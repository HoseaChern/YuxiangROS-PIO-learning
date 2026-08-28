/**
 * @file main.cpp
 * @brief test10: 两轮自平衡直立环固件 (阶段一: 仅直立环 PD)
 *
 * 功能: MPU6050 读姿态 -> 直立环 PD (D 项直接用陀螺仪角速度) -> 左右轮同值 PWM;
 *       串口命令: s=启停 / c=标定机械中值 / d=极性自检。
 *
 * 轴向: 小车前进方向为 X 负方向, pitch=-getAngleY() (前倾为正)。
 *
 * 详细设计说明 / 调参指南 / 后续速度环转向环计划见 docs/Balance_Car_Notes.md。
 * 编译/烧录: pio run -e test10_balance -t upload --no-monitor
 */

#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <MPU6050_light.h>
#include <PIDController.h>
#include <SemanticEnums.h>
#include <Wire.h>

#include "config.h"

namespace {

// ---- 固件本地常量 (跨固件共用参数见 lib/RobotConfig/config.h) ----

constexpr uint16_t CALIB_CYCLES = 40;   // 中值标定采样周期数 (40 * 5ms = 0.2s)
constexpr uint32_t SELF_TEST_MS = 1000; // 极性自检输出时长, 单位 ms
constexpr int16_t SELF_TEST_PWM = 100;  // 极性自检输出 PWM

enum class BalanceState : uint8_t {
    kIdle,    // 停止: 输出关闭, 等待武装且姿态进入中值窗口
    kRunning, // 直立控制: PD 闭环输出
};

// ---- 可变全局状态 (仅在 balance_task 中读写, 无跨任务竞争) ----

Esp32McpwmMotor motor;                            // 电机驱动对象
MPU6050 mpu(Wire);                                // MPU6050 对象, 使用 Wire 作为 I2C 总线
PIDController balance_pid;                        // 直立环 PD 控制器
BalanceState balance_state = BalanceState::kIdle; // 当前状态机状态
bool balance_armed = false;                       // 武装标志 ('s' 命令切换, 倒地自动解除)
float zero_pitch_deg = BALANCE_ZERO_PITCH_DEG;    // 机械中值, 可由 'c' 命令在线标定

uint16_t calib_remaining = 0;    // 中值标定剩余采样周期数, >0 表示正在采样
float calib_sum = 0.0f;          // 中值标定采样累加和
uint32_t self_test_until_ms = 0; // 极性自检截止时间戳 (millis), 0 表示未在进行

// ---- 函数前向声明 (内部链接) ----

void handle_serial_command(float pitch);
void control_step();
void balance_task(void* param);

} // namespace

void setup() {
    // 初始化调试串口 (115200), 等待 USB 串口就绪
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化 I2C 总线 (引脚见 config.h: IMU_SDA_PIN / IMU_SCL_PIN) 并探测 MPU6050
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    byte status = mpu.begin();
    if (status != 0) {
        // 探测失败 (I2C 地址无应答): 打印错误码后死循环, 便于排查接线
        Serial.printf("[IMU] MPU6050 initialise failed, status=%u, shutdown\n", status);
        while (true) {
            delay(1000);
        }
    }

    // 陀螺仪零偏校准: 期间必须保持小车静止平放, 校准结果作为角度零点基准
    Serial.println("[IMU] calibrating gyro offset, keep the car still and level...");
    delay(BALANCE_CALM_DELAY_MS); // 静置等待传感器稳定后再采样
    mpu.calcOffsets();
    Serial.println("[IMU] calibration done. Commands: s=arm/stop c=calibrate-zero d=test-polarity");

    // 初始化两路电机与直立环控制器: 目标值设为机械中值, 输出限幅对齐 PWM 范围
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);
    balance_pid.update_pid(BALANCE_KP, BALANCE_KI, BALANCE_KD);
    balance_pid.output_limit(BALANCE_PWM_LIMIT);
    balance_pid.update_target(zero_pitch_deg);

    // 创建控制任务: 5ms 固定节拍, 钉在 core1 避开 core0 的 WiFi 协议栈抖动
    xTaskCreatePinnedToCore(
        balance_task,
        "balance_task",
        BALANCE_STACK_SIZE,
        nullptr,
        BALANCE_TASK_PRIO,
        nullptr,
        BALANCE_TASK_CORE
    );
}

void loop() {
    delay(1000); // 控制与命令处理全部在 balance_task 中, 主循环空转
}

namespace {

/**
 * @brief 处理串口单字符命令 (每控制周期轮询一次)
 *
 * 命令表:
 *   's' 启动/停止切换: 武装后姿态进入中值窗口自动起控, 再按一次解除武装;
 *   'c' 标定机械中值: 仅停止状态有效, 手扶车体大致直立静止后发送, 取 0.2s 平均 pitch;
 *   'd' 极性自检: 仅停止状态有效, 两轮输出 +PWM 约 1s, 轮子应朝车头方向转动;
 *       若反向, 对调 config.h 中对应电机的 MOTOR_x_PIN_A/PIN_B 定义即可。
 *
 * @param pitch 当前 pitch 角 (deg), 'c' 标定时用于累加采样
 */
void handle_serial_command(float pitch) {
    while (Serial.available() > 0) {
        char cmd = static_cast<char>(Serial.read()); // 读入单字符命令
        switch (cmd) {
        case 's':
            // 切换武装标志, 并在串口回显当前状态 (纯文本, 无绘图依赖)
            balance_armed = !balance_armed;
            Serial.println(balance_armed ? "[CMD] armed, wait for pitch window" : "[CMD] stopped");
            break;

        case 'c':
        case 'd':
            // 标定与极性自检都要求车轮静止, 运行中直接忽略
            if (balance_state != BalanceState::kIdle) {
                Serial.println("[CMD] ignored: running");
                break;
            }
            if (cmd == 'c') {
                // 启动中值标定: 置剩余采样周期数, 清零累加和, 由 control_step 逐周期累加
                calib_remaining = CALIB_CYCLES;
                calib_sum = 0.0f;
                Serial.println("[CALIB] sampling 0.2s, hold the car upright...");
            } else {
                // 启动极性自检: 记录截止时间戳, control_step 在窗口内输出固定 PWM
                self_test_until_ms = millis() + SELF_TEST_MS;
                Serial.println("[CMD] polarity test: wheels should turn toward the front");
            }
            break;

        default:
            break; // 忽略回车/换行等其他字符
        }
    }
}

/**
 * @brief 单周期控制步骤: 读 IMU -> 命令处理 -> 标定/自检/状态机 -> PD 输出 -> 文本打印
 *
 * 由 balance_task 以 5ms 固定节拍调用。内部顺序即完整控制链路:
 * 先更新传感器数据, 再响应串口命令, 然后按状态机/自检标志决定本轮 PWM,
 * 最后以低频文本行打印状态供串口监视器观察。
 */
void control_step() {
    // 1. 更新姿态并转换到本固件符号约定: 前倾为正 (见文件头轴向说明)
    mpu.update();
    float pitch = -mpu.getAngleY();        // 俯仰角, 前倾为正
    float pitch_rate = -mpu.getGyroY();    // 俯仰角速度, 前倾趋势为正

    // 2. 处理串口命令 (可能启动标定或极性自检)
    handle_serial_command(pitch);

    // 3. 机械中值在线标定: 采样期间保持停止, 采样结束后取均值生效
    if (calib_remaining > 0) {
        calib_sum += pitch;                 // 累加当前 pitch
        calib_remaining--;                  // 剩余采样周期递减
        if (calib_remaining == 0) {
            // 采样完成: 更新机械中值并同步到 PID 目标值
            zero_pitch_deg = calib_sum / CALIB_CYCLES;
            balance_pid.update_target(zero_pitch_deg);
            Serial.printf("[CALIB] zero_pitch=%.2f deg\n", zero_pitch_deg);
        }
    }

    int16_t pwm_balance = 0; // 本周期实际输出的 PWM (默认 0)
    uint32_t now = millis();

    // 4. 输出决策: 极性自检优先于状态机, 窗口结束后恢复
    if (self_test_until_ms != 0) {
        // 极性自检: 窗口内两轮输出固定 PWM (不做闭环), 便于人工观察转向
        pwm_balance = SELF_TEST_PWM;
        if (now >= self_test_until_ms) {
            // 窗口结束: 关闭自检标志, 本轮 PWM 归零, 下一周期回到状态机
            self_test_until_ms = 0;
            pwm_balance = 0;
        }
        motor.updateMotorSpeed(MOTOR_LEFT, pwm_balance);
        motor.updateMotorSpeed(MOTOR_RIGHT, pwm_balance);
    } else {
        switch (balance_state) {
        case BalanceState::kIdle:
            // 停止态: 保持两轮输出关闭
            motor.updateMotorSpeed(MOTOR_LEFT, 0);
            motor.updateMotorSpeed(MOTOR_RIGHT, 0);
            // 起控条件: 已武装 且 |pitch-中值| 进入起控窗口 -> 复位控制器积分后起控
            if (balance_armed && fabsf(pitch - zero_pitch_deg) <= BALANCE_ARM_ANGLE_DEG) {
                balance_pid.reset(); // 清零积分/微分历史, 避免起控瞬间的旧状态冲击
                balance_state = BalanceState::kRunning;
                Serial.println("[STATE] running");
            }
            break;

        case BalanceState::kRunning:
            // 倒地保护: 姿态超出安全窗口 -> 解除武装并切回停止, 下一周期关闭输出
            if (fabsf(pitch - zero_pitch_deg) >= BALANCE_FALL_ANGLE_DEG) {
                balance_armed = false;
                balance_state = BalanceState::kIdle;
                Serial.println("[SAFE] fall detected, disarmed");
                break;
            }

            // 直立环输出: PID 内部 = KP*(中值-pitch) - KD*pitch_rate,
            // 取负后前倾(正)输出正 PWM, 两轮向车头方向转动接住重心
            float pid_inputs[2] = {pitch, pitch_rate}; // 索引见 PidInputID 枚举
            pwm_balance = static_cast<int16_t>(-balance_pid.update_pwm_with_rate(pid_inputs));
            motor.updateMotorSpeed(MOTOR_LEFT, pwm_balance);
            motor.updateMotorSpeed(MOTOR_RIGHT, pwm_balance);
            break;
        }
    }

    // 5. 低频状态打印 (100ms): 人类可读文本行, 供串口监视器观察与调参
    static uint32_t last_print_ms = 0;
    if (now - last_print_ms >= BALANCE_PRINT_MS) {
        last_print_ms = now;
        Serial.printf(
            "state=%s pitch=%.2f rate=%.2f pwm=%d\n",
            balance_state == BalanceState::kRunning ? "RUN" : "IDLE",
            pitch,
            pitch_rate,
            pwm_balance
        );
    }
}

/**
 * @brief 直立环控制任务: 以 BALANCE_PERIOD_MS 为固定节拍循环调用 control_step
 *
 * vTaskDelayUntil 保证节拍严格 5ms, 不受其他任务调度与 loop() 抖动影响;
 * 任务钉在 core1, 避开 core0 上 WiFi 协议栈的调度抖动。
 *
 * @param param 未使用 (xTaskCreatePinnedToCore 固定参数)
 */
void balance_task(void* param) {
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BALANCE_PERIOD_MS)); // 等下一个节拍
        control_step(); // 执行一个完整控制周期
    }
}

} // namespace
