/**
 * @file main.cpp
 * @brief test10_upright: 两轮自平衡直立环固件 (阶段一: 直立环 PD)
 *
 * 功能: MPU6050 读姿态 -> 直立环 PD -> 左右轮同值 PWM;
 *       串口命令: s=启停 / c=标定机械中值。
 *
 * 控制原理 (严格对应 docs/Balance_Car_Notes.md 直立环公式):
 *       PWM = Kp * (theta_0 - theta) - Kd * omega
 *   即 Kp*e_k - Kd*omega, D 项系数为负 (陀螺仪角速度, 见 1.4 符号推导)。
 *   方向约定: 小车前进方向为 x 负方向, 故"前倾"时 getAngleY 读数为负;
 *   直接采用传感器原始读数作为控制量: theta = mpu.getAngleY(), omega = mpu.getGyroY() (后倾为正)。
 *   PID 库契约不变 (误差 = 目标 - 实际, 微分项取 -rate, 见 1.4 符号推导), 极性经实机验证通过。
 *   前提: 正 PWM 驱动两轮向车头(即前进)方向转动; 若实测反向, 应反接电机而非取负。
 *   theta_0  = zero_pitch_deg  (机械中值, 'c' 在线标定)。
 *
 * 详细设计说明 / 调参指南 / 后续速度环转向环计划见 docs/Balance_Car_Notes.md。
 * 编译/烧录: pio run -e test10_upright -t upload
 */

#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <MPU6050_light.h>
#include <Wire.h>

#include <cmath>

#include <PIDController.h>
#include <SemanticEnums.h>

#include "config.h"

namespace {

// ---- 固件本地常量 (跨固件共用参数见 lib/RobotConfig/config.h) ----

enum class BalanceState : uint8_t {
    kIdle,    // 停止: 输出关闭, 等待武装且姿态进入中值窗口
    kRunning, // 直立控制: PD 闭环输出
};

// ---- 可变全局状态 (仅在 balance_task 中读写, 无跨任务竞争) ----

Esp32McpwmMotor motor;                            // 电机驱动对象
MPU6050 mpu(Wire);                                // MPU6050 对象, 使用 Wire 作为 I2C 总线
PIDController balance_pid;                        // 直立环 PD 控制器 (P/I/D 参数在 setup 中配置)
BalanceState balance_state = BalanceState::kIdle; // 当前状态机状态
bool balance_armed = false;                       // 武装标志 ('s' 命令切换, 倒地自动解除)
float zero_pitch_deg = UPRIGHT_ZERO_PITCH_DEG;    // 机械中值 theta_0, 可由 'c' 命令在线标定

// ---- 函数前向声明 (内部链接) ----

void handle_serial_command(float theta);
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
            delay(IMU_FAIL_LOOP_MS);
        }
    }

    // 陀螺仪零偏校准: 期间必须保持小车静止平放, 校准结果作为角度零点基准
    Serial.println("[IMU] calibrating gyro offset, keep the car still and level...");
    delay(UPRIGHT_CALM_DELAY_MS); // 静置等待传感器稳定后再采样
    mpu.calcOffsets();
    Serial.println("[IMU] calibration done. Commands: s=arm/stop c=calibrate-zero");

    // 初始化两路电机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);

    // 配置直立环 PD 控制器: 库层强制纯 PD 无 I 项 (update_pwm_upright 忽略 ki_), 输出限幅对齐 MCPWM 占空比范围
    // update_pwm_upright 的 D 项取 -rate, 传陀螺仪角速度即得 -Kd*omega (见 1.4 符号推导)
    balance_pid.update_pid(UPRIGHT_KP, UPRIGHT_KI, UPRIGHT_KD);
    balance_pid.output_limit(UPRIGHT_PWM_LIMIT);

    // 创建控制任务: 5ms 固定节拍, 钉在 core1 避开 core0 的 WiFi 协议栈抖动
    // 参数依次为: 任务函数, 任务名称, 任务堆栈字节数, 传递给任务函数的参数, 任务优先级, 任务句柄, 绑定的 CPU 核号
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
    delay(IDLE_LOOP_MS); // 控制与命令处理全部在 balance_task 中, 主循环空转
}

namespace {

/**
 * @brief 处理串口单字符命令 + 机械中值在线标定采样 (每控制周期轮询一次)
 *
 * 命令表:
 *   's' 启动/停止切换: 武装后姿态进入中值窗口自动起控, 再按一次解除武装;
 *   'c' 标定机械中值: 仅停止状态有效, 手扶车体大致直立静止后发送。
 *       随后每个控制周期由本函数逐周期累加 theta, 取 0.2s 均值作为 theta_0,
 *       采样期间小车保持静止, 完成即自动生效并打印结果。
 *
 * @param theta 当前控制角 theta (deg, 后倾为正), 'c' 标定时用于累加采样
 */
void handle_serial_command(float theta) {
    // 中值标定采样状态仅本函数内使用, 按最小作用域原则设为函数内局部静态 (跨周期保留)
    static uint16_t calib_remaining = 0; // 剩余采样周期数, >0 表示正在采样
    static float calib_sum = 0.0f;       // 采样累加和

    // 机械中值标定采样: 逐周期累加, 与命令轮询共用同一次周期调用
    if (calib_remaining > 0) {
        calib_sum += theta; // 累加当前 theta
        calib_remaining--;  // 剩余采样周期递减
        if (calib_remaining == 0) {
            // 采样完成: 更新机械中值 theta_0
            zero_pitch_deg = calib_sum / UPRIGHT_CALIB_CYCLES;
            Serial.printf("[CALIB] zero_pitch=%.2f deg\n", zero_pitch_deg);
        }
    }

    while (Serial.available() > 0) {
        char cmd = static_cast<char>(Serial.read()); // 读入单字符命令
        switch (cmd) {
        case 's':
            // 切换武装标志, 并在串口回显当前状态 (纯文本, 无绘图依赖)
            balance_armed = !balance_armed;
            Serial.println(balance_armed ? "[CMD] armed, wait for pitch window" : "[CMD] stopped");
            break;

        case 'c':
            // 标定要求车轮静止, 运行中直接忽略
            if (balance_state != BalanceState::kIdle) {
                Serial.println("[CMD] ignored: running");
                break;
            }
            // 启动中值标定: 置剩余采样周期数, 清零累加和, 由本函数逐周期累加
            calib_remaining = UPRIGHT_CALIB_CYCLES;
            calib_sum = 0.0f;
            Serial.println("[CALIB] sampling 0.2s, hold the car upright...");
            break;

        default:
            break; // 忽略回车/换行等其他字符
        }
    }
}

/**
 * @brief 单周期控制步骤: 读 IMU -> 命令处理(含在线标定采样) -> 状态机 -> PD 输出 -> 文本打印
 *
 * 由 balance_task 以 5ms 固定节拍调用。内部顺序即完整控制链路:
 * 先更新传感器数据(直接采用原始读数, 后倾为正), 再响应串口命令
 * (机械中值在线标定内聚于 handle_serial_command), 然后按状态机决定本轮 PWM,
 * 最后以低频文本行打印状态供串口监视器观察。
 */
void control_step() {
    // 1. 更新姿态: 直接采用传感器原始读数 (后倾为正, 见文件头方向约定)
    mpu.update();
    const float theta = mpu.getAngleY(); // theta: 控制俯仰角 (deg), 后倾为正
    const float omega = mpu.getGyroY();  // omega: 控制角速度 (deg/s), 后倾方向为正

    // 2. 处理串口命令
    handle_serial_command(theta);

    int16_t pwm_balance = 0; // 本周期实际输出的 PWM (默认 0)

    // 3. 输出决策: 按状态机决定本轮 PWM
    switch (balance_state) {
    case BalanceState::kIdle:
        // 停止态: 保持两轮输出关闭
        motor.updateMotorSpeed(MOTOR_LEFT, 0);
        motor.updateMotorSpeed(MOTOR_RIGHT, 0);

        // 起控条件: 已武装 且 |theta - theta_0| 进入起控窗口 -> 起控
        if (balance_armed && fabsf(theta - zero_pitch_deg) <= UPRIGHT_ARM_ANGLE_DEG) {
            balance_pid.reset(); // 清零 PID 内部状态 (error 差分/积分), 避免上次残留
            balance_state = BalanceState::kRunning;
            Serial.println("[STATE] running");
        }
        break;

    case BalanceState::kRunning:
        // 倒地保护: 姿态超出安全窗口 -> 解除武装并切回停止, 下一周期关闭输出
        if (fabsf(theta - zero_pitch_deg) >= UPRIGHT_FALL_ANGLE_DEG) {
            balance_armed = false;
            balance_state = BalanceState::kIdle;
            Serial.println("[SAFE] fall detected, disarmed");
            break;
        }

        // 直立环输出: PWM = Kp*(theta_0 - theta) - Kd*omega
        // update_pwm_upright 目标角度直接入参: 此处为机械中值 theta_0
        const float inputs[2] = {theta, omega}; // [当前角度, 当前角速度]
        // = Kp*(theta_0 - theta) - Kd*omega
        pwm_balance = balance_pid.update_pwm_upright(zero_pitch_deg, inputs);

        motor.updateMotorSpeed(MOTOR_LEFT, pwm_balance);
        motor.updateMotorSpeed(MOTOR_RIGHT, pwm_balance);
        break;
    }

    // 5. 低频状态打印: 100ms 由宏 BALANCE_PRINT_MS 体现, 定义于 lib/RobotConfig/config.h (值 100 即 10Hz);
    //    下方 if 判断 "now - last_print_ms >= BALANCE_PRINT_MS" 即 100ms 到点才整行打印一次, 供串口监视器观察。
    const uint32_t now = millis();
    static uint32_t last_print_ms = 0;
    if (now - last_print_ms >= BALANCE_PRINT_MS) {
        last_print_ms = now;
        Serial.printf(
            "state=%s theta=%.2f omega=%.2f pwm=%d\n",
            balance_state == BalanceState::kRunning ? "RUN" : "IDLE",
            theta,
            omega,
            pwm_balance
        );
    }
}

/**
 * @brief 直立环控制任务: 定时调度器, 每 BALANCE_PERIOD_MS (见 config.h) 调用一次 control_step
 *
 * 任务与 control_step 的关系: balance_task 只负责"定时唤起", 每个固定节拍唤醒后调一次
 * control_step 执行完整控制链路 (读 IMU -> PD -> 输出); 本任务钉在 core1, 避开 core0 的 WiFi 抖动。
 *
 * @param param 未使用 (xTaskCreatePinnedToCore 固定参数)
 */
void balance_task(void* param) {
    // xTaskGetTickCount(): 无参数, 返回调度器启动以来累计的 tick 数, 用作"当前时刻"基准
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        // vTaskDelayUntil 参数依次为: 上次唤醒时刻指针, 延时 tick 数
        // pdMS_TO_TICKS 参数: 毫秒值 (BALANCE_PERIOD_MS=5), 返回换算后的 tick 数 (5ms -> 5 tick)
        // vTaskDelayUntil 执行"绝对"延时: 锚定到"上一唤醒时刻 + 延时"这个绝对时间点, 自带last_wake更新,
        //                   不受本周期执行耗时影响, 从而让控制节拍稳定不漂移 (区别于 vTaskDelay 相对延时)。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BALANCE_PERIOD_MS));
        control_step(); // 唤醒后执行一个完整控制周期 (读 IMU -> PD -> 输出)
    }
}

} // namespace
