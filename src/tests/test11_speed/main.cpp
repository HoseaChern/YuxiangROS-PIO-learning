/**
 * @file main.cpp
 * @brief test11_speed: 两轮自平衡串级固件 (阶段二: 速度环 PI + 直立环 PD 串级)
 *
 * 功能: MPU6050 读姿态 + 编码器测速 -> 速度环 PI -> 直立环 PD -> 左右轮同值 PWM;
 *       串口命令: s=启停 / c=标定机械中值 / w=目标速度+10 / x=目标速度-10 / v=显示目标速度。
 *
 * 控制原理 (严格对应 docs/Balance_Car_Notes.md 3.3 串级公式):
 *   速度环(外环, PI):  speed_output = Kp'*(v_set - v) + Ki'*Σe_j, 输出为期望角度增量 (deg)
 *   直立环(内环, PD):  PWM = Kp*(target_angle - theta) - Kd*omega
 *   串级嵌套:           target_angle = speed_output + theta_0
 *   其中 v_set 为期望车体速度 (mm/s, 串口 'w'/'x' 调整), v 为编码器反馈速度 (两轮平均, mm/s)。
 *
 * 方向约定 (沿用 test10):
 *   小车前进方向为 x 负方向, "前倾"时 getAngleY 读数为正;
 *   控制坐标系统一"前倾为正": theta = -mpu.getAngleY(), omega = -mpu.getGyroY()。
 *   正 PWM 驱动两轮向车头(即前进)方向转动; 若实测反向, 应反接电机而非取负。
 *   编码器读数方向: 前进时 getTicks 递增, 故前进速度 v 为正; 若实测反向, 应取负或反接。
 *   theta_0 = zero_pitch_deg ('c' 在线标定)。
 *
 * 详细设计说明 / 调参指南见 docs/Balance_Car_Notes.md。
 * 编译/烧录: pio run -e test11_speed -t upload
 */

#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <MPU6050_light.h>
#include <Wire.h>

#include <cmath>

#include <Kinematics.h>
#include <PIDController.h>
#include <SemanticEnums.h>

#include "config.h"

namespace {

// ---- 固件本地常量 (跨固件共用参数见 lib/RobotConfig/config.h) ----

enum class BalanceState : uint8_t {
    kIdle,    // 停止: 输出关闭, 等待武装且姿态进入中值窗口
    kRunning, // 直立控制: 速度环 + 直立环串级闭环输出
};

// ---- 可变全局状态 (仅在 balance_task 中读写, 无跨任务竞争) ----

Esp32McpwmMotor motor;                            // 电机驱动对象
Esp32PcntEncoder encoders[2];                     // 编码器对象数组 (两轮)
MPU6050 mpu(Wire);                                // MPU6050 对象, 使用 Wire 作为 I2C 总线
PIDController speed_pid;                          // 速度环 PI 控制器 (外环, 参数在 setup 中配置)
PIDController balance_pid;                        // 直立环 PD 控制器 (内环, 参数在 setup 中配置)
Kinematics kinematics;                            // 运动学对象: 编码器测速 + 正解
BalanceState balance_state = BalanceState::kIdle; // 当前状态机状态
bool balance_armed = false;                       // 武装标志 ('s' 命令切换, 倒地自动解除)
float zero_pitch_deg = BALANCE_ZERO_PITCH_DEG;    // 机械中值 theta_0, 可由 'c' 命令在线标定
float target_speed_mm_s = SPEED_SETPOINT_MM_S;    // 期望车体速度 v_set, 可由 'w'/'x' 命令调整

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
    delay(BALANCE_CALM_DELAY_MS); // 静置等待传感器稳定后再采样
    mpu.calcOffsets();
    Serial.println(
        "[IMU] calibration done. Commands: s=arm/stop c=calibrate-zero w=+speed x=-speed"
    );

    // 初始化两路编码器
    encoders[MOTOR_LEFT].init(MOTOR_LEFT, ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
    encoders[MOTOR_RIGHT].init(MOTOR_RIGHT, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

    // 配置运动学参数: 单脉冲距离与轮间距
    kinematics.set_motor_param(DISTANCE_PER_TICK_MM);
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);

    // 初始化两路电机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);

    // 配置速度环 PI 控制器 (外环, 无 D 项): 输出为期望角度增量 (deg), 限幅防目标角过大失衡
    speed_pid.update_pid(SPEED_KP, SPEED_KI, SPEED_KD);
    speed_pid.output_limit(SPEED_OUTPUT_LIMIT);

    // 配置直立环 PD 控制器: 库层强制纯 PD 无 I 项 (update_pwm_upright 忽略 ki_), 输出限幅对齐 MCPWM 占空比范围
    // update_pwm_upright 的 D 项取 -rate, 传陀螺仪角速度即得 -Kd*omega (见 1.4 符号推导)
    balance_pid.update_pid(BALANCE_KP, BALANCE_KI, BALANCE_KD);
    balance_pid.output_limit(BALANCE_PWM_LIMIT);

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
 *   'w'/'x' 目标速度调整: 每按一次按 SPEED_STEP_MM_S 步进加减 (运行中/停止均可)。
 *   'v' 显示当前目标速度与反馈速度。
 *
 * @param theta 当前控制角 theta (deg, 前倾为正), 'c' 标定时用于累加采样
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
            zero_pitch_deg = calib_sum / BALANCE_CALIB_CYCLES;
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
            calib_remaining = BALANCE_CALIB_CYCLES;
            calib_sum = 0.0f;
            Serial.println("[CALIB] sampling 0.2s, hold the car upright...");
            break;

        case 'w':
            // 目标速度 +步进
            target_speed_mm_s += SPEED_STEP_MM_S;
            Serial.printf("[CMD] target speed=%.1f mm/s\n", target_speed_mm_s);
            break;

        case 'x':
            // 目标速度 -步进
            target_speed_mm_s -= SPEED_STEP_MM_S;
            Serial.printf("[CMD] target speed=%.1f mm/s\n", target_speed_mm_s);
            break;

        case 'v':
            // 显示当前目标速度
            Serial.printf("[CMD] target speed=%.1f mm/s\n", target_speed_mm_s);
            break;

        default:
            break; // 忽略回车/换行等其他字符
        }
    }
}

/**
 * @brief 单周期控制步骤: 读 IMU + 测速 -> 命令处理(含在线标定采样) -> 状态机 -> 串级输出 -> 文本打印
 *
 * 由 balance_task 以 5ms 固定节拍调用。内部顺序即完整控制链路:
 * 先更新传感器数据(读取处对 theta/omega 取负, 统一"前倾为正")与编码器速度,
 * 再响应串口命令 (机械中值在线标定内聚于 handle_serial_command), 然后按状态机决定本轮 PWM
 * (运行态执行 速度环 PI -> 直立环 PD 串级), 最后以低频文本行打印状态供串口监视器观察。
 */
void control_step() {
    // 1. 更新姿态: 控制坐标系统一为"前倾为正" (前进方向为 -X, 见文件头方向约定)
    mpu.update();
    const float theta = -mpu.getAngleY(); // theta: 控制俯仰角 (deg), 前倾为正
    const float omega = -mpu.getGyroY();  // omega: 控制角速度 (deg/s), 前倾方向为正

    // 2. 编码器测速 + 运动学正解: 车体前进速度 v (mm/s), 用作速度环反馈
    int32_t ticks[2] = {encoders[MOTOR_LEFT].getTicks(), encoders[MOTOR_RIGHT].getTicks()};
    kinematics.update_motor_speed(millis(), ticks);
    float motor_speeds[2] = {
        kinematics.get_motor_speed(MOTOR_LEFT),
        kinematics.get_motor_speed(MOTOR_RIGHT)
    };
    float body_vel[2] = {0.0f, 0.0f};
    kinematics.kinematics_forward(motor_speeds, body_vel);
    const float speed_mm_s = body_vel[VEL_LINEAR];

    // 3. 处理串口命令
    handle_serial_command(theta);

    int16_t pwm_balance = 0; // 本周期实际输出的 PWM (默认 0)

    // 4. 输出决策: 按状态机决定本轮 PWM
    switch (balance_state) {
    case BalanceState::kIdle:
        // 停止态: 保持两轮输出关闭
        motor.updateMotorSpeed(MOTOR_LEFT, 0);
        motor.updateMotorSpeed(MOTOR_RIGHT, 0);

        // 起控条件: 已武装 且 |theta - theta_0| 进入起控窗口 -> 起控
        if (balance_armed && fabsf(theta - zero_pitch_deg) <= BALANCE_ARM_ANGLE_DEG) {
            // 清零两环 PID 内部状态 (误差差分/积分), 避免上次残留
            balance_pid.reset();
            speed_pid.reset();
            balance_state = BalanceState::kRunning;
            Serial.println("[STATE] running");
        }
        break;

    case BalanceState::kRunning:
        // 倒地保护: 姿态超出安全窗口 -> 解除武装并切回停止, 下一周期关闭输出
        if (fabsf(theta - zero_pitch_deg) >= BALANCE_FALL_ANGLE_DEG) {
            balance_armed = false;
            balance_state = BalanceState::kIdle;
            Serial.println("[SAFE] fall detected, disarmed");
            break;
        }

        // 速度环 (外环, PI): 期望速度 target_speed_mm_s, 反馈两轮平均 speed_mm_s
        // 输出为期望角度增量 (deg): speed_output = Kp'*(v_set - v) + Ki'*Σe
        const int16_t speed_output = speed_pid.update_pwm_speed(target_speed_mm_s, speed_mm_s);

        // 串级嵌套: 直立环目标角度 = 速度环输出 + 机械中值 theta_0 (docs 3.3 公式 ③)
        const float target_angle = static_cast<float>(speed_output) + zero_pitch_deg;

        // 直立环 (内环, PD): PWM = Kp*(target_angle - theta) - Kd*omega
        const float inputs[2] = {theta, omega}; // [当前角度, 当前角速度]
        // = Kp*(target_angle - theta) - Kd*omega
        pwm_balance = balance_pid.update_pwm_upright(target_angle, inputs);

        motor.updateMotorSpeed(MOTOR_LEFT, pwm_balance);
        motor.updateMotorSpeed(MOTOR_RIGHT, pwm_balance);
        break;
    }

    // 6. 低频状态打印: 100ms 由宏 BALANCE_PRINT_MS 体现, 定义于 lib/RobotConfig/config.h (值 100 即 10Hz);
    //    下方 if 判断 "now - last_print_ms >= BALANCE_PRINT_MS" 即 100ms 到点才整行打印一次, 供串口监视器观察。
    const uint32_t now = millis();
    static uint32_t last_print_ms = 0;
    if (now - last_print_ms >= BALANCE_PRINT_MS) {
        last_print_ms = now;
        Serial.printf(
            "state=%s theta=%.2f omega=%.2f speed=%.1f target=%.1f pwm=%d\n",
            balance_state == BalanceState::kRunning ? "RUN" : "IDLE",
            theta,
            omega,
            speed_mm_s,
            target_speed_mm_s,
            pwm_balance
        );
    }
}

/**
 * @brief 串级控制任务: 定时调度器, 每 BALANCE_PERIOD_MS (见 config.h) 调用一次 control_step
 *
 * 任务与 control_step 的关系: balance_task 只负责"定时唤起", 每个固定节拍唤醒后调一次
 * control_step 执行完整控制链路 (读 IMU + 测速 -> 串级 -> 输出); 本任务钉在 core1, 避开 core0 的 WiFi 抖动。
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
        control_step(); // 唤醒后执行一个完整控制周期 (读 IMU + 测速 -> 串级 -> 输出)
    }
}

} // namespace
