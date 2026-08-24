#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PIDController.h>

namespace {

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 编码器引脚 (编码器0: 4/5, 编码器1: 14/15) ----

constexpr uint8_t ENC0_PIN_A = 4;
constexpr uint8_t ENC0_PIN_B = 5;
constexpr uint8_t ENC1_PIN_A = 14;
constexpr uint8_t ENC1_PIN_B = 15;

// ---- 电机引脚 (电机0: 10/11, 电机1: 12/13) ----

constexpr uint8_t MOTOR0_PIN_A = 10;
constexpr uint8_t MOTOR0_PIN_B = 11;
constexpr uint8_t MOTOR1_PIN_A = 12;
constexpr uint8_t MOTOR1_PIN_B = 13;

// ---- PID 参数 ----

constexpr float PID_KP = 0.625f;           // 比例增益
constexpr float PID_KI = 0.125f;           // 积分增益
constexpr float PID_KD = 0.0f;             // 微分增益
constexpr float PID_OUTPUT_LIMIT = 100.0f; // 输出限幅 ±100

// ---- 运动学参数 ----

constexpr float WHEEL_DISTANCE_MM = 175.0f;        // 轮间距, 单位 mm
constexpr float DISTANCE_PER_TICK_MM = 0.1427138f; // 单个脉冲对应的轮子前进距离, 单位 mm

// ---- 目标速度 ----

constexpr float TARGET_LINEAR_SPEED_MM_S = 50.0f;  // 目标线速度, 单位 mm/s
constexpr float TARGET_ANGULAR_SPEED_RAD_S = 0.1f; // 目标角速度, 单位 rad/s

// ---- 控制周期 ----

constexpr uint32_t LOOP_DELAY_MS = 10; // 主循环调度节拍, 单位 ms

// ---- 可变全局状态 (跨 setup/update_and_control 共享) ----

Esp32McpwmMotor motor;           // 电机驱动对象 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器对象数组 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器对象数组 (setup/loop 共享)
Kinematics kinematics;           // 运动学正逆解对象 (setup/loop 共享)

// ---- 函数前向声明（内部链接） ----

void update_and_control();

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化编码器
    encoders[0].init(0, ENC0_PIN_A, ENC0_PIN_B);
    encoders[1].init(1, ENC1_PIN_A, ENC1_PIN_B);

    // 初始化电动机
    motor.attachMotor(0, MOTOR0_PIN_A, MOTOR0_PIN_B);
    motor.attachMotor(1, MOTOR1_PIN_A, MOTOR1_PIN_B);

    // 初始化 PID 控制器参数
    pid_controller[0].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[1].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[0].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT
    pid_controller[1].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT

    // 初始化轮子间距和电动机参数
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);
    kinematics.set_motor_param(DISTANCE_PER_TICK_MM); // 标定量标量化: 两电机共用

    // 运动学逆解: 目标线速度和角速度 -> 目标左轮速度和右轮速度
    // 逆解输出仅本次使用, 声明为局部变量, 作用域最小化 (仿照 main.cpp)
    const float body_velocities[2] = {
        TARGET_LINEAR_SPEED_MM_S,
        TARGET_ANGULAR_SPEED_RAD_S
    }; // 车体速度: [0]=线速度 mm/s, [1]=角速度 rad/s
    float motor_speeds[2]; // 电机转速: [0]=左, [1]=右, 单位 mm/s, 仅用于本次逆解计算
    kinematics.kinematics_inverse(body_velocities, motor_speeds);

    // PID 初始化目标轮速
    pid_controller[0].update_target(motor_speeds[0]);
    pid_controller[1].update_target(motor_speeds[1]);
}

void loop() {
    delay(LOOP_DELAY_MS);
    update_and_control();

    Serial.printf(
        "x = %f, y = %f, yaw = %f\n",
        kinematics.get_odom().x,
        kinematics.get_odom().y,
        kinematics.get_odom().yaw
    );
}

namespace {

/**
 * @brief 更新编码器速度并通过 PID 控制电机输出
 *
 * 调用运动学 update_motor_speed 根据编码器 tick 计算当前轮速,
 * 再经 PID 控制器输出 PWM 值更新电机。
 */
void update_and_control() {
    const int32_t ticks[2] = {
        encoders[0].getTicks(),
        encoders[1].getTicks()
    }; // 编码器 tick: [0]=左, [1]=右
    kinematics.update_motor_speed(millis(), ticks);

    motor.updateMotorSpeed(0, pid_controller[0].update_pwm(kinematics.get_motor_speed(0)));
    motor.updateMotorSpeed(1, pid_controller[1].update_pwm(kinematics.get_motor_speed(1)));
}

} // namespace
