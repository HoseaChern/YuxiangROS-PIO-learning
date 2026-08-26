#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PIDController.h>
#include <SemanticEnums.h>

#include "config.h"

namespace {

// ---- 可变全局状态 (跨 setup/update_and_control 共享) ----
// 编译期常量 (串口/引脚/PID/运动学/目标速度/控制周期) 见 lib/RobotConfig/config.h

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
    encoders[MOTOR_LEFT].init(MOTOR_LEFT, ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
    encoders[MOTOR_RIGHT].init(MOTOR_RIGHT, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

    // 初始化电动机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);

    // 初始化 PID 控制器参数
    pid_controller[MOTOR_LEFT].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[MOTOR_RIGHT].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[MOTOR_LEFT].output_limit(PID_OUTPUT_LIMIT);  // 对称输出限幅 ±PID_OUTPUT_LIMIT
    pid_controller[MOTOR_RIGHT].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT

    // 初始化轮子间距和电动机参数
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);
    kinematics.set_motor_param(DISTANCE_PER_TICK_MM); // 标定量标量化: 两电机共用

    // 运动学逆解: 目标线速度和角速度 -> 目标左轮速度和右轮速度
    // 逆解输出仅本次使用, 声明为局部变量, 作用域最小化 (仿照 main.cpp)
    // 车体速度: [VEL_LINEAR]=线速度 mm/s, [VEL_ANGULAR]=角速度 rad/s
    const float body_velocities[2] = {TARGET_LINEAR_SPEED_MM_S, TARGET_ANGULAR_SPEED_RAD_S};
    // 电机转速: [MOTOR_LEFT]=左, [MOTOR_RIGHT]=右, 单位 mm/s, 仅用于本次逆解计算
    float motor_speeds[2];
    kinematics.kinematics_inverse(body_velocities, motor_speeds);

    // PID 初始化目标轮速
    pid_controller[MOTOR_LEFT].update_target(motor_speeds[MOTOR_LEFT]);
    pid_controller[MOTOR_RIGHT].update_target(motor_speeds[MOTOR_RIGHT]);
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
    // 编码器 tick: [MOTOR_LEFT]=左, [MOTOR_RIGHT]=右
    const int32_t ticks[2] = {encoders[MOTOR_LEFT].getTicks(), encoders[MOTOR_RIGHT].getTicks()};
    kinematics.update_motor_speed(millis(), ticks);

    motor.updateMotorSpeed(
        MOTOR_LEFT,
        pid_controller[MOTOR_LEFT].update_pwm(kinematics.get_motor_speed(MOTOR_LEFT))
    );
    motor.updateMotorSpeed(
        MOTOR_RIGHT,
        pid_controller[MOTOR_RIGHT].update_pwm(kinematics.get_motor_speed(MOTOR_RIGHT))
    );
}

} // namespace
