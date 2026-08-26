#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <SemanticEnums.h>

#include "config.h"

namespace {

// ---- 可变全局状态 (跨 setup/loop 共享) ----
// 编译期常量 (串口/引脚/测试参数) 见 lib/RobotConfig/config.h

Esp32McpwmMotor motor;

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);
}

void loop() {
    Serial.println("forward");
    motor.updateMotorSpeed(MOTOR_LEFT, MOTOR_SPEED);
    motor.updateMotorSpeed(MOTOR_RIGHT, MOTOR_SPEED);
    delay(STEP_DELAY_MS);

    Serial.println("backward");
    motor.updateMotorSpeed(MOTOR_LEFT, -MOTOR_SPEED);
    motor.updateMotorSpeed(MOTOR_RIGHT, -MOTOR_SPEED);
    delay(STEP_DELAY_MS);

    Serial.println("stop");
    motor.updateMotorSpeed(MOTOR_LEFT, 0);
    motor.updateMotorSpeed(MOTOR_RIGHT, 0);
    delay(STEP_DELAY_MS);
}
