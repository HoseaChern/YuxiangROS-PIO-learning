#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <SemanticEnums.h>

namespace {

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 电机引脚 (电机0: 4/5, 电机1: 7/6) ----

constexpr uint8_t MOTOR_LEFT_PIN_A = 4;
constexpr uint8_t MOTOR_LEFT_PIN_B = 5;
constexpr uint8_t MOTOR_RIGHT_PIN_A = 7;
constexpr uint8_t MOTOR_RIGHT_PIN_B = 6;

// ---- 测试参数 ----

constexpr uint32_t STEP_DELAY_MS = 2000; // 每个方向保持时间, 单位: ms
constexpr int16_t MOTOR_SPEED = 70;      // 测试转速, 范围 [-100, 100]

// ---- 可变全局状态 (跨 setup/loop 共享) ----

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
