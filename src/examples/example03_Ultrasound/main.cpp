#include <Arduino.h>

namespace {

// ---- 引脚与串口 ----
constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率
constexpr uint8_t TRIG_PIN = 37;         // 超声波触发引脚
constexpr uint8_t ECHO_PIN = 21;         // 超声波回波引脚

// ---- 测距参数 ----
constexpr uint32_t TRIGGER_PULSE_US = 10;        // 触发脉冲宽度, 单位: us
constexpr float SOUND_SPEED_CM_PER_US = 0.0343f; // 声速, 单位: cm/us
constexpr float DISTANCE_DIVISOR = 2.0f;         // 声波往返, 距离需除 2
constexpr uint32_t MEASURE_INTERVAL_MS = 500;    // 测量间隔, 单位: ms

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
    // 发送高电平脉冲触发超声波模块
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(TRIGGER_PULSE_US);
    digitalWrite(TRIG_PIN, LOW);

    // 检测回波高电平持续时间, 单位: us
    const double echo_time_us = pulseIn(ECHO_PIN, HIGH);
    // 计算目标距离, 单位: cm
    const float distance_cm = echo_time_us * SOUND_SPEED_CM_PER_US / DISTANCE_DIVISOR;

    Serial.printf("distance: %f cm\n", distance_cm);
    delay(MEASURE_INTERVAL_MS);
}
