#include <Arduino.h>
#include <Esp32PcntEncoder.h>

namespace {

// ---- 编码器引脚 (编码器0: 32/33, 编码器1: 26/25) ----
constexpr uint8_t ENC0_PIN_A = 32;
constexpr uint8_t ENC0_PIN_B = 33;
constexpr uint8_t ENC1_PIN_A = 26;
constexpr uint8_t ENC1_PIN_B = 25;

// ---- 串口与轮径标定参数 ----
constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率
constexpr uint32_t LOOP_DELAY_MS = 10;   // 采样间隔, 单位: ms

// ---- 可变全局状态 (跨 setup/loop 共享) ----
Esp32PcntEncoder encoders[2];

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    encoders[0].init(0, ENC0_PIN_A, ENC0_PIN_B);
    encoders[1].init(1, ENC1_PIN_A, ENC1_PIN_B);
}

void loop() {
    delay(LOOP_DELAY_MS);
    int32_t tick0 = encoders[0].getTicks();
    int32_t tick1 = encoders[1].getTicks();

    Serial.printf("tick1=%d,tick2=%d", tick0, tick1);
}
