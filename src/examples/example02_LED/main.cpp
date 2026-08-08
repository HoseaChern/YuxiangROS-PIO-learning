#include <Arduino.h>

namespace {

// ---- LED 引脚与闪烁参数 ----
constexpr uint8_t LED_PIN = 2;               // 板载 LED 引脚
constexpr uint32_t BLINK_INTERVAL_MS = 1000; // 亮/灭保持时间, 单位: ms

} // namespace

void setup() { pinMode(LED_PIN, OUTPUT); }

void loop() {
    // 输出为低电平有效: LOW 点亮, HIGH 熄灭
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_INTERVAL_MS);
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_INTERVAL_MS);
}
