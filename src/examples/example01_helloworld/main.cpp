#include <Arduino.h>

namespace {

// ---- 串口参数 ----
constexpr uint32_t SERIAL_BAUD = 115200;     // 串口波特率
constexpr uint32_t PRINT_INTERVAL_MS = 1000; // 打印间隔, 单位: ms

} // namespace

// setup 函数, 启动时仅调用一次
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }
}

// loop 函数, 启动后循环调用
void loop() {
    Serial.println("Hello World!");
    delay(PRINT_INTERVAL_MS);
}
