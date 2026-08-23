#include <Arduino.h>
#include <Esp32PcntEncoder.h>

namespace {

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 编码器引脚 (编码器0: 15/16, 编码器1: 18/17) ----

constexpr uint8_t ENC0_PIN_A = 15;
constexpr uint8_t ENC0_PIN_B = 16;
constexpr uint8_t ENC1_PIN_A = 18;
constexpr uint8_t ENC1_PIN_B = 17;

// ---- 测试参数 ----

constexpr uint32_t LOOP_DELAY_MS = 10; // 采样间隔, 单位: ms

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

    Serial.printf("tick1=%d,tick2=%d\n", tick0, tick1);
}

/**
 * 如何测量单次脉冲对应小车前进距离 l:
 * 
 * 预先准备:
 * 1. 测量小车的轮径 D, 单位: mm
 * 
 * 测量步骤:
 * 1. 编译并烧录当前程序
 * 2. 保持一轮固定, 转动另一轮, 使用串口监视器监视输出
 * 3. 转动 N 圈后, 记录当前被转动轮的 tick 数
 * 4. 根据公式计算即可: l = N * PI * D/ tick, PI 取较精确值即可
 * 
 * 最新测量值:
 * N = 10, tick = 14931, D = 68 mm
 * 得到:
 * l = 0.14307702 mm / tick
 */