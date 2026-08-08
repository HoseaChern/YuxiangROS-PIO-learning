#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>

namespace {

// ---- 电机引脚 (电机0: 10/11, 电机1: 12/13) ----

constexpr uint8_t MOTOR0_PIN_A = 10;
constexpr uint8_t MOTOR0_PIN_B = 11;
constexpr uint8_t MOTOR1_PIN_A = 12;
constexpr uint8_t MOTOR1_PIN_B = 13;

// ---- 编码器引脚 (编码器0: 4/5, 编码器1: 14/15) ----

constexpr uint8_t ENC0_PIN_A = 4;
constexpr uint8_t ENC0_PIN_B = 5;
constexpr uint8_t ENC1_PIN_A = 14;
constexpr uint8_t ENC1_PIN_B = 15;

// ---- 串口与控制参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率
constexpr uint32_t LOOP_DELAY_MS = 10;   // 采样周期, 单位: ms
constexpr int MOTOR_SPEED = 70;          // 测试转速, 范围 [-100, 100]

// ---- 编码器标定参数 ----
// 距离比时间获取速度: 当前速度 = delta_ticks * 单脉冲距离 / 时间差
// 单位: mm/ms, 等价于 m/s

constexpr float DISTANCE_PER_TICK_MM = 0.1051566f;

// ---- 可变全局状态 (跨 setup/loop 共享) ----

Esp32McpwmMotor motor;        // 电机驱动对象
Esp32PcntEncoder encoders[2]; // 编码器对象数组

int64_t last_ticks[2];     // 上一次读取的计数器数值
int32_t delta_ticks[2];    // 两次读取之间的计数器差值
uint64_t last_update_time; // 上一次更新时间
float current_speeds[2];   // 当前两个电动机的速度

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化编码器
    encoders[0].init(0, ENC0_PIN_A, ENC0_PIN_B);
    encoders[1].init(1, ENC1_PIN_A, ENC1_PIN_B);

    // 初始化电动机并设置速度
    motor.attachMotor(0, MOTOR0_PIN_A, MOTOR0_PIN_B);
    motor.attachMotor(1, MOTOR1_PIN_A, MOTOR1_PIN_B);
    motor.updateMotorSpeed(0, MOTOR_SPEED);
    motor.updateMotorSpeed(1, MOTOR_SPEED);

    // 初始化采样基线, 避免首次 loop 时间差过大
    last_update_time = millis();
    last_ticks[0] = encoders[0].getTicks();
    last_ticks[1] = encoders[1].getTicks();
}

void loop() {
    delay(LOOP_DELAY_MS);

    // 计算时间差
    uint64_t now = millis();
    uint64_t dt = now - last_update_time;

    // 计算编码器差值
    delta_ticks[0] = static_cast<int32_t>(encoders[0].getTicks() - last_ticks[0]);
    delta_ticks[1] = static_cast<int32_t>(encoders[1].getTicks() - last_ticks[1]);

    // 距离比时间获取速度, 单位 mm/ms, 相当于 m/s
    if (dt != 0) {
        current_speeds[0] =
            static_cast<float>(delta_ticks[0] * DISTANCE_PER_TICK_MM) / static_cast<float>(dt);
        current_speeds[1] =
            static_cast<float>(delta_ticks[1] * DISTANCE_PER_TICK_MM) / static_cast<float>(dt);
    }

    // 更新数据
    last_update_time = now;
    last_ticks[0] = encoders[0].getTicks();
    last_ticks[1] = encoders[1].getTicks();

    // 输出数据
    Serial.printf("speed1=%fm/s,speed2=%fm/s\n", current_speeds[0], current_speeds[1]);
}
