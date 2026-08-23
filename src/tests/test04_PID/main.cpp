#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <PIDController.h>

namespace {

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 电机引脚 (电机0: 4/15, 电机1: 7/6) ----

constexpr uint8_t MOTOR0_PIN_A = 4;
constexpr uint8_t MOTOR0_PIN_B = 5;
constexpr uint8_t MOTOR1_PIN_A = 7;
constexpr uint8_t MOTOR1_PIN_B = 6;

// ---- 编码器引脚 (编码器0: 15/16, 编码器1: 18/17) ----

constexpr uint8_t ENC0_PIN_A = 15;
constexpr uint8_t ENC0_PIN_B = 16;
constexpr uint8_t ENC1_PIN_A = 18;
constexpr uint8_t ENC1_PIN_B = 17;

// ---- PID 参数 ----

constexpr float PID_KP = 0.625f;           // 比例增益
constexpr float PID_KI = 0.125f;           // 积分增益
constexpr float PID_KD = 0.0f;             // 微分增益
constexpr float PID_OUTPUT_LIMIT = 100.0f; // 输出限幅 ±100

// ---- 测试参数 ----

constexpr uint32_t LOOP_DELAY_MS = 10; // 控制周期, 单位 ms
constexpr float MS_TO_S = 1000.0f;     // mm/ms -> mm/s 换算系数

// ---- 编码器标定参数 ----
// 距离比时间获取速度: 当前速度 = delta_ticks * 单脉冲距离 / 时间差
// 单位: mm/ms, 等价于 m/s

constexpr float DISTANCE_PER_TICK_MM = 0.1427138f; // 单个脉冲对应的轮子前进距离, 单位 mm

// ---- 目标速度 ----

constexpr float TARGET_SPEED_MM_S = 100.0f; // 目标轮速, 单位 mm/s

// ---- 可变全局状态 (跨 setup/loop/motor_speed_control 共享) ----

Esp32McpwmMotor motor;           // 电机驱动对象 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器对象数组 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器对象数组 (setup/loop 共享)

// ---- 函数前向声明（内部链接） ----

void motor_speed_control();

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
    pid_controller[0].update_PID(PID_KP, PID_KI, PID_KD);
    pid_controller[1].update_PID(PID_KP, PID_KI, PID_KD);
    pid_controller[0].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT
    pid_controller[1].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT

    // 初始化目标速度, 单位 mm/s
    pid_controller[0].update_target(TARGET_SPEED_MM_S);
    pid_controller[1].update_target(TARGET_SPEED_MM_S);
}

void loop() {
    delay(LOOP_DELAY_MS);
    motor_speed_control();
}

namespace {

/**
 * @brief 电机速度控制函数
 *
 * 根据编码器差值计算当前轮速, 通过 PID 控制器更新电机 PWM 输出,
 * 并在串口打印当前速度。
 */
void motor_speed_control() {
    // 静态局部变量: 采样基线跨多次调用保持
    static uint64_t last_update_time = 0;  // 上一次更新时间
    static int64_t last_ticks[2] = {0, 0}; // 上一次读取的计数器数值
    static bool is_first_run = true;       // 首次进入标志

    if (is_first_run) {
        // 初始化采样基线, 避免首次控制周期时间差过大
        last_update_time = millis();
        last_ticks[0] = encoders[0].getTicks();
        last_ticks[1] = encoders[1].getTicks();
        is_first_run = false;
    }

    // 普通局部变量: 每次调用重新计算的临时量
    uint64_t now = millis();
    uint64_t dt = now - last_update_time;
    int32_t delta_ticks[2] = {0, 0};              // 两次读取之间的计数器差值
    float current_motor_speeds[2] = {0.0f, 0.0f}; // 当前两个电动机的速度, 单位 mm/s

    // 计算编码器差值
    delta_ticks[0] = static_cast<int32_t>(encoders[0].getTicks() - last_ticks[0]);
    delta_ticks[1] = static_cast<int32_t>(encoders[1].getTicks() - last_ticks[1]);

    // 距离比时间获取速度: delta_ticks * 单脉冲距离 / 时间差
    // 原始单位为 mm/ms, 乘以 1000 转换为 mm/s, 方便 PID 计算与观察
    if (dt != 0) {
        current_motor_speeds[0] = static_cast<float>(delta_ticks[0]) * DISTANCE_PER_TICK_MM /
                                  static_cast<float>(dt) * MS_TO_S;
        current_motor_speeds[1] = static_cast<float>(delta_ticks[1]) * DISTANCE_PER_TICK_MM /
                                  static_cast<float>(dt) * MS_TO_S;
    }

    // 更新上一次状态
    last_update_time = now;
    last_ticks[0] = encoders[0].getTicks();
    last_ticks[1] = encoders[1].getTicks();

    // 根据当前速度, 更新电机 0 和电机 1 的 PWM 输出
    motor.updateMotorSpeed(0, pid_controller[0].update_pwm(current_motor_speeds[0]));
    motor.updateMotorSpeed(1, pid_controller[1].update_pwm(current_motor_speeds[1]));

    // 输出数据
    Serial.printf(
        "speed1=%f mm/s, speed2=%f mm/s\n",
        current_motor_speeds[0],
        current_motor_speeds[1]
    );
}

} // namespace
