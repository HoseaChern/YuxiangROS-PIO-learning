/**
 * @file main.cpp
 * @brief test12_turn: 两轮自平衡转向固件 (阶段三: 转向环差模叠加, 承接 test11 串级)
 *
 * 功能: 在 test11 (速度环 PI + 直立环 PD 串级) 基础上新增转向环, 差速量 Δ 对称叠加进左右轮;
 *       两种转向模式: 抑制转向 (走直线阻尼, 默认) 与 开环转动 (串口 'l'/'r' 步进转角指令)。
 *
 * 控制原理 (严格对应 docs/Balance_Car_Notes.md 5.2/5.3):
 *   共模: pwm_base = balance_pwm + speed_pwm (直立环 PD + 速度环 PI 串级, 同 test11)
 *   差模: Δ = 转向环输出 (模式 A 抑制 / 模式 B 开环), 合成 pwm_L = base + Δ, pwm_R = base - Δ
 *   模式 A 抑制转向: Δ = -TURN_KD*ωz, 复用 update_pwm (target=0, kp 装 TURN_KD), ωz = Gyro_Z
 *   模式 B 开环转动: Δ = TURN_KP*θ_target, 用 update_pwm_turn_openloop (docs 5.3 模式 B)
 *
 * 方向约定 (沿用 test10/test11):
 *   前进方向为 -X, "前倾"时 getAngleY 为正; 控制坐标系统一"前倾为正": theta = -getAngleY(),
 *   omega_pitch = -getGyroY(), omega_z = getGyroZ()。
 *   转向差速符号 (docs 5.2): pwm_L = base + Δ 且 Δ>0 → 左轮快 → 右转 (顺时针);
 *   故左转指令取负、右转取正; 实测反向时调换 'l'/'r' 符号即可。模式 A 符号自洽:
 *   车左转时 ωz>0, Δ = -TURN_KD*ωz < 0 → 右轮快 → 反向阻尼, 无需额外取负。
 *   正 PWM 驱动两轮向车头(前进)方向转动; 若实测反向应反接电机而非取负。
 *
 * 串口命令: s=启停 / c=标定机械中值 / w=目标速度+ / x=目标速度- / v=显示目标速度
 *          / t=切换转向模式 / l=左转步进 / r=右转步进 / o=开环转角归零。
 * 详细设计 / 调参指南见 docs/Balance_Car_Notes.md 6。编译: pio run -e test12_turn -t upload
 */

#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <MPU6050_light.h>
#include <Wire.h>

#include <cmath>

#include <PIDController.h>
#include <SemanticEnums.h>

#include "config.h"

namespace {

// ---- 固件本地常量 (跨固件共用参数见 lib/RobotConfig/config.h) ----

enum class BalanceState : uint8_t {
    kIdle,    // 停止: 输出关闭, 等待武装且姿态进入中值窗口
    kRunning, // 直立控制: 速度环 + 直立环串级 + 转向环差模叠加输出
};

enum class TurnMode : uint8_t {
    kStraight,     // 抑制转向 (走直线): Δ = -TURN_KD*ωz, 阻尼自发偏航 (docs 5.3 模式 A)
    kOpenloopTurn, // 开环转动: Δ = TURN_KP*θ_target, 跟踪串口转角指令 (docs 5.3 模式 B)
};

// ---- 可变全局状态 (仅在 balance_task 中读写, 无跨任务竞争) ----

Esp32McpwmMotor motor;        // 电机驱动对象
Esp32PcntEncoder encoders[2]; // 编码器对象数组 (两轮)
MPU6050 mpu(Wire);            // MPU6050 对象, 使用 Wire 作为 I2C 总线
PIDController speed_pid;      // 速度环 PI 控制器 (外环, 参数在 setup 中配置)
PIDController balance_pid;    // 直立环 PD 控制器 (内环, 参数在 setup 中配置)
PIDController turn_pid;       // 转向环控制器 (差模, 参数随模式在 setup/'t' 中配置)
BalanceState balance_state = BalanceState::kIdle; // 当前状态机状态
bool balance_armed = false;                       // 武装标志 ('s' 命令切换, 倒地自动解除)
float zero_pitch_deg = BALANCE_ZERO_PITCH_DEG;    // 机械中值 theta_0, 可由 'c' 命令在线标定
float target_speed_mm_s = SPEED_SETPOINT_MM_S;    // 期望车体速度 v_set, 可由 'w'/'x' 命令调整
TurnMode turn_mode = TurnMode::kStraight;         // 当前转向模式, 可由 't' 命令切换
float turn_target_angle_deg = 0.0f;               // 开环期望转角 θ_target, 'l'/'r' 步进 ('o' 归零)

// ---- 函数前向声明 (内部链接) ----

void handle_serial_command(float theta);
float measure_speed_mm_s();
void configure_turn_pid();
void control_step();
void balance_task(void* param);

} // namespace

void setup() {
    // 初始化调试串口 (115200), 等待 USB 串口就绪
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化 I2C 总线 (引脚见 config.h: IMU_SDA_PIN / IMU_SCL_PIN) 并探测 MPU6050
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    byte status = mpu.begin();
    if (status != 0) {
        // 探测失败 (I2C 地址无应答): 打印错误码后死循环, 便于排查接线
        Serial.printf("[IMU] MPU6050 initialise failed, status=%u, shutdown\n", status);
        while (true) {
            delay(1000);
        }
    }

    // 陀螺仪零偏校准: 期间必须保持小车静止平放, 校准结果作为角度零点基准
    Serial.println("[IMU] calibrating gyro offset, keep the car still and level...");
    delay(BALANCE_CALM_DELAY_MS); // 静置等待传感器稳定后再采样
    mpu.calcOffsets();
    Serial.println(
        "[IMU] calibration done. Commands: s=arm/stop c=calibrate-zero w=+speed x=-speed "
        "t=switch-turn-mode l=left-turn r=right-turn o=zero-turn"
    );

    // 初始化两路编码器
    encoders[MOTOR_LEFT].init(MOTOR_LEFT, ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
    encoders[MOTOR_RIGHT].init(MOTOR_RIGHT, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

    // 初始化两路电机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);

    // 配置速度环 PI 控制器 (外环, 无 D 项): 输出为期望角度增量 (deg), 限幅防目标角过大失衡
    speed_pid.update_pid(SPEED_KP, SPEED_KI, 0.0f);
    speed_pid.output_limit(SPEED_OUTPUT_LIMIT);

    // 配置直立环 PD 控制器: 库层强制纯 PD (update_pwm_upright 忽略 ki_), 输出限幅对齐 MCPWM 占空比
    balance_pid.update_pid(BALANCE_KP, BALANCE_KI, BALANCE_KD);
    balance_pid.output_limit(BALANCE_PWM_LIMIT);

    // 配置转向环 (差模): 默认抑制模式, Δ 经 update_pwm 以 target=0 复用 (docs 5.3 模式 A)
    configure_turn_pid();

    // 创建控制任务: 5ms 固定节拍, 钉在 core1 避开 core0 的 WiFi 协议栈抖动
    xTaskCreatePinnedToCore(
        balance_task,
        "balance_task",
        BALANCE_STACK_SIZE,
        nullptr,
        BALANCE_TASK_PRIO,
        nullptr,
        BALANCE_TASK_CORE
    );
}

void loop() {
    delay(1000); // 控制与命令处理全部在 balance_task 中, 主循环空转
}

namespace {

/**
 * @brief 按当前转向模式配置转向环 PID (增益与输出限幅, 切换模式时先 reset 清状态)
 *
 * 模式 A (kStraight): kp 装 TURN_KD, ki=kd=0, target 固定 0, 用 update_pwm 得 Δ = -TURN_KD*ωz
 *   (抑制转向复用通用 update_pwm, 未新增方法, 见 lib/PIDController docs/README.md);
 * 模式 B (kOpenloopTurn): kp 装 TURN_KP, ki=kd=0, 用 update_pwm_turn_openloop 得 Δ = TURN_KP*θ_target。
 * 输出限幅均为 TURN_PWM_LIMIT, 防止差速量过大超出直立环调节能力破坏平衡 (docs 5.5)。
 */
void configure_turn_pid() {
    turn_pid.reset(); // 切换模式时清零内部状态, 避免模式 A 残留的误差差分/积分污染模式 B
    if (turn_mode == TurnMode::kOpenloopTurn) {
        turn_pid.update_pid(TURN_KP, 0.0f, 0.0f);
    } else {
        turn_pid.update_pid(TURN_KD, 0.0f, 0.0f);
        turn_pid.update_target(0.0f); // 抑制转向: 期望角速度恒为 0
    }
    turn_pid.output_limit(TURN_PWM_LIMIT);
}

/**
 * @brief 处理串口单字符命令 + 机械中值在线标定采样 (每控制周期轮询一次)
 *
 * 命令表:
 *   's' 启动/停止切换: 武装后姿态进入中值窗口自动起控, 再按一次解除武装;
 *   'c' 标定机械中值: 仅停止状态有效, 手扶车体大致直立静止后发送。
 *       随后每个控制周期由本函数逐周期累加 theta, 取 0.2s 均值作为 theta_0,
 *       采样期间小车保持静止, 完成即自动生效并打印结果。
 *   'w'/'x' 目标速度调整: 每按一次按 SPEED_STEP_MM_S 步进加减 (运行中/停止均可)。
 *   'v' 显示当前目标速度与反馈速度。
 *   't' 切换转向模式 (抑制 <-> 开环): 切换时重配 turn_pid 增益并清零目标转角 (docs 5.3 A/B)。
 *   'l'/'r' 开环转角步进: 仅开环模式有效, 每按一次 turn_target_angle_deg 增减 TURN_ANGLE_STEP_DEG;
 *       符号约定: Δ>0 → 右转 (见文件头方向约定), 故 'l' 左转取负、'r' 右转取正, 实测反向时调换。
 *   'o' 开环转角归零: 目标转角回 0 (两种模式均可用, 开环模式下即恢复直行)。
 *
 * @param theta 当前控制角 theta (deg, 前倾为正), 'c' 标定时用于累加采样
 */
void handle_serial_command(float theta) {
    // 中值标定采样状态仅本函数内使用, 按最小作用域原则设为函数内局部静态 (跨周期保留)
    static uint16_t calib_remaining = 0; // 剩余采样周期数, >0 表示正在采样
    static float calib_sum = 0.0f;       // 采样累加和

    // 机械中值标定采样: 逐周期累加, 与命令轮询共用同一次周期调用
    if (calib_remaining > 0) {
        calib_sum += theta; // 累加当前 theta
        calib_remaining--;  // 剩余采样周期递减
        if (calib_remaining == 0) {
            // 采样完成: 更新机械中值 theta_0
            zero_pitch_deg = calib_sum / BALANCE_CALIB_CYCLES;
            Serial.printf("[CALIB] zero_pitch=%.2f deg\n", zero_pitch_deg);
        }
    }

    while (Serial.available() > 0) {
        char cmd = static_cast<char>(Serial.read()); // 读入单字符命令
        switch (cmd) {
        case 's':
            // 切换武装标志, 并在串口回显当前状态 (纯文本, 无绘图依赖)
            balance_armed = !balance_armed;
            Serial.println(balance_armed ? "[CMD] armed, wait for pitch window" : "[CMD] stopped");
            break;

        case 'c':
            // 标定要求车轮静止, 运行中直接忽略
            if (balance_state != BalanceState::kIdle) {
                Serial.println("[CMD] ignored: running");
                break;
            }
            // 启动中值标定: 置剩余采样周期数, 清零累加和, 由本函数逐周期累加
            calib_remaining = BALANCE_CALIB_CYCLES;
            calib_sum = 0.0f;
            Serial.println("[CALIB] sampling 0.2s, hold the car upright...");
            break;

        case 'w':
            // 目标速度 +步进
            target_speed_mm_s += SPEED_STEP_MM_S;
            Serial.printf("[CMD] target speed=%.1f mm/s\n", target_speed_mm_s);
            break;

        case 'x':
            // 目标速度 -步进
            target_speed_mm_s -= SPEED_STEP_MM_S;
            Serial.printf("[CMD] target speed=%.1f mm/s\n", target_speed_mm_s);
            break;

        case 'v':
            // 显示当前目标速度
            Serial.printf("[CMD] target speed=%.1f mm/s\n", target_speed_mm_s);
            break;

        case 't':
            // 切换转向模式: 抑制(走直线) <-> 开环转动; 重配增益并清零目标转角
            turn_mode =
                (turn_mode == TurnMode::kStraight) ? TurnMode::kOpenloopTurn : TurnMode::kStraight;
            turn_target_angle_deg = 0.0f; // 模式切换后目标转角归零, 避免遗留转角指令
            configure_turn_pid();
            Serial.printf(
                "[CMD] turn mode=%s, target_angle=%.1f deg\n",
                turn_mode == TurnMode::kStraight ? "straight" : "openloop",
                turn_target_angle_deg
            );
            break;

        case 'l':
            // 左转步进: 仅开环模式有效 (抑制模式无转角指令概念)
            if (turn_mode != TurnMode::kOpenloopTurn) {
                Serial.println("[CMD] ignored: turn mode is straight");
                break;
            }
            // Δ>0 → 左轮快 → 右转 (docs 5.2), 故左转目标转角取负 (实测反向时调换 'l'/'r')
            turn_target_angle_deg -= TURN_ANGLE_STEP_DEG;
            Serial.printf("[CMD] turn target=%.1f deg\n", turn_target_angle_deg);
            break;

        case 'r':
            // 右转步进: 仅开环模式有效
            if (turn_mode != TurnMode::kOpenloopTurn) {
                Serial.println("[CMD] ignored: turn mode is straight");
                break;
            }
            turn_target_angle_deg += TURN_ANGLE_STEP_DEG;
            Serial.printf("[CMD] turn target=%.1f deg\n", turn_target_angle_deg);
            break;

        case 'o':
            // 开环转角归零: 恢复直行
            turn_target_angle_deg = 0.0f;
            Serial.println("[CMD] turn target=0 deg");
            break;

        default:
            break; // 忽略回车/换行等其他字符
        }
    }
}

/**
 * @brief 编码器测速: 返回车体前进速度 (mm/s, 两轮平均)
 *
 * 差值法 (同 test03/test11): 本周期与上一周期编码器 tick 差值 * 单脉冲距离 / 时间差。
 * 每控制周期 (5ms) 由 control_step 调用一次, 内部用函数静态量维护采样基线。
 * 单位换算: delta_ticks * DISTANCE_PER_TICK_MM / dt_ms 得 mm/ms, 再乘 MS_TO_S 得 mm/s。
 *
 * @return 车体前进速度 v (mm/s); 前进为正 (依赖编码器读数方向, 见文件头方向约定)
 */
float measure_speed_mm_s() {
    static uint32_t last_time_ms = 0;      // 上一次采样时刻
    static int32_t last_ticks[2] = {0, 0}; // 上一次编码器读数
    static bool is_first_run = true;       // 首次调用标志: 仅建基线, 不产生速度

    const uint32_t now_ms = millis();
    if (is_first_run) {
        // 初始化采样基线, 避免首次控制周期时间差过大
        last_time_ms = now_ms;
        last_ticks[MOTOR_LEFT] = encoders[MOTOR_LEFT].getTicks();
        last_ticks[MOTOR_RIGHT] = encoders[MOTOR_RIGHT].getTicks();
        is_first_run = false;
        return 0.0f;
    }

    const uint32_t dt_ms = now_ms - last_time_ms; // 距上次采样的时间差
    if (dt_ms == 0) {
        return 0.0f; // 时间差为 0 时不计算, 避免除零
    }

    // 各轮 delta_ticks * 单脉冲距离 / 时间差 = mm/ms, 乘 MS_TO_S 得 mm/s
    const float v_left =
        static_cast<float>(encoders[MOTOR_LEFT].getTicks() - last_ticks[MOTOR_LEFT]) *
        DISTANCE_PER_TICK_MM / static_cast<float>(dt_ms) * MS_TO_S;
    const float v_right =
        static_cast<float>(encoders[MOTOR_RIGHT].getTicks() - last_ticks[MOTOR_RIGHT]) *
        DISTANCE_PER_TICK_MM / static_cast<float>(dt_ms) * MS_TO_S;

    // 更新采样基线
    last_time_ms = now_ms;
    last_ticks[MOTOR_LEFT] = encoders[MOTOR_LEFT].getTicks();
    last_ticks[MOTOR_RIGHT] = encoders[MOTOR_RIGHT].getTicks();

    return (v_left + v_right) * 0.5f; // 车体前进速度 = 两轮平均 (直行时两轮同速)
}

/**
 * @brief 单周期控制步骤: 读 IMU + 测速 -> 命令处理(含在线标定采样) -> 状态机 -> 共模串级 + 差模转向 -> 文本打印
 *
 * 由 balance_task 以 5ms 固定节拍调用。内部顺序即完整控制链路:
 * 先更新传感器数据(读取处对 pitch 取负统一"前倾为正", Z 轴角速度直接取)与编码器速度,
 * 再响应串口命令 (机械中值在线标定内聚于 handle_serial_command), 然后按状态机决定本轮 PWM
 * (运行态执行 速度环 PI -> 直立环 PD 串级得到共模 base, 转向环得到差模 Δ,
 *  合成 pwm_L = base + Δ, pwm_R = base - Δ), 最后以低频文本行打印状态供串口监视器观察。
 */
void control_step() {
    // 1. 更新姿态: 控制坐标系统一为"前倾为正" (前进方向为 -X, 见文件头方向约定)
    mpu.update();
    const float theta = -mpu.getAngleY();      // theta: 控制俯仰角 (deg), 前倾为正
    const float omega_pitch = -mpu.getGyroY(); // omega_pitch: 控制俯仰角速度 (deg/s), 前倾方向为正
    const float omega_z = mpu.getGyroZ();      // omega_z: 偏航角速度 (deg/s), 用作转向环反馈/阻尼

    // 2. 编码器测速: 车体前进速度 (mm/s), 用作速度环反馈
    const float speed_mm_s = measure_speed_mm_s();

    // 3. 处理串口命令
    handle_serial_command(theta);

    int16_t pwm_balance = 0; // 本周期共模部分输出的 PWM (默认 0)
    int16_t pwm_delta = 0;   // 本周期差模转向量 Δ (默认 0)
    int16_t pwm_left = 0;    // 本周期左轮实际 PWM
    int16_t pwm_right = 0;   // 本周期右轮实际 PWM

    // 4. 输出决策: 按状态机决定本轮 PWM
    switch (balance_state) {
    case BalanceState::kIdle:
        // 停止态: 保持两轮输出关闭
        motor.updateMotorSpeed(MOTOR_LEFT, 0);
        motor.updateMotorSpeed(MOTOR_RIGHT, 0);

        // 起控条件: 已武装 且 |theta - theta_0| 进入起控窗口 -> 起控
        if (balance_armed && fabsf(theta - zero_pitch_deg) <= BALANCE_ARM_ANGLE_DEG) {
            // 清零各环 PID 内部状态 (误差差分/积分), 避免上次残留
            balance_pid.reset();
            speed_pid.reset();
            turn_pid.reset();
            balance_state = BalanceState::kRunning;
            Serial.println("[STATE] running");
        }
        break;

    case BalanceState::kRunning:
        // 倒地保护: 姿态超出安全窗口 -> 解除武装并切回停止, 下一周期关闭输出
        if (fabsf(theta - zero_pitch_deg) >= BALANCE_FALL_ANGLE_DEG) {
            balance_armed = false;
            balance_state = BalanceState::kIdle;
            Serial.println("[SAFE] fall detected, disarmed");
            break;
        }

        // 5. 共模部分: 速度环 (外环, PI) -> 直立环 (内环, PD) 串级 (同 test11, docs 3.3/4.1)
        // 速度环: output = Kp'*(v_set - v) + Ki'*Σe, 输出为期望角度增量 (deg)
        const int16_t speed_output = speed_pid.update_pwm_speed(target_speed_mm_s, speed_mm_s);

        // 串级嵌套: 直立环目标角度 = 速度环输出 + 机械中值 theta_0
        const float target_angle = static_cast<float>(speed_output) + zero_pitch_deg;

        // 直立环: = Kp*(target_angle - theta) - Kd*omega (库层强制纯 PD)
        const float inputs[2] = {theta, omega_pitch}; // [角度, 角速度]
        pwm_balance = balance_pid.update_pwm_upright(target_angle, inputs);

        // 6. 差模部分: 转向环输出 Δ (docs 5.3)
        if (turn_mode == TurnMode::kOpenloopTurn) {
            // 模式 B 开环转动: Δ = TURN_KP*θ_target (无反馈, 开环转角指令)
            pwm_delta = turn_pid.update_pwm_turn_openloop(turn_target_angle_deg);
        } else {
            // 模式 A 抑制转向: Δ = -TURN_KD*ωz (target=0 的角速度比例跟踪, 阻尼自发偏航)
            pwm_delta = turn_pid.update_pwm(omega_z);
        }

        // 7. 合成: 共模 base + 差模 Δ 对称叠加 (docs 5.2)
        // 在 int 域求和避免 int16 整数提升的隐式窄化告警; 两环输出均独立限幅
        // (BALANCE_PWM_LIMIT / TURN_PWM_LIMIT), 和值远小于 int16_t 范围, 显式转换安全
        pwm_left = static_cast<int16_t>(static_cast<int>(pwm_balance) + pwm_delta);
        pwm_right = static_cast<int16_t>(static_cast<int>(pwm_balance) - pwm_delta);

        // 8. 输出: 正值驱动两轮向车头方向转动 (实测反向应反接电机)
        motor.updateMotorSpeed(MOTOR_LEFT, pwm_left);
        motor.updateMotorSpeed(MOTOR_RIGHT, pwm_right);
        break;
    }

    // 9. 文本打印: 以 10Hz 低频输出一行状态 (纯文本, 无绘图依赖), 供串口监视器观察
    static uint32_t last_print_ms = 0; // 上次打印时刻 (函数内静态, 跨周期保留)
    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms >= 100) {
        last_print_ms = now_ms;
        Serial.printf(
            "state=%s theta=%.2f omega=%.2f omega_z=%.2f speed=%.1f target=%.1f "
            "turn=%s turn_cmd=%.1f delta=%d pwm_L=%d pwm_R=%d\n",
            balance_state == BalanceState::kIdle ? "idle" : "run",
            theta,
            omega_pitch,
            omega_z,
            speed_mm_s,
            target_speed_mm_s,
            turn_mode == TurnMode::kStraight ? "straight" : "openloop",
            turn_target_angle_deg,
            pwm_delta,
            pwm_left,
            pwm_right
        );
    }
}

/**
 * @brief 控制任务主体: 5ms 固定节拍调用 control_step
 *
 * 参数在 setup 中经 xTaskCreatePinnedToCore 传入 (见 config.h BALANCE_* 常量)。
 */
void balance_task(void* param) {
    (void)param; // 任务参数未使用

    const uint32_t period_ms = BALANCE_PERIOD_MS; // 控制周期 (5ms)
    uint32_t last_wake_ms = millis();

    while (true) {
        control_step();                                   // 执行一周期控制
        const uint32_t elapsed = millis() - last_wake_ms; // 本周期已耗时
        if (elapsed < period_ms) {
            vTaskDelay(pdMS_TO_TICKS(period_ms - elapsed)); // 补足剩余时间, 保持固定节拍
        }
        last_wake_ms = millis();
    }
}

} // namespace
