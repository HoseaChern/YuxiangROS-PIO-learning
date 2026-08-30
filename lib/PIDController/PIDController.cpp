#include "PIDController.h"

#include <cmath>

// static constexpr 类外定义，避免 C++11 ODR-use 链接错误
constexpr float PIDController::INTEGRAL_SUP_LIMIT;

void PIDController::update_pid(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::output_limit(float limit) { output_limit_ = limit; }

void PIDController::update_target(float target) { target_ = target; }

/**
 * @brief 更新 PID 控制器（数值微分变体）
 * @param current 当前值
 * @return PWM 输出，四舍五入取整，范围 ±output_limit_
 */
int16_t PIDController::update_pwm(float current) {
    float error = target_ - current;
    d_error_ = error - error_last_; // D 项差分 e_k - e_{k-1}
    error_last_ = error;

    error_sum_ += error;
    error_sum_ =
        std::fmax(-INTEGRAL_SUP_LIMIT, std::fmin(INTEGRAL_SUP_LIMIT, error_sum_)); // 积分限幅

    float output = kp_ * error + ki_ * error_sum_ + kd_ * d_error_;
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output)); // 输出限幅

    // 四舍五入，避免向零截断导致低速占空比系统性偏小
    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

/**
 * @brief 更新直立环 PID 控制器（纯 PD）
 *
 * 数学形式 u = kp·e + kd·d_error。
 * 为什么:
 *   D 项外部微分: 对含噪角度做数值微分会把噪声放大约 1/T，改用陀螺仪角速度规避;
 *   符号 e = target - angle，故 Δe = -(角速度)，即取角速度的相反数。
 *
 * @param target 期望角度（直立环目标，如 theta_0 或 速度环输出 + theta_0）
 * @param inputs [PID_INPUT_ANGLE]=角度, [PID_INPUT_ANGULAR_RATE]=角速度
 * @return PWM 输出，四舍五入取整，范围 ±output_limit_
 */
int16_t PIDController::update_pwm_upright(float target, const float inputs[2]) {
    float error = target - inputs[PID_INPUT_ANGLE];
    d_error_ = -inputs[PID_INPUT_ANGULAR_RATE]; // D 项 Δe = -角速度 = e_k - e_{k-1}

    float output = kp_ * error + kd_ * d_error_;                          // 纯 PD
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output)); // 输出限幅

    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

/**
 * @brief 更新速度环 PID 控制器（纯 PI）
 *
 * 数学形式 u = kp·e + ki·Σe，与文档速度环公式 output = Kp'·(v_set - v) + Ki'·Σe_j 对应。
 * 为什么:
 *   速度调节希望平缓连续，舍去微分（D）项以防高频振动（见 docs 速度环 3.1）。
 *   与 update_pwm_upright 保持独立方法：串级嵌套（速度环输出 → 直立环目标）由调用方实现。
 *
 * @param target 期望速度 v_set
 * @param measurement 编码器反馈速度 v
 * @return PWM 输出，四舍五入取整，范围 ±output_limit_
 */
int16_t PIDController::update_pwm_speed(float target, float measurement) {
    float error = target - measurement; // e_k = v_set - v

    error_sum_ += error;
    error_sum_ =
        std::fmax(-INTEGRAL_SUP_LIMIT, std::fmin(INTEGRAL_SUP_LIMIT, error_sum_)); // 积分限幅

    float output = kp_ * error + ki_ * error_sum_;                        // 纯 PI
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output)); // 输出限幅

    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

/**
 * @brief 更新转向环 PID 控制器（开环转动变体）
 *
 * 数学形式 u = kp·target，与文档转向环 5.3 模式 B「期望转向-开环」公式 Δ = Kp·θ_target 对应。
 * 为什么:
 *   开环: 期望转角为外部指令（遥控/上位机的模糊转角），无 yaw 反馈（docs 5.4: 六轴无绝对航向）;
 *   无积分: 转向存在偏航偏移与车轮滑动误差，"消静差"收益甚微反而引入漂移（docs 5.3）;
 *   无微分: 抑制转向（模式 A，纯 D 阻尼 Δ = -kd·ωz）复用 update_pwm（target=0）实现，本方法只管开环映射。
 *   注意: 本方法不读不写 error_sum_/d_error_/error_last_，reset() 对其无影响。
 *
 * @param target 期望转角 θ_target（deg，开环外部指令）
 * @return PWM 输出（差速量 Δ），四舍五入取整，范围 ±output_limit_
 */
int16_t PIDController::update_pwm_turn_openloop(float target) {
    float output = kp_ * target;                                          // 开环: Δ = kp·θ_target
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output)); // 输出限幅

    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

void PIDController::reset() {
    error_sum_ = 0.0f;
    d_error_ = 0.0f;
    error_last_ = 0.0f;
}
