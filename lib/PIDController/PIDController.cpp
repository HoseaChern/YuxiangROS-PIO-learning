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
 * @brief 更新 PID 控制器（外部微分变体）
 *
 * 数学形式与 update_pwm 一致（u = kp·e + ki·Σe + kd·d_error），D 项同为 Δe，仅计算来源不同。
 * 为什么: 对含噪测量做数值微分会把噪声放大约 1/T，改用传感器变化率规避。
 * 符号: e = target - measurement，故 D 项 Δe = -(measurement 的变化率)，即取测量变化率的相反数。
 *
 * @param inputs [PID_INPUT_MEASUREMENT]=测量值, [PID_INPUT_RATE]=测量变化率
 * @return PWM 输出，四舍五入取整，范围 ±output_limit_
 */
int16_t PIDController::update_pwm_with_rate(const float inputs[2]) {
    float error = target_ - inputs[PID_INPUT_MEASUREMENT];
    d_error_ = -inputs[PID_INPUT_RATE]; // D 项 Δe = -(测量 y 的变化率) = e_k - e_{k-1}

    error_sum_ += error;
    error_sum_ =
        std::fmax(-INTEGRAL_SUP_LIMIT, std::fmin(INTEGRAL_SUP_LIMIT, error_sum_)); // 积分限幅

    float output = kp_ * error + ki_ * error_sum_ + kd_ * d_error_;
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output)); // 输出限幅

    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

void PIDController::reset() {
    error_sum_ = 0.0f;
    d_error_ = 0.0f;
    error_last_ = 0.0f;
}
