#include "PIDController.h"

#include <cmath>

// C++11: 静态 constexpr 成员类外定义, 避免 ODR-use 链接错误
constexpr float PIDController::INTEGRAL_SUP_LIMIT;

void PIDController::update_pid(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::output_limit(float limit) { output_limit_ = limit; }

void PIDController::update_target(float target) { target_ = target; }

/**
 * @brief 更新PID控制器, 核心算法实现
 * 
 * @param current 当前值
 * @return int16_t PWM 输出值, 范围 ±output_limit_, 四舍五入取整
 */
int16_t PIDController::update_pwm(float current) {
    // 计算误差及其变化率
    float error = target_ - current; // 计算误差
    d_error_ = error_last_ - error;  // 微分项误差 (上次误差 - 当前误差)
    error_last_ = error;             // 更新上一次误差为当前误差

    // 计算积分项并进行积分限制
    error_sum_ += error; // 计算积分项
    error_sum_ = std::fmax(-INTEGRAL_SUP_LIMIT, std::fmin(INTEGRAL_SUP_LIMIT, error_sum_));

    // 计算输出并进行输出限制
    float output = kp_ * error + ki_ * error_sum_ + kd_ * d_error_;
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output));

    // 四舍五入取整: 直接截断 (static_cast<int16_t>) 会让 99.6 -> 99, 低速时占空比系统性偏小;
    // 四舍五入后 99.6 -> 100, 负数同样处理 (-99.6 -> -100)。输出已被 output_limit_
    // 限幅为 ±output_limit_ (远小于 int16_t 范围), 此转换无溢出风险。
    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

/**
 * @brief 更新PID控制器 (外部微分变体)
 *
 * 与 update_pwm 的区别: 微分项不由误差数值差分获得, 而由调用方通过 inputs 数组
 * 直接提供测量量的变化率 (如陀螺仪角速度), 避免对含噪测量做数值微分放大噪声。
 * 标准公式中 d(error)/dt = -d(measurement)/dt, 故微分项取 -Kd * measurement_rate。
 * P/I 逻辑与 update_pwm 完全一致并共享同一积分状态;
 * 同一实例请勿混用两种更新方式。
 *
 * @param inputs 输入数组, [PID_INPUT_MEASUREMENT]=测量值,
 *               [PID_INPUT_RATE]=测量量变化率 (与测量值同量纲, 以秒为单位)
 * @return int16_t PWM 输出值, 范围 ±output_limit_, 四舍五入取整
 */
int16_t PIDController::update_pwm_with_rate(const float inputs[2]) {
    // 计算误差 (微分项使用外部变化率, 不做数值差分, 不更新 error_last_)
    float error = target_ - inputs[PID_INPUT_MEASUREMENT];

    // 计算积分项并进行积分限制 (与 update_pwm 一致)
    error_sum_ += error;
    error_sum_ = std::fmax(-INTEGRAL_SUP_LIMIT, std::fmin(INTEGRAL_SUP_LIMIT, error_sum_));

    // 计算输出并进行输出限制: 微分项 = -Kd * 测量变化率
    float output = kp_ * error + ki_ * error_sum_ - kd_ * inputs[PID_INPUT_RATE];
    output = std::fmax(-output_limit_, std::fmin(output_limit_, output));

    // 四舍五入取整, 处理方式与 update_pwm 一致
    return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

void PIDController::reset() {
    error_sum_ = 0.0f;
    d_error_ = 0.0f;
    error_last_ = 0.0f;
}
