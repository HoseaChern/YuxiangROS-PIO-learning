#include "PIDController.h"

#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd) {
    reset();                // 初始化控制器
    update_PID(kp, ki, kd); // 更新PID参数
}

/**
 * @brief 更新PID控制器, 核心算法实现
 * 
 * @param current 当前值
 * @return float 控制输出值
 */
float PIDController::update_pwm(float current) {
    // 计算误差及其变化率
    float error = target_ - current; // 计算误差
    d_error_ = error_last_ - error;  // 计算误差变化率
    error_last_ = error;             // 更新上一次误差为当前误差

    // 计算积分项并进行积分限制
    error_sum_ += error; // 计算积分项
    if (error_sum_ > intergral_sup_) {
        error_sum_ = intergral_sup_; // 控制上界
    }
    if (error_sum_ < -intergral_sup_) {
        error_sum_ = -intergral_sup_; // 控制下界
    }

    // 计算输出并进行输出限制
    float output = kp_ * error + ki_ * error_sum_ + kd_ * d_error_;
    if (output > output_limit_) {
        output = output_limit_;
    }
    if (output < -output_limit_) {
        output = -output_limit_;
    }

    return output;
}

void PIDController::update_target(float target) { target_ = target; }

void PIDController::update_PID(float kp, float ki, float kd) {
    reset(); // 重置控制器
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::reset() {
    target_ = 0.0f;
    output_limit_ = 0.0f;
    kp_ = 0.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;
    error_last_ = 0.0f;
    error_sum_ = 0.0f;
    d_error_ = 0.0f;
}

void PIDController::output_limit(float limit) { output_limit_ = limit; }