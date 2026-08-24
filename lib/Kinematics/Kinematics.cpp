#include "Kinematics.h"

constexpr float Kinematics::MS_TO_S; // C++11: 静态 constexpr 成员类外定义, 避免 ODR-use 链接错误

void Kinematics::set_motor_param(uint8_t motor_id, float per_pulse_distance) {
    motor_params_[motor_id].per_pulse_distance = per_pulse_distance;
}

void Kinematics::set_wheel_distance(float wheel_distance) { wheel_distance_ = wheel_distance; }

/**
 * @brief 运动学正解计算
 * 
 * @param left_speed 左轮速度, 单位 mm/s
 * @param right_speed 右轮速度, 单位 mm/s
 * @param[out] output_linear_velocity 线速度, 单位 mm/s
 * @param[out] output_angular_velocity 角速度, 单位 rad/s
 */
void Kinematics::kinematics_forward(
    float left_speed, float right_speed, float& output_linear_velocity,
    float& output_angular_velocity
) {
    output_linear_velocity = (left_speed + right_speed) / 2.0;
    output_angular_velocity = (right_speed - left_speed) / wheel_distance_;
}

/**
 * @brief 运动学逆解计算
 * 
 * @param linear_velocity 线速度, 单位 mm/s
 * @param angular_velocity 角速度, 单位 rad/s
 * @param[out] output_left_speed 左轮速度, 单位 mm/s
 * @param[out] output_right_speed 右轮速度, 单位 mm/s
 */
void Kinematics::kinematics_inverse(
    float linear_velocity, float angular_velocity, float& output_left_speed,
    float& output_right_speed
) {
    output_left_speed = linear_velocity - angular_velocity * wheel_distance_ / 2.0;
    output_right_speed = linear_velocity + angular_velocity * wheel_distance_ / 2.0;
}

/**
 * @brief 更新电机速度和编码器数据
 * 
 * @param current_time 当前时间
 * @param left_tick 左轮编码器脉冲数
 * @param right_tick 右轮编码器脉冲数
 */
void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick) {
    // 计算时间差
    uint64_t dt = current_time - last_update_time_;
    // 更新上一次更新时间为当前时间
    last_update_time_ = current_time;

    // 计算电机编码器读数变化量
    int32_t delta_left_tick = left_tick - motor_params_[0].last_encoder_tick;
    int32_t delta_right_tick = right_tick - motor_params_[1].last_encoder_tick;
    // 更新上一次编码器读数为当前编码器读数
    motor_params_[0].last_encoder_tick = left_tick;
    motor_params_[1].last_encoder_tick = right_tick;

    // 轮子速度计算, mm/ms = m/s
    motor_params_[0].motor_speed =
        static_cast<float>(delta_left_tick * motor_params_[0].per_pulse_distance) / dt;
    motor_params_[1].motor_speed =
        static_cast<float>(delta_right_tick * motor_params_[1].per_pulse_distance) / dt;
    // 单位换算, mm/ms -> mm/s
    motor_params_[0].motor_speed *= MS_TO_S;
    motor_params_[1].motor_speed *= MS_TO_S;

    //更新里程计
    update_odom(dt);
}

int16_t Kinematics::get_motor_speed(uint8_t motor_id) {
    return motor_params_[motor_id].motor_speed;
}

void Kinematics::update_odom(uint16_t dt) {
    // 单位换算, ms -> s
    float dt_s = static_cast<float>(dt) / MS_TO_S;

    // 运动学正解
    this->kinematics_forward(
        motor_params_[0].motor_speed,
        motor_params_[1].motor_speed,
        odom_.linear_velocity,
        odom_.angular_velocity
    );
    // 单位换算, mm/s -> m/s
    odom_.linear_velocity /= MS_TO_S;

    // 计算角位置
    odom_.yaw += odom_.angular_velocity * dt_s;
    TransAngleInPI(odom_.yaw, odom_.yaw);

    // 计算位置
    odom_.x += odom_.linear_velocity * std::cos(odom_.yaw) * dt_s;
    odom_.y += odom_.linear_velocity * std::sin(odom_.yaw) * dt_s;
}

odom_t& Kinematics::get_odom() { return odom_; }

/**
 * @brief 将角度转换为[-PI, PI]区间
 * 
 * @param angle 输入角度
 * @param[out] output_angle 输出角度
 */
void Kinematics::TransAngleInPI(float angle, float& output_angle) {
    if (angle > PI) {
        output_angle -= 2 * PI;
    }
    if (angle < -PI) {
        output_angle += 2 * PI;
    }
}