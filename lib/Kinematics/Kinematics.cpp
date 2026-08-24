#include "Kinematics.h"

// C++11: 静态 constexpr 成员类外定义, 避免 ODR-use 链接错误
constexpr float Kinematics::MS_TO_S;
constexpr float Kinematics::PI_F;

void Kinematics::set_motor_param(float distance_per_tick_mm) {
    distance_per_tick_mm_ = distance_per_tick_mm;
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
    output_linear_velocity = (left_speed + right_speed) / 2.0f;
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
    output_left_speed = linear_velocity - angular_velocity * wheel_distance_ / 2.0f;
    output_right_speed = linear_velocity + angular_velocity * wheel_distance_ / 2.0f;
}

/**
 * @brief 更新电机速度和编码器数据
 * 
 * @param now 当前时间
 * @param left_tick 左轮编码器脉冲数
 * @param right_tick 右轮编码器脉冲数
 */
void Kinematics::update_motor_speed(uint64_t now, int32_t left_tick, int32_t right_tick) {
    // 静态局部变量: 采样基线跨多次调用保持
    static uint64_t last_update_time = 0;  // 上一次更新时间
    static int64_t last_ticks[2] = {0, 0}; // 上一次读取的计数器数值
    static bool is_first_run = true;       // 首次进入标志

    if (is_first_run) {
        // 初始化采样基线, 避免首次控制周期时间差过大
        last_update_time = now;
        last_ticks[0] = left_tick;
        last_ticks[1] = right_tick;
        is_first_run = false;
    }

    // 普通局部变量: 每次循环重新计算的临时量
    uint64_t dt = now - last_update_time; // 计算时间差

    // 计算电机编码器读数变化量
    int32_t delta_left_tick = static_cast<int32_t>(left_tick - last_ticks[0]);
    int32_t delta_right_tick = static_cast<int32_t>(right_tick - last_ticks[1]);

    // 距离比时间获取速度: delta_ticks * 单脉冲距离 / 时间差
    // 原始单位为 mm/ms, 乘以 1000 转换为 mm/s, 方便 PID 计算与观察
    if (dt != 0) {
        current_motor_speeds_[0] = static_cast<float>(delta_left_tick) * distance_per_tick_mm_ /
                                   static_cast<float>(dt) * MS_TO_S;
        current_motor_speeds_[1] = static_cast<float>(delta_right_tick) * distance_per_tick_mm_ /
                                   static_cast<float>(dt) * MS_TO_S;
    }

    // 更新上一次更新时间为当前时间
    last_update_time = now;
    // 更新上一次编码器读数为当前编码器读数
    last_ticks[0] = left_tick;
    last_ticks[1] = right_tick;

    // 更新里程计
    update_odom(dt);
}

float Kinematics::get_motor_speed(uint8_t motor_id) { return current_motor_speeds_[motor_id]; }

void Kinematics::update_odom(uint16_t dt) {
    // 单位换算, ms -> s
    float dt_s = static_cast<float>(dt) / MS_TO_S;

    // 运动学正解
    this->kinematics_forward(
        current_motor_speeds_[0],
        current_motor_speeds_[1],
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
    if (angle > PI_F) {
        output_angle -= 2.0f * PI_F;
    }
    if (angle < -PI_F) {
        output_angle += 2.0f * PI_F;
    }
}
