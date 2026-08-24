#include "Kinematics.h"

// C++11: 静态 constexpr 成员类外定义, 避免 ODR-use 链接错误
constexpr float Kinematics::MS_TO_S;
constexpr float Kinematics::PI_F;

void Kinematics::set_motor_param(float distance_per_tick_mm) {
    distance_per_tick_mm_ = distance_per_tick_mm;
}

void Kinematics::set_wheel_distance(float wheel_distance) { wheel_distance_ = wheel_distance; }

/**
 * @brief 运动学正解: 电机转速 -> 车体速度
 * 
 * @param motor_speeds 电机转速, [0]=左电机, [1]=右电机, 单位 mm/s
 * @param[out] body_velocities 车体速度, [0]=线速度, 单位 mm/s; [1]=角速度, 单位 rad/s
 * @note 仅用于里程计计算
 */
void Kinematics::kinematics_forward(const float motor_speeds[2], float body_velocities[2]) {
    body_velocities[0] = (motor_speeds[0] + motor_speeds[1]) / 2.0f; // 线速度
    // 防御: 轮间距未设置(<=0)时角速度无意义, 输出 0 避免除零得 inf
    body_velocities[1] = (wheel_distance_ > 0.0f)
                             ? (motor_speeds[1] - motor_speeds[0]) / wheel_distance_
                             : 0.0f; // 角速度
}

/**
 * @brief 运动学逆解: 车体速度 -> 电机目标转速
 * 
 * @param body_velocities 车体速度, [0]=目标线速度, 单位 mm/s; [1]=目标角速度, 单位 rad/s
 * @param[out] motor_speeds 电机目标转速, [0]=左电机, [1]=右电机, 单位 mm/s
 */
void Kinematics::kinematics_inverse(const float body_velocities[2], float motor_speeds[2]) {
    motor_speeds[0] = body_velocities[0] - body_velocities[1] * wheel_distance_ / 2.0f; // 左轮
    motor_speeds[1] = body_velocities[0] + body_velocities[1] * wheel_distance_ / 2.0f; // 右轮
}

/**
 * @brief 更新电机速度与里程计数据
 * 
 * @param now 当前时间, 单位 ms
 * @param ticks 编码器读数, [0]=左编码器, [1]=右编码器
 */
void Kinematics::update_motor_speed(uint64_t now, const int32_t ticks[2]) {
    // 静态局部变量: 采样基线跨多次调用保持
    static uint64_t last_update_time = 0;  // 上一次更新时间
    static int64_t last_ticks[2] = {0, 0}; // 上一次读取的计数器数值
    static bool is_first_run = true;       // 首次进入标志

    if (is_first_run) {
        // 初始化采样基线, 避免首次控制周期时间差过大
        last_update_time = now;
        last_ticks[0] = ticks[0];
        last_ticks[1] = ticks[1];
        is_first_run = false;
    }

    // 普通局部变量: 每次循环重新计算的临时量
    uint64_t dt = now - last_update_time; // 计算时间差

    // 计算电机编码器读数变化量
    int32_t delta_ticks[2] = {0, 0}; // 两次读取之间的计数器差值
    delta_ticks[0] = static_cast<int32_t>(ticks[0] - last_ticks[0]);
    delta_ticks[1] = static_cast<int32_t>(ticks[1] - last_ticks[1]);

    // 距离比时间获取速度: delta_ticks * 单脉冲距离 / 时间差
    // 原始单位为 mm/ms, 乘以 1000 转换为 mm/s, 方便 PID 计算与观察
    if (dt != 0) {
        current_motor_speeds_[0] = static_cast<float>(delta_ticks[0]) * distance_per_tick_mm_ /
                                   static_cast<float>(dt) * MS_TO_S;
        current_motor_speeds_[1] = static_cast<float>(delta_ticks[1]) * distance_per_tick_mm_ /
                                   static_cast<float>(dt) * MS_TO_S;
    }

    // 更新上一次更新时间为当前时间
    last_update_time = now;
    // 更新上一次编码器读数为当前编码器读数
    last_ticks[0] = ticks[0];
    last_ticks[1] = ticks[1];

    // 更新里程计
    update_odom(dt);
}

float Kinematics::get_motor_speed(uint8_t motor_id) const { return current_motor_speeds_[motor_id]; }

void Kinematics::update_odom(uint64_t dt) {
    // 单位换算, ms -> s
    float dt_s = static_cast<float>(dt) / MS_TO_S;

    // 运动学正解: 电机转速 -> 车体速度
    float body_velocities[2];
    this->kinematics_forward(current_motor_speeds_, body_velocities);
    odom_.linear_velocity = body_velocities[0];
    odom_.angular_velocity = body_velocities[1];
    // 单位换算, mm/s -> m/s
    odom_.linear_velocity /= MS_TO_S;

    // 计算角位置
    odom_.yaw += odom_.angular_velocity * dt_s;
    TransAngleInPI(odom_.yaw);

    // 计算位置
    odom_.x += odom_.linear_velocity * std::cos(odom_.yaw) * dt_s;
    odom_.y += odom_.linear_velocity * std::sin(odom_.yaw) * dt_s;
}

odom_t& Kinematics::get_odom() { return odom_; }
const odom_t& Kinematics::get_odom() const { return odom_; }

/**
 * @brief 将角度归一化到[-PI, PI]区间
 * 
 * @param[in,out] angle 输入角度, 归一化后原地返回
 */
void Kinematics::TransAngleInPI(float& angle) {
    // fmod 归一化: 先平移 PI 再取模, 消除多次旋转的累积误差
    angle = std::fmod(angle + PI_F, 2.0f * PI_F);
    if (angle < 0.0f) {
        angle += 2.0f * PI_F;
    }
    angle -= PI_F;
}
