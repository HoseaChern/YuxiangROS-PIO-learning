#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
#include <cstdint>

// 里程计参数
using odom_t = struct odom_t {
    float x;                // x 坐标, 单位 mm
    float y;                // y 坐标, 单位 mm
    float yaw;              // 偏航角, 单位 rad
    float linear_velocity;  // 线速度, 单位 mm/s
    float angular_velocity; // 角速度, 单位 rad/s
};

/**
 * @brief 运动学计算类
 * 
 * @note 
 * 运动学正逆解: \note
 * 正解: 轮转速 -> 直线速度和角速度 \note
 * 正解公式: \note
 * v = (v_left + v_right) / 2; omega = (v_right - v_left) / wheel_distance \note
 * 逆解: 直线速度和角速度 -> 轮转速 \note
 * 逆解公式: \note
 * v_left = v - omega * wheel_distance / 2; v_right = v + omega * wheel_distance / 2 \note
 * 
 * 里程计: \note
 * 里程计公式: \note
 * x = x + v * cos(yaw) * dt; y = y + v * sin(yaw) * dt; yaw = yaw + omega * dt \note
 */
class Kinematics {
  private:
    // 单位换算常量
    static constexpr float MS_TO_S = 1000.0f;  // 单位换算系数 (1 s = 1000 ms)
    static constexpr float PI_F = 3.14159265f; // 单精度 PI 常量, ESP32单精度优先

    float distance_per_tick_mm_;    // 单个脉冲对应的轮子前进距离, 单位 mm
    float current_motor_speeds_[2]; // 当前电机速度, 单位 mm/s
    float wheel_distance_;          // 轮子间距, 单位 mm
    odom_t odom_;                   // 存储里程计参数

  public:
    Kinematics() = default;  // 默认构造函数
    ~Kinematics() = default; // 默认析构函数

    void set_motor_param(float distance_per_tick_mm); // 设置电机参数
    void set_wheel_distance(float wheel_distance);    // 设置轮子间距

    // 运动学正解: 电机转速 -> 车体速度, 仅用于里程计计算
    // motor_speeds: [0]=左电机, [1]=右电机, 单位 mm/s
    // body_velocities: [0]=线速度, 单位 mm/s; [1]=角速度, 单位 rad/s
    void kinematics_forward(const float motor_speeds[2], float body_velocities[2]);
    // 运动学逆解: 车体速度 -> 电机目标转速
    // body_velocities: [0]=目标线速度, 单位 mm/s; [1]=目标角速度, 单位 rad/s
    // motor_speeds: [0]=左电机, [1]=右电机, 单位 mm/s
    void kinematics_inverse(const float body_velocities[2], float motor_speeds[2]);

    // 更新电机速度和编码器数据
    // ticks: [0]=左编码器, [1]=右编码器
    void update_motor_speed(uint64_t now, const int32_t ticks[2]);
    float get_motor_speed(uint8_t motor_id) const; // 获取电机速度

    void update_odom(uint64_t dt);            // 更新里程计数据
    odom_t& get_odom();                       // 获取里程计数据 (可写)
    const odom_t& get_odom() const;           // 获取里程计数据 (只读)
    static void TransAngleInPI(float& angle); // 将角度归一化到[-PI, PI]
};

#endif // KINEMATICS_H
