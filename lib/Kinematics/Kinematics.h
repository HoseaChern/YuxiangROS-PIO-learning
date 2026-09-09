#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <SemanticEnums.h>
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
 * 做什么: 由左右轮转速推算车体线速度/角速度 (正解), 由车体线速度/角速度推算左右轮转速 (逆解),
 *         并基于编码器采样与差速模型累积里程计位姿 (x, y, yaw)。
 * 为什么: 公式推导、单位约定与接口说明见库文档 docs/README.md 第 3 章「核心公式」。
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

    // 更新电机速度和编码器数据
    // ticks: [MOTOR_LEFT]=左编码器, [MOTOR_RIGHT]=右编码器
    void update_motor_speed(uint64_t now, const int32_t ticks[2]);

    // 运动学正解: 电机转速 -> 车体速度, 仅用于里程计计算
    // motor_speeds: [MOTOR_LEFT]=左电机, [MOTOR_RIGHT]=右电机, 单位 mm/s
    // body_velocities: [VEL_LINEAR]=线速度, 单位 mm/s; [VEL_ANGULAR]=角速度, 单位 rad/s
    void kinematics_forward(const float motor_speeds[2], float body_velocities[2]);
    // 运动学逆解: 车体速度 -> 电机目标转速
    // body_velocities: [VEL_LINEAR]=目标线速度, 单位 mm/s; [VEL_ANGULAR]=目标角速度, 单位 rad/s
    // motor_speeds: [MOTOR_LEFT]=左电机, [MOTOR_RIGHT]=右电机, 单位 mm/s
    void kinematics_inverse(const float body_velocities[2], float motor_speeds[2]);

    float get_motor_speed(MotorID motor_id) const; // 获取电机速度

    odom_t& get_odom();             // 获取里程计数据 (可写)
    const odom_t& get_odom() const; // 获取里程计数据 (只读)

  private:
    // 内部实现细节: 里程计随 update_motor_speed 每周期自动刷新, 无需外部调用
    void update_odom_(uint64_t dt); // 更新里程计数据
    // 内部实现细节: 角度归一化, 里程计累积的中间步骤
    static void trans_angle_in_pi_(float& angle); // 将角度归一化到[-PI, PI]
};

#endif // KINEMATICS_H
