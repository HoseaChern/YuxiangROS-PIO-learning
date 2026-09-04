#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <SemanticEnums.h>
#include <cstdint>

/**
 * @brief 位置式 PID 控制器（离散，增益已吸收采样周期 T）
 *
 * 做什么: 由目标值与当前值误差，按 P/I/D 三项求和输出控制量。
 * 为什么: 数学推导与符号约定见库文档 docs/README.md 第 1 章「数学原理」。
 */
class PIDController {
  private:
    // 积分上界，防止积分饱和
    static constexpr float INTEGRAL_SUP_LIMIT = 2500.0f;

    float target_ = 0.0f;       // 目标值
    float output_limit_ = 0.0f; // 输出限幅（对称 ±limit）
    float kp_ = 0.0f;           // P 增益
    float ki_ = 0.0f;           // I 增益（已吸收 T）
    float kd_ = 0.0f;           // D 增益（已吸收 1/T）

    float error_sum_ = 0.0f;  // 误差累积和（I 项）
    float d_error_ = 0.0f;    // 误差差分 e_k - e_{k-1}（D 项）
    float error_last_ = 0.0f; // 上一拍误差，供 D 项差分

  public:
    PIDController() = default; // 增益经 update_pid() 设定

    void update_pid(float kp, float ki, float kd); // 设定 P/I/D 增益，不重置内部状态
    void output_limit(float limit);                // 设定对称输出限幅 ±limit
    void update_target(float target);              // 设定目标值
    int16_t update_pwm(float current); // 数值微分变体：D 项取误差差分 Δe = e_k - e_{k-1}

    // 直立环变体：纯 PD（库层强制无 I 项，忽略 ki_ 不累加积分），D 项 Δe 取角速度的相反数(-omega)
    // target 为期望角度直接入参（串级时 = theta_0 - 速度环输出，动态变化，故不入内部状态）
    // inputs: [PID_INPUT_ANGLE]=角度(theta), [PID_INPUT_ANGULAR_RATE]=角速度(omega)
    int16_t update_pwm_upright(float target, const float inputs[2]);
    // 速度环变体：PI 控制（无 D 项），数学形式 u = kp*e + ki*Σe
    // target=期望速度, measurement=编码器反馈速度；与直立环为独立方法，串级嵌套由调用方实现
    int16_t update_pwm_speed(float target, float measurement);
    // 转向环变体（单一完整转向环）：Δ = kp*θ_cmd − kd*ωz（docs 5.3）
    // θ_cmd=目标转角(开环指令, 无指令=0, 驱动项 kp*θ_cmd)；ωz=偏航角速度 Gyro_Z（阻尼项 −kd*ωz,
    // 无指令时单独作用 = 走直线阻尼）；无积分(ki 占位)。target 不入内部状态, 每周期直接入参
    int16_t update_pwm_turn(float target_cmd, float omega_z);

    void reset(); // 清零内部状态，重新起控前调用
};

#endif // PIDCONTROLLER_H
