#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <SemanticEnums.h>
#include <cstdint>

/**
 * @brief PID控制器
 * 
 * @note 
 * PID原理: \note
 * PID控制器的原理是通过计算误差，并根据误差来调整输出值，以达到控制系统的目的。 \note
 * 其中，P（Proportional）控制器: 将 误差 按一定比例来调整输出值，以使系统的输出值与目标值相近。 \note
 * I（Integral）控制器: 将 误差累积和 按一定比例来调整输出值，以使系统的输出值与目标值相近。 \note
 * D（Differential）控制器: 将 误差变化率 按一定比例来调整输出值，以使系统的输出值与目标值相近。 \note
 * 公式: \note
 * 输出值 = Kp * 误差 + Ki * 误差累积和 + Kd * 误差变化率 \note
 */
class PIDController {
  private:
    // 编译期常量, 积分上界
    static constexpr float INTEGRAL_SUP_LIMIT = 2500.0f;

    float target_ = 0.0f;       // 目标值
    float output_limit_ = 0.0f; // 输出限幅 (对称 ±output_limit_)
    float kp_ = 0.0f;           // 比例系数
    float ki_ = 0.0f;           // 积分系数
    float kd_ = 0.0f;           // 微分系数

    float error_sum_ = 0.0f;  // 误差累积和
    float d_error_ = 0.0f;    // 微分项误差 (上次误差 - 当前误差, 即误差变化率取负)
    float error_last_ = 0.0f; // 上次误差

  public:
    PIDController() = default; // 默认构造函数, 参数通过 update_pid() 设置

    void update_pid(float kp, float ki, float kd); // 更新PID系数, 不重置内部状态
    void output_limit(float limit);                // 设置输出限幅, 对称限制在 [-limit, limit]
    void update_target(float target);              // 更新目标值
    int16_t update_pwm(float current); // 提供当前值, 返回 PWM 输出值 (int16_t, 范围 ±output_limit_)
    // 外部微分变体: 微分项由调用方提供测量变化率, 见实现处注释
    // inputs: [PID_INPUT_MEASUREMENT]=测量值, [PID_INPUT_RATE]=测量量变化率(以秒为单位)
    int16_t update_pwm_with_rate(const float inputs[2]);
    void reset(); // 清零内部状态 (误差累积和/微分项/上次误差), 用于重新起控前复位
};

#endif // PIDCONTROLLER_H
