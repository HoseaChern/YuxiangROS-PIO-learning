#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

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
    float target_;       // 目标值
    float output_limit_; // 输出限幅 (对称 ±output_limit_)
    float kp_;           // 比例系数
    float ki_;           // 积分系数
    float kd_;           // 微分系数

    float error_sum_;              // 误差累积和
    float d_error_;                // 误差变化率
    float error_last_;             // 上次误差
    float error_pre_last_;         // 上上次误差
    float intergral_sup_ = 2500.0; // 积分值上界

  public:
    PIDController() = default;                   //默认构造函数
    PIDController(float kp, float ki, float kd); // 构造函数, 传入PID系数

    float update(float current);                     // 提供当前值, 返回控制输出值
    void update_target(float target);                // 更新目标值
    void update_PID(float kp, float ki, float kd);   // 更新PID系数
    void reset();                                    // 重置PID控制器
    void output_limit(float limit);                  // 设置输出限幅, 对称限制在 [-limit, limit]
};

#endif // PIDCONTROLLER_H