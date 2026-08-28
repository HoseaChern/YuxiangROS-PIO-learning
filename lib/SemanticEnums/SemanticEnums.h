#ifndef SEMANTIC_ENUMS_H
#define SEMANTIC_ENUMS_H

#include <cstdint>

/**
 * @brief 电机标识: 左/右轮
 * 
 * 用于 motor_speeds / ticks 数组下标, 以及 get_motor_speed() 等电机维度接口参数。
 * 使用普通 enum (带底层类型) 而非 enum class:
 * - 可隐式转 int 直接作数组下标, 无需 static_cast
 * - 调用方传裸数字 0/1 会被编译器拦截, 防止左右写反 (写反不报错但行为错误)
 */
enum MotorID : uint8_t { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };

/**
 * @brief 车体速度标识: 线速度/角速度
 * 
 * 用于 body_velocities 数组下标, 区分车体速度的两个维度。
 */
enum VelocityID : uint8_t { VEL_LINEAR = 0, VEL_ANGULAR = 1 };

/**
 * @brief PID输入标识: 测量值/测量变化率
 *
 * 用于 update_pwm_with_rate() 的 inputs 数组下标, 区分外部微分模式的两个输入维度。
 */
enum PidInputID : uint8_t { PID_INPUT_MEASUREMENT = 0, PID_INPUT_RATE = 1 };

#endif // SEMANTIC_ENUMS_H
