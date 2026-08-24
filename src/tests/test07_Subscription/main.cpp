#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PIDController.h>
#include <SemanticEnums.h>
#include <WiFi.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "secrets.h"

namespace {

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 编码器引脚 (编码器0: 4/5, 编码器1: 14/15) ----

constexpr uint8_t ENC_LEFT_PIN_A = 4;
constexpr uint8_t ENC_LEFT_PIN_B = 5;
constexpr uint8_t ENC_RIGHT_PIN_A = 14;
constexpr uint8_t ENC_RIGHT_PIN_B = 15;

// ---- 电机引脚 (电机0: 10/11, 电机1: 12/13) ----

constexpr uint8_t MOTOR_LEFT_PIN_A = 10;
constexpr uint8_t MOTOR_LEFT_PIN_B = 11;
constexpr uint8_t MOTOR_RIGHT_PIN_A = 12;
constexpr uint8_t MOTOR_RIGHT_PIN_B = 13;

// ---- PID 参数 ----

constexpr float PID_KP = 0.625f;           // 比例增益
constexpr float PID_KI = 0.125f;           // 积分增益
constexpr float PID_KD = 0.0f;             // 微分增益
constexpr float PID_OUTPUT_LIMIT = 100.0f; // 输出限幅 ±100

// ---- 运动学参数 ----

constexpr float WHEEL_DISTANCE_MM = 175.0f;        // 轮间距, 单位 mm
constexpr float DISTANCE_PER_TICK_MM = 0.1427138f; // 单个脉冲对应的轮子前进距离, 单位 mm

// ---- 默认目标速度 (在收到 /cmd_vel 前使用) ----

constexpr float DEFAULT_LINEAR_SPEED_MM_S = 50.0f;  // 默认目标线速度, 单位 mm/s
constexpr float DEFAULT_ANGULAR_SPEED_RAD_S = 0.1f; // 默认目标角速度, 单位 rad/s

// ---- 单位换算 ----

constexpr float MPS_TO_MMPS = 1000.0f; // m/s -> mm/s

// ---- 控制周期 ----

constexpr uint32_t LOOP_DELAY_MS = 10; // 主循环调度节拍, 单位 ms

// ---- 网络配置 ----
// 注意: 凭据必须是可写 char 数组, 因库接口要求 char*, 故不能 constexpr
// 使用 secrets.h 存储 WIFI_SSID 和 WIFI_PASS

constexpr char AGENT_IP_STR[] = "192.168.4.136"; // 主机 IP(运行 micro-ROS Agent 的电脑)
constexpr uint16_t AGENT_PORT = 8888;            // Agent UDP 端口

// ---- 任务参数 ----

constexpr uint32_t MICRO_ROS_STACK_SIZE = 10240; // micro-ROS 任务栈字节数
constexpr uint8_t MICRO_ROS_TASK_PRIO = 1;       // 任务优先级
constexpr uint32_t TRANSPORT_SETUP_MS = 2000;    // 传输层设置等待时间, 单位 ms

// ---- 订阅参数 ----

constexpr uint8_t EXECUTOR_HANDLES = 1;                // 执行器句柄数(1 个订阅)
constexpr char CMD_VEL_TOPIC[] = "/cmd_vel";           // 速度指令话题名
constexpr char NODE_NAME[] = "fishbot_motion_control"; // 节点名

// ---- 可变全局状态 (跨 setup/loop/micro_ros_task/twist_callback 共享) ----

Esp32McpwmMotor motor;           // 电机驱动对象 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器对象数组 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器对象数组 (setup/twist_callback/loop 共享)
Kinematics kinematics;           // 运动学正逆解对象 (setup/twist_callback/loop 共享)

// ---- 函数前向声明（内部链接） ----

void micro_ros_task(void* parameter);
void twist_callback(const void* msg_in);
void update_and_control();

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化编码器
    encoders[MOTOR_LEFT].init(MOTOR_LEFT, ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
    encoders[MOTOR_RIGHT].init(MOTOR_RIGHT, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

    // 初始化电动机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);

    // 初始化 PID 控制器参数
    pid_controller[MOTOR_LEFT].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[MOTOR_RIGHT].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[MOTOR_LEFT].output_limit(PID_OUTPUT_LIMIT);  // 对称输出限幅 ±PID_OUTPUT_LIMIT
    pid_controller[MOTOR_RIGHT].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT

    // 初始化轮子间距和电动机参数
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);
    kinematics.set_motor_param(DISTANCE_PER_TICK_MM); // 标定量标量化: 两电机共用

    // 默认目标速度, 避免订阅消息到达前电机无目标
    // 逆解输出仅本次使用, 声明为局部变量, 作用域最小化 (仿照 main.cpp)
    // 车体速度: [VEL_LINEAR]=线速度 mm/s, [VEL_ANGULAR]=角速度 rad/s
    const float body_velocities[2] = {DEFAULT_LINEAR_SPEED_MM_S, DEFAULT_ANGULAR_SPEED_RAD_S};
    // 电机转速: [MOTOR_LEFT]=左, [MOTOR_RIGHT]=右, 单位 mm/s, 仅用于本次逆解计算
    float motor_speeds[2];
    kinematics.kinematics_inverse(body_velocities, motor_speeds);

    // PID 初始化目标轮速
    pid_controller[MOTOR_LEFT].update_target(motor_speeds[MOTOR_LEFT]);
    pid_controller[MOTOR_RIGHT].update_target(motor_speeds[MOTOR_RIGHT]);

    // 创建任务运行 micro-ROS
    xTaskCreate(micro_ros_task, "micro_ros", MICRO_ROS_STACK_SIZE, NULL, MICRO_ROS_TASK_PRIO, NULL);
}

void loop() {
    delay(LOOP_DELAY_MS);
    update_and_control();
}

namespace {

/**
 * @brief /cmd_vel 话题回调函数
 *
 * 将接收到的 Twist 消息转换为左右轮目标速度, 并更新 PID 控制器目标值。
 *
 * @param msg_in 接收到的消息指针
 */
void twist_callback(const void* msg_in) {
    const geometry_msgs__msg__Twist* twist_msg =
        static_cast<const geometry_msgs__msg__Twist*>(msg_in);

    // 运动学逆解: 车体速度 -> 电机目标转速
    // linear.x 单位 m/s 转换为 mm/s, angular.z 单位 rad/s
    const float body_velocities[2] = {
        // [VEL_LINEAR]=线速度, 单位换算 m/s -> mm/s
        static_cast<float>(twist_msg->linear.x * MPS_TO_MMPS),
        // [VEL_ANGULAR]=角速度, 单位 rad/s
        static_cast<float>(twist_msg->angular.z)
    };
    // 电机转速: [MOTOR_LEFT]=左, [MOTOR_RIGHT]=右, 单位 mm/s, 仅用于本次逆解计算
    float motor_speeds[2];
    kinematics.kinematics_inverse(body_velocities, motor_speeds);

    // PID 更新目标轮速
    pid_controller[MOTOR_LEFT].update_target(motor_speeds[MOTOR_LEFT]);
    pid_controller[MOTOR_RIGHT].update_target(motor_speeds[MOTOR_RIGHT]);
}

/**
 * @brief micro-ROS 任务
 *
 * 单独创建一个任务运行 micro-ROS, 相当于一个线程。
 *
 * @param parameter 任务参数
 */
void micro_ros_task(void* parameter) {
    (void)parameter;

    // 静态局部变量: 仅本函数使用, 声明为 static 保持任务期间有效 (仿照 main.cpp)
    static rcl_allocator_t allocator;         // 内存分配器, 用于动态内存分配管理
    static rclc_support_t support;            // 用于存储时钟、内存分配器和上下文, 提供支持
    static rclc_executor_t executor;          // 执行器, 用于管理订阅和计时器回调的执行
    static rcl_node_t node;                   // ROS 节点
    static rcl_subscription_t subscriber;     // 订阅者
    static geometry_msgs__msg__Twist sub_msg; // 存储订阅到的速度消息

    // 1. 设置传输协议并延时等待设置完成
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP_STR);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT);
    delay(TRANSPORT_SETUP_MS);

    // 2. 初始化内存分配器
    allocator = rcl_get_default_allocator();

    // 3. 初始化 support
    rclc_support_init(&support, 0, NULL, &allocator);

    // 4. 初始化 ROS 节点
    rclc_node_init_default(&node, NODE_NAME, "", &support);

    // 5. 初始化执行器
    unsigned int num_handles = EXECUTOR_HANDLES; // 订阅事件和定时器事件的句柄数
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);

    // 6. 初始化订阅者并添加到执行器
    rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    );
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

    // 7. 循环执行器
    rclc_executor_spin(&executor);
}

/**
 * @brief 更新编码器速度并通过 PID 控制电机输出
 *
 * 调用运动学 update_motor_speed 根据编码器 tick 计算当前轮速,
 * 再经 PID 控制器输出 PWM 值更新电机。
 */
void update_and_control() {
    // 编码器 tick: [MOTOR_LEFT]=左, [MOTOR_RIGHT]=右
    const int32_t ticks[2] = {encoders[MOTOR_LEFT].getTicks(), encoders[MOTOR_RIGHT].getTicks()};
    kinematics.update_motor_speed(millis(), ticks);

    motor.updateMotorSpeed(
        MOTOR_LEFT,
        pid_controller[MOTOR_LEFT].update_pwm(kinematics.get_motor_speed(MOTOR_LEFT))
    );
    motor.updateMotorSpeed(
        MOTOR_RIGHT,
        pid_controller[MOTOR_RIGHT].update_pwm(kinematics.get_motor_speed(MOTOR_RIGHT))
    );
}

} // namespace
