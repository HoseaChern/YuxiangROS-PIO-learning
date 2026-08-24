/**
 * @brief 小车运动控制主程序: micro-ROS 订阅 /cmd_vel, 运动学逆解 + PID 控制, 发布 /odom 里程计
 */

#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PIDController.h>
#include <SemanticEnums.h>

// micro-ROS 和 WiFi 通信
#include <WiFi.h>
#include <geometry_msgs/msg/twist.h> // 速度订阅消息类型
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h> // 里程计发布消息类型
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "secrets.h"

// ============================================================================
// 全局状态: 匿名命名空间限定为本文件（内部链接），符合 C++ 规范
// 规则: 编译期常量一律 constexpr; 可变全局集中于此并注明被谁跨函数共享
// ============================================================================
namespace {

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 电机引脚 (电机0: 4/5, 电机1: 7/6) ----

constexpr uint8_t MOTOR_LEFT_PIN_A = 4;
constexpr uint8_t MOTOR_LEFT_PIN_B = 5;
constexpr uint8_t MOTOR_RIGHT_PIN_A = 7;
constexpr uint8_t MOTOR_RIGHT_PIN_B = 6;

// ---- 编码器引脚 (编码器0: 15/16, 编码器1: 18/17) ----

constexpr uint8_t ENC_LEFT_PIN_A = 15;
constexpr uint8_t ENC_LEFT_PIN_B = 16;
constexpr uint8_t ENC_RIGHT_PIN_A = 18;
constexpr uint8_t ENC_RIGHT_PIN_B = 17;

// ---- PID 参数 ----

constexpr float PID_KP = 0.625f;           // 比例增益
constexpr float PID_KI = 0.125f;           // 积分增益
constexpr float PID_KD = 0.0f;             // 微分增益
constexpr float PID_OUTPUT_LIMIT = 100.0f; // 输出限幅 ±100

// ---- 运动学参数 ----

constexpr float WHEEL_DISTANCE_MM = 175.0f;        // 轮间距, 单位 mm
constexpr float DISTANCE_PER_TICK_MM = 0.1427138f; // 单个脉冲对应的轮子前进距离, 单位 mm

// ---- 目标速度 ----

constexpr float TARGET_LINEAR_SPEED_MM_S = 50.0f;  // 目标线速度, 单位 mm/s
constexpr float TARGET_ANGULAR_SPEED_RAD_S = 0.1f; // 目标角速度, 单位 rad/s

// ---- 单位换算 ----

constexpr float MPS_TO_MMPS = 1000.0f; // m/s -> mm/s
constexpr double S_TO_NS = 1e6;        // 秒 -> 纳秒 (时间戳换算)

// ---- 控制周期 ----

constexpr uint32_t LOOP_DELAY_MS = 10; // 主循环调度节拍, 单位 ms

// ---- 网络配置 ----
// 注意: 凭据必须是可写 char 数组, 因库接口要求 char*, 故不能 constexpr
// 使用 secrets.h 存储 WIFI_SSID 和 WIFI_PASS

constexpr char AGENT_IP_STR[] = "192.168.2.120"; // 主机 IP(运行 micro-ROS Agent 的电脑)
constexpr uint16_t AGENT_PORT = 8888;            // Agent UDP 端口

// ---- 任务参数 ----

constexpr uint32_t MICRO_ROS_STACK_SIZE = 10240; // micro-ROS 任务栈字节数
constexpr uint8_t MICRO_ROS_TASK_PRIO = 1;       // 任务优先级
constexpr uint32_t ODOM_PUBLISH_MS = 50;         // 里程计发布周期, 单位 ms
constexpr uint32_t TRANSPORT_SETUP_MS = 2000;    // 传输层设置等待时间, 单位 ms
constexpr uint32_t SYNC_ATTEMPT_MS = 1000;       // 时间同步单次尝试时长, 单位 ms
constexpr uint32_t SYNC_POLL_MS = 10;            // 时间同步轮询间隔, 单位 ms

// ---- 订阅参数 ----

constexpr uint8_t EXECUTOR_HANDLES = 2;                 // 执行器句柄数(速度订阅+里程计定时器)
constexpr char CMD_VEL_TOPIC[] = "/cmd_vel";            // 速度指令话题名
constexpr char NODE_NAME[] = "fishbot_motion_control";  // 节点名

// ---- 发布参数 ----

constexpr char ODOM_TOPIC[] = "/odom"; // 里程计话题名

// ---- 可变全局状态 (跨 setup/loop/micro_ros_task/twist_callback 共享) ----

Esp32McpwmMotor motor;           // 电机驱动对象 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器对象数组 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器对象数组 (setup/twist_callback/loop 共享)
Kinematics kinematics;           // 运动学正逆解对象 (setup/twist_callback/odom_callback/loop 共享)

nav_msgs__msg__Odometry pub_msg; // 里程计消息 (odom_callback 填充, micro_ros_task 初始化)
rcl_publisher_t publisher;       // 里程计发布者 (micro_ros_task 初始化)

// ---- 函数前向声明（内部链接） ----

void twist_callback(const void* msg_in);
void odom_callback(rcl_timer_t* timer, int64_t last_call_time);
void micro_ros_task(void* parameter);
void update_and_control();

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化编码器
    encoders[MOTOR_LEFT].init(MOTOR_LEFT, ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);     // 编码器0
    encoders[MOTOR_RIGHT].init(MOTOR_RIGHT, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B); // 编码器1

    // 初始化电动机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);    // 电动机0
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B); // 电动机1

    // 初始化 PID 控制器参数
    pid_controller[MOTOR_LEFT].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[MOTOR_RIGHT].update_pid(PID_KP, PID_KI, PID_KD);
    pid_controller[MOTOR_LEFT].output_limit(PID_OUTPUT_LIMIT);  // 对称输出限幅 ±PID_OUTPUT_LIMIT
    pid_controller[MOTOR_RIGHT].output_limit(PID_OUTPUT_LIMIT); // 对称输出限幅 ±PID_OUTPUT_LIMIT

    // 初始化轮子间距和电动机参数
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);
    kinematics.set_motor_param(DISTANCE_PER_TICK_MM); // 标定量标量化: 两电机共用

    // 默认目标速度, 避免订阅消息到达前电机无目标
    // 逆解输出仅本次使用, 声明为局部变量, 作用域最小化
    // 车体速度: [VEL_LINEAR]=线速度 mm/s, [VEL_ANGULAR]=角速度 rad/s
    const float body_velocities[2] = {TARGET_LINEAR_SPEED_MM_S, TARGET_ANGULAR_SPEED_RAD_S};
    // 电机转速: [MOTOR_LEFT]=左, [MOTOR_RIGHT]=右, 单位 mm/s, 仅用于本次逆解计算
    float motor_speeds[2];
    kinematics.kinematics_inverse(body_velocities, motor_speeds);

    // PID 初始化目标轮速
    pid_controller[MOTOR_LEFT].update_target(motor_speeds[MOTOR_LEFT]);
    pid_controller[MOTOR_RIGHT].update_target(motor_speeds[MOTOR_RIGHT]);

    // 创建任务运行 micro-ROS
    // 参数依次为: 任务函数, 任务名称, 任务堆栈字节数, 传递给任务函数的参数, 任务优先级, 任务句柄
    xTaskCreate(micro_ros_task, "micro_ros", MICRO_ROS_STACK_SIZE, NULL, MICRO_ROS_TASK_PRIO, NULL);
}

void loop() {
    delay(LOOP_DELAY_MS);

    update_and_control();

    // 输出里程计数据
    Serial.printf(
        "x = %f, y = %f, yaw = %f\n",
        kinematics.get_odom().x,
        kinematics.get_odom().y,
        kinematics.get_odom().yaw
    );
}

// ============================================================================
// 函数实现（内部链接）
// ============================================================================
namespace {

/**
 * @brief 速度消息订阅回调函数
 * 
 * @param msg_in 消息指针
 */
void twist_callback(const void* msg_in) {
    // 将订阅消息强制转换为 Twist 消息指针
    // 在 pure-C 中, 空指针可以泛指任意类型指针; 因此在使用时, 应当强制类型转换到期待的类型
    const geometry_msgs__msg__Twist* twist_msg =
        static_cast<const geometry_msgs__msg__Twist*>(msg_in);

    // 计算运动学逆解: 车体速度 -> 电机目标转速
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
 * @brief 里程计定时器回调函数
 * 
 * @attention 在 micro_ros_platformio.h 中, 已经有函数声明定义为 void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
 * 
 * @param timer 定时器指针
 * @param last_call_time 上一次调用时间
 */
void odom_callback(rcl_timer_t* timer, int64_t last_call_time) {
    odom_t odom = kinematics.get_odom();     // 获取里程计
    int64_t stamp = rmw_uros_epoch_millis(); // 获取当前时间

    pub_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000);                // 秒部分
    pub_msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000) * static_cast<uint32_t>(S_TO_NS); // 纳秒部分

    // 设置里程计消息
    pub_msg.pose.pose.position.x = odom.x;
    pub_msg.pose.pose.position.y = odom.y;
    pub_msg.pose.pose.orientation.x = 0.0;
    pub_msg.pose.pose.orientation.y = 0.0;
    pub_msg.pose.pose.orientation.z = std::sin(odom.yaw / 2.0);
    pub_msg.pose.pose.orientation.w = std::cos(odom.yaw / 2.0);
    pub_msg.twist.twist.linear.x = odom.linear_velocity;
    pub_msg.twist.twist.angular.z = odom.angular_velocity;

    // 发布里程计消息
    if (rcl_publish(&publisher, &pub_msg, NULL) != RCL_RET_OK) {
        Serial.printf("Error: odom publisher failed!\n");
    }
}

/**
 * @brief micro-ROS 任务
 * 
 * @param parameter 任务参数
 * @note 
 * 1. 单独创建一个任务运行 micro-ROS, 相当于一个线程 \note
 * 2. xTaskCreate() 要求的任务函数原型必须为: void task(void* parameter) \note
 */
void micro_ros_task(void* parameter) {
    (void)parameter; // 显式转换为 void，告诉编译器"我故意不用"

    static rcl_allocator_t allocator; // 内存分配器
    static rclc_support_t support;    // 存储时钟/内存分配器/上下文, 提供支持
    static rcl_node_t node;           // ROS节点
    static rclc_executor_t executor;  // 管理订阅回调和计时器回调的执行

    static rcl_subscription_t subscriber;     // 速度消息订阅者
    static geometry_msgs__msg__Twist sub_msg; // 订阅的速度消息
    static rcl_timer_t timer;                 // 定时器

    // 1. 设置传输协议并延时等待设置完成
    // 主机 IP 地址: hostname -I / ipconfig / ip addr show
    // 注意, lo(本地回环)和state DOWN/NO-CARRIER(未工作)两类应当忽略
    // 这里最好用 IPv4
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

    // 6. 初始化速度消息订阅者并添加到执行器
    // 参数依次为: 订阅者指针, 节点指针, 消息接口类型, 订阅话题
    // best_effort: 使用最大努力, 详见QoS
    rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    );
    // 参数依次为: 执行器指针, 订阅者指针, 订阅消息指针, 订阅回调函数指针, 调用类型宏
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

    // 7. 初始化发布者
    pub_msg.header.frame_id = micro_ros_string_utilities_set(pub_msg.header.frame_id, "odom");
    pub_msg.child_frame_id =
        micro_ros_string_utilities_set(pub_msg.child_frame_id, "base_footprint");
    rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        ODOM_TOPIC
    );

    // 8. 时间同步
    while (!rmw_uros_epoch_synchronized()) {
        // 如果没有同步, 则尝试进行时间同步
        rmw_uros_sync_session(SYNC_ATTEMPT_MS);
        delay(SYNC_POLL_MS);
    }

    // 9. 初始化定时器并添加到执行器
    // 每 50 ms 执行一次, 此函数的第三参数单位要求是 ns
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ODOM_PUBLISH_MS), odom_callback);
    rclc_executor_add_timer(&executor, &timer);

    // 10. 循环执行器
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
