/**
 * @brief also the test08_micro_ros_platformio
 */

#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PIDController.h>

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
// 全局状态：匿名命名空间限定为本文件（内部链接），符合 C++ 规范
// 规则: 编译期常量一律 constexpr; 可变全局集中于此并注明被谁跨函数共享
// ============================================================================
namespace {

// ---- 编译期常量 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// 编码器引脚 (编码器0: 4/5, 编码器1: 14/15)

constexpr uint8_t ENC0_PIN_A = 4;
constexpr uint8_t ENC0_PIN_B = 5;
constexpr uint8_t ENC1_PIN_A = 14;
constexpr uint8_t ENC1_PIN_B = 15;

// 电机引脚 (电机0: 10/11, 电机1: 12/13)

constexpr uint8_t MOTOR0_PIN_A = 10;
constexpr uint8_t MOTOR0_PIN_B = 11;
constexpr uint8_t MOTOR1_PIN_A = 12;
constexpr uint8_t MOTOR1_PIN_B = 13;

// PID 参数

constexpr float PID_KP = 0.625;           // 比例增益
constexpr float PID_KI = 0.125;           // 积分增益
constexpr float PID_KD = 0.0;             // 微分增益
constexpr float PID_OUTPUT_LIMIT = 100.0; // 输出限幅 ±100

// 运动学参数

constexpr float WHEEL_DISTANCE_MM = 175.0; // 轮间距, 单位 mm
constexpr float MOTOR_PARAM = 0.1427138f;  // 电机标定参数

// ---- 网络配置 ----
// 注意: 凭据必须是可写 char 数组, 因库接口要求 char*, 故不能 constexpr
// 使用 secrets.h 存储 WIFI_SSID 和 WIFI_PASS

constexpr char AGENT_IP_STR[] = "192.168.2.120"; // 主机 IP(运行 micro-ROS Agent 的电脑)
constexpr uint16_t AGENT_PORT = 8888;            // Agent UDP 端口

// 任务与定时器参数

constexpr uint32_t MICRO_ROS_STACK_SIZE = 10240; // micro-ROS 任务栈字节数
constexpr uint8_t MICRO_ROS_TASK_PRIO = 1;       // 任务优先级
constexpr uint32_t ODOM_PUBLISH_MS = 50;         // 里程计发布周期, 单位 ms
constexpr uint32_t LOOP_DELAY_MS = 10;           // 主循环调度节拍
constexpr uint8_t EXECUTOR_HANDLES = 2;          // 执行器句柄数(速度订阅+里程计定时器)
constexpr uint32_t TRANSPORT_SETUP_MS = 2000;    // 传输层设置等待时间
constexpr uint32_t SYNC_ATTEMPT_MS = 1000;       // 时间同步单次尝试时长
constexpr uint32_t SYNC_POLL_MS = 10;            // 时间同步轮询间隔

// 单位换算

constexpr float M_TO_MM = 1000.0; // m/s -> mm/s
constexpr double S_TO_NS = 1e6;   // 秒 -> 纳秒 (时间戳换算)

// ---- 可变全局状态 (跨函数共享, 需长期存活) ----

Esp32McpwmMotor motor;           // 电机控制 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器 (twist_callback/loop 共享)
Kinematics kinematics;           // 运动学正逆解 (twist_callback/odom_callback/loop 共享)

nav_msgs__msg__Odometry pub_msg; // 里程计消息 (odom_callback 填充, micro_ros_task 初始化)
rcl_publisher_t publisher;       // 里程计发布者 (micro_ros_task 初始化)

// ---- 函数前向声明（内部链接; 经函数指针传给 micro-ROS/FreeRTOS, 链接性不影响取址）----

void twist_callback(const void* msgin);
void odom_callback(rcl_timer_t* timer, int64_t last_call_time);
void micro_ros_task(void* parameter);
void update_and_control();

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);

    // 初始化编码器
    encoders[0].init(0, ENC0_PIN_A, ENC0_PIN_B); // 编码器0
    encoders[1].init(1, ENC1_PIN_A, ENC1_PIN_B); // 编码器1

    // 初始化电动机
    motor.attachMotor(0, MOTOR0_PIN_A, MOTOR0_PIN_B); // 电动机0
    motor.attachMotor(1, MOTOR1_PIN_A, MOTOR1_PIN_B); // 电动机1

    // 初始化PID控制器
    pid_controller[0].update_PID(PID_KP, PID_KI, PID_KD);
    pid_controller[1].update_PID(PID_KP, PID_KI, PID_KD);
    pid_controller[0].output_limit(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    pid_controller[1].output_limit(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

    // 初始化轮间距和电机参数
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);
    kinematics.set_motor_param(0, MOTOR_PARAM);
    kinematics.set_motor_param(1, MOTOR_PARAM);

    // 计算运动学逆解: 目标线速度和角速度 -> 目标左轮速度和右轮速度
    // 仅初始化用一次的测试目标值, 声明为 const 局部变量, 作用域最小化
    const float target_linear_velocity = 50.0; // 目标线速度, 单位 mm/s
    const float target_angular_velocity = 0.1; // 目标角速度, 单位 rad/s

    float output_left_speed;  // 目标左轮速度, 单位 mm/s, 临时中间变量，仅用于本次逆解计算
    float output_right_speed; // 目标右轮速度, 单位 mm/s, 临时中间变量，仅用于本次逆解计算
    kinematics.kinematics_inverse(
        target_linear_velocity,
        target_angular_velocity,
        output_left_speed,
        output_right_speed
    );

    // PID更新目标轮速
    pid_controller[0].update_target(output_left_speed);
    pid_controller[1].update_target(output_right_speed);

    // 创建任务运行
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
 * @param msgin 消息指针
 */
void twist_callback(const void* msgin) {
    // 将订阅消息强制转换为 Twist 消息指针
    // 在 pure-C 中, 空指针可以泛指任意类型指针; 因此在使用时, 应当强制类型转换到期待的类型
    const geometry_msgs__msg__Twist* twist_msg =
        static_cast<const geometry_msgs__msg__Twist*>(msgin);

    // 计算运动学逆解
    float output_left_speed;  // 目标左轮速度, 单位 mm/s, 临时中间变量，仅用于本次逆解计算
    float output_right_speed; // 目标右轮速度, 单位 mm/s, 临时中间变量，仅用于本次逆解计算
    kinematics.kinematics_inverse(
        twist_msg->linear.x * M_TO_MM, // 单位换算, m/s -> mm/s
        twist_msg->angular.z,
        output_left_speed,
        output_right_speed
    );

    // PID更新目标轮速
    pid_controller[0].update_target(output_left_speed);
    pid_controller[1].update_target(output_right_speed);
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
    pub_msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000) * S_TO_NS; // 纳秒部分

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

    // 4.初始化ROS节点 fishbot_motion_control
    rclc_node_init_default(&node, "fishbot_motion_control", "", &support);

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
        "/cmd_vel"
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
        "/odom"
    );

    // 8.时间同步
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
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());

    motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0)));
    motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1)));
}

} // namespace
