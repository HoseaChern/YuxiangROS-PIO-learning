/**
 * @file main.cpp
 * @brief 主固件: 运动控制(micro-ROS) + 激光雷达透传融合 (单板方案)
 *
 * 本固件把原先相互独立的两份已验证固件融合到同一块 ESP32-S3:
 *   - test08_Publisher: micro-ROS 订阅 /cmd_vel, 运动学逆解 + PID 控制, 发布 /odom
 *   - test09_bridge:    雷达 UART1->WiFi TCP 透传, M_CTR PWM 驱动雷达电机
 *
 * 调度架构 (三部分互不阻塞):
 *   - micro_ros_task (FreeRTOS): 统一初始化 WiFi, 运行 micro-ROS executor
 *                                (订阅 /cmd_vel + 定时发布 /odom)
 *   - bridge_task    (FreeRTOS): 雷达透传, 只等待 WiFi 就绪, 绝不重复 
 *                                WiFi.begin() (避免打断 micro-ROS 的 UDP 会话)
 *   - loop()                     : 电机 PID + 编码器闭环控制
 *
 * 资源占用核验 (无冲突):
 *   - GPIO: 电机 4/5/6/7, 编码器 15/16/17/18, 雷达 RX=14, M_CTR=13
 *   - 外设: 电机用 MCPWM(fishros 库), 雷达 M_CTR 用 LEDC, 彼此独立
 *   - 网络: micro-ROS UDP:8888 与透传 TCP:8889 共用同一 WiFi/lwIP, 端口独立
 *
 * 接线 (X2L, MX1.25-4P 线序 M_CTR->GND->Tx->VCC):
 *   VCC -> 5V;  GND -> GND;  Tx -> GPIO14 (UART1 RX);  M_CTR -> GPIO13 (PWM)
 *
 * 编译/烧录: pio run -e esp32-s3-devkitc-1 -t upload
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

#include "config.h"
#include "net_boot.h"

// ============================================================================
// 全局状态: 匿名命名空间限定为本文件（内部链接），符合 C++ 规范
// 规则: 编译期常量一律 constexpr; 可变全局集中于此并注明被谁跨函数共享
// ============================================================================
namespace {

// micro-ROS 执行器句柄数: /cmd_vel 订阅 + odom 发布定时器
constexpr uint8_t EXECUTOR_HANDLES = 2;

// ---- 可变全局状态 (跨 setup/loop/micro_ros_task/twist_callback 共享) ----

Esp32McpwmMotor motor;           // 电机驱动对象 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器对象数组 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器对象数组 (setup/twist_callback/loop 共享)
Kinematics kinematics;           // 运动学正逆解对象 (setup/twist_callback/odom_callback/loop 共享)

nav_msgs__msg__Odometry pub_msg; // 里程计消息 (odom_callback 填充, micro_ros_task 初始化)
rcl_publisher_t publisher;       // 里程计发布者 (micro_ros_task 初始化)

WiFiClient tcp_client; // 透传 TCP client (连上位机 ros_serial2wifi, bridge_task 共享)

// ---- 函数前向声明（内部链接） ----

void twist_callback(const void* msg_in);
void odom_callback(rcl_timer_t* timer, int64_t last_call_time);
void micro_ros_task(void* parameter);
void bridge_task(void* parameter);
void update_and_control();

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }
    Serial.println("\n=== fishbot: 运动控制 + 雷达透传 ===");

    // 雷达 UART1: 只配 RX (X2L 无数据 RX 引脚), 115200 8N1
    // 增大 RX 缓冲: 扫描数据流满载时防 FIFO 溢出丢帧 (需在 begin 前调用)
    Serial1.setRxBufferSize(4096);
    Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_UART_RX_PIN, -1);

    // M_CTR 电机 PWM (LEDC): 上电即驱动雷达电机旋转
    ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RES);
    ledcAttachPin(LIDAR_MOTOR_CTRL_PIN, LIDAR_PWM_CHANNEL);
    ledcWrite(LIDAR_PWM_CHANNEL, LIDAR_MOTOR_SPEED);
    Serial.printf(
        "[LIDAR] M_CTR PWM: pin=%u chan=%u freq=%uHz duty=%u\n",
        LIDAR_MOTOR_CTRL_PIN,
        LIDAR_PWM_CHANNEL,
        LIDAR_PWM_FREQ,
        LIDAR_MOTOR_SPEED
    );

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

    // 创建任务运行 micro-ROS (参数: 任务函数, 名称, 栈字节数, 参数, 优先级, 句柄)
    xTaskCreate(micro_ros_task, "micro_ros", MICRO_ROS_STACK_SIZE, NULL, MICRO_ROS_TASK_PRIO, NULL);
    // 创建任务运行雷达透传
    xTaskCreate(bridge_task, "bridge", BRIDGE_STACK_SIZE, NULL, BRIDGE_TASK_PRIO, NULL);
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
 * @param timer 定时器指针
 * @param last_call_time 上一次调用时间
 */
void odom_callback(rcl_timer_t* timer, int64_t last_call_time) {
    odom_t odom = kinematics.get_odom();     // 获取里程计
    int64_t stamp = rmw_uros_epoch_millis(); // 获取当前时间

    pub_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
    pub_msg.header.stamp.nanosec =
        static_cast<uint32_t>(stamp % 1000) * static_cast<uint32_t>(S_TO_NS); // 纳秒部分

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
 * @brief micro-ROS 任务 (统一初始化 WiFi)
 * 
 * @param parameter 任务参数
 * @note 
 * 1. 单独创建一个任务运行 micro-ROS, 相当于一个线程
 * 2. 本任务负责 wifi_role_boot() 网络自举 (建立 WiFi 连接并注册 UDP transport);
 *    bridge_task 只轮询 wifi_network_ready(), 不重复建网, 避免打断本任务的 UDP 会话。
 */
void micro_ros_task(void* parameter) {
    (void)parameter;

    static rcl_allocator_t allocator;
    static rclc_support_t support;
    static rcl_node_t node;
    static rclc_executor_t executor;

    static rcl_subscription_t subscriber;
    static geometry_msgs__msg__Twist sub_msg;
    static rcl_timer_t timer;

    // 1. 网络自举并延时等待设置完成 (STA 接入 / AP 自组网由 WIFI_ROLE_AP 决定)
    IPAddress agent_ip;
    wifi_role_boot(agent_ip);  // 按 WIFI_ROLE_AP 决定 STA 接入或 AP 自组网
    delay(TRANSPORT_SETUP_MS); // 等待传输层设置完成

    // 2. 初始化内存分配器
    allocator = rcl_get_default_allocator();

    // 3. 初始化 support
    rclc_support_init(&support, 0, NULL, &allocator);

    // 4. 初始化 ROS 节点
    rclc_node_init_default(&node, NODE_NAME, "", &support);

    // 5. 初始化执行器
    unsigned int num_handles = EXECUTOR_HANDLES;
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);

    // 6. 初始化速度消息订阅者并添加到执行器
    rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    );
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
        rmw_uros_sync_session(SYNC_ATTEMPT_MS);
        delay(SYNC_POLL_MS);
    }

    // 9. 初始化定时器并添加到执行器
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ODOM_PUBLISH_MS), odom_callback);
    rclc_executor_add_timer(&executor, &timer);

    // 10. 循环执行器
    rclc_executor_spin(&executor);
}

/**
 * @brief 雷达透传任务 (UART1 <-> TCP)
 *
 * @param parameter 任务参数
 * @note 
 * 1. 只等待 WiFi 就绪(micro_ros_task 已建立), 绝不重复 WiFi.begin()
 * 2. WiFi/UART 初始化已在 setup 完成, 本任务只做连接与双向透传
 * 3. 雷达数据为单向出 (X2L), TCP 下行映射回 UART 保持契约, 忽略雷达侧
 */
void bridge_task(void* parameter) {
    (void)parameter;

    static uint8_t buf[512];
    uint32_t uart_rx_total = 0;
    uint32_t last_diag = 0;

    for (;;) {
        // 1. 等待网络就绪 (由 micro_ros_task 自举; AP 模式下 SoftAP 启动即就绪)
        if (!wifi_network_ready()) {
            delay(BRIDGE_RECONNECT_MS);
            continue;
        }

        // 2. 确保 TCP 已连接 (上位机 ros_serial2wifi tcp_server:8889)
        if (!tcp_client.connected()) {
            Serial.printf("[BRIDGE] 连接 %s:%u ...\n", AGENT_IP_STR, BRIDGE_TCP_PORT);
            if (tcp_client.connect(AGENT_IP_STR, BRIDGE_TCP_PORT)) {
                Serial.println("[BRIDGE] TCP 已连接");
            } else {
                delay(BRIDGE_RECONNECT_MS);
                continue;
            }
        }

        // 3. 双向透传: UART1 -> TCP
        size_t n = Serial1.available();
        if (n > 0) {
            if (n > sizeof(buf)) {
                n = sizeof(buf);
            }
            n = Serial1.read(buf, n);
            if (n > 0) {
                uart_rx_total += n;
                tcp_client.write(buf, n);
            }
        }
        // TCP -> UART1 (保留双向契约; X2L 无数据 RX, 雷达侧忽略)
        size_t m = tcp_client.available();
        if (m > 0) {
            if (m > sizeof(buf)) {
                m = sizeof(buf);
            }
            m = tcp_client.read(buf, m);
            if (m > 0) {
                Serial1.write(buf, m);
            }
        }

        // 4. 诊断: 每 2s 打印 UART1 累计接收字节 (独立于 WiFi/TCP 状态)
        if (millis() - last_diag >= 2000) {
            Serial.printf("[UART] 最近 2s 收到 %u 字节\n", uart_rx_total);
            uart_rx_total = 0;
            last_diag = millis();
        }
    }
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
