/**
 * @brief 小车运动控制主程序: micro-ROS 订阅 /cmd_vel, 运动学逆解 + PID 控制, 发布 /odom 里程计
 *
 * @note 本文件由原 src/main.cpp 迁移而来, 作为 test08_Publisher 环境保留已验证固件。
 *       主环境 src/main.cpp 已清空占位, 预留给后续新主程序 (雷达转接/运动控制+透传融合)。
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

// ---- micro-ROS 实体 (micro_ros_task 经 create_entities 建立, destroy_entities 释放) ----
// allocator 与 support 同级声明: rclc_support_init_with_options 把分配器地址存入
// support->allocator, 分配器对象的生命周期须不短于 support。

rcl_allocator_t allocator;         // 内存分配器
rclc_support_t support;            // 稳态时钟 + 内存分配器 + 上下文 (rmw 会话的载体)
rcl_node_t node;                   // ROS 节点
rclc_executor_t executor;          // 执行器: 管理句柄及其回调的执行
rcl_subscription_t subscriber;     // /cmd_vel 订阅者
geometry_msgs__msg__Twist sub_msg; // /cmd_vel 消息缓冲区 (执行器写入, twist_callback 读取)
rcl_timer_t timer;                 // 里程计发布定时器

nav_msgs__msg__Odometry pub_msg; // 里程计消息 (odom_callback 填充)
rcl_publisher_t publisher;       // 里程计发布者

// ---- 函数前向声明（内部链接） ----

bool create_entities();
void destroy_entities();
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
 * @brief 建立 micro-ROS 会话并按创建顺序装配实体 (在 micro_ros_task 中调用)
 *
 * 逐步检查返回值, 任一步失败立即返回 false, 由调用方 destroy_entities 统一逆序释放。
 * 依据: 初始化失败会把句柄留在无效状态, 继续注册或 spin 会沿无效句柄传播失败;
 * rclc_executor_init 对句柄容量 0 直接返回 RCL_RET_INVALID_ARGUMENT 且不初始化 executor,
 * 故句柄容量必须 >= 1。
 *
 * @return true=会话与全部实体就绪; false=任一步失败
 */
bool create_entities() {
    // 1. 初始化 support: 内部经 rmw 与 agent 建立会话, agent 不可达时在此失败
    rcl_ret_t ret = rclc_support_init(&support, 0, nullptr, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] support init failed (%d), agent unreachable, retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 2. 初始化 ROS 节点
    ret = rclc_node_init_default(&node, NODE_NAME, "", &support);
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] node init failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 3. 初始化执行器: 句柄容量 EXECUTOR_HANDLES >= 1, 满足 rclc 契约
    ret = rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] executor init failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 4. 初始化订阅者: best_effort 与上位机 QoS 匹配
    ret = rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    );
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] subscription init failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 5. 把订阅句柄注册到执行器: 仅新数据触发 twist_callback
    ret = rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &sub_msg,
        &twist_callback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] add subscription failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 6. 初始化发布者: 发布 nav_msgs/Odometry 到 ODOM_TOPIC
    ret = rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        ODOM_TOPIC
    );
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] publisher init failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 7. 时间同步: odom_callback 的时间戳取自 rmw_uros_epoch_millis(), 须先与 agent 对齐 epoch。
    //    rmw_uros_sync_session 返回 RMW_RET_OK 等价于 rmw_uros_epoch_synchronized() 为真
    //    (其内部仅在 uxr_sync_session 成功时置 session.synchronized), 故不需要 while 轮询;
    //    同步失败即会话不可用, 返回 false 由调用方释放并重建
    ret = rmw_uros_sync_session(SYNC_ATTEMPT_MS);
    if (ret != RMW_RET_OK) {
        Serial.printf(
            "[micro_ros] time sync failed (%d) within %u ms, retry in %u ms\n",
            ret,
            SYNC_ATTEMPT_MS,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 8. 初始化定时器: 每 ODOM_PUBLISH_MS 触发一次 odom_callback, 第三参数单位要求是 ns
    ret = rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ODOM_PUBLISH_MS), odom_callback);
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] timer init failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    // 9. 把定时器句柄注册到执行器
    ret = rclc_executor_add_timer(&executor, &timer);
    if (ret != RCL_RET_OK) {
        Serial.printf(
            "[micro_ros] add timer failed (%d), retry in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        return false;
    }

    return true;
}

/**
 * @brief 逆序释放 create_entities 装配的全部实体 (在 micro_ros_task 中调用)
 *
 * 释放顺序与创建顺序相反, 保证引用者先于被引用者释放 (定时器/发布者/订阅者先于 node, node 先于 support)。
 * 各 fini 对零初始化句柄返回 Ok 且不解引用, 故可全量调用; 返回值仅为错误码, 失败无补救动作。
 * 返回值统一以局部变量消费: rcl_node_fini 的声明带 warn_unused_result 属性, 强制转换到 void 无法消除该告警。
 * support 未建立 (context.impl 为空) 时其余实体必然未创建, 直接返回:
 * 该情况下 rclc_support_fini 会依次打印 rcl_clock_fini / rcl_shutdown 的告警。
 */
void destroy_entities() {
    // support 未建立, 则会话未建立, 其余实体未创建
    if (support.context.impl == nullptr) {
        return;
    }

    // 1. 释放定时器
    rcl_ret_t timer_fini_ret = rcl_timer_fini(&timer);
    (void)timer_fini_ret;

    // 2. 释放发布者 (其创建依赖 node, 必须先于 node 释放)
    rcl_ret_t publisher_fini_ret = rcl_publisher_fini(&publisher, &node);
    (void)publisher_fini_ret;

    // 3. 释放订阅者 (同上)
    rcl_ret_t subscription_fini_ret = rcl_subscription_fini(&subscriber, &node);
    (void)subscription_fini_ret;

    // 4. 释放执行器
    rcl_ret_t executor_fini_ret = rclc_executor_fini(&executor);
    (void)executor_fini_ret;

    // 5. 释放 node
    rcl_ret_t node_fini_ret = rcl_node_fini(&node);
    (void)node_fini_ret;

    // 6. 释放 support (稳态时钟 + 上下文, 最后释放)
    rcl_ret_t support_fini_ret = rclc_support_fini(&support);
    (void)support_fini_ret;
}

/**
 * @brief micro-ROS 任务
 *
 * 1. 单独创建一个任务运行 micro-ROS, 相当于一个线程;
 * 2. xTaskCreate() 要求的任务函数原型必须为: void task(void* parameter)。
 *
 * 生命周期: 网络自举 -> create_entities -> spin -> destroy_entities -> 延时重建, 任务永不返回。
 * 依据 1: 会话失效后 rmw 无重建会话路径 (uxr_create_session 只在 rmw_init 中调用),
 *         只有重建 support 才能恢复, 故 spin 出错后须释放并重建。
 * 依据 2: xTaskCreate 创建的任务函数一旦 return, FreeRTOS 会从已失效的任务栈指针继续调度,
 *         表现为 PC 奇数不对齐、IllegalInstruction 复位, 故不得越过函数尾返回。
 *
 * @param parameter 任务参数
 */
void micro_ros_task(void* parameter) {
    (void)parameter; // 显式转换为 void，告诉编译器"我故意不用"

    // 1. 网络自举并延时等待设置完成 (STA 接入 / AP 自组网由 WIFI_ROLE_AP 决定),
    //    仅执行一次: transport 注册与后续 rmw 会话建立解耦
    IPAddress agent_ip;
    wifi_role_boot(agent_ip);  // 按 WIFI_ROLE_AP 决定 STA 接入或 AP 自组网
    delay(TRANSPORT_SETUP_MS); // 等待传输层设置完成

    // 2. 初始化内存分配器
    allocator = rcl_get_default_allocator();

    // 3. 填充里程计消息的 frame_id: 字符串由 micro_ros_string_utilities_set 分配堆内存,
    //    消息对象 pub_msg 跨重建循环存活, 故只分配一次, 避免每次重建重复分配
    pub_msg.header.frame_id = micro_ros_string_utilities_set(pub_msg.header.frame_id, "odom");
    pub_msg.child_frame_id =
        micro_ros_string_utilities_set(pub_msg.child_frame_id, "base_footprint");

    // 4. 建立会话并运行: 创建失败或会话失效均回到循环头部重建
    for (;;) {
        if (!create_entities()) {
            destroy_entities();           // 释放已成功创建的部分
            delay(RECONNECT_INTERVAL_MS); // 延时后重试
            continue;                     // 回到循环头部重建
        }

        // 手动 spin 轮询: 等待集含 /cmd_vel 订阅与 odom 定时器两个句柄, rcl_wait 单轮最多阻塞
        // RCL_MS_TO_NS(10), 若无事件则到点返回; 仅当 agent 会话失效 (context 无效)
        // 时 spin_some 返回错误
        Serial.printf("[micro_ros] node \"%s\" ready, spinning\n", NODE_NAME);
        rcl_ret_t ret = RCL_RET_OK;
        while (true) {
            ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if ((ret != RCL_RET_OK) && (ret != RCL_RET_TIMEOUT)) {
                break; // 跳出 spin 层, 进入下方释放与重建
            }
            delay(1); // 让出 CPU, 下一轮重新等待句柄事件
        }

        // 5. 会话失效: 打印后释放实体并重建
        Serial.printf(
            "[micro_ros] spin_some failed (%d), agent session lost, rebuild in %u ms\n",
            ret,
            RECONNECT_INTERVAL_MS
        );
        destroy_entities();
        delay(RECONNECT_INTERVAL_MS); // 延时后回到循环头部重建
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
