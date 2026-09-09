#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PIDController.h>
#include <SemanticEnums.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "config.h"
#include "net_boot.h"

namespace {

// micro-ROS 执行器句柄容量: 本固件不注册订阅/定时器句柄, 仅手动 spin 保活;
// rclc 契约要求句柄容量 >= 1 (rclc_executor_init 对 0 直接返回 RCL_RET_INVALID_ARGUMENT
// 且不初始化 executor), 故取最小合法值 1, 避免 executor 初始化必然失败
constexpr uint8_t EXECUTOR_HANDLES = 1;

// ---- 可变全局状态 (跨 setup/loop/micro_ros_task 共享) ----

Esp32McpwmMotor motor;           // 电机驱动对象 (setup/loop 共享)
Esp32PcntEncoder encoders[2];    // 编码器对象数组 (setup/loop 共享)
PIDController pid_controller[2]; // PID 控制器对象数组 (setup/loop 共享)
Kinematics kinematics;           // 运动学正逆解对象 (setup/loop 共享)

// ---- 函数前向声明（内部链接） ----

void micro_ros_task(void* parameter);
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

    // 运动学逆解: 目标线速度和角速度 -> 目标左轮速度和右轮速度
    // 逆解输出仅本次使用, 声明为局部变量, 作用域最小化 (仿照 main.cpp)
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
}

namespace {

/**
 * @brief micro-ROS 任务
 *
 * 单独创建一个任务运行 micro-ROS, 相当于一个线程。
 * xTaskCreate() 要求的任务函数原型必须为: void task(void* parameter)
 *
 * 本函数执行 "初始化 -> spin -> 释放 -> 重建" 的完整生命周期, 任务永不返回。
 * 依据: xTaskCreate 创建的任务函数一旦 return, FreeRTOS 会从已失效的任务栈指针
 * 继续调度, 表现为 PC 奇数不对齐、IllegalInstruction 复位; 故初始化失败或
 * agent 会话失效时必须循环重建, 不得越过函数尾返回。
 *
 * @param parameter 任务参数
 */
void micro_ros_task(void* parameter) {
    (void)parameter; // 任务参数未使用

    // 静态局部变量: 仅本函数使用, static 保持其跨重建循环存活, 避免栈上反复分配
    static rcl_allocator_t allocator; // 内存分配器, 用于动态内存分配管理
    static rclc_support_t support;    // 用于存储时钟、内存分配器和上下文, 提供支持
    static rclc_executor_t executor;  // 执行器, 用于管理订阅和计时器回调的执行
    static rcl_node_t node;           // ROS 节点

    // 1. 网络自举并延时等待设置完成 (STA 接入 / AP 自组网由 WIFI_ROLE_AP 决定),
    //    仅执行一次: transport 注册与后续 rmw 会话建立解耦
    IPAddress agent_ip;
    wifi_role_boot(agent_ip);  // 按 WIFI_ROLE_AP 决定 STA 接入或 AP 自组网
    delay(TRANSPORT_SETUP_MS); // 等待传输层设置完成

    // 2. 初始化内存分配器
    allocator = rcl_get_default_allocator();

    // 3-6. 每步检查返回值: 任一失败即释放已成功初始化的对象并延时重建;
    //     若不检查, 失败会传播到未初始化句柄, 或使 spin 异常返回后越过函数尾触发复位
    for (;;) {
        // 3. 初始化 support: 内部经 rmw 与 agent 建立会话, agent 不可达时在此失败
        rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
        // 检查 support 是否成功创建
        if (ret != RCL_RET_OK) {
            // 若失败
            Serial.printf(
                "[micro_ros] support init failed (%d), agent unreachable, retry in %u ms\n",
                ret,
                RECONNECT_INTERVAL_MS
            );

            delay(RECONNECT_INTERVAL_MS); // 延时后重试
            continue;                     // 回到循环头部重新初始化
        }

        // 4. 初始化 ROS 节点
        ret = rclc_node_init_default(&node, NODE_NAME, "", &support);
        // 检查 node 是否成功创建
        if (ret != RCL_RET_OK) {
            // 若失败
            Serial.printf(
                "[micro_ros] node init failed (%d), retry in %u ms\n",
                ret,
                RECONNECT_INTERVAL_MS
            );

            // node 未初始化成功, 逆序释放已成功的 support
            rcl_ret_t support_fini_ret = rclc_support_fini(&support);
            (void)support_fini_ret; // fini 失败无补救动作, 消费返回值

            delay(RECONNECT_INTERVAL_MS); // 延时后重试
            continue;                     // 回到循环头部重新初始化
        }

        // 5. 初始化执行器: 句柄容量 EXECUTOR_HANDLES >= 1, 满足 rclc 契约
        ret = rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator);
        // 检查 executor 是否成功创建
        if (ret != RCL_RET_OK) {
            // 若失败
            Serial.printf(
                "[micro_ros] executor init failed (%d), retry in %u ms\n",
                ret,
                RECONNECT_INTERVAL_MS
            );

            // 逆序释放已成功的 node
            rcl_ret_t node_fini_ret = rcl_node_fini(&node);
            (void)node_fini_ret; // fini 失败无补救动作, 消费返回值

            // node 未初始化成功, 逆序释放已成功的 support
            rcl_ret_t support_fini_ret = rclc_support_fini(&support);
            (void)support_fini_ret; // fini 失败无补救动作, 消费返回值

            delay(RECONNECT_INTERVAL_MS); // 延时后重试
            continue;                     // 回到循环头部重新初始化
        }

        // 6. 手动 spin 轮询: 本固件无注册句柄, rcl_wait 对空等待集立即返回,
        //    因此 rclc_executor_spin 会退化为无休眠忙循环独占 core0;
        //    改用 spin_some + delay 显式让出 CPU。仅当 agent 会话失效(context 无效)
        //    时 spin_some 返回错误, 跳出本层循环进入重建
        Serial.printf("[micro_ros] node \"%s\" ready, spinning\n", NODE_NAME);
        while (true) {
            ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if ((ret != RCL_RET_OK) && (ret != RCL_RET_TIMEOUT)) {
                Serial.printf(
                    "[micro_ros] spin_some failed (%d), agent session lost, rebuild in %u ms\n",
                    ret,
                    RECONNECT_INTERVAL_MS
                );
                break; // 跳出 spin 层, 进入下方逆序释放与重建
            }
            delay(1); // 让出 CPU, 避免空句柄下无休眠轮询独占 core0
        }

        // 逆序释放本生命周期内已初始化的对象, 回到循环头部重建
        // 先释放执行器
        rcl_ret_t executor_fini_ret = rclc_executor_fini(&executor);
        (void)executor_fini_ret; // 消费返回值

        // 再释放 node
        rcl_ret_t node_fini_ret = rcl_node_fini(&node);
        (void)node_fini_ret; // 消费返回值

        // 最后释放 support
        rcl_ret_t support_fini_ret = rclc_support_fini(&support);
        (void)support_fini_ret; // 消费返回值

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
