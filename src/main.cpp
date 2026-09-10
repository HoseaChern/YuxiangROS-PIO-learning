/**
 * @file main.cpp
 * @brief 主固件: 两轮自平衡无线操控 + 激光雷达透传融合 (单板方案)
 *
 * 本固件把两份固件融合到同一块 ESP32-S3:
 *   - test13_balance: micro-ROS 无线遥控 (订阅 /cmd_vel + /balance_enable) 与三环串级直立控制
 *   - test09_bridge:  雷达 UART1->WiFi TCP 透传, M_CTR PWM 驱动雷达电机
 *
 * 相对 test13 的增量: /odom 里程计发布 (nav_msgs/Odometry, 与 test08_Publisher 一致)
 * 与 bridge_task 雷达透传任务。
 *
 * 调度架构 (三部分互不阻塞):
 *   - balance_task  (core1, 5ms 固定节拍): 姿态读取 + 编码器测速 + 三环 PID + 串口命令
 *   - micro_ros_task(默认核)            : 统一初始化 WiFi, 运行 micro-ROS executor
 *                                        (订阅 /cmd_vel + /balance_enable, 定时发布 /odom)
 *   - bridge_task   (core0)             : 雷达透传, 只等待 WiFi 就绪, 绝不重复
 *                                        WiFi.begin() (避免打断 micro-ROS 的 UDP 会话)
 *   - loop()                            : 空转 (控制全在 balance_task)
 *
 * 资源占用核验 (无冲突):
 *   - GPIO: 电机 5/4/6/7, 编码器 16/15/17/18, IMU SDA=10/SCL=9, 雷达 RX=14, M_CTR=13
 *   - 外设: 电机用 MCPWM, 编码器用 PCNT, IMU 用 I2C, 雷达 M_CTR 用 LEDC, 彼此独立
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
#include <MPU6050_light.h>
#include <Wire.h>

// micro-ROS 与 WiFi 通信
#include <WiFi.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>

#include <cmath>

#include <Kinematics.h>
#include <PIDController.h>
#include <SemanticEnums.h>

#include "config.h"
#include "net_boot.h"

namespace {

// ---- 固件本地常量 (跨固件共用参数见 include/RobotConfig/config.h) ----

// micro-ROS 执行器句柄数: /cmd_vel 订阅 + /balance_enable 订阅 + odom 发布定时器
constexpr uint8_t EXECUTOR_HANDLES = 3;
// micro-ROS 节点名
constexpr char BALANCE_NODE_NAME[] = "fishbot_balance";

constexpr float MPS_TO_MM_S = 1000.0f;        // m/s -> mm/s 换算 (ROS /cmd_vel 指令单位为 m/s)
constexpr uint32_t AGENT_RECONNECT_MS = 1000; // Agent 会话断开后的重连等待, 单位 ms

enum class BalanceState : uint8_t {
    kIdle,    // 停止: 输出关闭, 等待武装且姿态进入中值窗口
    kRunning, // 直立控制: 速度环 + 直立环串级 + 单一转向环差模输出
};

// ---- 可变全局状态 ----

// 仅 balance_task 访问
Esp32McpwmMotor motor;        // 电机驱动对象
Esp32PcntEncoder encoders[2]; // 编码器对象数组 (两轮)
MPU6050 mpu(Wire);            // MPU6050 对象, 使用 Wire 作为 I2C 总线
PIDController speed_pid;      // 速度环 PI 控制器 (外环, 参数在 setup 中配置)
PIDController balance_pid;    // 直立环 PD 控制器 (内环, 参数在 setup 中配置)
PIDController turn_pid;       // 转向环控制器 (差模单一完整转向环, 参数在 setup 中配置)
BalanceState balance_state = BalanceState::kIdle; // 当前状态机状态
bool balance_armed = false;                       // 武装标志 (cmd_enable 同步而来, 倒地自动解除)
float zero_pitch_deg = UPRIGHT_ZERO_PITCH_DEG;    // 机械中值 theta_0, 可由 'c' 命令在线标定

// balance_task 写 / odom_callback 读, 临界区保护 (见 odom_mux)
Kinematics kinematics; // 运动学对象: 编码器测速 + 正解 + 里程计积分

// 里程计发布 (odom_callback 填充, micro_ros_task 初始化)
nav_msgs__msg__Odometry pub_msg;
rcl_publisher_t publisher;

// 透传 TCP client (连上位机 ros_serial2wifi, bridge_task 共享)
WiFiClient tcp_client;

// ---- 跨任务共享变量 (micro-ROS 回调写 / balance_task 读, 临界区保护) ----
// micro_ros_task 与 balance_task 分属不同核心, 跨核共享非原子 float 必须用临界区,
// 否则 balance_task 可能读到撕裂值 (float 非原子, 多字段一致性亦无法保证)。

portMUX_TYPE cmd_mux = portMUX_INITIALIZER_UNLOCKED; // 命令共享变量临界区锁
volatile float cmd_linear_mps = 0.0f;                // /cmd_vel linear.x 原始值, 单位 m/s
volatile float cmd_angular_rps = 0.0f;               // /cmd_vel angular.z 原始值, 单位 rad/s
volatile bool cmd_enable = false;                    // /balance_enable data: true=请求武装

// 里程计临界区: balance_task 积分写 / odom_callback 拷贝读, 防多字段读到混合状态
portMUX_TYPE odom_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- 函数前向声明 (内部链接) ----

void handle_serial_command(float theta);
void control_step();
void balance_task(void* param);
void micro_ros_task(void* param);
bool connect_tcp();
void bridge_task(void* param);

} // namespace

void setup() {
    // 初始化调试串口 (115200), 等待 USB 串口就绪
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    // 初始化 I2C 总线 (引脚见 config.h: IMU_SDA_PIN / IMU_SCL_PIN) 并探测 MPU6050
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    byte status = mpu.begin();
    if (status != 0) {
        // 探测失败 (I2C 地址无应答): 打印错误码后死循环, 便于排查接线
        Serial.printf("[IMU] MPU6050 initialise failed, status=%u, shutdown\n", status);
        while (true) {
            delay(IMU_FAIL_LOOP_MS);
        }
    }

    // 陀螺仪零偏校准: 期间必须保持小车静止平放, 校准结果作为角度零点基准
    // 雷达电机在下一步才启动: 电机振动经结构传导至 IMU 会污染零偏采样, 故校准先于雷达上电
    Serial.println("[IMU] calibrating gyro offset, keep the car still and level...");
    delay(UPRIGHT_CALM_DELAY_MS); // 静置等待传感器稳定后再采样
    mpu.calcOffsets();
    Serial.println(
        "[IMU] calibration done. Commands: s=arm/stop c=calibrate-zero. "
        "ROS: /cmd_vel + /balance_enable"
    );

    // 雷达 UART1: 只配 RX (X2L 无数据 RX 引脚), 115200 8N1
    // 增大 RX 缓冲: 扫描数据流满载时防 FIFO 溢出丢帧 (需在 begin 前调用)
    Serial1.setRxBufferSize(4096);
    Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_UART_RX_PIN, -1);

    // M_CTR 电机 PWM (LEDC): 上电即驱动雷达电机旋转
    ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RES);
    ledcAttachPin(LIDAR_MOTOR_CTRL_PIN, LIDAR_PWM_CHANNEL);
    ledcWrite(LIDAR_PWM_CHANNEL, LIDAR_MOTOR_SPEED);
    Serial.printf(
        "[MOTOR] M_CTR PWM: pin=%u chan=%u freq=%uHz duty=%u\n",
        LIDAR_MOTOR_CTRL_PIN,
        LIDAR_PWM_CHANNEL,
        LIDAR_PWM_FREQ,
        LIDAR_MOTOR_SPEED
    );

    // 初始化两路编码器
    encoders[MOTOR_LEFT].init(MOTOR_LEFT, ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
    encoders[MOTOR_RIGHT].init(MOTOR_RIGHT, ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

    // 配置运动学参数: 单脉冲距离与轮间距
    kinematics.set_motor_param(DISTANCE_PER_TICK_MM);
    kinematics.set_wheel_distance(WHEEL_DISTANCE_MM);

    // 初始化两路电机
    motor.attachMotor(MOTOR_LEFT, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B);
    motor.attachMotor(MOTOR_RIGHT, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);

    // 配置速度环 PI 控制器 (外环, 无 D 项): 输出为期望角度增量 (deg), 限幅防目标角过大失衡
    speed_pid.update_pid(SPEED_KP, SPEED_KI, SPEED_KD);
    speed_pid.output_limit(SPEED_OUTPUT_LIMIT);

    // 配置直立环 PD 控制器: 库层强制纯 PD 无 I 项 (update_pwm_upright 忽略 ki_), 输出限幅对齐 MCPWM 占空比范围
    balance_pid.update_pid(UPRIGHT_KP, UPRIGHT_KI, UPRIGHT_KD);
    balance_pid.output_limit(UPRIGHT_PWM_LIMIT);

    // 与 test12 共用 TURN_KP/TURN_KI/TURN_KD 三元组 (KI 恒 0 占位), 无模式切换
    turn_pid.update_pid(TURN_KP, TURN_KI, TURN_KD);
    turn_pid.output_limit(TURN_PWM_LIMIT);

    // 创建控制任务: 5ms 固定节拍, 钉在 core1 避开 core0 的 WiFi 协议栈抖动
    xTaskCreatePinnedToCore(
        balance_task,
        "balance_task",
        BALANCE_STACK_SIZE,
        nullptr,
        BALANCE_TASK_PRIO,
        nullptr,
        BALANCE_TASK_CORE
    );

    // 创建 micro-ROS 任务: 不钉核, 低于控制任务优先级
    xTaskCreate(
        micro_ros_task,
        "micro_ros_task",
        MICRO_ROS_STACK_SIZE,
        nullptr,
        MICRO_ROS_TASK_PRIO,
        nullptr
    );

    // 创建雷达透传任务: 钉在 core0, 与 WiFi 协议栈同核, 不占用 core1 的控制节拍
    xTaskCreatePinnedToCore(
        bridge_task,
        "bridge_task",
        BRIDGE_STACK_SIZE,
        nullptr,
        BRIDGE_TASK_PRIO,
        nullptr,
        BRIDGE_TASK_CORE
    );
}

void loop() {
    delay(IDLE_LOOP_MS); // 控制与命令处理全部在 balance_task 中, 主循环空转
}

namespace {

// micro-ROS 订阅消息缓冲区 (仅 micro_ros_task 中的 executor 回调使用, 无跨任务竞争)
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Bool enable_msg;

/**
 * @brief /cmd_vel 订阅回调 (在 micro_ros_task 上下文执行)
 *
 * 把 linear.x / angular.z 原始值写入临界区保护的命令变量, balance_task 每周期读取快照,
 * 避免跨核读写撕裂 float。
 *
 * @param msg 收到的 Twist 消息指针
 */
void twist_callback(const void* msg) {
    const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(msg);
    portENTER_CRITICAL(&cmd_mux);
    cmd_linear_mps = static_cast<float>(twist->linear.x);
    cmd_angular_rps = static_cast<float>(twist->angular.z);
    portEXIT_CRITICAL(&cmd_mux);
}

/**
 * @brief /balance_enable 订阅回调 (在 micro_ros_task 上下文执行)
 *
 * data=true 请求武装, false 请求解除; 状态切换由 balance_task 统一打印。
 *
 * @param msg 收到的 Bool 消息指针
 */
void enable_callback(const void* msg) {
    const auto* en = static_cast<const std_msgs__msg__Bool*>(msg);
    portENTER_CRITICAL(&cmd_mux);
    cmd_enable = en->data;
    portEXIT_CRITICAL(&cmd_mux);
}

/**
 * @brief 里程计定时器回调函数 (在 micro_ros_task 上下文执行)
 *
 * 位姿与速度取自 balance_task 积分的里程计快照, 临界区拷贝避免读到多字段混合状态。
 *
 * @param timer 定时器指针
 * @param last_call_time 上一次调用时间
 */
void odom_callback(rcl_timer_t* timer, int64_t last_call_time) {
    portENTER_CRITICAL(&odom_mux);
    odom_t odom = kinematics.get_odom(); // 获取里程计
    portEXIT_CRITICAL(&odom_mux);
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
 * @brief 处理串口命令 (在 control_step 中逐周期调用)
 *
 * 字符集: 's' 武装/解除 (等价 /balance_enable 翻转), 'c' 标定机械中值 (仅 kIdle)。
 * 主命令通道为 /cmd_vel 话题, 串口仅调试辅助, 故与 test12 有两处刻意差异:
 *   - 每周期至多消费一个字符 (10-12 用 while 清空积压): 串口积压摊薄到多周期处理,
 *     避免单周期长时间读 UART 拉长 5ms 控制节拍;
 *   - 标定采样期间 return, 不响应新命令 (10-12 采样期间仍轮询): 保证 0.2s 采样窗口
 *     不被 's' 等命令打断, 采样数据纯净。
 *
 * @param theta 当前控制角 (deg, 后倾为正), 标定时逐周期累加求平均
 */
void handle_serial_command(float theta) {
    static uint16_t calib_remaining = 0; // 标定剩余采样次数, 0 表示未在采样
    static float calib_sum = 0.0f;       // 标定采样累加和

    // 标定采样进行中: 逐周期累加 theta (而非阻塞 delay), 保持 5ms 控制节拍
    if (calib_remaining > 0) {
        calib_sum += theta;
        --calib_remaining;
        if (calib_remaining == 0) {
            zero_pitch_deg = calib_sum / UPRIGHT_CALIB_CYCLES;
            Serial.printf("[CALIB] zero_pitch=%.2f deg\n", zero_pitch_deg);
        }
        return;
    }

    if (!Serial.available()) {
        return;
    }
    const char cmd = static_cast<char>(Serial.read());
    switch (cmd) {
    case 's': {
        // 翻转武装请求 (与 /balance_enable 等价): 经临界区写, balance_task 下周期快照生效
        portENTER_CRITICAL(&cmd_mux);
        cmd_enable = !cmd_enable;
        portEXIT_CRITICAL(&cmd_mux);
        break;
    }
    case 'c': {
        // 标定机械中值: 仅停止状态可标定, 启动后由 control_step 逐周期累加
        if (balance_state == BalanceState::kIdle) {
            calib_remaining = UPRIGHT_CALIB_CYCLES;
            calib_sum = 0.0f;
            Serial.println("[CALIB] sampling 0.2s, hold the car upright...");
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 单周期控制流程 (5ms 节拍, 由 balance_task 调用)
 *
 * 顺序: 姿态读取 -> 编码器测速 -> 命令快照/限幅 -> 串口命令 -> 状态机输出 -> 周期打印。
 * 坐标系与符号约定见文件头"方向约定", 与 test10/11/12 完全一致。
 */
void control_step() {
    // 1. 姿态读取: mpu.update() 必须先于 getAngle/getGyro, 否则数据不刷新
    mpu.update();
    const float theta = mpu.getAngleY();      // 控制角 (deg): 后倾为正
    const float omega_pitch = mpu.getGyroY(); // 俯仰角速度 (deg/s): 后倾为正
    const float omega_z = mpu.getGyroZ();     // 偏航角速度 (deg/s): 左转为正

    // 2. 编码器测速 + 运动学正解: 车体前进速度 v (mm/s), 用作速度环反馈
    //    临界区: 本处积分写 odom, odom_callback (core0) 拷贝读, 防多字段读到混合状态
    int32_t ticks[2] = {encoders[MOTOR_LEFT].getTicks(), encoders[MOTOR_RIGHT].getTicks()};
    portENTER_CRITICAL(&odom_mux);
    kinematics.update_motor_speed(millis(), ticks);
    portEXIT_CRITICAL(&odom_mux);
    float motor_speeds[2] = {
        kinematics.get_motor_speed(MOTOR_LEFT),
        kinematics.get_motor_speed(MOTOR_RIGHT)
    };
    float body_vel[2] = {0.0f, 0.0f};
    kinematics.kinematics_forward(motor_speeds, body_vel);
    const float speed_mm_s = body_vel[VEL_LINEAR];

    // 3. 读命令快照 (临界区) 并换算限幅: /cmd_vel 与 /balance_enable 均在此同步
    portENTER_CRITICAL(&cmd_mux);
    const float cmd_linear = cmd_linear_mps;
    const float cmd_angular = cmd_angular_rps;
    const bool enable_request = cmd_enable;
    portEXIT_CRITICAL(&cmd_mux);

    // 速度目标 (mm/s): 限幅兜底, teleop 默认 0.5 m/s 超出小车调节能力
    float target_speed_mm_s = cmd_linear * MPS_TO_MM_S;
    target_speed_mm_s =
        fabsf(target_speed_mm_s) <= CMD_MAX_LINEAR_MM_S
            ? target_speed_mm_s
            : (target_speed_mm_s > 0.0f ? CMD_MAX_LINEAR_MM_S : -CMD_MAX_LINEAR_MM_S);

    // 偏航角速度目标 (deg/s): 限幅兜底, teleop 默认 1.0 rad/s 偏大
    float omega_z_target = cmd_angular * RAD_S_TO_DEG_S;
    omega_z_target = fabsf(omega_z_target) <= CMD_MAX_ANGULAR_DEG_S
                         ? omega_z_target
                         : (omega_z_target > 0.0f ? CMD_MAX_ANGULAR_DEG_S : -CMD_MAX_ANGULAR_DEG_S);

    // 武装请求同步 (解除立即生效, 请求武装待 kIdle 起控判断); 状态变化打印一次
    if (balance_armed != enable_request) {
        balance_armed = enable_request;
        Serial.println(balance_armed ? "[CMD] arm requested" : "[CMD] stop requested");
    }

    // 4. 串口命令处理 (标定采样/武装翻转, 非阻塞)
    handle_serial_command(theta);

    // 5. 状态机: 输出关闭/直立控制
    int16_t pwm_balance = 0;
    int16_t pwm_delta = 0;
    int16_t pwm_left = 0;
    int16_t pwm_right = 0;
    switch (balance_state) {
    case BalanceState::kIdle:
        // 停止: 关闭输出, 等待武装且姿态进入中值窗口后起控
        motor.updateMotorSpeed(MOTOR_LEFT, 0);
        motor.updateMotorSpeed(MOTOR_RIGHT, 0);
        if (balance_armed && fabsf(theta - zero_pitch_deg) <= UPRIGHT_ARM_ANGLE_DEG) {
            balance_state = BalanceState::kRunning;
            speed_pid.reset(); // 起控清零积分, 防止残留误差导致起步冲击
            balance_pid.reset();
            turn_pid.reset();
            Serial.println("[CTRL] running");
        }
        break;
    case BalanceState::kRunning: {
        // 解除请求 (/balance_enable false / Agent 断开 / 串口 's') 或倒地: 立即退出直立控制
        if (!balance_armed || fabsf(theta - zero_pitch_deg) >= UPRIGHT_FALL_ANGLE_DEG) {
            balance_state = BalanceState::kIdle;
            balance_armed = false;
            portENTER_CRITICAL(&cmd_mux);
            cmd_enable = false; // 同步清除请求, 防止 ROS 侧 true 残留导致重新起控
            portEXIT_CRITICAL(&cmd_mux);
            motor.updateMotorSpeed(MOTOR_LEFT, 0);
            motor.updateMotorSpeed(MOTOR_RIGHT, 0);
            Serial.println("[CTRL] stopped: disarm or fall detected");
            break;
        }

        // 速度环 PI (外环): 目标 v_set (mm/s), 输出为期望角度增量 (deg), 限幅防目标角过大
        const int16_t speed_output = speed_pid.update_pwm_speed(target_speed_mm_s, speed_mm_s);

        // 直立环 (内环, PD): PWM = Kp*(target_angle - theta) - Kd*omega_pitch (库层强制纯 PD, 见 setup)
        // target_angle = 机械中值 - 速度环输出 (串级嵌套)
        const float target_angle = zero_pitch_deg - static_cast<float>(speed_output);
        const float inputs[2] = {theta, omega_pitch}; // [当前角度, 当前角速度]
        pwm_balance = balance_pid.update_pwm_upright(target_angle, inputs);

        // 转向环 (单一完整转向环, docs 7.2): 遥控角速度指令折算为目标转角指令
        // θ_cmd = -(Kd/Kp)·ωz_set (折算系数量纲 deg/(deg/s)=s, 见 docs 7.2 公式;
        // 无指令 (teleop 松键发全零, ωz_set=0) 时 θ_cmd=0, 转向环仅剩阻尼项走直线
        const float turn_cmd_deg = -(TURN_KD / TURN_KP) * omega_z_target;
        pwm_delta = turn_pid.update_pwm_turn(turn_cmd_deg, omega_z);

        // 差模合成 (int 域求和, 避免窄化隐式告警): pwm_L = base + Δ, pwm_R = base - Δ
        pwm_left = static_cast<int16_t>(static_cast<int>(pwm_balance) + pwm_delta);
        pwm_right = static_cast<int16_t>(static_cast<int>(pwm_balance) - pwm_delta);
        motor.updateMotorSpeed(MOTOR_LEFT, pwm_left);
        motor.updateMotorSpeed(MOTOR_RIGHT, pwm_right);
        break;
    }
    }

    // 6. 文本打印: 以 10Hz 低频输出一行状态 (纯文本, 无绘图依赖), 供串口监视器观察
    static uint32_t last_print_ms = 0; // 上次打印时刻 (函数内静态, 跨周期保留)
    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms >= BALANCE_PRINT_MS) {
        last_print_ms = now_ms;
        Serial.printf(
            "state=%s theta=%.2f omega=%.2f omega_z=%.2f speed=%.1f target=%.1f "
            "wz_cmd=%.1f delta=%d pwm_L=%d pwm_R=%d\n",
            balance_state == BalanceState::kIdle ? "IDLE" : "RUN",
            theta,
            omega_pitch,
            omega_z,
            speed_mm_s,
            target_speed_mm_s,
            omega_z_target,
            pwm_delta,
            pwm_left,
            pwm_right
        );
    }
}

/**
 * @brief 平衡控制任务 (core1, 5ms 固定节拍)
 *
 * 用 millis 绝对时刻补足节拍, 不沿用 test10/11/12 的 vTaskDelayUntil 骨架:
 * micro-ROS executor 与 WiFi 协议栈处理的临界区/中断可延迟 balance_task 的 tick 唤醒超过一个节拍 (控制周期超时)。
 * vTaskDelayUntil 超时后把唤醒锚点重写到理论时刻 (其实现为 *pxPreviousWakeTime = xTimeToWake), 会连续补跑 control_step 直至相位追平;
 * 无线链路抖动时追赶式补跑放大控制输出突变。
 *
 * 关键鲁棒行为在函数体 else 分支 (wait_ms <= 0 时):
 *     next_time_ms = millis() + BALANCE_PERIOD_MS;
 * 该句丢弃超时积压并重锚到当前时刻, 只损失一拍相位, 不产生追赶补跑。
 */
void balance_task(void* param) {
    (void)param;
    uint32_t next_time_ms = millis();
    for (;;) {
        control_step();
        next_time_ms += BALANCE_PERIOD_MS;
        const int32_t wait_ms = static_cast<int32_t>(next_time_ms - millis());
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        } else {
            next_time_ms = millis() + BALANCE_PERIOD_MS;
        }
    }
}

/**
 * @brief 连接上位机 TCP server (ros_serial2wifi, 端口 8889)
 */
bool connect_tcp() {
    if (tcp_client.connected()) {
        return true;
    }
    Serial.printf("[TCP] 连接 %s:%u ...\n", AGENT_IP_STR, BRIDGE_TCP_PORT);
    if (tcp_client.connect(AGENT_IP_STR, BRIDGE_TCP_PORT)) {
        Serial.println("[TCP] 已连接");
        return true;
    }
    Serial.println("[TCP] 连接失败");
    return false;
}

/**
 * @brief 雷达透传任务 (core0, UART1 <-> TCP)
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
        if (!connect_tcp()) {
            delay(BRIDGE_RECONNECT_MS);
            continue;
        }

        // 3. 双向透传: UART1 <-> TCP
        //    上位机 5s 无数据交换会断开连接, 需持续转发雷达数据;
        //    若断开, connect_tcp() 下一轮自动重连。
        //    批量读写: 逐字节 write 在高数据量下会溢出 UART FIFO 导致丢帧,
        //    这里每次最多读 512 字节一次性转发, 大幅减少 TCP 调用次数。
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
 * @brief micro-ROS 通信任务 (默认核, 优先级低于控制任务)
 *
 * 建立 WiFi + UDP 会话并运行 executor; spin 返回错误 (Agent 断开/重连) 时
 * 清空命令与武装请求, 让 balance_task 下一周期停止输出, 防止失控时小车携带指令奔跑。
 */
void micro_ros_task(void* param) {
    (void)param;
    // 任务起点打印: 置于首个 delay 之前, 上电后立即可见, 用于判别固件是否包含最新代码
    Serial.println("[ROS] micro_ros_task start");

    // 网络自举: STA 接入 / AP 自组网由 WIFI_ROLE_AP 决定, 并注册 UDP transport;
    // 连接前后的诊断打印由 wifi_role_boot 内部提供 (阻塞连接失败时串口可见)
    IPAddress agent_ip;
    wifi_role_boot(agent_ip);  // 按 WIFI_ROLE_AP 决定 STA 接入或 AP 自组网
    delay(TRANSPORT_SETUP_MS); // 等待传输层设置完成

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, nullptr, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, BALANCE_NODE_NAME, "", &support);

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator);

    // 两个 best-effort 订阅: 控制指令 /cmd_vel 与武装开关 /balance_enable
    rcl_subscription_t twist_sub;
    rcl_subscription_t enable_sub;
    rclc_subscription_init_best_effort(
        &twist_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    );
    rclc_subscription_init_best_effort(
        &enable_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        BALANCE_ENABLE_TOPIC
    );

    rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(
        &executor,
        &enable_sub,
        &enable_msg,
        &enable_callback,
        ON_NEW_DATA
    );

    // 里程计发布者: frame_id=odom, child_frame_id=base_footprint (与 test08 一致)
    pub_msg.header.frame_id = micro_ros_string_utilities_set(pub_msg.header.frame_id, "odom");
    pub_msg.child_frame_id =
        micro_ros_string_utilities_set(pub_msg.child_frame_id, "base_footprint");
    rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        ODOM_TOPIC
    );

    // 时间同步: 位姿时间戳取自 rmw_uros_epoch_millis(), 须先与 Agent 对齐 epoch
    while (!rmw_uros_epoch_synchronized()) {
        // 如果没有同步, 则尝试进行时间同步
        rmw_uros_sync_session(SYNC_ATTEMPT_MS);
        delay(SYNC_POLL_MS);
    }

    // 里程计定时器: 每 ODOM_PUBLISH_MS 执行一次, 此函数的第三参数单位要求是 ns
    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ODOM_PUBLISH_MS), odom_callback);
    rclc_executor_add_timer(&executor, &timer);

    for (;;) {
        const rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (rc != RCL_RET_OK) {
            // Agent 会话断开: 清命令与武装请求, balance_task 下周期读取后停止输出
            Serial.println("[ROS] agent session lost, disarm");
            portENTER_CRITICAL(&cmd_mux);
            cmd_linear_mps = 0.0f;
            cmd_angular_rps = 0.0f;
            cmd_enable = false;
            portEXIT_CRITICAL(&cmd_mux);
            delay(AGENT_RECONNECT_MS); // 等待重连窗口后继续 spin
        }
        delay(1); // 让出 CPU, 避免独占 core0
    }
}

} // namespace
