/**
 * @file config.example.h
 * @brief 机器人运行配置模板: 硬件接线、部署环境与通用参数
 *
 * 使用方式: 复制本文件为 config.h, 并按实际硬件/部署修改。
 * config.h 已被 .gitignore 忽略, 不入版本库, 因此调整引脚/IP 不会污染 git 工作区。
 * 本文件与 config.h 使用同一 include guard, 同一翻译单元内只应包含其一。
 *
 * 收纳范围 (src/main.cpp 与 src/tests/ 各测试固件共用):
 *   - 串口波特率与控制周期
 *   - 电机/编码器引脚
 *   - PID 与运动学标定参数
 *   - 默认目标速度与单位换算
 *   - WiFi 凭据与 micro-ROS Agent 网络配置 (SSID/密码/IP/端口)
 *   - micro-ROS 任务/发布/时间同步参数
 *   - 话题名与节点名
 *
 * 不收纳: 按各固件 micro-ROS 订阅/定时器数量取值的常量
 * (EXECUTOR_HANDLES 在 main=2 / test06=0 / test07=1), 保留在各固件本地定义。
 */
#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <cstdint> // uint8_t / uint16_t / uint32_t / int16_t

// ---- 串口参数 ----

constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率

// ---- 电机引脚 (电机0: 5/4, 电机1: 6/7) ----

constexpr uint8_t MOTOR_LEFT_PIN_A = 5;
constexpr uint8_t MOTOR_LEFT_PIN_B = 4;
constexpr uint8_t MOTOR_RIGHT_PIN_A = 6;
constexpr uint8_t MOTOR_RIGHT_PIN_B = 7;

// ---- 编码器引脚 (编码器0: 16/15, 编码器1: 17/18) ----

constexpr uint8_t ENC_LEFT_PIN_A = 16;
constexpr uint8_t ENC_LEFT_PIN_B = 15;
constexpr uint8_t ENC_RIGHT_PIN_A = 17;
constexpr uint8_t ENC_RIGHT_PIN_B = 18;

// ---- PID 参数 ----

constexpr float PID_KP = 0.625f;           // 比例增益
constexpr float PID_KI = 0.125f;           // 积分增益
constexpr float PID_KD = 0.0f;             // 微分增益
constexpr float PID_OUTPUT_LIMIT = 100.0f; // 输出限幅 ±100

// ---- 运动学参数 ----

constexpr float WHEEL_DISTANCE_MM = 175.0f;        // 轮间距, 单位 mm
constexpr float DISTANCE_PER_TICK_MM = 0.1427138f; // 单个脉冲对应的轮子前进距离, 单位 mm

// ---- 默认目标速度 (无外部指令时的标定目标) ----

constexpr float TARGET_LINEAR_SPEED_MM_S = 50.0f;  // 目标线速度, 单位 mm/s
constexpr float TARGET_ANGULAR_SPEED_RAD_S = 0.1f; // 目标角速度, 单位 rad/s

// ---- 测试专用参数 ----

constexpr uint32_t STEP_DELAY_MS = 2000;    // test01: 每个方向保持时间, 单位 ms
constexpr int16_t MOTOR_SPEED = 70;         // test01/03: 测试转速, 范围 [-100, 100]
constexpr float TARGET_SPEED_MM_S = 100.0f; // test04: 目标轮速, 单位 mm/s

// ---- 单位换算 ----

constexpr float MPS_TO_MMPS = 1000.0f; // m/s -> mm/s
constexpr float MS_TO_S = 1000.0f;     // mm/ms -> mm/s
constexpr double S_TO_NS = 1e6;        // 秒 -> 纳秒 (时间戳换算)

// ---- 控制周期 ----

constexpr uint32_t LOOP_DELAY_MS = 10; // 主循环调度节拍, 单位 ms

// ---- 网络配置 ----

constexpr char AGENT_IP_STR[] = "192.168.2.115"; // 主机 IP(运行 micro-ROS Agent 的电脑)
constexpr uint16_t AGENT_PORT = 8888;            // Agent UDP 端口

// ---- WiFi 凭据 ----
// 注意: 凭据须为可写 char 数组, 因 set_microros_wifi_transports 接口要求 char*, 不能 constexpr

char WIFI_SSID[] = "YOUR_WIFI_SSID";
char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";

// ---- 任务/发布/时间同步参数 ----

constexpr uint32_t MICRO_ROS_STACK_SIZE = 10240; // micro-ROS 任务栈字节数
constexpr uint8_t MICRO_ROS_TASK_PRIO = 1;       // 任务优先级
constexpr uint32_t TRANSPORT_SETUP_MS = 2000;    // 传输层设置等待时间, 单位 ms
constexpr uint32_t ODOM_PUBLISH_MS = 50;         // 里程计发布周期, 单位 ms
constexpr uint32_t SYNC_ATTEMPT_MS = 1000;       // 时间同步单次尝试时长, 单位 ms
constexpr uint32_t SYNC_POLL_MS = 10;            // 时间同步轮询间隔, 单位 ms

// ---- 雷达透传任务参数 (主环境融合固件: bridge_task) ----

constexpr uint32_t BRIDGE_STACK_SIZE = 8192; // bridge_task 任务栈字节数
constexpr uint8_t BRIDGE_TASK_PRIO = 1;      // bridge_task 任务优先级

// ---- 话题与节点 ----

constexpr char CMD_VEL_TOPIC[] = "/cmd_vel";           // 速度指令话题名
constexpr char NODE_NAME[] = "fishbot_motion_control"; // 节点名
constexpr char ODOM_TOPIC[] = "/odom";                 // 里程计话题名

// ---- 激光雷达转接 (test09_bridge 透传固件) ----
// X2L 接口 (MX1.25-4P, 线序从左到右 M_CTR->GND->Tx->VCC):
//   VCC(5V) -> 电源 5V; GND -> GND; Tx -> 本固件 UART RX; M_CTR -> PWM 调速
// 注意: X2L 无数据 RX 引脚, 数据仅从 Tx 出 (单通道)。

constexpr uint8_t LIDAR_UART_RX_PIN = 14; // 雷达 Tx -> ESP32 UART1 RX (普通 GPIO, 避开 strapping)
constexpr uint8_t LIDAR_MOTOR_CTRL_PIN = 13; // 雷达 M_CTR 电机调速 (LEDC PWM 输出)
constexpr uint32_t LIDAR_BAUD = 115200;      // 雷达串口波特率 (与上位机 ydlidar.yaml 一致)
constexpr uint32_t LIDAR_PWM_FREQ = 10000;   // M_CTR PWM 频率, 单位 Hz (X2L 手册规格典型值 10kHz)
constexpr uint8_t LIDAR_PWM_RES = 8;         // M_CTR PWM 分辨率, 单位 bit
constexpr uint8_t LIDAR_PWM_CHANNEL = 0;     // M_CTR LEDC 通道号 (test09 独占, 取 0)
constexpr uint32_t LIDAR_MOTOR_SPEED =
    89; // M_CTR 初始占空比 35% (89/255, 对齐 X2L 手册 PWM 典型值)

constexpr uint16_t BRIDGE_TCP_PORT = 8889;     // 上位机 ros_serial2wifi tcp_server 端口
constexpr uint32_t BRIDGE_RECONNECT_MS = 1000; // TCP 断线重连间隔, 单位 ms

// ---- 两轮自平衡 (test10_balance 直立环固件) ----
// I2C 引脚从 N16R8 空闲集合 {3,46,9,10,11,12} 中选取:
//   避开 strapping 引脚 3/46, 取 9/10 (无其他复用冲突); MPU6050 模块板载上拉电阻

constexpr uint8_t IMU_SDA_PIN = 10;           // MPU6050 SDA
constexpr uint8_t IMU_SCL_PIN = 9;            // MPU6050 SCL

constexpr uint32_t BALANCE_PERIOD_MS = 5;     // 控制节拍, 单位 ms (200Hz, 三环统一节拍)
constexpr uint32_t BALANCE_STACK_SIZE = 4096; // balance_task 任务栈字节数
constexpr uint8_t BALANCE_TASK_PRIO = 5;      // 任务优先级 (硬实时控制, 高于网络类任务)
constexpr uint8_t BALANCE_TASK_CORE = 1;      // 任务核心号 (避开 core0 的 WiFi 协议栈抖动)

constexpr float BALANCE_KP = 25.0f;          // 直立环比例增益, 单位 PWM/deg (经验起点, 待实测整定)
constexpr float BALANCE_KI = 0.0f;           // 直立环积分增益 (直立环不用 I, 相位滞后致振荡)
constexpr float BALANCE_KD = 0.5f;           // 直立环微分增益, 单位 PWM/(deg/s), D 项用陀螺仪角速度
constexpr float BALANCE_PWM_LIMIT = 255.0f;  // 直立环输出限幅, 对齐 MCPWM 占空比范围
constexpr float BALANCE_ZERO_PITCH_DEG = 0.0f; // 机械中值角, 实测车身静止站立的平均 pitch 后修正

constexpr float BALANCE_ARM_ANGLE_DEG = 8.0f;  // 起控阈值: |pitch| 小于该值才使能输出
constexpr float BALANCE_FALL_ANGLE_DEG = 45.0f; // 倒地保护阈值: |pitch| 大于该值立即停机
constexpr uint32_t BALANCE_CALM_DELAY_MS = 2000; // 校准前静置时长, 单位 ms
constexpr uint32_t BALANCE_PRINT_MS = 100;       // 调试打印周期, 单位 ms

#endif // ROBOTCONFIG_H
