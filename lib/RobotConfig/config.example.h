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

// ---- 话题与节点 ----

constexpr char CMD_VEL_TOPIC[] = "/cmd_vel";           // 速度指令话题名
constexpr char NODE_NAME[] = "fishbot_motion_control"; // 节点名
constexpr char ODOM_TOPIC[] = "/odom";                 // 里程计话题名

#endif // ROBOTCONFIG_H
