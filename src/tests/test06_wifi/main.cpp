/**
 * @file main.cpp
 * @brief micro-ROS 无线网络连接最小测试程序（独立固件）
 *
 * @details
 * 本程序用于验证 ESP32-S3 开发板能否通过 WiFi 成功连接主机 ROS2 Jazzy 上
 * 运行的 micro-ROS Agent，是"网络连接测试"任务的独立最小测试固件，
 * 不修改 src/main.cpp 中的运动控制主程序。
 *
 * @section 验收标准
 * 主机执行如下命令启动 Agent 后，开发板应能成功连接并周期性发布心跳：
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 * 主机端可用 ros2 topic echo /test_heartbeat 验证双向通信。
 *
 * @section 与参考书写法（src/main.cpp 的 micro_ros_task）的差异
 * 1. WiFi 连接带超时检测，失败不会死等
 *    （库的 set_microros_wifi_transports 内部是 while 死等，这里先手动连接）
 * 2. 用 rmw_uros_ping_agent 在 rmw 初始化前探测 Agent 是否在线
 * 3. 时间同步带超时，失败不会阻塞卡死
 * 4. 每个阶段都有状态打印，便于定位是哪一环出问题
 *
 * @section 代码组织（符合 C++ 规范）
 * - 全局状态集中在匿名命名空间（内部链接），编译期常量一律 constexpr
 * - 采用"前向声明 + setup/loop 入口在前 + 函数定义在后"的组织方式
 *
 * @section 编译烧录
 *   pio run -e esp32-s3-devkitc-1-wifi-test -t upload
 */

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h> // 心跳消息类型: std_msgs/Int32

// ============================================================================
// 全局状态：匿名命名空间限定为本文件（内部链接），符合 C++ 规范
// 规则: 编译期常量一律 constexpr; 可变全局集中于此并注明被谁跨函数共享
// ============================================================================
namespace {

// ---- 网络配置（与 src/main.cpp 保持一致，请按你的实际网络修改）----
// 注意: 凭据必须是可写 char 数组, 因库接口要求 char*, 故不能 constexpr
char WIFI_SSID[] = "Xiaomi_5844";     // 热点 SSID
char WIFI_PASS[] = "Zhong.2wsxdr5";   // 热点密码
static const IPAddress AGENT_IP(192, 168, 2, 120); // 主机 IP(运行 Agent 的电脑)
constexpr uint16_t AGENT_PORT = 8888; // Agent UDP 端口

// ---- 各阶段超时参数(单位 ms)：控制每步最多等多久, 保证不会卡死 ----
constexpr uint32_t WIFI_TIMEOUT_MS = 20000; // WiFi 连接最大等待时间
constexpr int PING_TIMEOUT_MS = 200;        // ping Agent 单次超时
constexpr uint8_t PING_ATTEMPTS = 5;        // ping Agent 尝试次数
constexpr uint32_t SYNC_TIMEOUT_MS = 5000;  // 时间同步最大等待时间

// ---- 系统级参数 ----
constexpr uint32_t SERIAL_BAUD = 115200;    // 串口波特率
constexpr uint32_t SERIAL_WAIT_MS = 1000;   // 等待串口就绪, 避免开头日志丢失
constexpr uint32_t WIFI_POLL_MS = 500;      // WiFi 连接状态轮询间隔
constexpr uint32_t SYNC_ATTEMPT_MS = 100;   // 时间同步单次尝试时长
constexpr uint32_t SYNC_POLL_MS = 10;       // 时间同步轮询间隔
constexpr uint32_t HEARTBEAT_MS = 500;      // 心跳发布周期
constexpr uint32_t SPIN_TIMEOUT_MS = 100;   // 执行器单次 spin 超时
constexpr uint32_t RESTART_DELAY_MS = 5000; // 失败后重启等待时间

// ---- 话题与节点名称 ----
constexpr char HEARTBEAT_TOPIC[] = "/test_heartbeat";
constexpr char NODE_NAME[] = "wifi_test";

// ---- micro-ROS 对象：全局而非任务栈, 避免大对象占用任务栈空间 ----
// 由 micro_ros_init 初始化, 由 heartbeat_callback / loop 持续使用
rcl_allocator_t allocator;          // 内存分配器
rclc_support_t support;             // 存储时钟/内存分配器/上下文
rcl_node_t node;                    // ROS 节点
rclc_executor_t executor;           // 执行器(驱动定时器回调)
rcl_publisher_t heartbeat_pub;      // 心跳发布者
std_msgs__msg__Int32 heartbeat_msg; // 心跳消息(自增计数)
rcl_timer_t heartbeat_timer;        // 心跳定时器

bool test_passed = false; // 测试是否通过, 控制 loop 行为 (setup 写入, loop 读取)

// ---- 函数前向声明（本文件内部链接）----
bool wifi_connect_with_timeout();
bool ping_agent_with_timeout();
void sync_time_with_timeout();
void heartbeat_callback(rcl_timer_t* timer, int64_t last_call_time);
bool micro_ros_init();

} // namespace

// ============================================================================
// Arduino 入口（外部链接, 框架要求）
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(SERIAL_WAIT_MS); // 等待串口就绪, 避免开头日志丢失
    Serial.printf("\n========== micro-ROS 网络连接测试开始 ==========\n");

    // 阶段 1: 先手动连接 WiFi(带超时, 不卡死)
    if (!wifi_connect_with_timeout()) {
        test_passed = false;
        return; // WiFi 都连不上, 无需继续
    }

    // 阶段 2~4: 配置传输、探测 Agent、初始化 micro-ROS
    test_passed = micro_ros_init();

    if (test_passed) {
        Serial.printf("========== 测试通过, 开始发布心跳 ==========\n");
    } else {
        Serial.printf("========== 测试失败, 将在 loop 中重启重试 ==========\n");
    }
}

void loop() {
    if (test_passed) {
        // 测试通过: 由定时器驱动心跳发布; spin_some 非阻塞
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
    } else {
        // 测试失败: 周期性提示后软重启, 便于主机 Agent 就绪后自动重试
        Serial.printf("[Test] 连接失败, %u 秒后重启重试...\n", RESTART_DELAY_MS / 1000);
        delay(RESTART_DELAY_MS);
        ESP.restart();
    }
}

// ============================================================================
// 函数实现
// ============================================================================
namespace {

// ============================================================================
// 阶段 1: 带超时的 WiFi 连接
// ============================================================================
/**
 * @brief 手动连接 WiFi, 带超时检测, 避免库内部 while 死等导致卡死
 * @return true 连接成功, false 失败
 * @note 连接成功后, 再调用 set_microros_wifi_transports 时其内部的
 *       while(WiFi.status() != WL_CONNECTED) 会立即通过, 不会阻塞
 */
bool wifi_connect_with_timeout() {
    Serial.printf("[WiFi] 开始连接热点: %s\n", WIFI_SSID);

    WiFi.mode(WIFI_STA); // 设为 Station 模式(连接路由器/热点)
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) {
            // 超时: 打印状态码并失败返回
            Serial.printf(
                "[WiFi] 连接超时(%u ms), 状态码=%d\n"
                "       请检查: 1.热点名称/密码是否正确 2.热点是否开启 "
                "3.开发板与主机是否在同一网络\n",
                WIFI_TIMEOUT_MS,
                WiFi.status()
            );
            return false;
        }
        // 每 WIFI_POLL_MS 打印一次状态, 便于观察进度
        Serial.printf("[WiFi] 等待连接中... 状态码=%d\n", WiFi.status());
        delay(WIFI_POLL_MS);
    }

    Serial.printf("[WiFi] 连接成功! 本机 IP=%s\n", WiFi.localIP().toString().c_str());
    return true;
}

// ============================================================================
// 阶段 2: 探测 micro-ROS Agent 是否在线
// ============================================================================
/**
 * @brief 在 rmw 初始化之前探测 Agent 是否在线
 * @return true Agent 可达, false 不可达
 * @note rmw_uros_ping_agent 不依赖 rcl 上下文, 是"先探测后初始化"的标准做法
 */
bool ping_agent_with_timeout() {
    Serial.printf(
        "[Agent] 探测 Agent(%s:%u), 单次超时 %d ms, 共 %d 次...\n",
        AGENT_IP.toString().c_str(),
        AGENT_PORT,
        PING_TIMEOUT_MS,
        PING_ATTEMPTS
    );

    rmw_ret_t ret = rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS);
    if (ret == RMW_RET_OK) {
        Serial.printf("[Agent] 探测成功, Agent 在线!\n");
        return true;
    }

    Serial.printf(
        "[Agent] 探测失败(错误码=%d)\n"
        "       请确认主机已执行: ros2 run micro_ros_agent micro_ros_agent udp4 --port %u\n"
        "       且开发板与主机在同一局域网、防火墙未拦截 UDP %u 端口\n",
        (int)ret,
        AGENT_PORT,
        AGENT_PORT
    );
    return false;
}

// ============================================================================
// 阶段 3: 带超时的时间同步(失败不卡死)
// ============================================================================
/**
 * @brief 循环尝试时间同步直到成功或超时
 * @note 参考书里是 while(!rmw_uros_epoch_synchronized()) 无限循环,
 *       一旦 Agent 异常会永久卡死; 这里加上限避免阻塞
 */
void sync_time_with_timeout() {
    uint32_t start = millis();
    while (!rmw_uros_epoch_synchronized()) {
        if (millis() - start > SYNC_TIMEOUT_MS) {
            Serial.printf(
                "[TimeSync] 时间同步超时(%u ms), 继续运行(仅影响消息时间戳精度)\n",
                SYNC_TIMEOUT_MS
            );
            return; // 超时: 不阻塞, 继续执行后续流程
        }
        rmw_uros_sync_session(SYNC_ATTEMPT_MS); // 每次尝试时长
        delay(SYNC_POLL_MS);
    }
    Serial.printf("[TimeSync] 时间同步完成\n");
}

// ============================================================================
// 心跳定时器回调: 每触发一次计数 +1 并发布
// ============================================================================
/**
 * @brief 定时器回调, 用于证明"开发板 -> 主机"方向数据可达
 * @param timer          触发本次回调的定时器
 * @param last_call_time 上一次回调触发时间
 */
void heartbeat_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)timer;          // 本回调不需要定时器句柄, 显式忽略避免编译警告
    (void)last_call_time; // 同上

    heartbeat_msg.data++; // 心跳计数自增

    if (rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL) != RCL_RET_OK) {
        Serial.printf("[Heartbeat] 发布失败!\n");
    } else {
        Serial.printf("[Heartbeat] 已发布 count=%d\n", heartbeat_msg.data);
    }
}

// ============================================================================
// micro-ROS 初始化主流程(阶段 2~4)
// ============================================================================
/**
 * @brief 配置传输、初始化 rclc、创建节点/发布者/定时器并做时间同步
 * @return true 初始化成功(测试通过), false 失败(测试失败)
 */
bool micro_ros_init() {
    // ---- 阶段 2: 配置 WiFi UDP 传输层 ----
    // 此时 WiFi 已由 wifi_connect_with_timeout 连好, 该函数内部的
    // while 死等会立即通过, 不会阻塞
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);

    // ---- 阶段 2.5: 探测 Agent 是否在线(rmw 初始化前) ----
    if (!ping_agent_with_timeout()) {
        return false; // Agent 不可达, 测试失败
    }

    // ---- 阶段 3: 初始化内存分配器与 support ----
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        Serial.printf("[Init] rclc_support_init 失败\n");
        return false;
    }

    // ---- 阶段 4: 创建 ROS 节点 ----
    if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
        Serial.printf("[Init] rclc_node_init_default 失败\n");
        return false;
    }
    Serial.printf("[Init] 节点 %s 创建成功\n", NODE_NAME);

    // ---- 阶段 5: 初始化执行器(1 个句柄: 心跳定时器) ----
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        Serial.printf("[Init] rclc_executor_init 失败\n");
        return false;
    }

    // ---- 阶段 6: 初始化心跳发布者(best_effort, 与主程序风格一致) ----
    // 参数依次为: 发布者指针, 节点指针, 消息类型支持, 话题名称
    if (rclc_publisher_init_best_effort(
            &heartbeat_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            HEARTBEAT_TOPIC
        ) != RCL_RET_OK) {
        Serial.printf("[Init] 心跳发布者初始化失败\n");
        return false;
    }
    Serial.printf("[Init] 心跳话题 %s 创建成功\n", HEARTBEAT_TOPIC);

    // 初始化消息对象并清零计数
    std_msgs__msg__Int32__init(&heartbeat_msg);
    heartbeat_msg.data = 0;

    // ---- 阶段 7: 初始化心跳定时器(每 500 ms 触发)并加入执行器 ----
    // 第三参数单位为 ns: RCL_MS_TO_NS 负责 ms -> ns 换算
    if (rclc_timer_init_default(
            &heartbeat_timer,
            &support,
            RCL_MS_TO_NS(HEARTBEAT_MS),
            heartbeat_callback
        ) != RCL_RET_OK) {
        Serial.printf("[Init] 心跳定时器初始化失败\n");
        return false;
    }
    rclc_executor_add_timer(&executor, &heartbeat_timer);

    // ---- 阶段 8: 时间同步(带超时, 失败不卡死) ----
    sync_time_with_timeout();

    return true;
}

} // namespace
