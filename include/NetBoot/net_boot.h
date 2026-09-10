/**
 * @file net_boot.h
 * @brief 网络自举入口: 按 WIFI_ROLE_AP 在 STA 接入 / AP 自组网之间切换
 *
 * 设计目标: 纯增量, 默认 STA 路径与改造前完全一致 (set_microros_wifi_transports)。
 * AP 模式仅新增一个与官方 WiFi transport 同构的"纯 UDP"注册函数, 跳过 WiFi.begin(STA),
 * 复用 platformio_transport_* 回调 (open/write/read 与 AP/STA 角色无关)。
 *
 * 网络契约 (AP 模式):
 *   ESP32 SoftAP 自身: 192.168.4.1 (自带 DHCP server)
 *   上位机(agent 宿主): 静态 192.168.4.100 (须手动配置, 否则编译期无法确定 agent IP)
 */
#ifndef NET_BOOT_H
#define NET_BOOT_H

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include "config.h"

/**
 * @brief 注册 micro-ROS 纯 UDP transport (AP 模式用, 不发起 WiFi.begin)
 *
 * 与 micro_ros_platformio 的 set_microros_wifi_transports 同构, 差异仅在:
 * 后者先 WiFi.begin(ssid, pass) 阻塞等待 STA 连接; 本函数假设 AP 已由调用方开启,
 * 直接复用同一批 transport 回调注册到 rmw。
 *
 * @param agent_ip 运行 micro-ROS Agent 的主机 IP (须编译期已知)
 * @param agent_port Agent UDP 端口
 */
static inline void set_microros_wifi_ap_transports(IPAddress agent_ip, uint16_t agent_port) {
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void*)&locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}

/**
 * @brief 网络自举: 解析 agent IP 并按 WIFI_ROLE_AP 建立网络 + 注册 transport
 *
 * @param[out] agent_ip 由 AGENT_IP_STR 解析得到的 agent 地址
 */
static inline void wifi_role_boot(IPAddress& agent_ip) {
    agent_ip.fromString(AGENT_IP_STR);

#if WIFI_ROLE_AP == 1
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
    Serial.printf(
        "[WiFi] softAP \"%s\" ready, IP=%s, agent %s:%u\n",
        WIFI_AP_SSID,
        WiFi.softAPIP().toString().c_str(),
        AGENT_IP_STR,
        AGENT_PORT
    );
    set_microros_wifi_ap_transports(agent_ip, AGENT_PORT);
#else
    // 诊断打印: 官方函数内 WiFi.begin 阻塞等待关联, 失败(频段/凭据不符)时无输出无法定位,
    // 故连接前打印目标凭据, 连接后打印本地 IP (与 AP 分支的 softAP ready 打印对称)
    Serial
        .printf("[WiFi] connecting \"%s\", agent %s:%u...\n", WIFI_SSID, AGENT_IP_STR, AGENT_PORT);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT);
    Serial.printf("[WiFi] connected, local IP=%s\n", WiFi.localIP().toString().c_str());
#endif
}

/**
 * @brief 网络就绪判定 (供依赖网络的其他任务轮询, 如主固件的雷达透传任务)
 *
 * STA 模式判 WiFi 关联成功; AP 模式判 SoftAP 已启动。
 * 原因: AP 模式下 STA 接口未启用, WiFi.status() 恒不等于 WL_CONNECTED,
 * 直接依赖该判定的任务在 AP 模式下会永远等待。
 */
static inline bool wifi_network_ready() {
#if WIFI_ROLE_AP == 1
    return (WiFi.getMode() & WIFI_AP) != 0;
#else
    return WiFi.status() == WL_CONNECTED;
#endif
}

#endif // NET_BOOT_H
