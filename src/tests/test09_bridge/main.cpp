/**
 * @file main.cpp
 * @brief test09: 激光雷达 UART->WiFi TCP 透传固件 (ESP32-S3 兼任转接板)
 *
 * 功能:
 *   1. WiFi 连接
 *   2. TCP client 主动连接上位机 ros_serial2wifi tcp_server (端口 8889)
 *   3. UART1 读取雷达 Tx 数据 (115200, 单通道), 转发到 TCP
 *   4. TCP 下行数据转发回 UART (保留双向透传契约; X2L 无数据 RX, 雷达侧忽略)
 *   5. M_CTR 输出 PWM 驱动雷达电机 (DTR 无法穿透 TCP/pty, 电机须由本端驱动)
 *
 * 上位机链路: ros_serial2wifi(tcp_server:8889) -> /tmp/tty_laser -> ydlidar_ros2
 * 接口契约:  TCP client 主动连上位机 IP:8889 双向透传 UART, 上位机零改动。
 *
 * 接线 (X2L, MX1.25-4P 线序 M_CTR->GND->Tx->VCC):
 *   VCC -> 5V;  GND -> GND;  Tx -> GPIO14 (UART1 RX);  M_CTR -> GPIO13 (PWM)
 *
 * 编译/烧录: pio run -e test09_bridge -t upload
 */

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"

namespace {

WiFiClient tcp_client; // TCP client (连上位机 ros_serial2wifi)

/**
 * @brief 连接 WiFi, 失败则阻塞重试
 */
bool connect_wifi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }
    Serial.print("[WiFi] 连接中: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (millis() - start > 15000) {
            Serial.println("[WiFi] 连接超时, 重试...");
            WiFi.disconnect();
            return false;
        }
    }
    Serial.print("[WiFi] 已连接, IP: ");
    Serial.println(WiFi.localIP());
    return true;
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

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println("\n=== test09_bridge: UART->WiFi 透传 ===");

    // 雷达 UART1: 只配 RX (X2L 无数据 RX 引脚), 115200 8N1
    // 增大 RX 缓冲: 扫描数据流满载时防 FIFO 溢出丢帧 (需在 begin 前调用)
    Serial1.setRxBufferSize(4096);
    Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_UART_RX_PIN, -1);

    // M_CTR 电机 PWM (LEDC): 上电即驱动雷达电机旋转
    ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RES);
    ledcAttachPin(LIDAR_MOTOR_CTRL_PIN, LIDAR_PWM_CHANNEL);
    ledcWrite(LIDAR_PWM_CHANNEL, LIDAR_MOTOR_SPEED);
    Serial.printf("[MOTOR] M_CTR PWM: pin=%u chan=%u freq=%uHz duty=%u\n",
                  LIDAR_MOTOR_CTRL_PIN, LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_MOTOR_SPEED);
}

void loop() {
    // 0. 诊断: 周期性打印 UART1 累计接收字节数 (排查用, 独立于 WiFi/TCP 状态)
    static uint32_t uart_rx_total = 0;
    static uint32_t last_diag = 0;
    if (millis() - last_diag >= 2000) {
        Serial.printf("[UART] 最近 2s 收到 %u 字节\n", uart_rx_total);
        uart_rx_total = 0;
        last_diag = millis();
    }

    // 1. 确保 WiFi 已连接
    if (!connect_wifi()) {
        delay(BRIDGE_RECONNECT_MS);
        return;
    }

    // 2. 确保 TCP 已连接
    if (!connect_tcp()) {
        delay(BRIDGE_RECONNECT_MS);
        return;
    }

    // 3. 双向透传: UART1 <-> TCP
    //    上位机 5s 无数据交换会断开连接, 需持续转发雷达数据;
    //    若断开, connect_tcp() 下一轮自动重连。
    //    批量读写: 逐字节 write 在高数据量下会溢出 UART FIFO 导致丢帧,
    //    这里每次最多读 512 字节一次性转发, 大幅减少 TCP 调用次数。
    static uint8_t buf[512];
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
}
