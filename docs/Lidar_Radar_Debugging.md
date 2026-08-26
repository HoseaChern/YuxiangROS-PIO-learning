# 激光雷达接入调试记录（阶段一：test09_bridge 独立透传）

> **任务界限**：本文只记录**已完成**的调试内容——阶段一 `test09_bridge` 独立
> 透传验证的完整操作时序。阶段二（最终形态）的软件部分已并入 `src/main.cpp`
> 并通过编译，尚未整机实测；建图 / 导航**未开始**。

## 0. 任务流程界限

| 阶段   | 内容                                              | 状态   |
| ------ | ------------------------------------------------- | ------ |
| 阶段一 | test09_bridge 独立透传验证（本文第 1 节全部内容） | 已完成 |
| 阶段二 | 融合固件 main.cpp（软件完成）→ 建图 / 导航        | 进行中 |

## 1. 阶段一：test09_bridge 独立透传（已完成）

### 1.1 目标与验收

**目标**：不动运动控制，用独立透传固件 `test09_bridge` 打通雷达数据全链路：

```text
雷达 Tx(115200) -> ESP32 UART1 RX(GPIO14) -> WiFi TCP client
  -> 上位机 :8889 (ros_serial2wifi) -> /tmp/tty_laser -> ydlidar_ros2 -> /scan
```

**验收 4 项，全部满足才算阶段一通过**：

1. 下位机串口（115200）每 2s 打印 `[UART]` 计数持续增长，雷达在出数据；
2. 下位机打印 `[TCP] 已连接`，ESP32 已连上上位机 8889；
3. 上位机 `/tmp/tty_laser` 存在且持续有数据流入；
4. `ros2 topic hz /scan` 约 13.7 Hz。

### 1.2 硬件接线（第 0 步）

YDLidar X2L 接口为 **MX1.25-4P 母座**，线序从左到右 **M_CTR → GND → Tx → VCC**
（不是 PH2.0；Tx 单通道只出数据，X2L 无数据 RX 引脚）。

| 雷达引脚 | 电气特性          | 接到 ESP32-S3      | 说明                      |
| -------- | ----------------- | ------------------ | ------------------------- |
| VCC      | 5V（4.8~5.2V）    | 电源 5V            | 供电正极                  |
| GND      | 0V                | GND                | 供电地                    |
| Tx       | 115200 8N1 单通道 | GPIO14（UART1 RX） | 数据仅雷达到主控          |
| M_CTR    | 默认 1.8V，0~3.3V | GPIO13（LEDC PWM） | 电机调速端，电压/PWM 均可 |

M_CTR 是独立电机调速端、不是数据 RX，这是最易误判点。GPIO14/13 都在 N16R8
（Octal PSRAM）可用 GPIO 集合内，避开 strapping 引脚。

**供电负载提醒**：雷达启动时电机电流瞬态较大（工作电流可达数百 mA 级），
5V 电源**必须保证带载能力**（建议独立 5V ≥1A 输出，不要依赖开发板 USB 口 5V
直供）。供电不足的典型表现：`[MOTOR]` 有输出但电机不转 / 转一阵就停，或
`[UART]` 计数为 0 / 乱码（Tx 电平被拉低）。

### 1.3 调试过程（按启动顺序）

> 原则：先验证雷达本身，再逐跳验证链路，最后才看 `/scan`。
> 顺序一句话：**先烧录并观察下位机 → 再起上位机 tcp_server → 最后起雷达驱动**。

#### 1.3.1 第 1 步：烧录 test09，先观察雷达是否出数据

```bash
pio run -e test09_bridge -t upload
pio device monitor -e test09_bridge   # 115200
```

**先观察下位机**（这一步不依赖 WiFi/TCP，用于把"雷达/接线问题"与"链路问题"分开）：

| 日志                         | 含义                           | 判定                        |
| ---------------------------- | ------------------------------ | --------------------------- |
| `[MOTOR] ... duty=89`        | M_CTR PWM 已输出，雷达电机应转 | 电机不转查供电与 PWM 脚     |
| `[UART] 最近 2s 收到 N 字节` | N 持续增长 = 雷达数据进 UART1  | N 恒 0 查 5V 供电与 Tx 接线 |

**注意 `[MOTOR]` 只在上电时打印一次**（见 `main.cpp` setup），而 `pio device
monitor` 通常晚于上电才连上串口，日志可能已错过。若没看到 `[MOTOR]`，**按下
开发板 RST 键重启**，让串口监视器捕获完整的启动日志，再观察电机是否起转。

`[UART]` 计数打印在 WiFi 连接逻辑之前（见 `main.cpp` loop 第 0 步），即使 WiFi
未连、上位机未起也能观测，这是该固件的设计要点。

#### 1.3.2 第 2 步：下位机上电运行，上位机先启动 tcp_server

**下位机侧**（无需任何手动操作）：第 1 步烧录后板子即上电运行，固件自动连
WiFi 并作为 TCP client 周期性重连上位机 8889（失败每 `BRIDGE_RECONNECT_MS`
重试一次）。若中途断电/重启，固件上电后会自动重新发起连接，无需干预；唯一
前提是**板子保持上电、串口监视器保持打开**。

**上位机侧**：先启动 TCP server（等 ESP32 来连）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/ROS/YuXiangROS/Chap9/Robot_ws/install/setup.bash
ros2 run ros_serial2wifi tcp_server --serial_port /tmp/tty_laser --port 8889
```

**为什么上位机先启动**：ESP32 是 TCP client，client 连不存在的 server 只会反复
失败重试，server 必须先就位（固件每 `BRIDGE_RECONNECT_MS` 重试一次）。

**再观察（两端同时看）**：

| 观察位置                      | 预期现象       | 判定                               |
| ----------------------------- | -------------- | ---------------------------------- |
| 下位机串口                    | `[TCP] 已连接` | 失败则查 `AGENT_IP_STR`、端口 8889 |
| 上位机 tcp_server 日志        | client 接入    | 无接入查防火墙/网段                |
| 上位机 `ls -l /tmp/tty_laser` | 文件存在       | 存在即 pty 映射就绪                |

#### 1.3.3 第 3 步：最后启动雷达驱动，观察 /scan

```bash
ros2 launch ydlidar_ros2 ydlidar_a1.launch.py
```

**为什么最后启动**：ydlidar 打开 `/tmp/tty_laser` 前，该虚拟串口必须先由
tcp_server（第 2 步）创建。

**最后观察**：

| 命令                    | 预期          | 判定                             |
| ----------------------- | ------------- | -------------------------------- |
| `ros2 topic hz /scan`   | 约 13.7 Hz    | 明显偏低查 FIFO 溢出（见 1.4.2） |
| `ros2 topic echo /scan` | 角度/距离正常 | 全为 0 查链路未通                |
| `rviz2` 添加 LaserScan  | 轮廓正确      | 轮廓错查雷达朝向/安装            |

**QoS 关键点（最容易踩的坑）**：ydlidar_ros2 发布 `/scan` 用的是
`rclcpp::SensorDataQoS()`，即 **reliability=Best Effort + history=Keep Last 5**
（见 `ydlidar_ros2/src/ydlidar_node.cpp` 第 165 行）。而 ROS2 CLI 与 rviz2 的
默认订阅 QoS 是 **Reliable**，与发布端不匹配——现象是"命令无输出 / rviz 空白但
不报错"。因此：

- `ros2 topic echo /scan` 需显式指定 QoS：

  ```bash
  ros2 topic echo /scan --qos-reliability best_effort
  ```

- `ros2 topic hz /scan` 若不出数，同样加 `--qos-reliability best_effort`；
- **rviz2 详细设定**（按顺序操作）：

  1. 启动：`rviz2`；
  2. 左上角 Global Options → Fixed Frame 改为 `laser_frame`（与
     `ydlidar_ros2/params/ydlidar.yaml` 的 `frame_id` 一致）；
  3. Displays 面板 → Add → By topic → 选择 `/scan`（类型 LaserScan）→ OK；
  4. **在 Displays 面板展开刚添加的 LaserScan 项 → 展开 Topic → 把 QoS Policy
     从 "Default" 改为 "Sensor Data"**（即 Best Effort + Keep Last 5）。改完后
     rviz2 会重新订阅，轮廓即出现；不改则一直空白且无任何报错。

至此阶段一验收 4 项全部满足，`test09` 独立调试结束。

#### 1.3.4 等价一键入口（调试通过后可选用）

`robot_bringup/bringup.launch.py` 把 agent + tcp_server + ydlidar 串起，ydlidar
延时 5s 等 `/tmp/tty_laser` 就绪，与手动启动等价。注意：**阶段一不需要
micro_ros_agent（那是阶段二融合固件的 UDP 中转）**，一键入口把它一并拉起是为
阶段二（融合固件）准备的，阶段一多起无副作用。

### 1.4 本次调试产生的关键决策

#### 1.4.1 雷达电机由 ESP32 驱动，上位机必须关 SDK 的 DTR 控制

- 原书转接板固件自带 `motor_speed` 配置项输出 PWM 驱动雷达电机；
- 本方案若沿用 ydlidar 默认 `support_motor_dtr: true`，驱动 `startMotor()` 会调
  `setDTR()`（Linux 上即 `TIOCM_DTR` ioctl，操作串口物理 modem 控制线），但链路
  是 TCP/pty 透传（`ros_serial2wifi` 的 `tcpserver.py` 用 `pty.openpty()`，全程
  无 DTR/RTS 处理）——DTR 电平物理上到不了 ESP32 的 M_CTR，电机不会转；
- 因此电机改由 ESP32 LEDC PWM 驱动（GPIO13，10kHz，35% 占空比 89/255，对齐 X2L
  手册典型值），上位机 `ydlidar_ros2/params/ydlidar.yaml` 改为
  `support_motor_dtr: false`（已在鱼香ROS仓库本地 fishbot 分支提交）。

#### 1.4.2 UART RX 缓冲 4096

扫描数据流满载时 FIFO 溢出会丢帧，`setRxBufferSize(4096)` 且必须在
`Serial1.begin` 前调用。

#### 1.4.3 批量读写

每次最多 512 字节一次性转发；逐字节 write 在高数据量下会溢出 UART FIFO 丢帧。

#### 1.4.4 TCP 断线自动重连

上位机约 5s 无数据交换会断开连接；固件持续转发雷达数据并每 1s 重连
（`BRIDGE_RECONNECT_MS`）。

### 1.5 排错速查（阶段一实际踩点）

| 现象                        | 原因                             | 处理                                     |
| --------------------------- | -------------------------------- | ---------------------------------------- |
| `[MOTOR]` 未打印            | monitor 接入晚于上电，日志已错过 | 按 RST 键重启板子再观察                  |
| 有 `[MOTOR]` 但电机不转     | 5V 带载能力不足 / PWM 脚错       | 换 ≥1A 独立 5V 供电，核对 GPIO13         |
| `[UART]` 恒为 0             | 供电不足 / Tx 接错脚             | 核对线序 M_CTR→GND→Tx→VCC，确认 5V       |
| `[TCP]` 反复失败            | server 未起 / IP 或端口错        | 先起 tcp_server，核对 192.168.2.115:8889 |
| `/tmp/tty_laser` 缺失       | tcp_server 未起 / client 未连    | 确认监听 8889 且 ESP32 已连              |
| 雷达转但 `/scan` 空         | `support_motor_dtr` 仍为 true    | 改为 false（见 1.4.1）                   |
| `/scan` 频率低或丢帧        | UART FIFO 溢出                   | `setRxBufferSize(4096)` 且 begin 前调用  |
| `echo/hz` 无输出、rviz 空白 | 订阅 QoS 不匹配                  | 按 1.3.3 指定 best_effort 订阅           |

## 2. 阶段二：融合固件 + 建图 / 导航

阶段二是最终形态，分两部分：

**软件部分（已完成，代码已并入 `src/main.cpp`，编译通过）**：透传与运动控制
融合为三任务 `micro_ros_task` / `bridge_task` / `loop`，待整机实测。

**建图 / 导航（未开始）**：基于 `/scan` 进行建图与导航。

整机实测的联调顺序：先 `micro_ros_agent`（udp4:8888，运动控制 UDP 中转）→ 再
tcp_server → 最后 ydlidar，详见 README「激光雷达转接」节。
