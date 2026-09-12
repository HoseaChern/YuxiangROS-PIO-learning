# 激光雷达接入调试记录

## 1. 阶段一：test09_bridge 独立透传

### 1.1 目标与验收

**目标** ：不动运动控制，用独立透传固件 `test09_bridge` 打通雷达数据全链路：

```text
雷达 Tx(115200) -> ESP32 UART1 RX(GPIO14) -> WiFi TCP client
  -> 上位机 :8889 (ros_serial2wifi) -> /tmp/tty_laser -> ydlidar -> /scan
```

**验收 4 项，全部满足才算阶段一通过** ：

| 编号 | 验收项       | 判据                                                | 观测位置      |
| ---- | ------------ | --------------------------------------------------- | ------------- |
| 1    | 雷达出数据   | `[UART]` 计数稳定在数千字节量级（判据含义见 1.4.2） | 下位机串口    |
| 2    | 双向链路建立 | `[TCP] 已连接` 与上位机 client 接入日志同时出现     | 两端串口/终端 |
| 3    | 虚拟串口就绪 | `/tmp/tty_laser -> /dev/pts/N`（路径来源见 1.3.2）  | 上位机终端    |
| 4    | 话题出数     | `ros2 topic hz /scan` 持续出数，约 13.7 Hz          | 上位机终端    |

第 1、4 项的量级依据：115200 8N1 每字节占 10 位，故净荷上限为
$115200/10 = 11520\ \text{B/s}$；实测雷达数据率 $13776\ \text{B}/2\ \text{s} = 6888\ \text{B/s}$，
占上限约 60%。净荷与 UART 上限同量级，故接收缓冲溢出是真实风险，RX 缓冲取 4096 字节（1.4.4）。

第 1 项与第 3 项都出现「字节」，但观测点不同：前者是 ESP32 侧 UART1 的累计接收量，后者是
上位机 pty 侧可被读出的字节，两者判据不可互推，区别见 1.4.2。

### 1.2 硬件接线（第 0 步）

YDLidar X2L 接口为 **MX1.25-4P 母座** ，线序从左到右 **M_CTR → GND → Tx → VCC**
（不是 PH2.0；Tx 为单通道输出，X2L 无数据 RX 引脚）。

| 雷达引脚 | 电气特性          | 接到 ESP32-S3      | 说明                      |
| -------- | ----------------- | ------------------ | ------------------------- |
| VCC      | 5V（4.8~5.2V）    | 电源 5V            | 供电正极                  |
| GND      | 0V                | GND                | 供电地                    |
| Tx       | 115200 8N1 单通道 | GPIO14（UART1 RX） | 数据仅雷达到主控          |
| M_CTR    | 默认 1.8V，0~3.3V | GPIO13（LEDC PWM） | 电机调速端，电压/PWM 均可 |

M_CTR 是独立电机调速端、不是数据 RX，这是最易误判点；GPIO14/13 都在 N16R8（Octal PSRAM）
可用 GPIO 集合内，避开 strapping 引脚。

**供电** ：雷达电机工作电流可达数百 mA 且启动瞬态较大，5V 必须独立输出 ≥1 A，不要用开发板
USB 口 5V 直供。供电不足表现为 `[MOTOR]` 有输出而电机不转或转一阵即停，`[UART]` 计数为 0
或乱码（Tx 电平被拉低）。

### 1.3 调试过程（按启动顺序）

> 原则：先验证雷达本身，再逐跳验证链路，最后才看 `/scan`。
> 顺序一句话：**先烧录并观察下位机 → 再起上位机 tcp_server → 最后起雷达驱动**。

**网络前置** ：ESP32 与上位机须处于同一子网，且固件 `AGENT_IP_STR`（`config.h:84`，STA 分支
默认 `10.42.0.1`）等于上位机在该子网中的 IP；建网与 `nmcli` 配置见
`docs/Network_Setup_Notes.md` 第 3 节。网络只影响第 2 步及之后，1.3.1 的两项判据只看板子本身。

**终端分工** （后面按此编号引用，每个常驻进程独占一个终端）：

| 终端   | 位置            | 工作目录                                    | 承担步骤             |
| ------ | --------------- | ------------------------------------------- | -------------------- |
| 下位机 | 本仓库根目录    | `fishbot_motion_control`                    | 1.3.1 烧录与串口监视 |
| 终端 1 | 上位机 Robot_ws | `~/Documents/ROS/YuXiangROS/Chap9/Robot_ws` | 1.3.2 tcp_server     |
| 终端 2 | 上位机 Robot_ws | 同上                                        | 1.3.3 ydlidar 驱动   |
| 终端 3 | 上位机任意目录  | 任意                                        | 1.3.3 话题验证命令   |

#### 1.3.0 判读前提（两条实现约束）

日志判读依赖两条约束，机制与实测依据见 1.4.1、1.4.2：

- 只有存在进程打开 `/tmp/tty_laser`（pty 从端）时，tcp_server 才保持与 ESP32 的连接；
  无读者期间每接受一个连接即关闭，两侧表现为反复打印「连接已建立」；
- `[UART]` 的数值只反映「雷达 → UART1」，与网络状态无关；该行末尾的 `TCP 未连/已连` 仅作旁证。

#### 1.3.1 第 1 步：烧录 test09，先观察雷达是否出数据

下位机终端（本仓库根目录）：

```bash
pio run -e test09_bridge -t upload
pio device monitor -e test09_bridge -b 115200
```

`-b 115200` 不可省略：`platformio.ini` 未声明 `monitor_speed`，`pio device monitor` 默认
9600，与固件的 `SERIAL_BAUD = 115200`（`config.h:16`）不一致时只能看到乱码。

**先观察下位机** ：这一步只看雷达与接线，上位机无需任何操作（无需热点、无需 tcp_server）。

| 日志                                    | 含义                                        | 判定                        |
| --------------------------------------- | ------------------------------------------- | --------------------------- |
| `[MOTOR] ... duty=89`                   | M_CTR PWM 已输出，雷达电机应转              | 电机不转查供电与 PWM 脚     |
| `[UART] 最近 2s 收到 N 字节 (TCP 未连)` | N 稳定在数千 = 雷达数据已进 UART1           | N 恒 0 查 5V 供电与 Tx 接线 |
| `[UART] 最近 2s 收到 N 字节 (TCP 已连)` | 同上；`TCP 已连` 只说明此刻连接处于保持状态 | 不以 TCP 状态作判据         |

`[UART]` 的读取与计数在 `loop()` 第 0 步无条件执行，先于 WiFi/TCP 逻辑
（`test09_bridge/main.cpp:97-128`，判据含义见 1.4.2），故 WiFi 未连、上位机未起时同样可观测，
判据就是「N 稳定在数千」。

**`[MOTOR]` 只在上电时打印一次** （`setup()`，`test09_bridge/main.cpp:88-94`），而
`pio device monitor` 通常晚于上电才连上串口，日志可能已错过。若没看到 `[MOTOR]`，
**按下开发板 RST 键重启** ，让串口监视器捕获完整启动日志，再观察电机是否起转。

#### 1.3.2 第 2 步：下位机上电运行，上位机先启动 tcp_server

**下位机侧** （无需手动操作）：第 1 步烧录后板子即上电运行，固件自动连 WiFi 并作为 TCP
client 周期性重连上位机 8889（失败每 `BRIDGE_RECONNECT_MS` 重试一次）；中途断电/重启后
自动重新发起连接。唯一前提是板子保持上电、串口监视器保持打开。

**上位机侧** ：先启动 TCP server（等 ESP32 来连），上位机终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/ROS/YuXiangROS/Chap9/Robot_ws/install/setup.bash
ros2 run ros_serial2wifi tcp_server \
  --ros-args -p serial_port:=/tmp/tty_laser -p tcp_port:=8889
```

**参数必须写成 `--ros-args -p 名:=值`** ：`serial_port`、`tcp_port` 是 ROS 参数
（`tcpserver.py:17-18`），不是命令行开关。`ros2 run` 把可执行文件之后的裸参数原样交给进程，
而 rclpy 只解析 `--ros-args` 段，故裸的 `--serial_port /tmp/tty_laser` 不会成为参数值，节点
静默使用默认值 `/tmp/laserport`：现象是符号链接建在 `/tmp/laserport`，`/tmp/tty_laser`
始终不存在。节点启动日志打印的路径即实际生效路径，用它核对参数最直接。

**为什么先起 tcp_server** ：ESP32 是 TCP client，连不存在的 server 只会反复失败重试，
server 必须先就位（ESP32 侧无需干预，固件按 `BRIDGE_RECONNECT_MS` 自动重试）。

**再观察（按顺序核对四处）** ：

| 检查点   | 命令或位置              | 预期                                                                      |
| -------- | ----------------------- | ------------------------------------------------------------------------- |
| 参数生效 | tcp_server 启动日志     | `已映射到串口设备:/tmp/tty_laser`                                         |
| 监听就绪 | `ss -tlnp \| grep 8889` | `0.0.0.0:8889` 处于 LISTEN                                                |
| pty 就绪 | `ls -l /tmp/tty_laser`  | `/tmp/tty_laser -> /dev/pts/N`                                            |
| 两端可见 | 下位机串口 / 上位机日志 | `[TCP] 已连接` 与 `来自(...)的连接已建立`（尚无读者时反复出现，见 1.4.1） |

#### 1.3.3 第 3 步：最后启动雷达驱动，验证 /scan（阶段一成功标志）

上位机终端 2：

```bash
ros2 launch ydlidar ydlidar_launch.py
```

**包名是 `ydlidar`，不是 `ydlidar_ros2`** 。三处判据一致：`package.xml` 的
`<name>ydlidar</name>`；launch 文件内 `get_package_share_directory('ydlidar')`
（`ydlidar_launch.py:29`）；可执行文件位于 `install/ydlidar/lib/ydlidar/ydlidar_node`。
包名写错时 `ros2 launch` 直接报包不存在。可用 `ros2 pkg list | grep ydlidar` 核对。

**为什么最后启动** ：两个必要条件。

1. **`/tmp/tty_laser` 必须由 tcp_server（第 2 步）创建并指向当前 pty 从端** ：tcp_server
   每次启动都重建 pty，若先起 ydlidar 再起 tcp_server，驱动持有的 fd 指向已被删除的旧
   `/dev/pts/N`，表现为设备打不开或读到 EOF（1.4.3）。
2. **从端必须先有读者，tcp_server 才保持与 ESP32 的连接** （1.4.1）。ydlidar 自身即该读者：
   它打开从端后，下位机经历一次显式重连（≤ `BRIDGE_RECONNECT_MS` 1000 ms，`config.h:133`）
   并恢复送数，故驱动侧首帧晚到约 1 s，`laser.initialize()` 的重试窗口需覆盖这段等待
   （实测可正常通过）。

**驱动读取的参数** ：launch 默认加载 `install/ydlidar/share/ydlidar/params/ydlidar.yaml`，
与本链路相关的三项为 `port: /tmp/tty_laser`、`frame_id: laser_link`、
`support_motor_dtr: false`（依据见 1.4.4）。`port` 必须与 tcp_server 实际生效的
`serial_port` 逐字符相同，否则驱动报设备打不开。

**阶段一成功标志（命令行订阅到数据即通过）**：

| 命令                                                  | 预期               | 判定                                 |
| ----------------------------------------------------- | ------------------ | ------------------------------------ |
| `ros2 topic info /scan -v`                            | Publisher count: 1 | 为 0 说明驱动未起或包名/路径错       |
| `ros2 topic hz /scan`                                 | 约 13.7 Hz         | 无输出说明无发布者；偏低查 FIFO 溢出 |
| `ros2 topic echo /scan --qos-reliability best_effort` | ranges 有非零值    | 全为 0 说明链路未通                  |

**QoS 不匹配（最容易踩的坑）** ：ydlidar 发布 `/scan` 用 `rclcpp::SensorDataQoS()`，即
reliability=Best Effort + history=Keep Last 5（`ydlidar_node.cpp:165`），而 CLI 与 rviz2 的
默认订阅是 Reliable，两者不兼容，现象为命令无输出或 rviz 空白且不报错。处理方式：

```bash
ros2 topic echo /scan --qos-reliability best_effort   # echo 支持该参数, 必须显式指定
ros2 topic hz /scan                                   # hz 不接受 QoS 参数, 直接运行
```

`ros2 topic hz` 的订阅把 QoS 硬编码为 `qos_profile_sensor_data`（Jazzy 的
`ros2topic/verb/hz.py:268`），其 `add_arguments` 未定义任何 `--qos-*` 选项，传入即报
`ros2: error: unrecognized arguments`；故 hz 天然是 Best Effort，无需也无法指定。

#### 1.3.4 第 4 步（可选）：rviz2 可视化辅助验证

rviz2 不是验收项，仅用于确认点云轮廓与雷达安装朝向：

1. 启动 `rviz2`；
2. Global Options → Fixed Frame 改为 `laser_link`（与 `ydlidar.yaml` 的 `frame_id` 一致）；
3. Displays → Add → By topic → 选择 `/scan`（LaserScan）→ OK；
4. 展开刚添加的 LaserScan 项 → Topic → 把 Reliability 从 Default 改为 Best Effort（预设项
   "Sensor Data" 即 Best Effort + Keep Last 5，选它等价）。不改则一直空白且无任何报错。

至此阶段一验收 4 项全部满足。

### 1.4 链路机制与判读依据

#### 1.4.1 pty 从端读者决定 TCP 连接是否稳定

tcp_server 把主端 `master` 注册进 poll（`tcpserver.py:37`），任一 fd 可读即 `os.read`
（`tcpserver.py:52`），异常一律按断连处理（`tcpserver.py:64-68`）。从端 `/dev/pts/N` 无人
打开时，主端 poll 立即返回 `POLLHUP`，`os.read(master)` 抛
`OSError: [Errno 5] Input/output error`，于是每接受一个 ESP32 连接就在一个 poll 周期内关闭：
下位机表现为反复 `[TCP] 已连接`，上位机表现为反复打印「来自…的连接已建立」。只有 ydlidar
（或临时充当读者的测量脚本，1.4.3）打开从端后，连接才被持续保持，雷达数据才真正写进 pty。

不改上游的依据：向无从端读者的主端写入本身不报错（实测写入 100 字节成功），若屏蔽该异常并
继续持有连接，数据只会堆进从端输入队列，队列写满即阻塞整个节点（`accept()` 亦不再执行），
比立即断连更糟。缺的条件是「存在读者」，属启动顺序问题，故保持 `ros_serial2wifi` 原实现。

#### 1.4.2 两处「字节」观测点的区别

**（1）`[UART]` 计数** ：ESP32 侧 UART1 的累计接收量，只说明雷达在出数。其读取与计数在
`loop()` 第 0 步无条件执行，先于 WiFi/TCP 逻辑（`test09_bridge/main.cpp:97-128`），故与网络
状态无关；行末的 `TCP 未连/已连` 仅用于区分连接状态。

**（2）pty 侧字节** ：上位机 `/tmp/tty_laser` 可读出的字节，除雷达在出数外还要求存在从端
读者（1.4.1）。两者判据不可互推：「`[UART]` 有数而 pty 无字节」是常见组合，原因是上位机侧
无读者，不是雷达或接线故障；反之 pty 有字节则 UART 必然有数。

#### 1.4.3 pty 侧字节测量（可选）

测量仅用于排查 TCP 上行是否通，判据 1 与判据 4 已覆盖同一信息，故非必需。需要时在 ydlidar
启动之前独占从端运行：

```bash
python3 - <<'EOF'
import os, time, tty, select
fd = os.open('/tmp/tty_laser', os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd)                      # 等价于 stty raw: 关掉 ICANON/ECHO/ISIG
t0, n = time.time(), 0
while time.time() - t0 < 5:
    if select.select([fd], [], [], 0.5)[0]:
        n += len(os.read(fd, 4096))
dt = time.time() - t0
print(f"bytes={n}  window={dt:.1f}s  rate={n/dt:.0f} B/s")
EOF
```

判定只看量级：5 s 窗口读出 ≥1e4 字节（≥2000 B/s）即视为上行通畅。满速参考值 6888 B/s
（1.1），但窗口起点含一次 ESP32 重连（≤ `BRIDGE_RECONNECT_MS` 1000 ms，`config.h:133`），
实测约为满速的一半，故不设精确期望值。

三条使用约束：

1. **必须独占从端，且早于 ydlidar** ：pty 从端是单一字节队列，多读者会瓜分同一字节流，脚本
   与 ydlidar 并存时两者都丢帧；脚本结束即关闭从端，连接随即回到不稳定状态（1.4.1），属预期
   现象。
2. **必须置 raw 模式** ：从端默认处于规范模式（ICANON + ECHO + ISIG），二进制流被按行缓冲，
   0x03 等字节被解释为控制字符，直接 `cat` 会卡住或丢字节；脚本用 `tty.setraw()` 在同一 fd
   上完成，ydlidar 的串口层自行切 raw，驱动侧无需设置。
3. **重启 tcp_server 后必须重启 ydlidar** ：tcp_server 每次启动都重建 pty 并覆盖符号链接
   （`tcpserver.py:30-33`），`/dev/pts/N` 随之改变；ESP32 侧无需干预，固件按
   `BRIDGE_RECONNECT_MS`（`config.h:133`，1000 ms）自动重连。

#### 1.4.4 关键决策一览

| 决策                                                                                                           | 依据                                                                                                                                               | 落点                                                               |
| -------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| 雷达电机由 ESP32 侧 LEDC PWM 驱动（GPIO13、10 kHz、占空比 89/255 即 35%），上位机取 `support_motor_dtr: false` | 本链路为 TCP/pty 透传（`pty.openpty()`，无 DTR/RTS 处理），驱动 `startMotor()` 的 `setDTR()`（即 `TIOCM_DTR` ioctl）到不了 ESP32 的 M_CTR          | `config.h:124-131`、`test09_bridge/main.cpp:84-94`、`ydlidar.yaml` |
| UART RX 缓冲 4096 字节                                                                                         | 满速 6888 B/s 与净荷上限 11520 B/s 同量级，FIFO 溢出即丢帧                                                                                         | `setRxBufferSize(4096)`，须在 `Serial1.begin` 前调用               |
| 每次最多 512 字节批量转发                                                                                      | 逐字节 write 在高数据量下溢出 UART FIFO                                                                                                            | UART 收发循环                                                      |
| 断线后每 1 s 重连                                                                                              | 上位机约 5 s 无数据交换即断开                                                                                                                      | `BRIDGE_RECONNECT_MS`                                              |
| `frame_id: laser_link`                                                                                         | TF 链中雷达坐标系的唯一来源是 `base_footprint → base_link → laser_link`；取其他名字则 rviz 报 `No transform`，slam_toolbox 查询 scan→base 必然失败 | `ydlidar.yaml`、`fishbot.urdf`                                     |
| 包名 `ydlidar`；参数写成 `--ros-args -p 名:=值`                                                                | 包名三处判据见 1.3.3；`ros2 run` 之后的裸参数不会成为 ROS 参数值（1.3.2）                                                                          | launch 文件与 CLI                                                  |

### 1.5 排错速查

| 现象                                                        | 原因                                                                      | 处理                                                                                                                      |
| ----------------------------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `[MOTOR]` 未打印                                            | monitor 接入晚于上电，日志已错过                                          | 按 RST 键重启板子再观察（1.3.1）                                                                                          |
| 有 `[MOTOR]` 但电机不转                                     | 5V 带载能力不足 / PWM 脚错                                                | 换 ≥1A 独立 5V 供电，核对 GPIO13（1.2）                                                                                   |
| `[UART]` 恒为 0                                             | 供电不足 / Tx 接错脚                                                      | 核对线序 M_CTR→GND→Tx→VCC，确认 5V（1.2）                                                                                 |
| `[TCP]` 反复失败                                            | server 未起 / IP 或端口错                                                 | 先起 tcp_server；核对 `AGENT_IP_STR`（STA 分支默认 `10.42.0.1`，`config.h:84`）与本机 `ip -4 addr` 网段一致（1.3.2）      |
| `/tmp/tty_laser` 不存在                                     | tcp_server 未起，或 `serial_port` 未生效（链接建在默认 `/tmp/laserport`） | 看 tcp_server 启动日志的实际路径；启动时写 `--ros-args -p serial_port:=/tmp/tty_laser`（1.3.2）                           |
| `/tmp/tty_laser` 存在但无字节，或两端反复打印「连接已建立」 | 无进程打开从端，tcp_server 接受后即断开 ESP32                             | 先起 ydlidar；临时测速用 1.4.3 脚本（它自身即读者）                                                                       |
| pty 侧测出 0 字节或远小于期望                               | 测量窗口含着 ESP32 重连（≤1 s），或设备未打开                             | 用 1.4.3 脚本并只判量级（≥1e4 字节）                                                                                      |
| `ros2 launch` 报包不存在                                    | 包名写成 `ydlidar_ros2`                                                   | 用 `ros2 launch ydlidar ydlidar_launch.py`（1.3.3）                                                                       |
| `ros2 topic info /scan` 无发布者                            | 驱动未启动，或 `ydlidar.yaml` 的 `port` 与 tcp_server 实际路径不一致      | 核对两处路径逐字符相同（1.3.3）                                                                                           |
| 雷达在转但 `/scan` 空                                       | `support_motor_dtr: true`                                                 | 取 false（1.4.4）                                                                                                         |
| `/scan` 频率低或丢帧                                        | UART FIFO 溢出                                                            | `setRxBufferSize(4096)`，须在 `Serial1.begin` 前调用（1.4.4）                                                             |
| `echo` 无输出、rviz 空白                                    | 订阅端默认 Reliable，与发布端 Best Effort 不兼容                          | `echo` 加 `--qos-reliability best_effort`；rviz 该 topic 的 Reliability 选 Best Effort 或预设 Sensor Data（1.3.3、1.3.4） |
| `hz` 报 `unrecognized arguments`                            | `hz` 无 QoS 参数                                                          | 去掉 `--qos-reliability`，`hz` 内部固定 sensor QoS（1.3.3）                                                               |
| 重启 tcp_server 后 `/scan` 断流                             | pty 号已变，驱动仍持有旧 `/dev/pts/N`                                     | 先起 tcp_server，再重启 ydlidar（1.4.3）                                                                                  |

### 1.6 结束与清理

- 停止顺序与启动顺序相反：终端 2 停 ydlidar（Ctrl-C）→ 终端 1 停 tcp_server
  （Ctrl-C）→ 下位机停串口监视器（Ctrl-C，板子可保持上电）。
- 停掉 ydlidar 之后、tcp_server 仍在运行的那段时间，从端已无读者，tcp_server 会立刻
  断开 ESP32 并反复打印「等待接受连接..／来自…的连接已建立」，下位机相应反复
  `[TCP] 已连接`。这是 1.4.1 的直接结果，不是故障。
- tcp_server 退出后 `/tmp/tty_laser` 成为悬空符号链接（指向已关闭的 `/dev/pts/N`）。
  下次启动 tcp_server 会先删除再重建该链接（`tcpserver.py:31-33`），无需手工清理；
  但悬空期间不要启动 ydlidar，否则打不开设备。
- 阶段一通过后，雷达链路在阶段二由 bringup 一键入口接管，下位机改烧融合固件
  （第 2 节）；test09_bridge 仅用于本次独立透传验证。

## 2. 阶段二：融合固件 + 建图 / 导航（完整计划）

阶段二是最终形态。分工原则：硬件操作（上电、按 RST、遥控走位）由人工完成；
软件改动、编译验证与流程指导按本节执行。流程沿用原书 9.5.2~9.5.4 的路径框架：
**TF 构建（9-55~9-61）→ 上下位互通验证 → 一键 bringup 联调（9-62）→ 建图
（9-63）→ 存图（9-64）→ 导航（9-65/9-66）**，各代码清单与本工程的对应见 2.0。
除特别注明外，所有命令均在上位机新终端执行，每个常驻进程独占一个终端，
机器人保持上电。两侧仓库与执行位置：

- 上位机（ROS 2 侧）：远程仓库 `YuXiangROS-jazzy-learning`，本地工作空间
  `~/Documents/ROS/YuXiangROS/Chap9/Robot_ws`（下称 Robot_ws），2.1~2.5 的
  ros2 / colcon 命令均在此执行（新终端先 `source install/setup.bash`）；
- 下位机（ESP32-S3 固件侧）：远程仓库 `YuxiangROS-PIO-learning`（本仓库），
  仅 2.1 前置的固件烧录涉及。

**起控契约**：融合固件即主固件 `src/main.cpp`，其本身是两轮自平衡控制固件：直立环
PD 直接输出 PWM，`/cmd_vel` 只作为速度环外环的目标速度。未武装时状态机停在 IDLE
且不输出任何 PWM，故一切依赖运动的联调（T1、建图遥控、导航）都必须先武装：

```bash
ros2 topic pub --once /balance_enable std_msgs/msg/Bool "{data: true}"
```

武装后倾角进入机械中值窗口（`UPRIGHT_ARM_ANGLE_DEG`）即进入 RUN；以下三种情况
立即停机：`/balance_enable` 置 false、倾角超出 `UPRIGHT_FALL_ANGLE_DEG`、micro-ROS
会话断开（Agent 关闭或断网）。

### 2.0 原书代码清单与本工程对应

| 原书清单 | 内容                   | 本工程对应                                                   |
| -------- | ---------------------- | ------------------------------------------------------------ |
| 9-55     | fishbot.urdf           | robot_description/urdf/fishbot.urdf，TF 静态链来源           |
| 9-57     | urdf2tf.launch.py      | robot_description/launch/urdf2tf.launch.py                   |
| 9-59     | 启动 urdf2tf           | 见 2.1 步骤 1                                                |
| 9-60     | odom2tf.cpp            | robot_bringup/src/odom2tf.cpp，可执行名 odom2tf              |
| 9-62     | bringup.launch.py      | robot_bringup/launch/bringup.launch.py，ydlidar 延时 5s 启动 |
| 9-63     | slam_toolbox 在线建图  | 见 2.3，默认帧参数与本工程 TF 链一致                         |
| 9-64     | map_saver_cli 保存地图 | 见 2.4，保存后拷入 maps/ 并重新编译                          |
| 9-65     | 复制 nav2 参数模板     | config/nav2_params.yaml 已存在，参数修改见 2.5               |
| 9-66     | 启动导航               | 包名为 robot_navigation2，见 2.5                             |

### 2.1 分步联调：构建 TF 链与上下位互通（原书 9-55~9-61 路径）

分步目的：逐段验证 TF 与话题，出问题时能定位到段。前置（下位机侧，本仓库）：
烧录融合固件 `pio run -e esp32-s3-devkitc-1 -t upload`；其余步骤均在上位机。
上位机与 ESP32 同网段。

融合固件的串口同时输出两类日志：透传链路的 `[MOTOR]`（setup 打印一次）、`[UART]`
（每 2s 一次，第 0 步无条件计数，末尾附 `TCP 未连/已连`）、`[TCP]`（连接状态变化时），
以及控制链路的 `state=... theta=... pwm_L=... pwm_R=...`（10Hz），据此可在同一串口区分
雷达链路与平衡控制的状态。注意 1.4.1 的 pty 从端读者约束在阶段二同样成立：`bringup` 里
ydlidar 由 TimerAction 延时 5s 启动，在此之前 tcp_server 会反复断开 ESP32，`[UART]` 行末
显示 `TCP 未连` 且数字仍应为数千，属正常现象。

#### 步骤 1（终端 1）：启动 urdf2tf（原书 9-59）

```bash
ros2 launch robot_description urdf2tf.launch.py
```

验证：`ros2 run tf2_tools view_frames` 生成 frames.pdf，应出现
base_footprint → base_link → laser_link 静态链。

#### 步骤 2（终端 2）：启动 micro_ros_agent，随后给机器人上电

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

验证：agent 打印上下位机连接成功；另开终端 `ros2 topic hz /odom` 约 20Hz
（融合固件发布，TF：odom → base_footprint；里程计发布与是否武装无关）。

#### 步骤 3（终端 3）：启动 odom2tf 节点（原书 9-60 的可执行）

```bash
ros2 run robot_bringup odom2tf
```

#### 步骤 4（终端 4）：rqt_tf_tree 验证完整 TF 链

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

预期一条完整链：odom → base_footprint → base_link → laser_link
（前段来自 odom2tf，后段来自 urdf2tf）。

### 2.2 一键联调（原书 9-62）与整机实测 T1~T3（上位机）

分步验证通过后，改用 bringup 一键入口（agent + urdf2tf + odom2tf +
tcp_server + ydlidar 串起；因雷达驱动依赖串口转 WiFi 驱动，且 ydlidar 打开从端前
tcp_server 无法保持与 ESP32 的连接（1.4.1），ydlidar 用 TimerAction 延时 5s 启动，
用于覆盖 tcp_server 建 pty 与 ESP32 的一次重连，与分步等价）。该入口内 tcp_server 的 `serial_port`
以节点 `parameters` 形式传入，不存在 1.3.2 节的裸参数问题。先重新构建再运行：

```bash
colcon build --packages-select robot_bringup
ros2 launch robot_bringup bringup.launch.py
```

随后给机器人重新上电，在各节点正常运行后检查各话题和 TF 结构
（原书 9-62 段落）。运动检查前先武装（见本节起控契约）：

```bash
ros2 topic pub --once /balance_enable std_msgs/msg/Bool "{data: true}"
```

检查项：

| 步骤    | 操作                            | 判定                                                               |
| ------- | ------------------------------- | ------------------------------------------------------------------ |
| T1 运动 | 武装后 pub /cmd_vel             | 底盘响应；`ros2 topic hz /odom` 约 20Hz                            |
| T2 雷达 | bringup 已含 tcp_server+ydlidar | `ros2 topic hz /scan` 不低于 13 Hz（hz 自带 sensor QoS，不加参数） |
| T3 压测 | T1+T2 同时运行 5 分钟以上       | 频率稳定；无 TCP 重连风暴；`/odom` 连续无中断                      |

### 2.3 建图（原书 9-63）（上位机）

终端安排：终端 1 保持 bringup 运行，终端 2 启动 slam_toolbox，终端 3 遥控：

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

说明：slam_toolbox 默认 base_frame=base_footprint、odom_frame=odom、
scan_topic=/scan，与本工程 TF 链一致，无需自定义配置；遥控慢速走遍场地，
slam_toolbox 只消费数据不产生运动。可选另开终端 rviz2（Fixed Frame 设 map、
添加 /map 显示）观察建图质量。

遥控前机器人必须处于武装状态（见本节起控契约），否则 `/cmd_vel` 不产生任何运动。
若建图途中倒地停机，里程计不会被清零（固件在停止状态下仍逐周期积分，只是轮速为 0），
重新武装后位姿从原值续算；倒地过程中车轮空转产生的位姿误差无法被里程计观测，
会逐渐累积到地图上，故倒地后建议重新建图。

### 2.4 保存地图并入库（原书 9-64）（上位机）

建图满意后，在任意终端（工作目录即保存位置）执行：

```bash
ros2 run nav2_map_server map_saver_cli -f room
cp room.pgm room.yaml <Robot_ws>/src/robot_navigation2/maps/
colcon build --packages-select robot_navigation2
```

注意：`navigation2.launch.py` 默认从 install 空间的 `maps/room.yaml`
加载地图，不拷贝、不重编译则 map_server 加载失败。

### 2.5 导航（原书 9-65 / 9-66）（上位机）

前置：关闭 slam_toolbox——建图与定位都发布 map→odom TF，并存会冲突。

修改 `config/nav2_params.yaml`（原书 9-65 模板的既有拷贝）：

- MPPI FollowPath `vx_max`: 0.5 → 0.25；velocity_smoother `max_velocity`:
  [0.5, 0.0, 2.0] → [0.25, 0.0, 1.6]（底盘实际极速约 0.2~0.3 m/s）；
- `inflation_radius`: 0.7 → 0.35（local_costmap 与 global_costmap 两处）。

启动（原书 9-66，包名为 robot_navigation2，终端 1 保持 bringup）：

```bash
ros2 launch robot_navigation2 navigation2.launch.py use_sim_time:=False
```

随后 rviz2 用 2D Pose Estimate 给 AMCL 初始位姿（不给则定位发散），再用
Nav2 Goal 下发目标点，观察路径跟踪与避障。

Nav2 通过 `/cmd_vel` 驱动机器人，故整个导航过程必须保持武装（见本节起控契约）；
中途因倒地或 Agent 断开停机时，重新武装并重给初始位姿后再下发目标点。

### 2.6 执行顺序总览

```text
2.1 分步联调 → 2.2 一键联调+T1~T3 → 2.3 建图 → 2.4 存图入库 → 2.5 导航
```
