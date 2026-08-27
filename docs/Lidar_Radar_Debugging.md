# 激光雷达接入调试记录

## 1. 阶段一：test09_bridge 独立透传

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

#### 1.3.3 第 3 步：最后启动雷达驱动，验证 /scan（阶段一成功标志）

```bash
ros2 launch ydlidar_ros2 ydlidar_launch.py
```

**为什么最后启动**：ydlidar 打开 `/tmp/tty_laser` 前，该虚拟串口必须先由
tcp_server（第 2 步）创建。

**阶段一成功标志（命令行订阅到数据即通过）**：

| 命令                    | 预期          | 判定                             |
| ----------------------- | ------------- | -------------------------------- |
| `ros2 topic hz /scan`   | 约 13.7 Hz    | 明显偏低查 FIFO 溢出（见 1.4.2） |
| `ros2 topic echo /scan` | 角度/距离正常 | 全为 0 查链路未通                |

两条命令均需显式指定 QoS（原因见下），能稳定出数即阶段一验收通过
（对应 1.1 验收第 4 项，前三项在第 1、2 步已满足）。

**QoS 关键点（最容易踩的坑）**：ydlidar_ros2 发布 `/scan` 用的是
`rclcpp::SensorDataQoS()`，即 **reliability=Best Effort + history=Keep Last 5**
（见 `ydlidar_ros2/src/ydlidar_node.cpp` 第 165 行）。而 ROS2 CLI 与 rviz2 的
默认订阅 QoS 是 **Reliable**，与发布端不匹配——现象是"命令无输出 / rviz 空白但
不报错"。因此 `echo` / `hz` 需显式指定：

```bash
ros2 topic echo /scan --qos-reliability best_effort
ros2 topic hz /scan --qos-reliability best_effort
```

#### 1.3.4 第 4 步（可选）：rviz2 可视化辅助验证

rviz2 不是验收项，仅用于直观确认点云轮廓与雷达安装朝向，按顺序操作：

1. 启动：`rviz2`；
2. 左上角 Global Options → Fixed Frame 改为 `laser_link`（与
   `ydlidar_ros2/params/ydlidar.yaml` 的 `frame_id` 一致）；
3. Displays 面板 → Add → By topic → 选择 `/scan`（类型 LaserScan）→ OK；
4. **在 Displays 面板展开刚添加的 LaserScan 项 → 展开 Topic → 把 Reliability
   从 "Default" 改为 "Best Effort"**（rviz2 预设项 "Sensor Data" 即
   Best Effort + Keep Last 5，选它等价）。改完后 rviz2 会重新订阅，轮廓即
   出现；不改则一直空白且无任何报错。

至此阶段一验收 4 项全部满足，`test09` 独立调试结束。

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

#### 1.4.5 `/scan` 的 frame_id 必须与 URDF 雷达 link 名一致

`ydlidar.yaml` 的 `frame_id` 已由 `laser_frame` 改为 `laser_link`，与
`fishbot.urdf` 中雷达 link 名对齐（改动已在鱼香ROS仓库本地 fishbot 分支
提交）。原因：TF 树中雷达坐标系只有 `base_footprint → base_link →
laser_link` 这一条来源链，驱动若发布 `frame_id: laser_frame` 的
`/scan`，该坐标系在 TF 中无任何变换来源。话题层测试（`hz`/`echo`）
不受影响，但 rviz 点云会报 "No transform from laser_frame"，
slam_toolbox 建图查询 scan→base 变换时必然失败——建图前务必保证
scan 的 frame_id 可在 TF 树中解析。

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
（融合固件发布，TF：odom → base_footprint）。

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
tcp_server + ydlidar 串起；因雷达驱动依赖串口转 WiFi 驱动，ydlidar 用
TimerAction 延时 5s 启动，与分步等价）。先重新构建再运行：

```bash
colcon build --packages-select robot_bringup
ros2 launch robot_bringup bringup.launch.py
```

随后给机器人重新上电，在各节点正常运行后检查各话题和 TF 结构
（原书 9-62 段落），检查项：

| 步骤    | 操作                            | 判定                                             |
| ------- | ------------------------------- | ------------------------------------------------ |
| T1 运动 | bringup 后 pub /cmd_vel         | 底盘响应；`ros2 topic hz /odom` 约 20Hz          |
| T2 雷达 | bringup 已含 tcp_server+ydlidar | `ros2 topic hz /scan` 不低于 13Hz（best_effort） |
| T3 压测 | T1+T2 同时运行 5 分钟以上       | 频率稳定；无 TCP 重连风暴；`/odom` 连续无中断    |

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

### 2.6 执行顺序总览

```text
2.1 分步联调 → 2.2 一键联调+T1~T3 → 2.3 建图 → 2.4 存图入库 → 2.5 导航
```
