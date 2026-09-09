# 上位机 WiFi 局域网配置笔记

> 整理日期：2026-09-06
> 适用平台：Ubuntu 24.04 + NetworkManager（Windows 仅附备查）
> 固件侧参数入口：`include/RobotConfig/config.h`（`AGENT_IP_STR`、`AGENT_PORT=8888`、`WIFI_SSID`/`WIFI_PASS`）。

---

## 0. 计算机网络基础理论

### 0.1 分层模型：每层只解决一层问题

网络通信按"层"组织，每层只处理自己职责内的字段，对上层透明。TCP/IP 把 OSI 七层压成四层，本项目自底向上可映射为一张"故障归因表"：

| 层     | 职责           | 本项目实体             | 现象关键词        |
| ------ | -------------- | ---------------------- | ----------------- |
| 物理层 | 比特与电磁信号 | 2.4GHz 射频            | 搜不到 / 信号弱   |
| 链路层 | 帧、介质访问   | 802.11 SSID、AP 角色   | 关联失败 / 被隔离 |
| 网络层 | 跨网寻址与路由 | IPv4 `10.42.0.x`、子网 | ping 不通网关     |
| 传输层 | 端到端会话     | UDP 8888 / TCP 8889    | Agent 无会话      |
| 应用层 | 语义解释       | ROS2 / micro-ROS XRCE  | 话题无数据        |

发送方逐层"加头"（封装），接收方逐层"去头"（解封）：

```text
ESP32 侧封装                      电脑侧解封
[XRCE 载荷]                       [XRCE 载荷]
+UDP头 → [UDP|XRCE]       →       [UDP|XRCE] -UDP头
+IP头  → [IP|UDP|XRCE]            [IP|UDP|XRCE]  -IP头
+802.11 → [帧|IP|UDP|XRCE]   →    [帧|IP|UDP|XRCE] -帧头
```

这就是第 5 节排障表"自底向上"的原因：**上层故障会掩盖下层症状** ——频段不对时表现为"连不上"，绝不会先报"话题无数据"。

### 0.2 物理层：频率、波长与 2.4/5 GHz 之争

电磁波以光速传播，波长与频率满足：

$$
\lambda = \frac{c}{f}
$$

WiFi 两个频段都落在 ISM 免许可频段，波长差异直接决定传播特性：

- 2.4 GHz：$\lambda \approx 12.5\ \text{cm}$
- 5 GHz：$\lambda \approx 6\ \text{cm}$

波长越长，绕射能力越强、穿墙衰减越小——所以 2.4G 覆盖占优，5G 更"怕墙"。自由空间路径损耗定量看：

$$
\text{FSPL(dB)} = 20\lg d + 20\lg f + 32.44 \quad (d\ \text{单位 km},\ f\ \text{单位 MHz})
$$

同一距离下 5 GHz 比 2.4 GHz 多损耗：

$$
20\lg\frac{5000}{2400} \approx 6.4\ \text{dB}
$$

即 5G 天生比 2.4G 弱约 6.4 dB。但 2.4G 的代价是 **拥挤** ：WiFi 信道 1–13、蓝牙、Zigbee、微波炉（~2.45 GHz）全挤在 2.4 GHz 附近，同频干扰大、有效速率天花板低——20 MHz 下 802.11n 单流约 72 Mbps，对本项目每秒几百字节的 XRCE 消息绰绰有余。

**物理层结论**：能否收到某频段由 **射频前端硬件** 决定，不是软件选项。ESP32-S3 只做了 2.4G b/g/n，5G 信号对它物理不可见——这就是全文所有方案必须"把 AP 逼到 2.4G"的根因。

### 0.3 链路层：802.11 角色、SSID 与二层隔离

802.11 网络有两种角色：

- **AP（Access Point）**：基础设施模式中心，周期性广播 Beacon；
- **STA（Station）**：终端，经"扫描 → 认证 → 关联（Association）"挂到 AP 下。

**SSID** 是给人看的网络名；**BSSID** 是 AP 的 MAC，机器真正按它区分（同名 SSID 可有多个 AP——这正是"SSID 同名设备冲突"现象的来源）。

2.4G 信道中心频率：

$$
f_n = 2407 + 5n\ \text{MHz} \quad\Rightarrow\quad \text{信道1}=2412,\ \text{信道6}=2437,\ \text{信道11}=2462
$$

相邻信道间隔仅 5 MHz，而单信道占宽 20 MHz → 只有相隔 ≥5 信道的 1/6/11 互不重叠。这就是换信道时只考虑 1/6/11 的原因。

**二层隔离（AP isolation / client isolation）**：STA 之间的帧本应由 AP 按 MAC 二层转发；启用隔离后，AP **丢弃 client→client 的帧**，只放行 client↔AP↔上联口。这是纯二层的"横向通信禁令"，与 IP/网段无关——所以手机热点默认隔离时，**同网段也 ping 不通**，改 IP 无济于事（§2.2 的根源）。

### 0.4 网络层：IP、子网、网关与 NAT

IPv4 地址 + 掩码划出"网络号/主机号"。以 `10.42.0.1/24`（掩码 `255.255.255.0`）为例：

```text
网络号 10.42.0    本子网内直达（不查路由表）
主机号 .1          = 热点主机自身（网关/Agent）
.0                 网络地址（不可分配）
.255               广播地址
.2 ~ .254          终端（DHCP 租约分发，ESP32 常取 .2+）
```

同子网两台主机先 ARP 查对方 MAC 再二层直通；**跨子网必须交给网关转发**。NetworkManager 热点以 `shared` 模式自建 DHCP + NAT：自己固定 `10.42.0.1`，用内置 dnsmasq 给终端发 `10.42.0.x`，终端出公网流量经源地址转换（NAT）从上联口出去。

对本项目最关键的是： **Agent 与 ESP32 同处 `10.42.0.0/24`，互相直达，根本不经 NAT** ——NAT 只影响"电脑上要出公网的其他应用"。这也解释了为何要求网段对齐固件 `AGENT_IP_STR`：`10.42.0.1` 正是热点网关/主机地址，ESP32 发往它的包一跳即达。

### 0.5 传输层：端口、UDP 与 TCP

IP 负责"把包送到哪台主机"，**端口**负责"送到这台主机上的哪个进程"——同一 IP 上同时跑 8888（micro-ROS）与 8889（透传桥）互不干扰，靠的就是端口号分离。

**UDP**（8 字节头：源端口/目的端口/长度/校验和）：

- 无连接：发前不握手；
- 无重传、无有序保证：丢包即丢，由上层容忍；
- 适合周期性小报文：micro-ROS 的 XRCE 消息按周期重复发布，丢一两帧只表现为该周期跳变，无需可靠语义 → Agent 侧 `udp4` + `best-effort` QoS 正是为此设计。

**TCP**（20 字节头 + 三次握手建立）：

- 面向连接、字节流、可靠有序（序号/确认/重传）；
- 适合不可丢的连续字节流：激光雷达 8889 透传即是。

传输层最后一步是**绑定**：Agent 默认绑 `0.0.0.0`（所有接口）才能收到 ESP32 从无线口进来的入站包；若只绑 `127.0.0.1` 则外部永远连不上——"能 ping 通网关却无会话"的常见原因之一。

### 0.6 收束：一次数据包的旅程

把 0.1–0.5 串成一个实例：ESP32 上发一帧 odom 到 Agent。

```text
[ESP32 固件] rcl_publish(odom)
   ↓ XRCE 序列化
[UDP socket → 10.42.0.1:8888]     (0.5 端口分离)
   ↓ 加 UDP/IP 头，源 10.42.0.x
[802.11 STA 已关联 2.4G AP]       (0.3 关联/SSID)
   ↓ 空中 2.4GHz 帧                (0.2 频段)
[电脑网卡 AP] 解帧 → 内核 IP 栈
   ↓ 目的 10.42.0.1 命中本机      (0.4 同子网直达)
[Agent UDP:8888 socket] 解包 → ROS 话题 (0.1 应用层)
```

任一步断掉，症状各有归属：断在 0.2 → 搜不到；0.3 → 关联失败/被隔离；0.4 → ping 不通；0.5 → 无会话；0.6 之前全通而话题无数据 → 查 Agent/应用层。 **诊断永远先问"症状在哪一层"，再去动那一层的旋钮** ——这就是第 5 节排障表的结构依据。

---

## 1. 背景：为什么需要一条 WiFi 局域网

- 下位机 ESP32-S3 通过 WiFi 与上位机通信，两个端口共用同一 WiFi/lwIP：
  - **UDP 8888**：micro-ROS（`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`）；
  - **TCP 8889**：激光雷达透传桥（`ros_serial2wifi tcp_server`）。
- **硬件硬约束**：ESP32-S3 射频前端只支持 **2.4GHz（802.11 b/g/n）**。5GHz 信号对它"不存在"，搜都搜不到——这是硬件能力，不是配置问题。
- 因此上位机热点必须落在 **2.4GHz**，且网段需与固件写死的 `AGENT_IP_STR` 对齐。

## 2. 三种接入形态的演进与结论

### 2.1 普通路由器

- 双频并发，ESP32 只能连 2.4G 那个 SSID。
- 典型卡点：
  - **portal 认证** （企业与校园公网常见）：`WiFi.begin()` 能拿到 IP，但流量被劫持到认证页，Agent 会话建不起来；
  - **客户端隔离（client isolation）/ AP 隔离** ：同路由下设备互访被断；
  - 双频同名 SSID 时，设备可能被引导到 5GHz 而连不上。

### 2.2 手机热点

- 手机热点**可切换 2.4G/5G**（iOS 需手动开「最大兼容性」才锁 2.4GHz；Android 频段可设）。
- 更致命的是 **互联受限** （普遍行为，非个例）：
  - 手机热点默认启用 **客户端隔离** ，连入设备彼此不可见、不可互访，只开放 NAT 出口上网；
  - 不少 ROM 的该隔离 **没有开关可关** （路由器上的 AP 隔离大多还能关，手机热点常关不了）；
  - 手机不能同时作为 STA 连别的 WiFi 再转发（禁止 AP 互联/中继）。
- 结论： **"ESP32 与电脑都挂手机热点再互通"这条路在系统层就被禁止** ，不是改网段能解决的。

### 2.3 电脑网卡热点（当前方案）

- 用电脑无线网卡开 SoftAP，NetworkManager 热点 **默认共享网段 `10.42.0.0/24`，网关 `10.42.0.1`** ，恰好等于固件 `AGENT_IP_STR` → **固件零改动**。
- ESP32 与 Agent 同机直连，一跳直达，无中间转发。
- 前提（见 §3）：**无线网卡必须支持 AP 模式**。

### 2.4 形态对比总表

| 形态       | 频段供给                                 | ESP32 能否连            | 真正卡点                                                                            |
| ---------- | ---------------------------------------- | ----------------------- | ----------------------------------------------------------------------------------- |
| 普通路由器 | 双频并发                                 | 只连 2.4G SSID          | portal 认证；AP 隔离（多数可手动关）                                                |
| 手机热点   | 可切 2.4G/5G（iOS「最大兼容性」锁 2.4G） | 需热点**显式落在 2.4G** | ① 默认 5G → 搜不到；② 客户端隔离默认开且常无开关 → 设备互访被禁止；③ 网段与固件不符 |
| 电脑热点   | 网卡能力，可显式指定 `band bg`           | `band bg` 强制 2.4G     | 需**网卡支持 AP 模式**（§3）                                                        |

## 3. 电脑热点前置检查：网卡 SoftAP 能力

电脑热点（SoftAP）成立依赖「网卡硬件 + 驱动 + 固件」三者允许 AP 角色，**不是所有网卡默认支持**。

```bash
# Linux：查看网卡支持的接口模式，期望出现 "AP"
iw list | sed -n '/Supported interface modes/,/^$/p'

# 查看 wifi 网卡设备名
nmcli device status

# Windows 备查（需管理员）
netsh wlan show drivers        # 看"支持的承载网络: 是"
```

- 若驱动不支持，hostapd 会报 `driver doesn't support AP mode`，`nmcli` 热点起不来。
- 驱动生态差异：Atheros `ath9k`、MediaTek `mt76`、Realtek `rtw88/rtw89` 对 AP 支持较好；**Intel `iwlwifi` 较弱/受限**，不少型号没有可靠的基础设施 AP 模式。
- 拓扑限定：若电脑还要同时连外网（STA）又开热点（AP），还需网卡支持**并发多角色**；本项目 Agent 只需与 ESP32 直连、不依赖外网，天然只要求单角色 AP，因此该方案才是 **"最小化"** 。

## 4. nmcli 常用指令

### 4.1 一键开热点（2.4GHz）

```bash
# 先确认 wifi 网卡名（通常 wlan0 / wlp2s0）
nmcli device status

# 一键开热点：band bg 锁 2.4G，channel 用 1/6/11 中不拥挤的一个
nmcli device wifi hotspot \
  ifname wlan0 \
  ssid "<WIFI_SSID>" \
  password "<WIFI_PASS>" \
  band bg channel 6
```

说明：

- 首次执行会创建一个名为 `Hotspot` 的 NetworkManager 连接，`ipv4.method` 自动为 `shared`（即 NAT 共享、网关 10.42.0.1）；
- SSID/密码须与 `include/RobotConfig/config.h` 中 `WIFI_SSID` / `WIFI_PASS` 一致。

### 4.2 手动建 profile（更可控，推荐长期方案）

```bash
nmcli connection add type wifi ifname wlan0 con-name AgentAP autoconnect no \
  ssid "changli-Legion-Y7000-IRX9"
nmcli connection modify AgentAP 802-11-wireless.mode ap
nmcli connection modify AgentAP 802-11-wireless.band bg
nmcli connection modify AgentAP 802-11-wireless.channel 6
nmcli connection modify AgentAP wifi-sec.key-mgmt wpa-psk
nmcli connection modify AgentAP wifi-sec.psk "<WIFI_PASS>"
nmcli connection modify AgentAP ipv4.method shared
nmcli connection up AgentAP
```

### 4.3 查看状态 / 修改 / 关闭

```bash
nmcli connection show --active                        # 当前活动连接
nmcli connection show Hotspot                         # 查看某连接全部参数
nmcli -f IP4.ADDRESS,IP4.GATEWAY device show wlan0    # 热点自身 IP，应见 10.42.0.1/24
nmcli connection down Hotspot                         # 关闭热点（再次 up 可恢复）
nmcli connection delete Hotspot                       # 删除 profile
nmcli radio wifi                                      # wifi 射频开关状态
```

### 4.4 主机侧联调验证

```bash
# 终端 1：Agent 监听（绑定 0.0.0.0，收 ESP32 入站）
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 终端 2：观察下位机串口，应打印 [WiFi] connected, local IP=10.42.0.x

# 终端 3：话题级验证
ros2 topic pub /balance_enable std_msgs/msg/Bool "{data: true}" -r 5
ros2 topic hz /odom          # 期望约 20Hz
```

## 5. 排障速查（按层自底向上）

| 现象                                  | 先查什么                      | 常用命令/手段                                                                           |
| ------------------------------------- | ----------------------------- | --------------------------------------------------------------------------------------- |
| ESP32 搜不到 SSID                     | **频段**（最常见：热点在 5G） | 确认 `band bg`；iPhone 热点开「最大兼容性」                                             |
| 连上但拿不到 IP / ping 不通网关       | **网段/隔离**                 | `nmcli -f IP4.ADDRESS device show wlan0` 必须 10.42.0.1；手机热点基本无解（客户端隔离） |
| 拿到 IP、ping 通网关，但 Agent 无会话 | **端口/防火墙/Agent 绑定**    | Agent 用 `udp4 --port 8888` 默认绑 0.0.0.0；有防火墙放行 8888/8889                      |
| Agent 能看到客户端但数据断            | **同频干扰/信号**             | 2.4G 换信道（1/6/11）；SSID 同名设备冲突                                                |
| 热点根本起不来                        | **网卡无 AP 能力**            | `iw list` 看是否含 AP；换网卡/换驱动                                                    |
| 连上后一会儿就掉                      | **固件重连/时间同步**         | 看下位机串口 `[WiFi]`/`[BRIDGE]` 重连日志                                               |

## 6. Windows 侧备查（若在 Windows 上开热点）

```bat
netsh wlan show drivers          :: 看"支持的承载网络"
:: 承载网络模式（旧方案，驱动不支持则失败）
netsh wlan set hostednetwork mode=allow ssid=xxx key=yyy
netsh wlan start hostednetwork
```

新版本更推荐用系统「移动热点」GUI（同样依赖驱动 SoftAP 支持，默认网段常为 192.168.137.1，**与固件 10.42.0.1 不符，需改 `AGENT_IP_STR` 重编译或改用 Linux**）。

## 7. 关键结论备忘

1. ESP32-S3 只认 2.4GHz → 一切形态的第一步是"把 AP 逼到 2.4G"；
2. 手机热点"设备互访被禁"是系统级行为，网段对不上只是次要问题；
3. 电脑热点最小化成立的三前提：**热点落 2.4G + 网卡支持 AP + 拓扑只要求单角色 AP**；
4. NetworkManager 热点默认 10.42.0.1/24 与固件 `AGENT_IP_STR` 天然对齐，这是当前方案能"零改固件"的根本原因。

---

## 8. 固件网络拓扑与接入契约（STA 接入 / AP 自组网）

> 对应代码：`include/NetBoot/net_boot.h`（网络自举入口）、`include/RobotConfig/config.h`（拓扑开关与参数）。
> 第 2 节从"上位机侧怎么建网"出发；本节从"固件侧支持哪两种网络形态、各自契约是什么"出发，两者互补。

### 8.1 两种拓扑总览

固件通过编译期宏 `WIFI_ROLE_AP` 在两种拓扑间切换，`config.h` 中默认 `0`（可用命令行 `-DWIFI_ROLE_AP=1` 覆盖，便于不改文件做试验）：

| 维度         | STA 接入（默认，`WIFI_ROLE_AP=0`）                      | AP 自组网（`WIFI_ROLE_AP=1`，试验）                       |
| ------------ | ------------------------------------------------------- | --------------------------------------------------------- |
| ESP32 角色   | STA，接入外部 AP                                        | 自身开 2.4G SoftAP，兼任 AP + DHCP server                 |
| 上位机角色   | AP 宿主（电脑热点/路由器/手机热点）                     | STA，手动配静态 IP                                        |
| 网段         | 取决于外部 AP；电脑热点默认 `10.42.0.0/24`              | ESP32 固定 `192.168.4.0/24`，自身 `192.168.4.1`           |
| Agent 地址   | 固件固定 `AGENT_IP_STR`（电脑热点场景为 `10.42.0.1`）   | 固件固定 `AGENT_IP_STR` 为 `192.168.4.100`                |
| 固件建网方式 | `WiFi.begin` 阻塞等 STA 连接，再注册官方 wifi transport | 不发起 `WiFi.begin`，直接注册纯 UDP transport（复用回调） |
| 适用场景     | 有外部 AP/热点的室内常规联调                            | 户外、无路由、快速双机验证                                |
| 外网访问     | 经外部 AP 上联口（NAT）                                 | 无（仅局域网内互通，除非上位机额外开转发）                |

两种形态下 transport 回调（open/write/read）完全相同，与 AP/STA 角色无关，故 AP 模式只需"跳过一次 `WiFi.begin`"即可复用同一套 micro-ROS 传输栈。

### 8.2 STA 接入（默认路径）

- 链路：`wifi_role_boot` 解析 `AGENT_IP_STR` → `set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT)` 内部完成 `WiFi.begin` 阻塞连接并注册 transport。
- 网络侧配置见第 2、4、5 节（电脑热点方案网段与固件天然对齐，零改固件）。

### 8.3 AP 自组网（`WIFI_ROLE_AP=1`）

设计动机：联调环境可能没有可用的外部 AP（路由器被占用、手机热点隔离不可关、户外无网）。让 ESP32 自开 SoftAP，上位机作为 STA 连入，形成最小局域网。

网络契约（`net_boot.h` 头部注释）：

- ESP32 SoftAP 自身固定 `192.168.4.1`（Arduino-ESP32 SoftAP 默认网段），自带 DHCP server，给上位机分配 `192.168.4.x`。
- 上位机（Agent 宿主）必须手动配静态 IP `192.168.4.100`：Agent 地址编译期写死在固件 `AGENT_IP_STR`，无法运行期动态发现，故上位机 IP 必须落在编译期已知值上。
- SSID / WPA2 密码 / 信道由 `WIFI_AP_SSID` / `WIFI_AP_PASS` / `WIFI_AP_CHANNEL` 给定（`config.h`，信道默认 6，属 2.4G 不重叠集 1/6/11）。

transport 注册要点（`set_microros_wifi_ap_transports`）：

- 与官方 `set_microros_wifi_transports` 同构，差异仅在后者先 `WiFi.begin` 阻塞等 STA 连接，本函数假设 SoftAP 已由调用方开启。
- 通过 `rmw_uros_set_custom_transport` 注册，复用 `platformio_transport_*` 回调，会话建立逻辑（第 9 节生命周期）完全一致。

上位机侧静态 IP 配置（Linux + NetworkManager 示例，网卡名以实际为准）：

```bash
nmcli connection add type wifi ifname <wifi_dev> con-name AgentSTA autoconnect yes \
  ssid "<WIFI_AP_SSID>"
nmcli connection modify AgentSTA wifi-sec.key-mgmt wpa-psk
nmcli connection modify AgentSTA wifi-sec.psk "<WIFI_AP_PASS>"
nmcli connection modify AgentSTA ipv4.method manual ipv4.addresses 192.168.4.100/24
nmcli connection up AgentSTA
```

局限与前提：

- 仅支持编译期已知的单个 Agent 地址；上位机静态 IP 与固件 `AGENT_IP_STR` 不一致时，ESP32 可达但 Agent 收不到（包发往 `.100` 而实际地址不同），现象同"无会话"。
- SoftAP 单射频即 2.4G，ESP32-S3 硬件本就只支持 2.4G，无频段问题；同一 2.4G 频谱的干扰/信道排障规则沿用第 5 节。
- 上位机若同时要访问外网，需另开 NAT/转发（本拓扑默认不出网）。

### 8.4 固件代码映射与参数入口

| 代码位置                                                       | 作用                                                                           |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| `net_boot.h` `wifi_role_boot`                                  | 解析 `AGENT_IP_STR` 到 `IPAddress`，按 `WIFI_ROLE_AP` 分支建网并注册 transport |
| `net_boot.h` `set_microros_wifi_ap_transports`                 | AP 模式专用：纯 UDP transport 注册（不 `WiFi.begin`）                          |
| `config.h` `WIFI_ROLE_AP`                                      | 拓扑开关，`#ifndef` 包裹以支持 `-D` 覆盖                                       |
| `config.h` `AGENT_IP_STR` / `AGENT_PORT`                       | Agent 地址与 UDP 端口（两拓扑各有一组注释说明）                                |
| `config.h` `WIFI_SSID` / `WIFI_PASS`                           | STA 模式凭据（须为非 const 可写数组，接口要求 `char*`）                        |
| `config.h` `WIFI_AP_SSID` / `WIFI_AP_PASS` / `WIFI_AP_CHANNEL` | AP 模式广播参数                                                                |

---

## 9. micro-ROS 会话生命周期与故障处理

> 本固件的 micro-ROS 网络会话采用"初始化 -> spin -> 释放 -> 重建"的完整生命周期模型，能自愈两类常见故障（Agent 启动晚于固件、运行中会话断开）。
> 以下根因结论均引述本地编译内嵌的 rclc 源码（路径 `lib/micro_ros_platformio/build/mcu/src/rclc/rclc/src/rclc/`），并以函数与行号标注，可自行复核。

### 9.1 会话生命周期模型

micro-ROS 对象按依赖序初始化，失败/断开时逆序释放：

```text
初始化:  rclc_support_init(support)  ->  rclc_node_init_default(node)
         ->  rclc_executor_init(executor)
释放:    rclc_executor_fini  ->  rcl_node_fini  ->  rclc_support_fini   (严格逆序)
```

- 初始化链任一步失败：释放该步之前已成功的对象，延时 `RECONNECT_INTERVAL_MS` 后整体重来；
- spin 期间会话失效：`rclc_executor_spin_some` 返回错误，break 后走同一套逆序释放并重建；
- 任务函数永不返回。原因：`xTaskCreate` 创建的任务函数一旦 return，FreeRTOS 调度器从已失效的任务栈指针继续取指，表现为 PC 奇数不对齐、`IllegalInstruction` 复位。故失败路径只允许"延时重试"，不允许越过函数尾。

运行期轮询采用 `rclc_executor_spin_some`（单次限时 10 ms）而非 `rclc_executor_spin`：本固件未注册订阅/定时器句柄，等待集为空时 `rcl_wait` 立即返回，`spin` 会退化为无休眠忙循环独占 core0；`spin_some` 返回后显式 `delay(1)` 让出 CPU。仅当 Agent 会话失效（context 无效）时返回错误，正常超时返回 `RCL_RET_TIMEOUT` 需放行。

### 9.2 两类典型故障与根因

#### 9.2.1 Agent 启动晚于固件（固件先跑）

现象：串口循环打印 `[micro_ros] support init failed (<code>), agent unreachable`，间隔约 `RECONNECT_INTERVAL_MS`。

根因：`rclc_support_init` 内部调用 `rcl_init`（`init.c:70`），其 rmw 层需与 Agent 建立 XRCE 会话；Agent 不可达时 `rcl_init` 返回错误并向上传播，support 初始化即失败。此时固件尚未创建任何可用对象（`init.c:69` 已先把 context 重置为零初始化值），失败分支不调用 fini，直接延时重试。

处理：该分支天然实现"等 Agent 上线后自动建链"，无需人工干预；Agent 先启动时不会走到此分支。

#### 9.2.2 运行中会话断开（Agent 掉线或重启）

现象：串口打印 `[micro_ros] spin_some failed (<code>), agent session lost`，随后按重建节奏恢复，话题数据重新流动。

根因：`rclc_executor_spin_some` 入口校验上下文有效性（`executor.c:1808-1811`）：

```c
if (!rcl_context_is_valid(executor->context)) {
  PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_context_not_valid);
  return RCL_RET_ERROR;
}
```

rmw 层通过周期性 ping 维护会话，Agent 掉线超时后 context 被判无效，下一次 `spin_some` 即返回 `RCL_RET_ERROR`。固件据此跳出 spin 层进入重建；Agent 侧重启后无需任何操作，固件自动重新建链。

### 9.3 重建可行性与实现约束（rclc 契约）

生命周期可安全重建的前提是 rclc 对"重复 init / 重复 fini"的幂等保证，逐条引述：

| 约束                           | 源码依据                                                                                                                                  | 结论                                                                     |
| ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| 执行器句柄容量必须 ≥ 1         | `executor.c:108-111`：`number_of_handles == 0` 时直接 `return RCL_RET_INVALID_ARGUMENT`，且该 return 发生在 `executor.c:114` 整体清零之前 | 无句柄固件也须传最小合法值 1，否则 executor 初始化必然失败且不落任何状态 |
| 重复 `rclc_executor_init` 安全 | `executor.c:114`：每次 init 先 `(*executor) = rclc_executor_get_zero_initialized_executor();` 整体重置                                    | 半初始化/残留状态被清零覆盖，重建无需新建对象，可复用 static 实例        |
| 重复 `rclc_support_init` 安全  | `init.c:69`：每次 init 先 `support->context = rcl_get_zero_initialized_context();`                                                        | 失败后的 context 残留被重置，下轮 init 从干净状态开始                    |
| `rclc_executor_fini` 幂等      | `executor.c:199-201`：对无效 executor 走空分支，注释明确 "Repeated calls to fini or calling fini on a zero initialized executor is ok"    | 释放路径可安全重复执行                                                   |
| 失败分支不 fini 半初始化对象   | `executor.c:108-111` / `:126-129` 的失败点均不持有需手动释放的堆内存                                                                      | 直接交给下轮整体重置兜底；只有"完整初始化成功"的对象才进释放路径         |

释放顺序与初始化严格逆序，且每步 fini 返回值以 `(void)` 消费（fini 失败无补救动作）。

### 9.4 实现要点速查

- 参考实现：`src/tests/test06_wifi/main.cpp` 的 `micro_ros_task` 函数（上述模型在该测试固件的落地，可直接移植到主固件/演进固件）。
- 相关参数（`config.h`）：`MICRO_ROS_STACK_SIZE` / `MICRO_ROS_TASK_PRIO`（任务资源）、`TRANSPORT_SETUP_MS`（网络自举后等待时间）、`RECONNECT_INTERVAL_MS`（重建重试间隔）、`ODOM_PUBLISH_MS`（发布周期，若发布话题）。
- 诊断口径：以串口 `[micro_ros]` 前缀日志为准，三类日志分别对应"support 失败 / node 失败 / executor 失败 / spin 会话丢失"。
- 若在 Agent 先启动的正常联调中观察不到重建日志，说明未发生故障，生命周期静默维持。

### 9.5 验证方法

```bash
# 终端 1：先启动 Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 终端 2：后给 ESP32 上电，观察串口应一次进入 "ready, spinning"，无失败日志

# 故障注入：Ctrl-C 停掉 Agent 约数秒再重启
# 期望：串口出现一次 spin_some failed + 重建日志，话题随后恢复
ros2 topic hz /odom
```

> 引述版本说明：rclc 源码行号以当前工作区 `lib/micro_ros_platformio/build/mcu/src/rclc/rclc/src/rclc/` 内嵌副本为准（该副本即编译实际使用的版本）。
