# 上位机 WiFi 局域网配置笔记

> 整理日期：2026-09-06
> 适用平台：Ubuntu 24.04 + NetworkManager（Windows 仅附备查）
> 固件侧参数入口：`lib/RobotConfig/config.h`（`AGENT_IP_STR`、`AGENT_PORT=8888`、`WIFI_SSID`/`WIFI_PASS`）。

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
- SSID/密码须与 `lib/RobotConfig/config.h` 中 `WIFI_SSID` / `WIFI_PASS` 一致。

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
