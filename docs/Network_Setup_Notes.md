# 上位机 WiFi 局域网配置笔记

> 整理日期：2026-09-09 适用平台：Ubuntu 24.04 + NetworkManager；ROS 2 Jazzy 固件侧参数入口：`include/RobotConfig/config.h`（`AGENT_IP_STR`、`AGENT_PORT=8888`、`WIFI_ROLE_AP`、`WIFI_SSID`/`WIFI_PASS`、`WIFI_AP_SSID`/`WIFI_AP_PASS`/`WIFI_AP_CHANNEL`） 上位机 Agent 工作区：`~/Documents/ROS/YuXiangROS/Chap9/Robot_ws`（本文第 1、3 节引述） 拓扑方案原型固件：`test06_wifi`（`pio run -e test06_wifi`）

本笔记按五节组织：第 0 节是 TCP/IP 分层基础（一般性理论，与具体硬件解耦）；第 1 节是 micro-ROS 与 ESP32 网络配置工具（本工作区与 Agent 工作区源码引述）；第 2 节是当前网络拓扑方案（下位机 STA 为主、AP 备用，原型 test06）；第 3 节是完整使用流程（上位机 nmcli 配置与 Agent 启动，含首次启动特殊步骤）；第 4 节是排障记录（AP 模式首版崩溃的根因取证，以及崩溃条件的分解与跨固件对照）。

## 目录

- [上位机 WiFi 局域网配置笔记](#上位机-wifi-局域网配置笔记)
  - [目录](#目录)
  - [0. 计算机网络分层基础](#0-计算机网络分层基础)
    - [0.1 分层模型与封装解封](#01-分层模型与封装解封)
    - [0.2 物理层：频率、波长与 2.4/5 GHz 之争](#02-物理层频率波长与-245-ghz-之争)
    - [0.3 链路层：802.11 角色、SSID 与二层隔离](#03-链路层80211-角色ssid-与二层隔离)
    - [0.4 网络层：IPv4 地址、子网、路由与 NAT](#04-网络层ipv4-地址子网路由与-nat)
    - [0.5 传输层：端口号、UDP 与 TCP](#05-传输层端口号udp-与-tcp)
    - [0.6 收束：一次数据包的旅程](#06-收束一次数据包的旅程)
  - [1. micro-ROS 与 ESP32 的网络配置工具](#1-micro-ros-与-esp32-的网络配置工具)
    - [1.1 系统两侧与两条数据通道](#11-系统两侧与两条数据通道)
    - [1.2 固件侧配置工具](#12-固件侧配置工具)
    - [1.3 上位机侧工具](#13-上位机侧工具)
    - [1.4 第 0 节概念到两侧工具的映射](#14-第-0-节概念到两侧工具的映射)
  - [2. 当前网络拓扑方案：下位机 STA 为主、AP 备用](#2-当前网络拓扑方案下位机-sta-为主ap-备用)
    - [2.1 主备决策逻辑](#21-主备决策逻辑)
    - [2.2 外部 AP 的形态选择：为何以电脑热点为主载体](#22-外部-ap-的形态选择为何以电脑热点为主载体)
    - [2.3 两拓扑网络画像](#23-两拓扑网络画像)
    - [2.4 代码路径](#24-代码路径)
    - [2.5 局限](#25-局限)
  - [3. 使用指导：nmcli 建网与 Agent 启动](#3-使用指导nmcli-建网与-agent-启动)
    - [3.1 拓扑 A（主）：电脑热点 + 固件 STA](#31-拓扑-a主电脑热点--固件-sta)
    - [3.2 拓扑 B（备）：固件 AP 自组网 + 上位机 STA](#32-拓扑-b备固件-ap-自组网--上位机-sta)
    - [3.3 联调验证](#33-联调验证)
    - [3.4 排障速查（操作级，按层自底向上）](#34-排障速查操作级按层自底向上)
  - [4. 排障记录：AP 模式首版周期性复位](#4-排障记录ap-模式首版周期性复位)
    - [4.1 症状](#41-症状)
    - [4.2 复现与 bug 版本代码](#42-复现与-bug-版本代码)
    - [4.3 反汇编取证](#43-反汇编取证)
    - [4.4 根因链（结合 rclc/rmw 源码）](#44-根因链结合-rclcrmw-源码)
    - [4.5 修复与结论](#45-修复与结论)
    - [4.6 崩溃条件的分解与跨固件对照](#46-崩溃条件的分解与跨固件对照)
      - [4.6.1 三个必要条件](#461-三个必要条件)
      - [4.6.2 句柄容量非零时的条件性返回](#462-句柄容量非零时的条件性返回)
      - [4.6.3 跨固件对照](#463-跨固件对照)
      - [4.6.4 结论](#464-结论)

---

## 0. 计算机网络分层基础

> 分层的唯一目的：把任意一次通信故障归到某一层，诊断按 **"自底向上"** 顺序排查，因为上层故障会掩盖下层症状。

### 0.1 分层模型与封装解封

网络通信按"层"组织，每层只处理自己职责内的字段，对上层透明。TCP/IP 把 OSI 七层压成四层：

| 层     | 职责           | 典型实体（通用）                   |
| ------ | -------------- | ---------------------------------- |
| 物理层 | 比特与电磁信号 | 网线、光纤、射频天线、无线信道     |
| 链路层 | 帧、介质访问   | 以太网 MAC、802.11、交换机、AP/STA |
| 网络层 | 跨网寻址与路由 | IPv4/IPv6、子网掩码、网关、ARP     |
| 传输层 | 端到端会话     | TCP、UDP、端口号                   |
| 应用层 | 语义解释       | HTTP、DNS、SSH、自有应用协议       |

发送方逐层"加头"（封装），接收方逐层"去头"（解封）。以主机 A 向同局域网主机 B 发送一个 HTTP 请求为例：

```text
主机 A 侧封装                          主机 B 侧解封
[HTTP 载荷]                             [HTTP 载荷]
+TCP头   -> [TCP|HTTP]          ->      [TCP|HTTP]    -TCP头
+IP头    -> [IP|TCP|HTTP]               [IP|TCP|HTTP] -IP头
+以太网头 -> [帧|IP|TCP|HTTP]   ->      [帧|IP|TCP|HTTP] -帧头
```

诊断必须自底向上的原因：物理介质断开（物理层）时表现为"连不上"，绝不会先报"应用无响应"（应用层）。

### 0.2 物理层：频率、波长与 2.4/5 GHz 之争

电磁波以光速传播，波长与频率满足：

$$
\lambda = \frac{c}{f}
$$

WiFi 两个频段都落在 ISM 免许可频段，波长差异直接决定传播特性：

- 2.4 GHz：$\lambda \approx 12.5\ \text{cm}$
- 5 GHz：$\lambda \approx 6\ \text{cm}$

波长越长，绕射能力越强、穿墙衰减越小，故 2.4G 覆盖占优，5G 更"怕墙"。自由空间路径损耗（$d$ 单位 km、$f$ 单位 MHz）定量为：

$$
\text{FSPL(dB)} = 20\lg d + 20\lg f + 32.44
$$

同一距离下 5 GHz 比 2.4 GHz 多损耗约：

$$
20\lg\frac{5000}{2400} \approx 6.4\ \text{dB}
$$

2.4G 的代价是拥挤：WiFi 信道 1–13、蓝牙、Zigbee、微波炉（约 2.45 GHz）共享该频段，同频干扰大、有效速率天花板低。20 MHz 下 802.11n 单流约 65–72 Mbps，对传感器遥测这类每秒几 KB 的周期性载荷绰绰有余。

物理层一般性结论：设备能否接收某频段由射频前端硬件决定，不是软件选项。射频仅支持 2.4G 的终端对 5G 信号物理不可见，方案必须把无线接入点配置在 2.4G。

### 0.3 链路层：802.11 角色、SSID 与二层隔离

802.11 网络有两种角色：

- **AP（Access Point）**：基础设施模式中心，周期性广播 Beacon；
- **STA（Station）**：终端，经"扫描 -> 认证 -> 关联（Association）"挂到 AP 下。

SSID 是给人看的网络名；BSSID 是 AP 的 MAC 地址，机器按它区分设备。同名 SSID 可有多个 AP，这是"同名设备冲突"的来源。2.4G 信道中心频率：

$$
f_n = 2407 + 5n\ \text{MHz}
$$

相邻信道间隔仅 5 MHz，单信道占宽 20 MHz，只有相隔 5 个信道的 1/6/11 互不重叠。代入 $n=1,6,11$ 得中心频率 2412/2437/2462 MHz，这是换信道只考虑 1/6/11 的原因。

**二层隔离（AP isolation / client isolation）**：STA 之间的帧本应由 AP 按 MAC 二层转发；启用隔离后，AP 丢弃 client 到 client 的帧，只放行 client 与 AP 及上联口之间的帧。这是纯二层的横向通信禁令，与 IP 网段无关，所以默认隔离的手机热点里，同网段的两台终端也 ping 不通，改 IP 无济于事。

### 0.4 网络层：IPv4 地址、子网、路由与 NAT

IPv4 地址是 32 位无符号整数，习惯写成点分十进制：每 8 位一组、十进制、点分隔，取值范围 0.0.0.0 ~ 255.255.255.255。编址的核心思想是**分层**：地址高位是网络号，低位是主机号。路由器只按网络号转发、不感知网络内部的主机，路由表规模随网络数而非主机数增长。这是"跨网路由"与"同子网直达"在结构上的分界。

**子网掩码与 CIDR**：掩码是高位连续的 1，其位数即前缀长度，记法为"地址/前缀"。$192.168.1.10/24$ 表示前 24 位是网络号，等价于掩码 $255.255.255.0$。三类关键地址由掩码与地址按位运算得出：网络地址 $A \land M$（地址与掩码按位与），标识子网本身，不可分配；广播地址 $A \lor \lnot M$，发往它的包被子网内所有主机接收；介于两者之间的其余值才是可分配的主机地址。设前缀长度为 $p$、主机位 $h = 32 - p$，每子网可分配主机数为：

$$
2^h - 2
$$

减去的两个即网络地址与广播地址。举一反三：/24 有 $2^8 - 2 = 254$ 台，日常局域网最常见；/30 有 $2^2 - 2 = 2$ 台，只够两点直连，常用作路由器间链路段；/32 的 $h = 0$，不可作子网，表示"单台主机地址"。

编址历史：早期按首字节硬分 A/B/C 类（/8、/16、/24），粒度固定、地址浪费严重，已被 **CIDR**（无类别域间路由）取代。CIDR 允许任意前缀长度，地址空间按需切分，并把多条相邻路由聚合成一条，路由器表项随之减少。

常用保留地址段（RFC 1918 等，一般性知识）：

- 0.0.0.0/8：本网络。源地址 0.0.0.0 出现在主机"尚不知自身 IP"的启动期，如 DHCP 发现报文；目的 0.0.0.0 与掩码 /0 组成默认路由，指"除已知网络外一律走这里"；
- 127.0.0.0/8：环回。发给它的包不出本机，用于本机自测协议栈；
- 10.0.0.0/8、172.16.0.0/12、192.168.0.0/16：私有地址。公网路由不传播，任何组织可内部复用，主机数分别约 $2^{24}$、$2^{20}$、$2^{16}$；
- 169.254.0.0/16：链路本地。主机找不到 DHCP 服务端时自动配置此段地址（APIPA），同链路设备凭它仍能互通，但不跨网；
- 224.0.0.0/4：组播，一对多；255.255.255.255 是全网广播，不出本子网。

私有地址是"局域网互通不需要公网"的制度基础：同一私网内两台主机通信，包只在本地交换，不经过任何运营商网络。

**转发路径**：同子网内，发送方用 ARP 把目的 IP 解析成目的 MAC，帧二层直达；跨子网时，发送方把帧交给默认网关（路由器）的 MAC，路由器查路由表按"目的网络号 -> 下一跳"逐跳转发。每经一跳，IPv4 头部的 TTL（生存时间）减 1，减到 0 即丢弃并向源发 ICMP 超时报文——`traceroute` 正是靠递增 TTL 让沿途每跳都回一个超时来探测路径。`ping` 使用 ICMP Echo 请求/应答，能 ping 通只证明到该 IP 的网络层通路存在，不证明其上有任何服务在听。

**DHCP**：主机启动时广播"我要地址"，服务端从地址池取一个租约下发并记录期限，到期可续。地址池须避开网络地址与广播地址，故 /24 池的可用范围是去掉头尾后的一段。

**NAT**：IPv4 只有 32 位、总量约 43 亿，远少于接入设备数，私有地址靠 NAT 上网。内网大量主机共用一个或少量公网 IP：出站时路由器把内网源地址改写为自己的公网地址并记录映射，回包按映射还原。公网因此只能到达路由器本身，内网主机对外不可达——"外部无法主动连接内网设备"是 NAT 的结构性结果，不是安全配置。映射表区分不同内网主机的关键字段是源端口，端口的具体语义见 0.5 节。

### 0.5 传输层：端口号、UDP 与 TCP

IP 负责把包送到主机，端口负责把数据交给主机上的哪个进程。端口是 16 位无符号数，取值 0 ~ 65535，IANA 分三段：0–1023 知名端口，通常需特权绑定，如 80（HTTP）、443（HTTPS）、22（SSH）、53（DNS）；1024–49151 注册端口，如 3306（MySQL）、5432（PostgreSQL）；49152–65535 动态端口，客户端出站连接的源端口通常取自这里。

一个 socket 是"IP + 端口"二元组。TCP 连接由四元组唯一确定：源 IP、源端口、目的 IP、目的端口，改任一元即为不同连接；同一服务端口可承载海量并发连接，靠的就是各客户端源端口不同。UDP 无连接，报文只按"目的 IP:端口"投递。

**UDP（用户数据报协议）**：首部固定 8 字节，含源端口、目的端口、长度、校验和各 2 字节。无连接，发送即走，无握手、无状态；不重传、不保序、不防重复，可靠性义务全部转给上层。校验和在 IPv4 中可选（填 0 表示不计算），在 IPv6 中强制。UDP 保留报文边界，一次发送对应一次接收。适用画像：

- 查询-响应：如 DNS，一个请求一个应答，等不到就重发，无需连接；
- 实时流：音视频帧过期即弃，重传旧帧反而有害；
- 周期性遥测：丢一两帧可容忍，状态靠下一帧刷新；
- 在 UDP 上自建可靠层：QUIC（HTTP/3 的底层传输）把握手与重传搬进 UDP，换取连接建立延迟与队头阻塞的收益。

**TCP（传输控制协议）**：面向连接、面向字节流、可靠。首部最小 20 字节。可靠性由一组机制组合保证：

- 序号与确认：每个字节一个序号，接收方以 ACK 告知已连续收到的位置，发送方据此识别缺口；
- 超时重传：未在时限内确认的段重发，超时值随实测往返时间动态调整；
- 流量控制：接收方把剩余缓冲容量（窗口 rwnd）随 ACK 带回，发送窗口不得超过 rwnd，防止淹没慢接收方；
- 拥塞控制：发送方另持拥塞窗口 cwnd，慢启动从小值开始每轮倍增，丢包判拥塞则减半进入拥塞避免（每轮线性 +1），快重传/快恢复加速恢复。网络丢包率无法先验，只能靠探测逼近可用带宽，故拥塞控制是必要机制而非附加。

连接生命周期以状态机概括。建立阶段三次握手（SYN -> SYN+ACK -> ACK），同时交换初始序号，防止旧连接的迟到段误入新连接；数据阶段双工独立推进序号；关闭阶段四次挥手（FIN -> ACK -> FIN -> ACK），先收 FIN 的一方进入半关闭（只能发不能收），主动关闭方最后停留 TIME_WAIT，等迟到段老化后再释放四元组，避免复用的新连接收到旧段。

TCP 是字节流，不保留报文边界：一次发送可能被拆进多个段，多次发送也可能在一次接收中拼出，应用层必须自定消息边界（长度前缀、分隔符），否则解析错位——"粘包"由此而来。适用不可丢的连续流：文件传输、远程终端、数据库同步。与 UDP 对比：

| 维度          | UDP                     | TCP                        |
| ------------- | ----------------------- | -------------------------- |
| 连接          | 无                      | 三次握手建立，四次挥手关闭 |
| 可靠性        | 不保证                  | 序号/ACK/超时重传          |
| 顺序          | 不保证                  | 字节流有序                 |
| 边界          | 保留报文边界            | 字节流，应用自定界         |
| 流控/拥塞控制 | 无                      | rwnd 流控，cwnd 拥塞控制   |
| 首部          | 8 字节                  | 最小 20 字节               |
| 适用          | 查询-响应、实时流、遥测 | 文件、终端、事务           |

服务端进程接收前须先绑定监听地址：绑 0.0.0.0（通配）表示监听本机所有接口，任何接口进来的入站包都能命中；只绑 127.0.0.1 则仅本机进程能连。入站无响应时先分两问：包到没到（网络层，ping 验证）与进程有没有在听（传输层，端口是否监听）——"能 ping 通却连不上"是后者的典型症状。

### 0.6 收束：一次数据包的旅程

把 0.1–0.5 串成一个实例：主机 A 的浏览器向同网段主机 B 的 Web 服务请求一页。

```text
[主机 A 应用] 浏览器发起 HTTP GET
   | TCP 连接 80 端口                 (0.5 端口分离)
[TCP/IP 协议栈] 加 TCP/IP 头           (0.1 封装)
[网卡已关联 2.4G AP]                   (0.3 关联/SSID)
   | 空中 2.4GHz 帧                   (0.2 频段)
[AP 转发] 二层按 MAC 转给主机 B
[主机 B 网卡] 解帧 -> 内核 IP 栈
   | 目的 IP 命中本机                  (0.4 同子网直达)
[Web 服务 80 端口 socket] 解包 -> 响应 (0.1 应用层)
```

任一步断掉，症状各有归属：断在物理层 -> 搜不到；链路层 -> 关联失败/被隔离；网络层 -> ping 不通；传输层 -> 无会话；以上全通而应用无数据 -> 查应用本身。诊断永远先问"症状在哪一层"，再去动那一层的旋钮。

---

## 1. micro-ROS 与 ESP32 的网络配置工具

本节把两侧可配置、可编程的网络组件列全，并给出工作区内的源码级引述：固件侧在 `include/`、`lib/micro_ros_platformio/` 与 `platformio.ini`，上位机侧在 Agent 工作区 `~/Documents/ROS/YuXiangROS/Chap9/Robot_ws`。

### 1.1 系统两侧与两条数据通道

通信两端是 ROS 2 上位机（Agent）与 ESP32 下位机（micro-ROS 客户端），会话协议为 XRCE（eXtremely Resource Constrained Environments），承载于 UDP。本项目的微控制器网络栈同时承载两条通道：

| 通道 | 端口 | 协议 | 用途             | 下位机侧          | 上位机侧                     |
| ---- | ---- | ---- | ---------------- | ----------------- | ---------------------------- |
| A    | 8888 | UDP  | micro-ROS XRCE   | `AGENT_PORT`      | `micro_ros_agent udp4`       |
| B    | 8889 | TCP  | 激光雷达数据透传 | `BRIDGE_TCP_PORT` | `ros_serial2wifi tcp_server` |

通道 A 的载荷是周期性话题，丢失一两帧表现为单周期跳变，故 UDP 足够；通道 B 是连续字节流，不可丢，故用 TCP。端口号在两个工作区成对出现，是"端到端对齐"的检查点：`config.h` 与上位机 Agent、`config.h` 与 `ros_serial2wifi` 必须一致。

### 1.2 固件侧配置工具

参数入口 `include/RobotConfig/config.h`。本文件由 `config.example.h` 复制而来，被 .gitignore 忽略（文件头注释 `config.h:3-7`），本地改部署环境只动它。

网络相关参数集中在 `config.h:65-92`：

- `WIFI_ROLE_AP`（`config.h:70-72`）：用 `#ifndef` 包裹，默认 `0`，支持编译期 `-DWIFI_ROLE_AP=1` 覆盖，便于不改文件做 AP 试验；
- `AGENT_IP_STR`：运行 Agent 的主机地址。`WIFI_ROLE_AP==1` 分支（`config.h:76-81`）取 `192.168.4.100`，并连同 `WIFI_AP_SSID`/`WIFI_AP_PASS`/`WIFI_AP_CHANNEL` 组成 AP 组；`#else` 分支（`config.h:82-85`）取 `10.42.0.1`；
- `AGENT_PORT=8888`（`config.h:86`）；
- `WIFI_SSID`/`WIFI_PASS`（`config.h:91-92`）：凭据须为可写 `char` 数组而非 `constexpr`，因 `set_microros_wifi_transports` 接口要求 `char*`（注释 `config.h:89`）。

网络自举入口 `include/NetBoot/net_boot.h`。`wifi_role_boot(agent_ip)`（`net_boot.h:52-69`）先由 `AGENT_IP_STR` 解析出 Agent 地址，再按 `WIFI_ROLE_AP` 分流：

```cpp
#if WIFI_ROLE_AP == 1
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
    ...
    set_microros_wifi_ap_transports(agent_ip, AGENT_PORT);
#else
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT);
#endif
```

transport 注册层由 `micro_ros_platformio` 库提供：

- 构建开关 `board_microros_transport = wifi`（`platformio.ini` 主环境第 53 行、test06 第 113 行）选择 WiFi transport 实现并注入预编译 `libmicroros`；
- STA 官方路径 `set_microros_wifi_transports`（`platform_code/arduino/wifi/micro_ros_transport.h:9-28`）：先 `WiFi.begin` 阻塞等待关联（10-14 行），再 `rmw_uros_set_custom_transport` 注册四个回调（20-27 行）；
- UDP 收发由四个回调实现（`platform_code/arduino/wifi/micro_ros_transport.cpp`）：`platformio_transport_open` 绑本地端口（15-19 行）、`platformio_transport_write` 经 `beginPacket`/`endPacket` 发往 locator（27-42 行）、`platformio_transport_read` 轮询 `parsePacket`（44-60 行）；
- AP 自组网路径 `set_microros_wifi_ap_transports`（`net_boot.h:32-45`）与官方函数同构：差异是它假设 AP 已由调用方开启，直接复用同一批 transport 回调注册，不调用 `WiFi.begin`（注释 `net_boot.h:25-27`）。

### 1.3 上位机侧工具

micro-ROS Agent 是 ROS 2 包 `micro_ros_agent`（仓库名 micro-ROS-Agent，见 `package.xml`）。单条命令启动 UDP 会话：

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

其中 `udp4` 指定以 UDP over IPv4 承载 XRCE 会话，`--port 8888` 与固件 `AGENT_PORT` 对齐。

本地 Agent 工作区 `~/Documents/ROS/YuXiangROS/Chap9/Robot_ws` 的 `src/` 含 `micro-ROS-Agent`、`robot_bringup`、`ros_serial2wifi`、`ydlidar_ros2` 等包。聚合启动文件 `robot_bringup/launch/bringup.launch.py` 把两条通道的进程集中声明：

- micro-ROS Agent 节点（`bringup.launch.py:28-37`）：`executable="micro_ros_agent"`，`arguments=["udp4", "--port", "8888"]`；
- 雷达桥节点（`bringup.launch.py:39-44`）：`executable="tcp_server"`，`parameters=[{"serial_port": "/tmp/tty_laser"}]`。

雷达桥实现 `ros_serial2wifi/ros_serial2wifi/tcpserver.py` 声明参数 `tcp_port=8889`、`serial_port=/tmp/laserport`（17-18 行），绑定 `0.0.0.0`（27 行）并以伪终端符号链接暴露串口（31-33 行）。launch 覆盖 `serial_port` 为 `/tmp/tty_laser`。

### 1.4 第 0 节概念到两侧工具的映射

| 层     | 概念落点                  | 固件侧实现            | 上位机侧实现            |
| ------ | ------------------------- | --------------------- | ----------------------- |
| 链路层 | AP/STA 关联               | `WiFi.begin`/`softAP` | nmcli 建连接（第 3 节） |
| 网络层 | 同子网直达（网段对齐）    | `AGENT_IP_STR` 静态值 | 热点网段/静态 IP        |
| 传输层 | 监听 0.0.0.0 收无线口入站 | `udp_client.begin`    | Agent 默认监听所有接口  |
| 应用层 | XRCE 语义                 | `rclc`/`rmw` 栈       | `micro_ros_agent`       |

工具齐备后，第 2 节给出"按哪种拓扑连线"，第 3 节给出"怎么一步步操作"。

---

## 2. 当前网络拓扑方案：下位机 STA 为主、AP 备用

拓扑方案与原型均出自 test06 的专门工作：`env:test06_wifi` 固件同时实现 STA 接入与 AP 自组网两条路径，用于验证本方案。方案决策与代码路径见下文。

### 2.1 主备决策逻辑

- 主路径（默认，`WIFI_ROLE_AP=0`）：ESP32 作 STA 接入既有的 2.4G 无线接入点。理由：部署现场通常有可用 AP；该路径沿用官方 `set_microros_wifi_transports`，与改造前完全一致，属纯增量（注释 `net_boot.h:5-6`），改动面最小。
- 备用路径（`WIFI_ROLE_AP=1`）：ESP32 自开 SoftAP 自组网，上位机作 STA 连入。理由：STA 依赖外部基础设施存在，现场无路由/热点或须脱离一切外部设备时，主路径失效；自组网使通信闭环于两机之间。
- 主备切换是编译期宏（`config.h:69-72` 的 `#ifndef` 支持命令行覆盖），不是运行时行为；运行时的会话中断由固件按重建周期自动恢复（见 2.4）。

### 2.2 外部 AP 的形态选择：为何以电脑热点为主载体

主路径的"外部 AP"有普通路由器、手机热点、电脑网卡热点三种形态：

| 形态       | ESP32 能否连   | 隔离/认证卡点                | 本方案可用性       |
| ---------- | -------------- | ---------------------------- | ------------------ |
| 普通路由器 | 只连 2.4G SSID | portal 认证；AP 隔离多数可关 | 备选，需可控路由器 |
| 手机热点   | 需显式落 2.4G  | 客户端隔离默认开且常无开关   | 不可作主           |
| 电脑热点   | 需网卡支持 AP  | 网卡/驱动须支持 AP 角色      | 默认主载体         |

结论：

1. 手机热点默认启用二层客户端隔离（第 0 节 0.3），同网段互 ping 不通，系统层无法关，故不可作主；
2. 电脑热点由 NetworkManager 托管，`shared` 模式固定网关 `10.42.0.1`、DHCP 分发 `10.42.0.x`，恰好等于 STA 分支 `AGENT_IP_STR=10.42.0.1`（`config.h:84`），固件零改动，Agent 与 ESP32 一跳直达；
3. 电脑热点成立依赖"网卡硬件 + 驱动 + 固件"三者允许 AP 角色，第 3 节首次启动步骤先做能力检查。

### 2.3 两拓扑网络画像

| 项             | 拓扑 A（主）：STA 接入                 | 拓扑 B（备）：AP 自组网                |
| -------------- | -------------------------------------- | -------------------------------------- |
| `WIFI_ROLE_AP` | `0`（默认）                            | `1`（试验）                            |
| ESP32 角色     | STA，连外部 2.4G AP                    | SoftAP，自开 2.4G                      |
| ESP32 地址     | DHCP 租约 `10.42.0.x`（电脑热点）      | 固定 `192.168.4.1`，自带 DHCP          |
| Agent 宿主     | 外部 AP 侧（电脑热点网关 `10.42.0.1`） | 上位机 STA，须手动静态 `192.168.4.100` |
| `AGENT_IP_STR` | `10.42.0.1`                            | `192.168.4.100`                        |
| 频段           | 外部 AP 落 2.4G（`band bg`）           | SoftAP 单射频即 2.4G                   |
| 出公网         | 经热点 NAT                             | 默认无                                 |

共同前提：ESP32-S3 射频前端仅支持 2.4G（802.11 b/g/n），5G 对其物理不可见（第 0 节 0.2 的一般性结论落到本硬件）；因此无论哪种拓扑，无线侧都必须落在 2.4G。

### 2.4 代码路径

选型只改一处宏 `WIFI_ROLE_AP`（`config.h:70-72`），地址组随宏联动（`config.h:76-85`）。固件侧分流在 `net_boot.h` 的 `wifi_role_boot`（`net_boot.h:52-69`）：

```cpp
static inline void wifi_role_boot(IPAddress& agent_ip) {
    agent_ip.fromString(AGENT_IP_STR);

#if WIFI_ROLE_AP == 1
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
    Serial.printf("[WiFi] softAP \"%s\" ready, IP=%s, agent %s:%u\n", ...);
    set_microros_wifi_ap_transports(agent_ip, AGENT_PORT);
#else
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, agent_ip, AGENT_PORT);
#endif
}
```

- STA 分支：官方函数内 `WiFi.begin` 阻塞等待关联成功后注册 transport（`micro_ros_transport.h:10-14`）；
- AP 分支：`WiFi.softAP` 开启后走同构纯 UDP 注册，跳过 `WiFi.begin`（`net_boot.h:32-45`）。

固件任务骨架在 `src/tests/test06_wifi/main.cpp`：`setup` 中 `xTaskCreate(micro_ros_task, ...)`（75 行）；任务内 `wifi_role_boot` 只执行一次（110 行），随后进入"初始化 -> spin -> 释放 -> 重建"循环（118-208 行），任务永不返回（约束依据见注释 91-97）。`EXECUTOR_HANDLES=1`（21 行）满足 rclc"句柄容量须大于等于 1"的契约（注释 18-20）。

### 2.5 局限

- Agent 地址编译期已知且单一：拓扑 B 下上位机静态 IP 必须等于 `AGENT_IP_STR`，不一致时 ESP32 可达但 Agent 收不到，现象同"无会话"；
- SoftAP 无外网上联，拓扑 B 默认不出公网；
- 两拓扑切换需重新编译烧录（宏是编译期常量）。

---

## 3. 使用指导：nmcli 建网与 Agent 启动

本节按拓扑 A（主，STA 接入）与拓扑 B（备，AP 自组网）分别给出完整 shell 流程。每步注释标注执行时机：`[首次启动]` 表示只在首次部署执行一次的步骤，其余为每次上电的常规步骤。

共用准备（每会话一次）：

```bash
# ROS 2 环境与本地 Agent 工作区 (Jazzy)
source /opt/ros/jazzy/setup.bash
source ~/Documents/ROS/YuXiangROS/Chap9/Robot_ws/install/setup.bash

# 确认网卡设备名 (示例 wlp14s0, 实际以输出为准)
nmcli device status
```

### 3.1 拓扑 A（主）：电脑热点 + 固件 STA

固件侧为默认值：`WIFI_ROLE_AP=0`、`AGENT_IP_STR="10.42.0.1"`（`config.h:70-85`），`WIFI_SSID`/`WIFI_PASS` 与热点一致（`config.h:91-92`）。

上位机配置（拓扑 A）：

```bash
# [首次启动] 1. 确认网卡支持 AP 模式, 期望含 "AP"
iw list | sed -n '/Supported interface modes/,/^$/p'

# [首次启动] 2. 创建热点连接 (band bg 锁 2.4G; channel 取 1/6/11 中不拥挤者)
#             首次执行生成名为 Hotspot 的连接 profile, 此后不必重复本步
nmcli device wifi hotspot ifname wlp14s0 ssid <"YOUR_HOTPOT_SSID"> \
  password <"YOUR_HOTPOT_PASSWORD"> band bg channel 6

# [每次] 3. 开热点 (profile 已存在, 只拉起)
nmcli connection up Hotspot

# [每次] 4. 验证热点自身 IP 应为 10.42.0.1/24 (与固件 AGENT_IP_STR 对齐)
nmcli -f IP4.ADDRESS,IP4.GATEWAY device show wlp14s0
```

上位机启动 Agent（拓扑 A）：

```bash
# [每次] 单独启动 micro-ROS Agent (绑 0.0.0.0, 收 ESP32 无线口入站)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 或: 全量聚合启动 (含雷达桥 ros_serial2wifi 与 ydlidar, 见 1.3)
# ros2 launch robot_bringup bringup.launch.py
```

下位机（拓扑 A）：

```bash
# [首次启动] 确认 config.h 为 STA 分支 (默认即此), 编译烧录
pio run -e test06_wifi -t upload
```

### 3.2 拓扑 B（备）：固件 AP 自组网 + 上位机 STA

固件侧：`WIFI_ROLE_AP=1`、`AGENT_IP_STR="192.168.4.100"`、`WIFI_AP_SSID="fishbot-ap"`、`WIFI_AP_PASS="fishbot123"`、`WIFI_AP_CHANNEL=6`（`config.h:76-81`）。ESP32 自开 SoftAP（自身 `192.168.4.1` 并内置 DHCP）。

关键点（对应本节最重要的特殊步骤）：上位机**连入热点之前**必须预先建好 profile 并配静态 IP。原因：Agent 地址 `192.168.4.100` 是编译进固件的常量，若上位机走 DHCP，SoftAP 会从 `.2` 起分配地址，固件发往 `.100` 的包无人接收，表现为"连上了但无会话"。故静态 IP 必须等于 `AGENT_IP_STR`。

上位机配置（拓扑 B）：

```bash
# [首次启动] 1. 连下位机热点之前: 建 STA profile
#             autoconnect no: 备用拓扑手动拉起, 避免开机抢连
nmcli connection add type wifi ifname wlp14s0 con-name AgentSTA \
  autoconnect no ssid "fishbot-ap"

# [首次启动] 2. WPA2 凭据 (须与 config.h WIFI_AP_PASS 一致)
nmcli connection modify AgentSTA wifi-sec.key-mgmt wpa-psk
nmcli connection modify AgentSTA wifi-sec.psk "fishbot123"

# [首次启动] 3. 静态 IP 192.168.4.100/24, 必须等于固件 AGENT_IP_STR
nmcli connection modify AgentSTA ipv4.method manual \
  ipv4.addresses 192.168.4.100/24

# [首次启动] 4. 上述 1-3 完成后才首次连入 (本步也是 [每次] 的拉起命令)
nmcli connection up AgentSTA
```

上位机启动 Agent（拓扑 B，命令同拓扑 A，本机 IP 现为 `192.168.4.100`）：

```bash
# [每次] 拉起备用连接 (profile 已存在, 免去 1-3)
nmcli connection up AgentSTA

# [每次] 验证: 本机 IP 应为 192.168.4.100, 且能 ping 通 ESP32 网关
ip addr show wlp14s0
ping -c 3 192.168.4.1

# [每次] 启动 Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

下位机（拓扑 B）：

```bash
# [首次启动] 方式一: 直接编辑 config.h, 把 WIFI_ROLE_AP 置 1 后编译烧录
# [首次启动] 方式二: 不改文件, 用环境变量注入宏 (对应 config.h:69 的 #ifndef)
PLATFORMIO_BUILD_FLAGS="-DWIFI_ROLE_AP=1" pio run -e test06_wifi -t upload
```

### 3.3 联调验证

以 test06 为验证对象时，固件不发布话题，验证看串口日志：

```text
[WiFi] softAP "fishbot-ap" ready, IP=192.168.4.1, agent 192.168.4.100:8888   (拓扑 B)
[WiFi] connected, local IP=10.42.0.x                                          (拓扑 A)
[micro_ros] node "fishbot_motion_control" ready, spinning
```

上列串口日志取自 test06，其节点名为 `fishbot_motion_control`（`NODE_NAME`）；
主固件与 test13 的节点名为 `fishbot_balance`（各固件本地定义的 `BALANCE_NODE_NAME`）。
若换烧发布/订阅型固件（主固件、test08、test13），做话题级验证：

```bash
ros2 topic hz /odom                                                   # 主固件: 期望约 20 Hz
ros2 topic pub /balance_enable std_msgs/msg/Bool "{data: true}" -r 5   # 主固件 / test13
```

注意主固件与 test13 只在武装（`/balance_enable` 为 true）后输出 PWM，
`/cmd_vel` 本身不触发运动。

### 3.4 排障速查（操作级，按层自底向上）

| 现象                     | 先查什么          | 常用命令/手段                             |
| ------------------------ | ----------------- | ----------------------------------------- |
| 搜不到 SSID              | 频段（热点在 5G） | 确认 `band bg`；iPhone 开最大兼容性       |
| 拿不到 IP、ping 不通网关 | 网段/隔离         | 查 `IP4.ADDRESS`；拓扑 B 查静态 IP        |
| ping 通但 Agent 无会话   | 端口/地址对齐     | `AGENT_IP_STR` 是否等于本机 IP；放行 8888 |
| 有客户端但数据断         | 同频干扰/信号     | 换信道 1/6/11；查同名 SSID                |
| 热点起不来               | 网卡无 AP 能力    | `iw list` 查 AP；换网卡                   |

研发级根因取证（如第 4 节的 AP 崩溃）在源码层排障，不在本表范围。

---

## 4. 排障记录：AP 模式首版周期性复位

本故障发生于拓扑 B（AP 备用分支，第 2 节）的首版实现：提交 `eb9a540` 引入 STA/AP 双模自举，`67a314b` 修复。修复后的参考实现即当前 `src/tests/test06_wifi/main.cpp`。以下取证全部来自本仓库内嵌库源码、git 历史与本地编译产物。

### 4.1 症状

AP 模式（`WIFI_ROLE_AP=1`）下，串口周期性打印 UDP 发送失败，随后复位，与是否连接 Agent 无关：

```text
[WiFi] softAP "fishbot-ap" ready, IP=192.168.4.1, agent 192.168.4.100:8888
endPacket(): could not send data: 12
...
endPacket(): could not send data: 12
Guru Meditation Error: IllegalInstruction
PC     : 0x42002da8
Backtrace: 0x42002da5
```

`endPacket(): could not send data: 12` 约每 2 秒一次，持续约 11.2 秒后触发 `IllegalInstruction`，复位后重复同一周期。

### 4.2 复现与 bug 版本代码

用 `git worktree` 隔离出问题提交 `eb9a540`，本地编译生成 ELF 供反汇编：

```bash
git worktree add /tmp/fishbot_bug_eb9a540 eb9a540
cd /tmp/fishbot_bug_eb9a540
cp config.example.h include/RobotConfig/config.h
sed -i 's/#define WIFI_ROLE_AP 0/#define WIFI_ROLE_AP 1/' include/RobotConfig/config.h
pio run -e test06_wifi
```

该版本 `micro_ros_task` 的关键代码（`git show eb9a540:src/tests/test06_wifi/main.cpp`）：

```cpp
constexpr uint8_t EXECUTOR_HANDLES = 0;   // 句柄容量为 0

// 三处初始化返回值均不检查
rclc_support_init(&support, 0, NULL, &allocator);
rclc_node_init_default(&node, NODE_NAME, "", &support);
unsigned int num_handles = EXECUTOR_HANDLES;
rclc_executor_init(&executor, &support.context, num_handles, &allocator);

rclc_executor_spin(&executor);   // 末尾直接 spin, 函数体随后 return
```

### 4.3 反汇编取证

对 `firmware.elf` 反汇编，`micro_ros_task`（符号 `_ZN12_GLOBAL__N_114micro_ros_taskEPv`）起址 `0x42002e50`，函数体长度 `0x17e`。其尾部调用序列：

```text
42002f6e  call8  4206ad90 <rclc_support_init>
42002f7f  call8  4206af1c <rclc_node_init_default>
42002f91  call8  4206a7f8 <rclc_executor_init>
42002f99  call8  4206ad3c <rclc_executor_spin>
42002fcc  retw.n
```

三点证据：

1. 四个调用之间没有对返回值的分支判断，`rclc_executor_spin` 之后直接 `retw.n`，即任务函数在 spin 返回后立即越过函数尾返回；
2. `rclc_executor_init` 调用前 `a12` 被赋 `0`（`movi.n a12, 0`），对应 `EXECUTOR_HANDLES=0`；
3. 崩溃地址 `0x42002da8` 与回溯地址 `0x42002da5` 均低于 `micro_ros_task` 起址 `0x42002e50`，落在 `.flash.text` 段内 `_stext` 之后的常量池区域（该处反汇编显示数据字 `24 c8 03 60` 而非指令）；且 `0x42002da5` 为奇数，Xtensa 指令须 2 字节对齐，奇数取指即触发 `IllegalInstruction`。

结论：崩溃 PC 指向数据区而非代码区，符合"从失效任务栈取指"的机制。

### 4.4 根因链（结合 rclc/rmw 源码）

第一步，support 初始化失败但未检查。`rclc_support_init` 转调 `rclc_support_init_with_options`，其内部 `rcl_init` 失败即带错误码返回（内嵌 `rclc/.../init.c:69-74`）：

```c
support->context = rcl_get_zero_initialized_context();
rc = rcl_init(argc, argv, init_options, &support->context);
if (rc != RCL_RET_OK) {
  PRINT_RCLC_ERROR(rclc_init, rcl_init);
  return rc;
}
```

AP 模式下 Agent 不可达，`rcl_init` 经 rmw 层建 XRCE 会话失败，返回错误；bug 版本不检查，继续执行。

第二步，节点初始化路径内阻塞重试发送。`rclc_node_init_default` 内部经 `rmw_create_node` 调静态函数 `create_node`，其中 `run_xrce_session` 在 Agent 不可达时阻塞重试（内嵌 `rmw-microxrcedds/.../rmw_node.c:108-110`）：

```c
if (!run_xrce_session(
    custom_node->context, custom_node->context->creation_stream, participant_req,
    custom_node->context->creation_timeout))
```

`run_xrce_session` 每次发送经 `platformio_transport_write`（`micro_ros_transport.cpp:27-42`）调用 `udp_client.endPacket()`，其底层 `sendto` 失败后由 Arduino core `WiFiUdp.cpp:183-187` 打印 `could not send data: %d`（errno 12）。这正是症状中周期性日志的来源。

第三步，executor 初始化直接返回且未清零。`rclc_executor_init`（内嵌 `rclc/.../executor.c:108-111`）：

```c
if (number_of_handles == 0) {
  RCL_SET_ERROR_MSG("number_of_handles is 0. Must be larger or equal to 1");
  return RCL_RET_INVALID_ARGUMENT;
}
```

该返回发生在 `executor.c:114` 的整体清零赋值之前，故 `EXECUTOR_HANDLES=0` 时 executor 保持零初始化，其 `context` 字段为 NULL。

第四步，spin 因 context 无效立即返回。`rclc_executor_spin_some`（`executor.c:1808-1811`）：

```c
if (!rcl_context_is_valid(executor->context)) {
  PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_context_not_valid);
  return RCL_RET_ERROR;
}
```

而 `rclc_executor_spin`（`executor.c:1982-1987`）是 `while(true)` 调 `spin_some`，遇到非 OK/非 TIMEOUT 返回值即 return：

```c
while (true) {
  ret = rclc_executor_spin_some(executor, executor->timeout_ns);
  if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
    RCL_SET_ERROR_MSG("rclc_executor_spin_some error");
    return ret;
  }
}
```

于是 `rclc_executor_spin` 立即返回，`micro_ros_task` 越过函数尾（反汇编中的 `retw.n`）。任务函数 return 后，FreeRTOS 调度器从已失效的任务栈指针继续取指，PC 落入常量池数据区（`0x42002da8`），回溯地址 `0x42002da5` 奇数不对齐，触发 `IllegalInstruction`，复位后重复同一路径，形成约 11.2 秒周期的复位。

### 4.5 修复与结论

修复提交（`67a314b`）将 `EXECUTOR_HANDLES` 改为最小合法值 1，并为四步初始化补全返回值检查、逆序释放、`spin_some` 加 `delay` 的生命周期模型（当前 `src/tests/test06_wifi/main.cpp:118-208`），任务函数永不 return。

结论：

1. 崩溃根因是"初始化返回值不检查 + 句柄容量为 0"两处叠加：对象未就绪时任务函数越过函数尾返回，触发 FreeRTOS 失效栈取指；
2. `endPacket(): could not send data: 12` 是伴随症状而非根因，由 `run_xrce_session` 在 Agent 不可达时阻塞重试发送所致，与是否连接 Agent 无关；
3. `xTaskCreate` 任务函数永不 return 是基本约束，失败路径只能延时重建，不能越过函数尾。

### 4.6 崩溃条件的分解与跨固件对照

第 4.4 节的根因链可分解为三个相互独立的条件。判据均取自此仓库内嵌库源码，可逐行核对。

#### 4.6.1 三个必要条件

| 条件  | 含义                                         | 源码判据                                                                                                                                                                                                                                                                                                     |
| ----- | -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| $C_1$ | executor 句柄容量为 0，executor 从未被初始化 | `executor.c:108-111` 对 `number_of_handles == 0` 直接 `return RCL_RET_INVALID_ARGUMENT`；该返回早于 `executor.c:114-115` 的 `(*executor) = rclc_executor_get_zero_initialized_executor(); executor->context = context;`，故 `executor.context` 保持 `NULL`                                                   |
| $C_2$ | 初始化返回值不检查，控制流仍能到达 `spin`    | `rclc/init.c:69-74` 仅把 `rcl_init` 的错误码上抛，调用方不接即静默；`rcl/init.c:222-229` 在 `rmw_init` 失败时 `goto fail`；`rmw_init.c:308-313` 在 `uxr_create_session` 失败时置 `context->impl = NULL` 并返回 `RMW_RET_ERROR`；`rcl/context.c:105-109` 的 `__cleanup_context` 把 `instance_id_storage` 归零 |
| $C_3$ | `spin` 返回后任务函数可达函数尾              | 任务函数末尾无可退出的外层循环；`executor.c:1982-1988` 的 `rclc_executor_spin` 为 `while(true)` 调 `spin_some`，遇非 OK/非 TIMEOUT 即 `return`                                                                                                                                                               |

三者各自的作用：

- $C_1$ 使 `spin` 无条件立即返回。`executor.context == NULL` 时，`executor.c:1808-1811` 的 `rcl_context_is_valid(NULL)` 为假，`spin_some` 直接返回 `RCL_RET_ERROR`，与 support 是否初始化成功无关；
- $C_2$ 使控制流能到达 `spin`。任一步失败即 `return` 时，链路在到达 `spin` 之前终止；
- $C_3$ 把"`spin` 返回"升级为"任务函数返回"。只有越过函数尾，才触发第 4.3 节的失效栈取指。

因此该崩溃的充分条件链为：

$$
\text{crash} = C_1 \wedge C_2 \wedge C_3
$$

三项缺一，链路即断。这解释了同一缺陷类在不同固件上可见性的差异。

#### 4.6.2 句柄容量非零时的条件性返回

$C_1$ 不是 `spin` 返回的唯一来源。`number_of_handles >= 1` 时 `executor.context = &support.context`，`spin_some` 在 `rcl_wait` 上阻塞后返回 `RCL_RET_OK`（`executor.c:1954-1955` 以 `RCLC_UNUSED(rc)` 丢弃 `rcl_wait` 的返回值，稳态返回值来自其后的调度函数），外层 `while(true)` 因此不退出。此时 `spin` 是否返回取决于 support 是否初始化成功：

- support 成功：`rcl_context_is_valid`（`rcl/context.c:90-94`，判据为 `instance_id_storage != 0`）为真，`spin` 永不返回；
- support 失败：`instance_id_storage` 已被 `__cleanup_context` 归零，`spin` 每轮立即返回 `RCL_RET_ERROR`，此时 $C_3$ 若成立即崩溃。

句柄容量非零只是把"必然返回"降级为"条件返回"，条件为 support 初始化失败，并未消除 $C_3$。

#### 4.6.3 跨固件对照

本工程使用 micro-ROS 的固件共 5 个，即 `platformio.ini` 中配置 `board_microros_transport = wifi` 的 5 个环境：`esp32-s3-devkitc-1`（主固件）、`test06_wifi`、`test07_Subscription`、`test08_Publisher`、`test13_balance`。

| 固件                            | 句柄容量 | $C_1$  | $C_2$  | $C_3$  | 结果                         |
| ------------------------------- | -------- | ------ | ------ | ------ | ---------------------------- |
| `test06_wifi` 首版（`eb9a540`） | 0        | 成立   | 成立   | 成立   | 崩溃（第 4.1 节）            |
| `test06_wifi` 现状              | 1        | 不成立 | 不成立 | 不成立 | 安全                         |
| `test07_Subscription`           | 1        | 不成立 | 成立   | 成立   | 潜在崩溃                     |
| `test08_Publisher`              | 2        | 不成立 | 成立   | 成立   | 潜在崩溃（另有一处偶然屏蔽） |
| `test13_balance`                | 2        | 不成立 | 成立   | 不成立 | 结构不可达                   |
| `esp32-s3-devkitc-1`            | 3        | 不成立 | 成立   | 不成立 | 结构不可达                   |

$C_3$ 的判据是任务函数内是否存在不可退出的外层循环：

- `test13_balance/main.cpp:469-482` 与 `src/main.cpp:666-679` 的 `for (;;)` 内无 `break`、无 `return`，函数在结构上到不了函数尾；
- `test07_Subscription/main.cpp:158` 与 `test08_Publisher/main.cpp:249` 的 `rclc_executor_spin` 即函数最后一条语句，其后为函数尾。

`test08` 另有一处非设计意图的屏蔽，即 `test08_Publisher/main.cpp:237-241` 的时间同步循环：

```cpp
while (!rmw_uros_epoch_synchronized()) {
    rmw_uros_sync_session(SYNC_ATTEMPT_MS);
    delay(SYNC_POLL_MS);
}
```

Agent 不可达时该循环的退出条件永不满足，函数停于循环内而不返回。该屏蔽只覆盖"开机时 Agent 不可达"这一条路径。

`test07` 与 `test08` 实测未复现第 4.1 节的崩溃，原因是运行条件始终满足"Agent 可达、support 初始化成功"，使 `spin` 阻塞不返回，而非结构上不可能返回。开机时 Agent 不可达的路径为：`uxr_create_session` 失败（`rmw_init.c:308`）-> `rcl_init` 失败（`rcl/init.c:222-229`）-> `instance_id_storage` 归零（`rcl/context.c:109`）-> `spin_some` 返回 `RCL_RET_ERROR`（`executor.c:1808-1811`）-> `spin` 返回（`executor.c:1984-1987`）-> 任务函数越过函数尾。

#### 4.6.4 结论

1. 第 4.1 节崩溃的充分条件链是 $C_1 \wedge C_2 \wedge C_3$。其中 $C_1$ 是决定性一项：它使 `spin` 无条件立即返回，且与 support 是否初始化成功无关；
2. 句柄容量非零的固件，其是否崩溃由"support 是否初始化成功"与"任务函数能否越过函数尾"两因素决定；
3. 任务函数永不 return 是基本约束（同 4.5 节第 3 条）。不满足该约束的固件即使句柄容量合法，在 support 初始化失败时仍可复现同一崩溃。

---

> 引述版本说明：rclc 源码行号以工作区 `lib/micro_ros_platformio/build/mcu/src/rclc/rclc/src/rclc/` 内嵌副本为准；rmw 源码行号以 `lib/micro_ros_platformio/build/mcu/src/rmw-microxrcedds/` 内嵌副本为准；Arduino core `WiFiUdp.cpp` 以 PlatformIO 包 `framework-arduinoespressif32` 为准；雷达桥与 Agent 引述以 `~/Documents/ROS/YuXiangROS/Chap9/Robot_ws/src/` 下对应包源码为准；反汇编产物为 bug 提交 `eb9a540` 的本地编译 ELF。
