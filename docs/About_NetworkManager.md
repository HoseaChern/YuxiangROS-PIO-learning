# NetworkManager 与 nmcli 使用与维护

> 整理日期：2026-09-10
> 适用环境：Ubuntu 24.04 桌面版，NetworkManager 1.46.0（版本以 `nmcli --version` 输出为准）
> 权威参考：`man 1 nmcli`、`man 5 nm-settings-nmcli`、`man 5 nm-settings-keyfile`、
> `man 8 NetworkManager`、`man 7 nmcli-examples`；官方文档
> [networkmanager.dev/docs](https://networkmanager.dev/docs/)
> 本文档讲解通用机制。本项目热点与 STA 拓扑的实操序列见
> `docs/Network_Setup_Notes.md`。

---

## 目录

- [NetworkManager 与 nmcli 使用与维护](#networkmanager-与-nmcli-使用与维护)
  - [目录](#目录)
  - [1. 架构模型](#1-架构模型)
    - [1.1 组件与通信](#11-组件与通信)
    - [1.2 三层对象模型](#12-三层对象模型)
    - [1.3 profile 存储位置](#13-profile-存储位置)
  - [2. nmcli 命令体系](#2-nmcli-命令体系)
    - [2.1 语法与对象](#21-语法与对象)
    - [2.2 输出控制](#22-输出控制)
    - [2.3 profile 操作与设备操作的区分](#23-profile-操作与设备操作的区分)
  - [3. 自动激活与路由选择规则](#3-自动激活与路由选择规则)
    - [3.1 自动激活：profile 如何被选中](#31-自动激活profile-如何被选中)
    - [3.2 默认路由：metric 竞争](#32-默认路由metric-竞争)
    - [3.3 IPv4 配置方法](#33-ipv4-配置方法)
  - [4. 常用操作](#4-常用操作)
    - [4.1 状态查询（只读，安全）](#41-状态查询只读安全)
    - [4.2 WiFi 客户端](#42-wifi-客户端)
    - [4.3 热点（AP 模式）](#43-热点ap-模式)
    - [4.4 静态 IP、DNS 与路由](#44-静态-ipdns-与路由)
    - [4.5 断开、禁用与删除的语义区分](#45-断开禁用与删除的语义区分)
  - [5. 维护手段](#5-维护手段)
    - [5.1 keyfile 手工维护与重载](#51-keyfile-手工维护与重载)
    - [5.2 日志与运行时诊断](#52-日志与运行时诊断)
    - [5.3 服务与总开关](#53-服务与总开关)
    - [5.4 无线硬件开关](#54-无线硬件开关)
    - [5.5 故障速查表](#55-故障速查表)
  - [6. 与本项目文档的分工](#6-与本项目文档的分工)

## 1. 架构模型

### 1.1 组件与通信

Ubuntu 桌面版由 `NetworkManager.service` 守护进程管理有线、无线与虚拟网络
接口。`nmcli` 是该守护进程的命令行前端，经 D-Bus 总线接口
`org.freedesktop.NetworkManager` 与之通信。

由该架构可得两条边界规则：

- nmcli 的全部读写都作用于守护进程。配置修改立即持久化并生效，无需重启
  服务。
- 绕过 nmcli 直接操作接口（如 `ip link set wlp14s0 down`）会在守护进程之外
  改变设备状态。因为守护进程仍持有旧的设备视图，所以后续 nmcli 操作可能与
  实际状态失同步。排查网络问题时应以 nmcli 观察到的状态为准。

### 1.2 三层对象模型

nmcli 的一切命令都作用于下述三层对象：

| 层                | 概念         | 载体               | 说明                                                         |
| ----------------- | ------------ | ------------------ | ------------------------------------------------------------ |
| device            | 设备         | 内核网络接口       | 物理或虚拟接口，如 `wlp14s0`、`enp8s0`；状态由内核与驱动决定 |
| connection        | 连接 profile | keyfile 配置文件   | 一份命名的持久配置：SSID、凭据、IP 方法等                    |
| active connection | 活动连接     | 守护进程内运行实例 | profile 在某设备上的激活状态                                 |

三层之间的约束关系：

- 一台设备同一时刻至多持有一个活动连接；
- 一个 profile 可激活于任意兼容设备，未激活时仅以配置形式存在；
- 设备存在但无活动连接时为 `disconnected`；守护进程未接管的设备为
  `unmanaged`。

`nmcli d status` 中的 `p2p-dev-wlp14s0` 是 WiFi P2P（WiFi Direct）虚拟设备，
属正常现象，无需处理。

### 1.3 profile 存储位置

profile 以 ini 格式的 keyfile 存储（`man 5 nm-settings-keyfile`），三个目录
按语义分工：

| 目录                                          | 语义                                        |
| --------------------------------------------- | ------------------------------------------- |
| `/etc/NetworkManager/system-connections/`     | 管理员持久配置，`nmcli c modify` 的默认落点 |
| `/run/NetworkManager/system-connections/`     | 运行时临时配置，重启即失                    |
| `/usr/lib/NetworkManager/system-connections/` | 发行版预置配置                              |

安全约束：因为 keyfile 以明文存储凭据，所以守护进程拒绝加载任何非 root
用户可读或可写的文件。手工编辑后权限错误的典型症状是 profile 从
`nmcli c show` 中消失。规范权限为 `root:root` 且仅属主可读写：

```bash
sudo chmod 600 /etc/NetworkManager/system-connections/<profile>.nmconnection
```

## 2. nmcli 命令体系

### 2.1 语法与对象

语法（`man 1 nmcli`）：

```text
nmcli [选项] 对象 { 命令 | help }
```

对象一览（含最小简写）：

| 对象         | 简写 | 职责                          |
| ------------ | ---- | ----------------------------- |
| `general`    | `g`  | 守护进程整体状态、日志级别    |
| `networking` | `n`  | 网络总开关、连通性检查        |
| `radio`      | `r`  | 无线电开关（WiFi/WWAN 等）    |
| `connection` | `c`  | 连接 profile 的增删改查与激活 |
| `device`     | `d`  | 设备查询与操作                |
| `agent`      | `a`  | secret/polkit 代理            |
| `monitor`    | `m`  | 监听 NetworkManager 状态事件  |

每个对象支持 `nmcli <对象> help` 查看子命令；shell 的 Tab 补全在 Ubuntu 24.04
中默认可用。

### 2.2 输出控制

| 选项                    | 作用                                 | 典型用途            |
| ----------------------- | ------------------------------------ | ------------------- |
| `-f <字段,...>`         | 选择输出字段                         | 缩小关注范围        |
| `-t`                    | terse 模式，去表头与装饰             | 脚本逐行解析        |
| `-m tabular\|multiline` | 输出模式                             | 长字段值换行展示    |
| `-g <字段,...>`         | 等价于 `-m tabular -t -f` 的快捷方式 | 单值提取            |
| `-a`                    | 交互询问缺失参数                     | 密码不落 shell 历史 |
| `-s`                    | 允许显示密码等 secret                | 核对凭据            |

脚本取单值的标准写法（本机实测）：

```bash
nmcli -g ipv4.route-metric connection show BIT-Mobile
```

### 2.3 profile 操作与设备操作的区分

因为 `connection` 与 `device` 是两层对象，所以"断开"存在两种语义：

- `nmcli c down <profile>` 停用 profile 的活动实例，profile 配置保留；
- `nmcli d disconnect <device>` 断开设备，由该设备托管的全部活动连接一并
  停用。

配置修改后不生效的原因通常是：`modify` 只改配置，活动实例仍用旧值。所以
修改 IP、DNS 等运行参数后需执行 `nmcli c up <profile>` 重激活。

## 3. 自动激活与路由选择规则

### 3.1 自动激活：profile 如何被选中

设备出现（网线插入、无线驱动加载）时，守护进程从满足以下全部条件的 profile
中选择一个激活：

- `connection.autoconnect` 为 `yes`；
- profile 类型与设备兼容（WiFi profile 不能激活于有线设备）。

选择规则（`man 5 nm-settings-nmcli`，`connection.autoconnect-priority` 条目）。
设候选集合为 $P$，profile $i$ 的优先级为 $p_i$（范围 $-999$ 至 $999$，默认
$0$），最近一次激活时间戳为 $t_i$，则被激活的 profile 为

$$
i^* \;=\; \arg\max_{i \in P} \left( p_i,\; t_i \right),
$$

即先比 $p_i$，同优先级时比 $t_i$。若两者都相同，手册明确选择结果未定义。
工程结论：需要确定性行为时，为会竞争同一设备的 profile 显式设置互异的
`autoconnect-priority`。

### 3.2 默认路由：metric 竞争

多个活动连接同时提供默认路由时，内核选 metric 最小者。设各路由 metric 为
$m_i$，则默认路由取

$$
i^* \;=\; \arg\min_{i \in R} \; m_i .
$$

profile 的 `ipv4.route-metric` 默认为 $-1$，含义是由守护进程按设备类型自动
取值。本机实测（WiFi 未显式设置 metric，profile `BIT-Mobile`）：

```bash
ip -4 route show default
# default via 10.194.0.1 dev wlp14s0 proto dhcp src 10.194.106.85 metric 600
```

有线类型通常取值低于无线，所以网线插入时默认路由一般自动切向有线。具体
数值随版本与设备类型变化，以 `ip -4 route show default` 实测为准。需要固定
优先级时显式设置，例如给 WiFi profile 设 `ipv4.route-metric 600`、有线设
`ipv4.route-metric 100`。

### 3.3 IPv4 配置方法

`ipv4.method` 决定地址来源：

| 取值         | 机制                                                                                          |
| ------------ | --------------------------------------------------------------------------------------------- |
| `auto`       | DHCP 客户端获取地址、网关、DNS                                                                |
| `manual`     | 静态地址，配合 `ipv4.addresses` / `ipv4.gateway`                                              |
| `shared`     | 设备充当网关：守护进程启动内建 DHCP 服务（dnsmasq）向客户端分发地址并做 NAT，热点即基于此方法 |
| `link-local` | 仅链路本地地址（169.254.0.0/16）                                                              |
| `disabled`   | 不配置 IPv4                                                                                   |

IPv6 的 `ipv6.method` 与 `ipv6.*` 属性与之对称。

## 4. 常用操作

### 4.1 状态查询（只读，安全）

```bash
nmcli general status                          # 守护进程整体状态
nmcli networking connectivity                 # 连通性: none/portal/limited/full
nmcli device status                           # 设备一览
nmcli -f NAME,UUID,TYPE,AUTOCONNECT,DEVICE connection show  # profile 一览
nmcli connection show --active                # 仅活动连接
nmcli -f IP4,IP4.GATEWAY device show wlp14s0  # 指定设备的 IP 详情
nmcli -f WIFI-PROPERTIES device show wlp14s0  # 无线能力
```

`connectivity` 的语义：`full` 为完全可达，`limited` 为本地可达但无外网，
`portal` 为需网页认证，`none` 为不可达。

### 4.2 WiFi 客户端

```bash
nmcli device wifi rescan                     # 主动重扫
nmcli -f SSID,BSSID,CHAN,RATE,SIGNAL,SECURITY device wifi list  # 列表
nmcli device wifi connect <SSID> password '<密码>'               # 即连即存 profile
nmcli connection up <profile>                # 用已存凭据连接
nmcli connection modify <profile> 802-11-wireless.powersave 2    # 关节能省电但增延迟, 按需
```

### 4.3 热点（AP 模式）

最小命令（机制见 3.3 的 `shared` 方法）：

```bash
sudo nmcli device wifi hotspot ifname wlp14s0 ssid '<SSID>' \
  password '<密码>' band bg channel 6
```

`band bg` 锁 2.4 GHz，`channel` 取 1/6/11 中不拥挤者。本项目的完整热点与
STA 序列（含与 micro-ROS Agent 地址对齐的约束）见
`docs/Network_Setup_Notes.md` 第 "热点" 章节。

### 4.4 静态 IP、DNS 与路由

```bash
nmcli connection modify <profile> \
  ipv4.method manual \
  ipv4.addresses 192.168.4.100/24 \
  ipv4.gateway 192.168.4.1 \
  ipv4.dns "192.168.4.1" \
  ipv4.route-metric 100
nmcli connection up <profile>                # 重激活使修改生效
```

补充属性：`ipv4.ignore-auto-dns yes` 拒用 DHCP 下发的 DNS；
`ipv4.routes "10.42.0.0/24 192.168.4.1"` 追加静态路由；
`ipv4.never-default yes` 禁止该 profile 提供默认路由（常用于让辅助连接不
参与 3.2 的默认路由竞争）。

### 4.5 断开、禁用与删除的语义区分

| 命令                          | 作用对象            | 效果                                                    | 可逆性               |
| ----------------------------- | ------------------- | ------------------------------------------------------- | -------------------- |
| `nmcli c down <profile>`      | profile 实例        | 停用该活动连接                                          | 可再 `c up`          |
| `nmcli d disconnect <device>` | 设备                | 断开设备上全部活动连接                                  | 可再连接             |
| `nmcli radio wifi off`        | 无线电子系统(NM 层) | WiFi 逻辑关断                                           | `radio wifi on` 恢复 |
| `nmcli networking off`        | 守护进程全部接口    | 网络总开关关断                                          | `networking on` 恢复 |
| `nmcli c delete <profile>`    | keyfile             | 删除 profile 文件                                       | 不可逆，凭据丢失     |
| `nmcli d delete <device>`     | 设备                | 仅适用于软件设备（bond/bridge/vlan 等），物理设备不可删 | —                    |

## 5. 维护手段

### 5.1 keyfile 手工维护与重载

直接编辑 `/etc/NetworkManager/system-connections/*.nmconnection` 后，守护
进程不会自动感知编辑行为，需显式重载：

```bash
sudo nmcli connection reload                  # 重载全部
sudo nmcli connection load /etc/.../<profile>.nmconnection  # 重载单个
```

重载只更新配置库，活动中的连接不随之变化；需要应用新配置仍执行
`nmcli c up <profile>`。常规维护建议以 `nmcli c modify` 为主，因为 nmcli
保证属性语法与文件权限正确，手工编辑的常见故障是权限不满足 1.3 的约束与
属性名拼写错误。

### 5.2 日志与运行时诊断

```bash
journalctl -u NetworkManager -b               # 本次开机以来的守护进程日志
journalctl -u NetworkManager -f               # 跟踪输出
nmcli monitor                                 # 订阅状态事件(设备/profile/连接变化)
nmcli general logging level TRACE domains ALL # 运行时调高日志级别
nmcli general logging level INFO domains ALL  # 排查后恢复
```

手册（`man 8 NetworkManager`）注明两点：TRACE 级别输出量大，systemd-journald
可能对日志限速，必要时调整 `journald.conf` 的 `RateLimitIntervalSec` 与
`RateLimitBurst`；守护进程不记录任何 secret，日志可放心留存。

### 5.3 服务与总开关

```bash
systemctl status NetworkManager               # 服务状态
sudo systemctl restart NetworkManager         # 重读全部配置并断开现有活动连接
nmcli networking on                           # 等价的软启动总开关
```

`restart` 的代价是全部活动连接中断，所以日常配置变更不需要 restart，nmcli
的 modify 与 reload 已覆盖；仅当守护进程自身状态异常（如 D-Bus 无响应）时
使用。

### 5.4 无线硬件开关

无线不可用的排查分两层：内核层 rfkill 与守护进程层 radio 开关。

```bash
rfkill list                                   # 内核层: Soft blocked / Hard blocked
sudo rfkill unblock all                       # 解除软阻塞; Hard 需物理开关或 BIOS
nmcli radio all                               # 守护进程层各无线电开关状态
nmcli radio wifi on                           # 打开 WiFi 无线电
```

判断依据：`rfkill` 显示 blocked 而 `nmcli r wifi` 为 enabled，则问题在内核层
（物理开关、飞行模式、BIOS）；反之在守护进程层。

### 5.5 故障速查表

| 现象                      | 机理                                                                                         | 处理                                                                                                               |
| ------------------------- | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| 设备状态 `unmanaged`      | 守护进程未接管该设备：netplan 渲染器指向 networkd，或 `NetworkManager.conf` 将其标记为非托管 | 网卡属 netplan 管理时改 `renderer: NetworkManager`；临时接管用 `sudo nmcli d set <设备> managed yes`（运行时生效） |
| `nmcli d wifi list` 为空  | rfkill 阻塞、无线电关闭或驱动未就绪                                                          | 按 5.4 两层排查；`iw dev` 确认接口存在                                                                             |
| 热点无法启动              | 信道冲突、频段不被驱动支持或 `shared` 方法失败                                               | `journalctl -u NetworkManager -e` 看具体报错；换信道或 `band a`；确认 3.3 的机制未被 `ipv4.method` 覆盖            |
| DHCP 拿不到地址           | AP 侧隔离或 `ipv4.method` 配置错误                                                           | `nmcli -g ipv4.method c show <profile>` 应为 `auto`；热点场景确认客户端未设静态                                    |
| DNS 失败但 `ping <IP>` 通 | DNS 链路问题：`ipv4.dns` 缺失或 systemd-resolved 未接收到上游                                | `resolvectl status` 查当前上游；`nmcli c modify <profile> ipv4.dns "..."` 后 `c up`                                |
| 已连接但访问外网不通      | 多活动连接的默认路由 metric 竞争指向了错误出口                                               | 按 3.2 实测 `ip -4 route show default`；为辅助连接设 `ipv4.never-default yes` 或调高其 metric                      |

## 6. 与本项目文档的分工

- 本文：NetworkManager 通用机制、命令体系与维护手段，不含项目特定参数。
- `docs/Network_Setup_Notes.md`：本项目的热点拓扑（上位机开热点）与 STA
  拓扑（下位机开热点）实操序列，以及与固件 `config.h` 中
  `AGENT_IP_STR` 的对齐约束。
- 本机现状快照（2026-09-10 实测，`nmcli -t -f
  DEVICE,TYPE,STATE,CONNECTION device status`）：无线设备 `wlp14s0`；有线设备 `enp8s0` 与`enx00e04c680c2f`（unavailable）。
