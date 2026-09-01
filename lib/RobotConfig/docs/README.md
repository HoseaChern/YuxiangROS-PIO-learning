# RobotConfig 配置库

## 1. 库的作用

RobotConfig 是 YuxiangROS-PIO-learning(ESP32-S3, PlatformIO)项目中的共用编译期配置库，
收纳 `src/main.cpp` 与 `src/tests/`(test01 至 test11)各固件共用的"环境相关、经常调整"的常量：

- **硬件接线**：电机/编码器引脚、IMU I2C 引脚、雷达串口与调速引脚；
- **标定参数**：PID 系数（直立环/速度环）、运动学轮距与脉冲当量、默认目标速度、机械中值；
- **部署环境**：WiFi 凭据、micro-ROS Agent 的 IP 与端口；
- **通用参数**：串口波特率、控制周期、单位换算、任务/发布/时间同步、话题与节点名。

背景：这些常量原本以 `constexpr` 形式重复硬编码在 12 个固件内部，改引脚/换 IP 会修改
被 git 跟踪的 `.cpp` 文件，频繁污染工作区；同一常量多处声明也容易改动遗漏、相互不一致。
RobotConfig 将之统一收纳，`config.h` 不入版本库，使配置调整与代码改动彻底解耦。

## 2. 文件说明

| 文件               | 说明                                       |
| ------------------ | ------------------------------------------ |
| `config.example.h` | 模板（纳入版本库），含占位符凭据与默认值   |
| `config.h`         | 本地真实配置（被 .gitignore 忽略，不入库） |
| `docs/README.md`   | 本文档，库的使用说明                       |

两个文件使用同一 include guard（`ROBOTCONFIG_H`），同一翻译单元内只应包含其一。
`config.h` 由 `config.example.h` 复制而来，初始值与当前硬件/部署环境一致，复制后无需修改即可编译。

## 3. 收纳范围

| 分组       | 常量                                                                                                                                                                             | 说明                                  |
| ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------- |
| 串口参数   | `SERIAL_BAUD`                                                                                                                                                                    | 串口波特率                            |
| 电机引脚   | `MOTOR_LEFT_PIN_A/B`、`MOTOR_RIGHT_PIN_A/B`                                                                                                                                      | 电机 PWM 引脚                         |
| 编码器引脚 | `ENC_LEFT_PIN_A/B`、`ENC_RIGHT_PIN_A/B`                                                                                                                                          | 编码器输入引脚                        |
| PID 参数   | `PID_KP`、`PID_KI`、`PID_KD`、`PID_OUTPUT_LIMIT`                                                                                                                                 | 速度环标定参数                        |
| 运动学参数 | `WHEEL_DISTANCE_MM`、`DISTANCE_PER_TICK_MM`                                                                                                                                      | 轮距与脉冲当量                        |
| 目标速度   | `TARGET_LINEAR_SPEED_MM_S`、`TARGET_ANGULAR_SPEED_RAD_S`                                                                                                                         | 无外部指令时的标定目标                |
| 测试专用   | `STEP_DELAY_MS`、`MOTOR_SPEED`、`TARGET_SPEED_MM_S`                                                                                                                              | test01/03/04 使用                     |
| 单位换算   | `MPS_TO_MMPS`、`MS_TO_S`、`S_TO_NS`                                                                                                                                              | 速度与时间戳换算                      |
| 控制周期   | `LOOP_DELAY_MS`                                                                                                                                                                  | 主循环调度节拍                        |
| 网络配置   | `AGENT_IP_STR`、`AGENT_PORT`                                                                                                                                                     | micro-ROS Agent 地址                  |
| WiFi 凭据  | `WIFI_SSID`、`WIFI_PASS`                                                                                                                                                         | WiFi 账号密码（可写数组）             |
| 任务参数   | `MICRO_ROS_STACK_SIZE`、`MICRO_ROS_TASK_PRIO`、`TRANSPORT_SETUP_MS`、`ODOM_PUBLISH_MS`、`SYNC_ATTEMPT_MS`、`SYNC_POLL_MS`                                                        | 任务栈、发布周期、时间同步            |
| 话题与节点 | `CMD_VEL_TOPIC`、`NODE_NAME`、`ODOM_TOPIC`                                                                                                                                       | ROS 图元素名称                        |
| IMU 与任务 | `IMU_SDA_PIN`、`IMU_SCL_PIN`、`BALANCE_PERIOD_MS`、`BALANCE_STACK_SIZE`、`BALANCE_TASK_PRIO`、`BALANCE_TASK_CORE`                                                                | test10/test11 I2C 引脚与控制任务参数  |
| 直立环参数 | `UPRIGHT_KP`、`UPRIGHT_KI`、`UPRIGHT_KD`、`UPRIGHT_PWM_LIMIT`、`UPRIGHT_ZERO_PITCH_DEG`                                                                                          | 直立环 PD 标定与机械中值              |
| 安全与标定 | `UPRIGHT_ARM_ANGLE_DEG`、`UPRIGHT_FALL_ANGLE_DEG`、`UPRIGHT_CALM_DELAY_MS`、`UPRIGHT_CALIB_CYCLES`、`BALANCE_PRINT_MS`                                                           | 起控/倒地保护阈值、中值标定、打印周期 |
| 速度环参数 | `SPEED_KP`、`SPEED_KI`、`SPEED_OUTPUT_LIMIT`、`SPEED_SETPOINT_MM_S`、`SPEED_STEP_MM_S`                                                                                           | test11 速度环 PI 标定与串口调速步进   |
| 雷达转接   | `LIDAR_UART_RX_PIN`、`LIDAR_MOTOR_CTRL_PIN`、`LIDAR_BAUD`、`LIDAR_PWM_FREQ`、`LIDAR_PWM_RES`、`LIDAR_PWM_CHANNEL`、`LIDAR_MOTOR_SPEED`、`BRIDGE_TCP_PORT`、`BRIDGE_RECONNECT_MS` | test09 激光雷达透传参数               |
| 桥接任务   | `BRIDGE_STACK_SIZE`、`BRIDGE_TASK_PRIO`                                                                                                                                          | 主环境 bridge_task 任务参数           |

> 说明：`WIFI_SSID`/`WIFI_PASS` 由原 `lib/Secrets` 合并而来，须为可写 `char` 数组，
> 因 `set_microros_wifi_transports` 接口要求 `char*`，不能声明为 `constexpr`。

### 不收纳

- `EXECUTOR_HANDLES`：micro-ROS 执行器句柄数，按各固件订阅/定时器数量取值
  （main=2 / test06=0 / test07=1），保留在各固件本地定义。
- `BalanceState`：test10/test11 的两轮自平衡状态机枚举（`kIdle`/`kRunning`），属行为逻辑
  而非配置数据，且仅两个固件使用，保留在各固件本地定义。

## 4. 使用流程

首次使用：

```bash
cp lib/RobotConfig/config.example.h lib/RobotConfig/config.h
```

调整配置时只修改 `lib/RobotConfig/config.h`（更换 Agent IP、调整 PID 参数、修改 WiFi 凭据等），
git 工作区保持干净。

新增固件时 `#include "config.h"` 即可使用全部共享常量，无需重复定义；
若需本固件独有常量，在固件内匿名 namespace 中定义并注释说明理由。

## 5. 为什么这样做（优点）

- **工作区解耦**：引脚/IP/WiFi 等环境相关改动不再污染 git 工作区；
- **单一事实来源**：同一常量全仓库只有一处定义，消除 12 个固件中的重复声明；
- **模板可审计**：`config.example.h` 入库，便于 code review 与新手初始化；
- **配置与代码分离**：`config.h` + 模板 + `.gitignore` 的组合已在本仓库验证，
  与原先 `lib/Secrets` 的凭据管理模式对称（Secrets 现已并入本库）。

## 6. 缺点与代价

- 新增常量需先加入模板再复制到本地 `config.h`，否则本地副本缺失该常量
  （模板更新不会自动同步到已存在的 `config.h`）；
- 配置分散在两处（模板/本地），需注意二者同步；
- 依赖 PlatformIO 的库机制：本库为纯头文件库，`pio run -t compiledb` 不会自动
  注入其 include 路径，需 `tools/merge_ccdb.py` 在 `HEADER_ONLY_LIBS` 中登记
  `lib/RobotConfig`，否则 clangd 无法解析 `config.h`。

## 7. 注意事项

- 新增/修改常量时，模板与本地副本必须同步更新，避免两处不一致；
- `WIFI_SSID`/`WIFI_PASS` 是凭据，提交代码时确认只提交 `config.example.h`
  （占位符），本地 `config.h` 已被 `.gitignore` 忽略；
- `config.h` 与 `config.example.h` 共用同一 include guard，同一翻译单元内只能包含其一。
