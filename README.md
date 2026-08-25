# YuxiangROS-PIO-learning

> 配套书籍：《ROS 2 机器人开发：从入门到实践》（桑欣 著，fishros 出品）。
> 本仓库是第 9 章"实体机器人"的 ESP32-S3 两轮差速底盘运动控制固件：通过
> micro-ROS 订阅 `/cmd_vel`，经运动学逆解与 PID 速度闭环驱动电机，并发布 `/odom`。
> 特色：原书每小节直接在上小节代码上改、没有整合工程；本仓库把各小节散落的
> 代码块切片为 12 个独立可编译工程（examples / tests / main），便于对照原书逐节阅读。
> 仓库名 PIO 即 PlatformIO（下位机固件侧），与主仓库
> [YuXiangROS-jazzy-learning](https://github.com/HoseaChern/YuXiangROS-jazzy-learning)
> （ROS 2 上位机侧）互补配套。

[English Version](./README_EN.md)

## 目录

- [YuxiangROS-PIO-learning](#yuxiangros-pio-learning)
  - [目录](#目录)
  - [项目简介](#项目简介)
  - [与原书的对应关系](#与原书的对应关系)
  - [硬件与引脚](#硬件与引脚)
  - [platformio.ini 关键设计](#platformioini-关键设计)
  - [目录结构](#目录结构)
  - [依赖库](#依赖库)
  - [编译与烧录](#编译与烧录)
  - [运行与联调](#运行与联调)
  - [自主优化](#自主优化)
  - [开发环境](#开发环境)
    - [compile\_commands.json 生成（12 环境合并）](#compile_commandsjson-生成12-环境合并)
  - [致谢与参考](#致谢与参考)
  - [许可证](#许可证)

## 项目简介

固件运行在 ESP32-S3（PlatformIO + Arduino framework）上，与上位机（ROS 2 Jazzy）
通过 WiFi + micro-ROS 通信：

1. 订阅 `/cmd_vel`（`geometry_msgs/msg/Twist`）速度指令；
2. 运动学逆解将车体速度转为左右轮目标转速（纯算法库 `Kinematics`）；
3. PID 速度闭环输出 PWM 驱动电机（纯算法库 `PIDController`）；
4. 编码器读数积分里程计，50 ms 周期发布 `/odom`（`nav_msgs/msg/Odometry`）。

```text
/cmd_vel (Twist) -> 逆解 -> 目标轮速 -> PID -> PWM -> 电机+编码器 -> 里程计 -> /odom
```

## 与原书的对应关系

原书第 9 章开发路径：单片机开发基础（9.2）→ 控制系统实现（9.3）→ micro-ROS 接入
（9.4）。**原书下位机代码以"一小节的代码块"散落书中，每小节直接在上小节基础上改，
没有整合的 PIO 工程**。本仓库把这些代码块切片为独立可编译工程，一一对应：

| 原书小节（代码块）             | 本仓库切片工程                                     | 验证内容                   |
| ------------------------------ | -------------------------------------------------- | -------------------------- |
| 9.2.2 第一个 Hello World 工程  | `examples/example01_helloworld`                    | 串口 Hello World           |
| 9.2.3 使用代码点亮 LED 灯      | `examples/example02_LED`                           | GPIO 输出、LED 闪烁        |
| 9.2.4 使用超声波测量距离       | `examples/example03_Ultrasound`                    | 超声波传感器读取           |
| 9.2.5 使用开源库驱动 IMU       | `examples/example04_IMU`                           | MPU6050 姿态解算           |
| 9.3.1 使用开源库驱动多路电动机 | `tests/test01_motor`                               | MCPWM 电机驱动             |
| 9.3.2 电动机速度测量与转换     | `tests/test02_encoder`、`tests/test03_speed_trans` | 编码器读数、速度换算       |
| 9.3.3 使用 PID 控制轮子转速    | `tests/test04_PID`                                 | PID 速度闭环               |
| 9.3.4 运动学正逆解的实现       | `tests/test05_Kinematics`                          | 逆解 + PID 综合控制        |
| 9.3.5 机器人里程计计算         | 主固件 `src/main.cpp`（里程计部分）                | 里程计积分                 |
| 9.4.1 第一个节点               | `tests/test06_wifi`                                | micro-ROS WiFi 连接        |
| 9.4.2 订阅话题控制机器人       | `tests/test07_Subscription`                        | `/cmd_vel` 订阅 + 运动控制 |
| 9.4.3 发布机器人里程计话题     | 主固件 `src/main.cpp`                              | `/odom` 发布 + 全流程集成  |

> 说明：9.2.1（平台介绍）不涉及代码；9.3.2 同时覆盖"速度测量"与"速度转换"两段代码，
> 故映射两个切片；主固件是 9.3.5 与 9.4.3 的收敛（内嵌里程计 + 发布 `/odom`）。

## 硬件与引脚

| 器件     | 左（Motor/Encoder 0） | 右（Motor/Encoder 1） |
| -------- | --------------------- | --------------------- |
| 电机 PWM | GPIO 5 / 4            | GPIO 6 / 7            |
| 编码器   | GPIO 16 / 15          | GPIO 17 / 18          |

| 项目     | 配置                                                        |
| -------- | ----------------------------------------------------------- |
| 主控     | ESP32-S3-DevKitC-1（Xtensa LX7，Arduino framework）         |
| 电机驱动 | `Esp32McpwmMotor`（MCPWM）                                  |
| 编码器   | `Esp32PcntEncoder`（PCNT 脉冲计数）                         |
| 通信     | micro-ROS over WiFi（UDP），默认 Agent `192.168.2.120:8888` |
| 控制周期 | 主循环 10 ms，里程计发布 50 ms                              |

> 硬件差异：原书使用 Adafruit Feather 开发板，本仓库改用 ESP32-S3-DevKitC-1。
> 固件与板型解耦，换板只需改 `platformio.ini` 的 `board` 与引脚——证明可灵活变通。

## platformio.ini 关键设计

`platformio.ini` 是本仓库的核心配置：12 个环境（1 主 + 4 示例 + 7 测试），每个
示例/测试只编译自身 `main.cpp`，与主固件互不干扰。关键点：

- **`build_src_filter` 环境隔离**：主固件 `+<*> -<examples> -<tests>`；每个
  示例/测试只保留自己的目录。否则 `src/` 下多个 `setup()/loop()` 符号重复定义，
  链接失败。
- **`lib_deps` 按环境声明**：公共段不写 `lib_deps`（各环境依赖无公共交集），
  每环境只装自己需要的库，`lib_ignore` 完全不需要。
- **micro-ROS 只声明在使用它的环境**（主环境、test06/07）：其 `extra_script.py`
  构建钩子在被安装的环境无条件执行（注入宏、链接预编译 `libmicroros`），无法用
  `lib_ignore` 阻止，故不能装进无关环境。
- **IntelliSense 兜底 include**：`MPU6050_light` 只装在 example04 环境，公共段加
  `-I` 指向其头文件，保证任何激活环境下 IDE 都能解析该头（编译层面多余但无害）。
- **凭据分离**：WiFi 账号密码存于 `lib/Secrets/secrets.h`，仓库只保留
  `secrets.example.h` 模板。

```ini
[env]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

[env:esp32-s3-devkitc-1]                 ; 主固件
build_src_filter = +<*> -<examples> -<tests>
board_microros_transport = wifi
lib_deps = Esp32McpwmMotor, Esp32PcntEncoder, micro_ros_platformio(fishros 镜像)

[env:example01_helloworld]               ; 示例：只编译自身目录
build_src_filter = +<examples/example01_helloworld>
```

## 目录结构

```text
YuxiangROS-PIO-learning/
├── include/                     # 项目头文件（预留）
├── lib/                         # 私有库
│   ├── Kinematics/              # 两轮差速运动学（正/逆解 + 里程计），纯算法
│   ├── PIDController/           # 位置式 PID，纯算法
│   ├── SemanticEnums/           # 语义化枚举（MotorID / VelocityID 等）
│   └── Secrets/                 # 凭据模板（secrets.example.h）
├── src/
│   ├── main.cpp                 # 主固件：micro-ROS + 运动控制
│   ├── examples/                # 4 个示例固件（example01~04）
│   └── tests/                   # 7 个测试固件（test01~07）
├── docs/                        # 学习笔记（About_PlatformIO、CLI 使用）
├── .clangd / .clang-format / .clang-tidy   # C/C++ 工具链规范
└── platformio.ini               # 12 环境工程配置
```

## 依赖库

| 库                   | 用途            | 来源                                                                 | 使用环境                      |
| -------------------- | --------------- | -------------------------------------------------------------------- | ----------------------------- |
| Esp32McpwmMotor      | MCPWM 电机驱动  | [fishros](https://github.com/fishros/Esp32McpwmMotor)                | 主环境、test01/03/04/05/06/07 |
| Esp32PcntEncoder     | PCNT 编码器读取 | [fishros](https://github.com/fishros/Esp32PcntEncoder)               | 主环境、test02/03/04/05/06/07 |
| micro_ros_platformio | micro-ROS 支持  | [fishros](https://github.com/fishros/micro_ros_platformio)（镜像版） | 主环境、test06/07             |
| MPU6050_light        | IMU 姿态解算    | [rfetick](https://github.com/rfetick/MPU6050_light)                  | example04                     |

> 为何用 fishros 预编译镜像：官方
> [micro-ROS/micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio)
> 的板型支持列表仅含通用 `esp32dev`（未覆盖 ESP32-S3），且构建需从源码全量编译
> 整个 micro-ROS 栈（cmake + meta-build，首次构建慢且易失败）——这是使用预编译
> 镜像的原因。

## 编译与烧录

```bash
# 准备凭据（复制模板并填写 WiFi 账号密码）
cp lib/Secrets/secrets.example.h lib/Secrets/secrets.h

# 编译 / 烧录主固件
pio run -e esp32-s3-devkitc-1
pio run -e esp32-s3-devkitc-1 -t upload

# 串口监视 / 编译烧录某个示例或测试（如 test01_motor）
pio device monitor -b 115200
pio run -e test01_motor -t upload
```

| 类型   | 环境名               | 说明                                                  |
| ------ | -------------------- | ----------------------------------------------------- |
| 主固件 | esp32-s3-devkitc-1   | 运动控制 + micro-ROS（订阅 `/cmd_vel`，发布 `/odom`） |
| 示例   | example01_helloworld | Hello World                                           |
| 示例   | example02_LED        | LED 闪烁                                              |
| 示例   | example03_Ultrasound | 超声波测距                                            |
| 示例   | example04_IMU        | MPU6050 姿态解算                                      |
| 测试   | test01_motor         | 电机驱动测试                                          |
| 测试   | test02_encoder       | 编码器读取与标定                                      |
| 测试   | test03_speed_trans   | 速度换算测试                                          |
| 测试   | test04_PID           | PID 速度闭环测试                                      |
| 测试   | test05_Kinematics    | 运动学逆解 + PID 控制测试                             |
| 测试   | test06_wifi          | micro-ROS WiFi 连接测试                               |
| 测试   | test07_Subscription  | `/cmd_vel` 订阅 + 运动控制测试                        |

## 运行与联调

1. 填写 `lib/Secrets/secrets.h` 的 WiFi 账号密码；
2. 将 `src/main.cpp` 中 `AGENT_IP_STR` 改为运行 micro-ROS Agent 的主机 IP；
3. 烧录固件后，上位机启动 Agent：

   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

4. 发布速度指令驱动底盘：

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
   ```

5. 查看里程计：`ros2 topic echo /odom`

## 自主优化

本固件在 fishros 的 micro-ROS 模板基础上自主实现，算法库（`Kinematics`、
`PIDController`）与原作差异集中于代码质量层面，要点如下：

- **去除魔法数，提取编译期常量**：`MS_TO_S`（单位换算 1000）、
  `INTEGRAL_SUP_LIMIT`（积分上界 2500）、`PI_F` 等以 `constexpr` 表达；
- **类型纪律**：显式整型/浮点宽度（`int16_t`/`uint32_t`/`float`）；
  `dt` 由 `uint32_t` 拓宽为 `uint64_t` 防毫秒计数溢出；`update_pwm` 返回
  `int16_t` 并四舍五入，避免低速占空比系统性偏小；
- **语义化枚举**：`SemanticEnums` 约束电机/速度下标，替代裸数字；
- **const 正确性**：只读方法一律 `const`，`get_odom()` 提供可写/只读双版本；
- **防御性编程**：除零防御（轮间距 `<= 0` 时角速度输出 0）、积分与输出限幅、
  角度归一化（`std::fmod` 重写，任意多次旋转收敛到 [-PI, PI]）；
- **接口瘦身与依赖收缩**：拆除 `motor_param_t` 结构体（左右轮标定参数一致，
  标量化）；算法层依赖由 `<Arduino.h>` 收缩为 `<cstdint>`/`<cmath>`，可移植可单测。

> 详细 16 项/9 项优化清单（含 commit）见对应库的 `docs/README.md`。

## 开发环境

采用"PIO 交叉编译 + LLVM 开发表层 + gdb 调试"三层结构，各层职责独立：

| 层       | 工具                         | 职责                         |
| -------- | ---------------------------- | ---------------------------- |
| 编译层   | xtensa-esp32s3-elf-g++ (gcc) | 唯一生产编译路径，产出固件   |
| 开发表层 | clangd / clang-format        | 智能提示、格式化             |
| 调试层   | platformio-debug (gdb)       | `pio debug`（OpenOCD + gdb） |

选择依据（与常见替代品的区别）：

- **编译层不可替换**：ESP32-S3 是 Xtensa LX7 架构，Arduino framework 与
  micro-ROS 预编译库都以 PIO 自带交叉编译器为编译底线；
- **开发表层选 clangd 而非 cpptools**：cpptools 无法处理"交叉编译器内置宏 +
  多环境 libdeps"的解析；clangd 依赖 `compile_commands.json` 承载完整编译命令；
- **调试层选 gdb 而非 CodeLLDB**：CodeLLDB 的 LLDB 不支持 Xtensa 架构，
  嵌入式调试只能走 OpenOCD + gdb。

仓库已入库 `.clangd`、`.clang-format`、`.clang-tidy` 三份规范（`.vscode/` 不入库）。

### compile_commands.json 生成（12 环境合并）

`pio run -t compiledb` 每次只产出当前激活环境，故循环生成再合并去重（本机生成物，
含绝对路径，不入库）：

```bash
pio=~/.platformio/penv/bin/pio
mkdir -p .pio/ccdbs
for env in esp32-s3-devkitc-1 example01_helloworld example02_LED \
           example03_Ultrasound example04_IMU test01_motor test02_encoder \
           test03_speed_trans test04_PID test05_Kinematics test06_wifi \
           test07_Subscription; do
  $pio run -e "$env" -t compiledb && mv compile_commands.json ".pio/ccdbs/$env.json"
done
python3 tools/merge_ccdb.py
```

要点：

- 公共 framework 源文件在各环境重复出现，须按 `file` 去重，否则 clangd 对同一
  文件多条 command 会冲突；
- 相对编译器名 `xtensa-esp32s3-elf-` 用 `startswith` 补绝对路径前缀；不要用
  `sed` 全局替换（该名字也存在于绝对路径内，会得到 `bin//home` 双前缀）；
- `tools/merge_ccdb.py` 会为每条命令补齐 header-only 库 `lib/SemanticEnums`
  的 `-I`：PIO 的 `-t compiledb` 漏注入无 `.cpp` 的纯头文件库，真实构建命令
  含该路径而 ccdb 缺失，导致 clangd 报 `'SemanticEnums.h' file not found`；
- 增删环境后重新执行本步骤。

常见问题速查：

| 现象                          | 处理                                                       |
| ----------------------------- | ---------------------------------------------------------- |
| clangd 报 driver not found    | `.clangd` 误写 `Compiler:`，删除该字段                     |
| `uint32_t` 等类型全报 unknown | compile_commands.json 缺失或编译器为相对名，重跑生成步骤   |
| clang-tidy 未生效             | 确认 `--clang-tidy` 参数与根目录 `.clang-tidy`             |
| 调试无法用 CodeLLDB           | Xtensa 无 LLDB 支持，改用 CLI `pio debug`（OpenOCD + gdb） |

## 致谢与参考

- 鱼香ROS（fishros）及《ROS 2 机器人开发：从入门到实践》（桑欣 著）；
- [fishros/ros2bookcode](https://github.com/fishros/ros2bookcode)：原书配套代码仓库；
- [fishros/micro_ros_platformio](https://github.com/fishros/micro_ros_platformio)：
  micro-ROS 固件模板（本工程配置与工具链基于其改写）；
- [fishros/fishbot](https://github.com/fishros/fishbot)：运动学与 PID 算法思路参考；
- [fishros/Esp32McpwmMotor](https://github.com/fishros/Esp32McpwmMotor)、
  [fishros/Esp32PcntEncoder](https://github.com/fishros/Esp32PcntEncoder)：
  电机与编码器驱动库；
- 主仓库 [YuXiangROS-jazzy-learning](https://github.com/HoseaChern/YuXiangROS-jazzy-learning)。

## 许可证

本仓库原创代码遵循 [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)
许可证（见 [LICENSE](./LICENSE)）；第三方库保留各自许可证。
