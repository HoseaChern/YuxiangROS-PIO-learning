# Kinematics 运动学库

## 1. 库的作用

Kinematics 是 YuxiangROS-PIO-learning(ESP32-S3, PlatformIO)项目中的运动学计算库,为两轮差速机器人提供:

- **运动学正解**:由左右轮转速(电机转速)推算车体线速度与角速度,供里程计计算使用。
- **运动学逆解**:由目标车体线速度与角速度推算左右轮目标转速,供运动控制(如 PID 速度环)下发。
- **里程计更新**:基于编码器采样数据与运动学模型,累积推算机器人位姿 `(x, y, yaw)` 与速度。

库为**纯算法层**,不依赖任何硬件外设(编码器、定时器等),输入输出均为基本数值类型,便于单元测试与跨平台移植。

## 2. 文件说明

| 文件             | 说明                                            |
| ---------------- | ----------------------------------------------- |
| `Kinematics.h`   | 头文件,声明 `odom_t` 数据结构与 `Kinematics` 类 |
| `Kinematics.cpp` | 实现文件,包含正逆解、速度采样、里程计更新等实现 |
| `docs/README.md` | 本文档,库的使用说明                             |

库的历次优化与提交记录已并入仓库根目录 [README.md](../../../README.md) 的
"Kinematics 运动学库优化（16 项）"章节。

## 3. 核心公式

两轮差速模型,`wheel_distance` 为轮间距(单位 mm)。

正解(轮转速 -> 车体速度):

```text
v      = (v_left + v_right) / 2
omega  = (v_right - v_left) / wheel_distance
```

逆解(车体速度 -> 轮转速):

```text
v_left  = v - omega * wheel_distance / 2
v_right = v + omega * wheel_distance / 2
```

里程计累积(单位已换算为秒):

```text
x    += v * cos(yaw) * dt
y    += v * sin(yaw) * dt
yaw  += omega * dt
```

## 4. 数据类型

### odom_t

里程计参数结构体:

| 字段               | 说明   | 单位  |
| ------------------ | ------ | ----- |
| `x`                | x 坐标 | mm    |
| `y`                | y 坐标 | mm    |
| `yaw`              | 偏航角 | rad   |
| `linear_velocity`  | 线速度 | mm/s  |
| `angular_velocity` | 角速度 | rad/s |

## 5. 公共接口

| 方法                                                                        | 说明                                                                 |
| --------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| `set_motor_param(float distance_per_tick_mm)`                               | 设置标定参数:单个编码器脉冲对应的轮子前进距离(mm),两轮共用           |
| `set_wheel_distance(float wheel_distance)`                                  | 设置轮间距(mm)                                                       |
| `update_motor_speed(uint64_t now, const int32_t ticks[2])`                  | 编码器采样:输入当前时间(ms)与左右编码器读数,内部计算速度并自动刷新里程计 |
| `kinematics_forward(const float motor_speeds[2], float body_velocities[2])` | 运动学正解:电机转速 -> 车体速度,仅用于里程计内部计算                 |
| `kinematics_inverse(const float body_velocities[2], float motor_speeds[2])` | 运动学逆解:车体速度 -> 电机目标转速                                  |
| `get_motor_speed(MotorID motor_id) const`                                   | 获取电机当前速度(mm/s),`motor_id`: MOTOR_LEFT=左, MOTOR_RIGHT=右     |
| `get_odom()` / `get_odom() const`                                           | 获取里程计数据,提供可写与只读两种重载                                |

> 说明:`update_odom_` 与角度归一化 `trans_angle_in_pi_` 为类私有实现细节,由
> `update_motor_speed` 每周期自动驱动,不对外暴露。

### 数组参数约定

正逆解与采样接口均使用**捆绑指针传参**(数组形参退化为指针),避免逐元素传参的冗长签名:

- `motor_speeds[2]`:电机转速(mm/s),`[0]` = 左电机,`[1]` = 右电机。
- `body_velocities[2]`:车体速度,`[0]` = 线速度(mm/s),`[1]` = 角速度(rad/s)。

正解与逆解的参数名**镜像统一**:正解把 `motor_speeds` 转为 `body_velocities`,逆解把 `body_velocities` 转为 `motor_speeds`,调用时一眼即可看出数据流向。

## 6. 语法与设计特性

### 单精度优先

`PI_F` 与 `MS_TO_S` 均定义为 `static constexpr float`。ESP32-S3 单精度浮点运算更快,使用 `float` 并在运算中使用 `f` 后缀字面量,避免隐式双精度提升。

### 纯算法层,不依赖硬件

`update_motor_speed` 接收 `const int32_t ticks[2]`(编码器读数数组)而非编码器
对象。这样库保持纯算法,不反向依赖硬件层(同时规避了
`Esp32PcntEncoder::getTicks()` 非 const 与常量传参的冲突)。

### 方法内静态采样基线

`update_motor_speed` 内部使用三个 `static` 局部变量维持采样基线:

```cpp
static uint64_t last_update_time = 0;
static int64_t  last_ticks[2]    = {0, 0};
static bool     is_first_run     = true;
```

- `is_first_run` 首次进入时仅建立基线,不提前 return,保证首轮 `dt` 不被放大。
- `dt == 0` 时跳过速度计算(保持上次速度),避免除零。
- 普通临时量(`dt`、`delta_ticks`)仍为栈局部变量,避免无谓的静态化。

### 除零防御

正解中,轮间距未设置或非法(`<= 0`)时角速度直接输出 `0`,避免除零得到 `inf`。

### const 正确性

- `get_motor_speed`、只读 `get_odom` 声明为 `const` 成员。
- 输入数组参数均以 `const` 修饰,表达只读语义。
- 提供 `get_odom()` 可写与 `get_odom() const` 只读两种重载,便于常量对象访问。

### C++11 兼容

静态 `constexpr` 成员在 `.cpp` 中提供类外定义,避免 ODR-use 链接错误。

### 单位约定

| 量                                             | 单位  |
| ---------------------------------------------- | ----- |
| 坐标 `x`/`y`                                   | mm    |
| 角度 `yaw`                                     | rad   |
| 速度 `current_motor_speeds_`/`linear_velocity` | mm/s  |
| 角速度 `angular_velocity`                      | rad/s |
| 时间参数 `now`/`dt`                            | ms    |

`MS_TO_S = 1000.0f` 为 ms 与 s 的换算系数;速度在采样处统一换算为 mm/s,里程计积分前再换算为 m/s(注:当前实现 `linear_velocity` 仍以 mm/s 存储,`update_odom_` 内换算后用于积分)。

## 7. 使用示例

```cpp
Kinematics kine;
kine.set_motor_param(0.06f);   // 标定: 每脉冲前进 0.06 mm
kine.set_wheel_distance(160.0f); // 轮间距 160 mm

// 控制周期内采样编码器(左右读数)并自动更新里程计
kine.update_motor_speed(millis(), left_ticks, right_ticks); // 经适配后为 ticks[2] 数组

// 速度环下发: 目标线速度 200 mm/s, 角速度 0
float body_velocities[2] = {200.0f, 0.0f};
float motor_speeds[2];
kine.kinematics_inverse(body_velocities, motor_speeds);

// 读取里程计
const odom_t& odom = kine.get_odom();
```

## 8. 已知遗留事项

- `main.cpp` 与 `test05`-`test07` 的调用点尚未适配新接口(旧版 `set_motor_param` 双参、`kinematics_inverse` 四参、`update_motor_speed` 三参),待后续统一修改。
