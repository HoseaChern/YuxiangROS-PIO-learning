# PIDController 库

## 1. 库的作用

PIDController 是 YuxiangROS-PIO-learning(ESP32-S3, PlatformIO)项目中的位置式 PID 控制器库,为电机速度环提供闭环控制:

- **控制计算**:由目标值与当前值的误差,按比例、积分、微分三项求和得到控制输出(PWM 占空比)。
- **限幅保护**:积分项限幅防止积分饱和,输出项限幅保护执行机构。
- **参数与状态管理**:支持运行时更新目标值、调整 PID 系数、设置输出限幅;参数统一通过 `update_pid()` 设定。

库为**纯算法层**,仅依赖 `<cstdint>` 与 `<cmath>`,不依赖 Arduino 头文件与硬件外设,便于单元测试与跨平台移植。

## 2. 文件说明

| 文件               | 说明                                 |
| ------------------ | ------------------------------------ |
| `PIDController.h`  | 头文件,声明 `PIDController` 类       |
| `PIDController.cpp`| 实现文件,包含 PID 核心算法与状态管理 |
| `docs/README.md`   | 本文档,库的使用说明                  |

库的历次优化与提交记录已并入仓库根目录 [README.md](../../../README.md) 的
"PIDController 控制器库优化（9 项）"章节。

## 3. 核心公式

离散位置式 PID:

```text
error      = target - current
error_sum += error            (限幅于 ±INTEGRAL_SUP_LIMIT)
d_error    = error_last - error   (注意: 为误差变化率取负)
output     = Kp * error + Ki * error_sum + Kd * d_error  (限幅于 ±output_limit_)
```

说明:

- 积分项持续累积误差,`INTEGRAL_SUP_LIMIT` 为编译期常量(默认 `2500.0f`),防止积分饱和。
- 微分项 `d_error_` 存储"上次误差 - 当前误差",即标准误差变化率的**负值**,调参时需注意符号约定。
- 输出限幅为对称限幅,由 `output_limit()` 设置。

## 4. 公共接口

| 方法                                            | 说明                                                       |
| ----------------------------------------------- | ---------------------------------------------------------- |
| `PIDController()`                               | 默认构造,全部成员类内初始化为 0,构造后即可安全使用         |
| `void update_pid(float kp, float ki, float kd)` | 更新 PID 系数,不重置内部状态                               |
| `void output_limit(float limit)`                | 设置输出限幅,对称限制在 [-limit, limit]                    |
| `void update_target(float target)`              | 更新目标值                                                 |
| `int16_t update_pwm(float current)`             | 输入当前值,返回 PWM 输出值(四舍五入取整)                   |

## 5. 语法与设计特性

### 位置式 PID 与积分限幅

控制器累加历史误差(位置式),积分项带 `INTEGRAL_SUP_LIMIT` 上下限,防止长时间偏差导致积分饱和、输出失控。

### 单精度优先

运算全程使用 `float` 与 `f` 后缀字面量,避免隐式双精度提升,适配 ESP32-S3 单精度浮点运算。

### 编译期常量与 C++11 兼容

积分上界定义为 `static constexpr float INTEGRAL_SUP_LIMIT`,并在 `.cpp` 中提供类外定义,避免 C++11 下 ODR-use 链接错误。

### 成员默认初始化

全部成员类内初始化 `= 0.0f`,默认构造后直接调用 `update_pwm()` 不产生未定义行为。

### 限幅收敛

积分限幅与输出限幅统一使用 `std::fmax`/`std::fmin` 单行收敛,减少重复分支。

### 四舍五入取整

`update_pwm` 返回 `int16_t`,采用四舍五入而非直接截断:直接截断会让
`99.6 -> 99`,低速时占空比系统性偏小;四舍五入后 `99.6 -> 100`,负数同样处理。
输出已限幅为 `±output_limit_`(远小于 int16_t 范围),转换无溢出风险。

### 纯算法层,不依赖硬件

库仅包含 `<cstdint>`(int16_t)与 `<cmath>`(fmin/fmax),不包含 Arduino 头文件,可在任意 C++11 环境编译测试。

## 6. 使用示例

```cpp
PIDController pid;
pid.update_pid(1.0f, 0.1f, 0.05f); // 设置 PID 系数
pid.output_limit(100.0f);           // 输出限幅 ±100
pid.update_target(200.0f);          // 目标速度 200 mm/s

// 控制周期内调用: 传入当前速度, 返回 PWM 输出值
int16_t pwm = pid.update_pwm(current_speed);
```

## 7. 注意事项

- `update_pid()` 不重置内部状态,积分与微分历史保留;对象为固件级静态实例,生命周期与固件相同,上电即全新,无需(也不存在)重置接口。
- 微分项符号为"上次误差 - 当前误差"(误差变化率取负),调参时注意 `Kd` 符号与标准公式的差异。
- 构造后需先调用 `output_limit()` 设置输出限幅,否则输出恒为 0(默认 `output_limit_ = 0.0f`)。
