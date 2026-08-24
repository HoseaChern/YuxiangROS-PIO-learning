# PID 库优化便签

记录 PID 库到目前为止的全部优化,按时间顺序排列,含对应的 git 提交(branch: `main`)。

## 优化清单

| 序号 | 优化内容                 | 说明                                                                                                                              | 提交               |
| ---- | ------------------------ | --------------------------------------------------------------------------------------------------------------------------------- | ------------------ |
| 1    | 输出限幅单参对称化       | `output_max_`/`output_min_` 双成员合并为 `output_limit_` 单成员,接口由双参改为单参对称限幅;`-1 * x` 简化为 `-x`                    | `95e6b75`          |
| 2    | 接口改名与冗余成员移除   | `update` 改名为 `update_pwm`,语义更明确;移除无用成员 `error_pre_last_`;浮点字面量统一加 `f` 后缀(`0.0` -> `0.0f`)                 | `5cb6b9c`          |
| 3    | 返回类型窄化             | `update_pwm` 返回 `float` 改为 `int16_t` 并四舍五入取整,避免直接截断导致低速占空比系统性偏小;依赖由 `<Arduino.h>` 收缩为 `<cstdint>` | `ac67527`          |
| 4    | 源文件格式统一           | 修复 Kinematics/PIDController 源文件末尾缺失换行符;同改动在 main 与 perf 分支各提交一次                                          | `05436b1` / `5af66c5` |
| 5    | 积分上界编译期常量       | 魔法数 `2500.0f` 提取为 `static constexpr float INTEGRAL_SUP_LIMIT`,删除拼写错误的成员 `intergral_sup_`                            | `27a99dc`          |
| 6    | 删除冗余 include         | `.cpp` 中重复包含的 `<cstdint>` 删除(头文件已包含)                                                                                | `d459446`          |
| 7    | 初始化与命名重构         | 构造函数改初始化列表消除双重 reset;成员类内默认初始化消除 UB;`update_PID` 改名 `update_pid` 且不再内部 `reset()`;限幅收敛为 `std::fmin`/`std::fmax` | `36ca49f` |
| 8    | 补充类外定义             | C++11 下静态 `constexpr` 成员提供类外定义,避免 ODR-use 链接错误                                                                   | `dd830d3`          |
| 9    | 代码格式化               | 构造函数由多行展开收敛为单行,统一风格                                                                                             | `296597d`          |

## 各优化要点说明

### 1. 输出限幅单参对称化(序号 1)

原接口 `output_limit(float out_min, float out_max)` 与成员 `output_min_`/`output_max_` 可表达非对称限幅,但实际使用场景均为对称限幅。重构为单成员 `output_limit_` 与单参接口,限幅逻辑简化为 `±output_limit_` 比较,并顺带将 `-1 * x` 简化为 `-x`。

### 2. 接口改名与冗余成员移除(序号 2)

- `update` 改名 `update_pwm`:方法用途是计算 PWM 输出值,新名避免与通用 `update` 混淆。
- 移除 `error_pre_last_`(上上次误差):微分项 `d_error_` 只依赖 `error_last_` 与当前误差,该成员从未被使用。
- 全部浮点字面量统一加 `f` 后缀,避免隐式双精度提升。

### 3. 返回类型窄化(序号 3)

`update_pwm` 返回类型由 `float` 改为 `int16_t`:

```cpp
return static_cast<int16_t>(output >= 0.0f ? output + 0.5f : output - 0.5f);
```

直接截断会让 `99.6 -> 99`,低速时占空比系统性偏小;四舍五入后 `99.6 -> 100`,负数同样处理。输出已被限幅为 `±output_limit_`(远小于 int16_t 范围),转换无溢出风险。同时依赖由宽泛的 `<Arduino.h>` 收缩为 `<cstdint>`。

### 4. 源文件格式统一(序号 4)

修复 Kinematics/PIDController 源文件末尾缺失换行符,消除 POSIX 换行符警告。同一改动在 main(`05436b1`)与 perf 分支(`5af66c5`)各提交一次。

### 5. 积分上界编译期常量(序号 5)

魔法数 `2500.0f` 提取为编译期常量:

```cpp
static constexpr float INTEGRAL_SUP_LIMIT = 2500.0f;
```

删除拼写错误的成员 `intergral_sup_`(正确拼写为 integral),积分限幅比较统一使用编译期常量。

### 6. 删除冗余 include(序号 6)

`.cpp` 中重复包含 `<cstdint>`(头文件已包含),删除。

### 7. 初始化与命名重构(序号 7)

- **消除双重 reset**:原构造函数先调 `reset()` 再调 `update_PID()`,而 `update_PID()` 内部又调一次 `reset()`,实际执行了两次。重构后构造函数改初始化列表,仅覆盖 PID 系数:

```cpp
PIDController::PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}
```

- **成员默认初始化**:全部成员类内初始化 `= 0.0f`,默认构造后直接调用 `update_pwm()` 不再有未定义行为。
- **改名与陷阱消除**:`update_PID` 改名 `update_pid`(与 `update_pwm`/`update_target` 的 snake_case 风格统一),且不再内部调用 `reset()`。原实现在运行中调整参数时会把 `output_limit_` 静默清零,导致后续输出恒为 0;新实现如需重置状态,由调用者显式调用 `reset()`。
- **限幅收敛**:积分限幅与输出限幅分别收敛为 `std::fmax(-LIMIT, std::fmin(LIMIT, x))` 单行,并引入 `<cmath>`。

### 8. 补充类外定义(序号 8)

C++11 下静态 `constexpr` 成员若被 ODR-use(如取地址、绑定引用),需要类外定义:

```cpp
constexpr float PIDController::INTEGRAL_SUP_LIMIT;
```

### 9. 代码格式化(序号 9)

构造函数由多行展开收敛为单行,统一整体代码风格。

## 提交链总览

```text
5b662a0  Initial commit (初始形态: 双参限幅, error_pre_last_, 魔法数, float 返回, 依赖 Arduino.h)
  -> 95e6b75  输出限幅单参对称化
  -> 5cb6b9c  update -> update_pwm, 移除 error_pre_last_, 统一 f 后缀
  -> ac67527  返回 int16_t + 四舍五入, <Arduino.h> -> <cstdint>
  -> 05436b1  源文件末尾换行符修复 (main)
  -> 5af66c5  与 main 同步修复 (perf 分支)
  -> 27a99dc  积分上界 -> INTEGRAL_SUP_LIMIT 编译期常量
  -> d459446  删除冗余 include
  -> 36ca49f  初始化列表 + 成员默认初始化 + update_PID 改名 + 限幅收敛
  -> dd830d3  补充类外定义
  -> 296597d  代码格式化
```

## 遗留事项

- 编译验证(ESP32 目标)尚未执行,待 `pio run` 确认。
- 本便签后续优化记录需同步追加。
