# Kinematics 库优化便签

记录 Kinematics 库到目前为止的全部优化,按时间顺序排列,含对应的 git 提交(branch: `kinematics_perf`)。

## 优化清单

| 序号 | 优化内容 | 说明 | 提交 |
| --- | --- | --- | --- |
| 1 | 单位换算常量提取 | 魔法数 1000 提取为编译期常量 `MS_TO_S`,消除散落的魔法数 | `b5e153e` |
| 2 | 速度计算重排 | `update_motor_speed` 内部统一"先转换后运算",消除冗余 cast;引入 `dt == 0` 时跳过速度计算的防御 | `6aea787` |
| 3 | 常量收敛与依赖收缩 | PI 宏替换为单精度常量 `PI_F`;头文件依赖收缩为 `cmath`/`cstdint`,移除宽泛头文件 | `cb81db2` |
| 4 | 源文件格式统一 | 与 main 同步,修复 Kinematics/PIDController 源文件末尾缺失换行符 | `5af66c5` |
| 5 | 接口参数化重构 | 拆除 `motor_param_t` 结构体,`distance_per_tick_mm_` 标量化(两电机共用,不再独立标定) | `8fb5b5d` |
| 6 | 采样基线静态化 | `update_motor_speed` 改用方法内静态三件套(`last_update_time`/`last_ticks`/`is_first_run`),`is_first_run` 只建基线不提前 return,`dt == 0` 跳过速度计算 | `8fb5b5d` |
| 7 | 捆绑指针传参 | 正逆解改为 `motor_speeds[2]` 与 `body_velocities[2]` 捆绑传参,参数名镜像统一;`update_motor_speed` 传 `const int32_t ticks[2]` 而非 encoder 对象,库保持纯算法不依赖硬件 | `4ed6329` |
| 8 | 文档注释同步 | 函数块注释与接口签名同步更新 | `3434957` |
| 9 | 合并参数分支 | `kinematics_param` 以非快进方式合并入 `kinematics_perf` | `8fecf73` |
| 10 | 预清理 | 注释空白对齐等格式整理 | `7855dbd` |
| 11 | 角度归一化重写 | `TransAngleInPI` 改用 `std::fmod` 单参归一化。原实现"判断用 angle、修改用 output_angle"仅当两参传同一变量才正确,且单次加减 2*PI 不收敛;新实现多次旋转可收敛,消除累积误差 | `1039347` |
| 12 | 时间差窄化 | `update_odom`/`update_motor_speed` 的时间差 `dt` 由 `uint32_t` 改为 `uint64_t`,避免毫秒计数长时间运行溢出 | `bf0e2c5` |
| 13 | const 重载 | `get_odom()` 提供可写与只读 const 重载,便于常量对象访问 | `604ee51` |
| 14 | 除零防御 | 正解中轮间距 `<= 0` 时角速度输出 0,避免除零得 `inf` | `1e1efdb` |
| 15 | const 化审查 | `get_motor_speed` 等只读方法加 `const` 修饰,输入参数以 const 表达只读语义 | `430b96a` |
| 16 | 代码格式化 | 单行实现展开为多行,头文件注释对齐,统一风格 | `1d737bd` |

## 各优化要点说明

### 1. 单位换算常量提取(序号 1)

速度换算中的魔法数 `1000` 提取为编译期常量 `MS_TO_S`(值为 `1000.0f`),并在 `.cpp` 中提供 C++11 类外定义,避免 ODR-use 链接错误。

### 2. 速度计算重排(序号 2)

`update_motor_speed` 内部由"先乘后转"改为"统一先转换后运算":

```cpp
static_cast<float>(delta_ticks) * distance_per_tick_mm_ / static_cast<float>(dt) * MS_TO_S;
```

消除了 `delta_ticks * distance` 先乘时产生的冗余 cast,同时引入 `dt != 0` 判断,`dt == 0` 时跳过速度计算。

### 3. 常量收敛与依赖收缩(序号 3)

PI 宏替换为单精度常量 `PI_F`,运算字面量统一加 `f` 后缀(如 `/ 2.0f`),避免隐式双精度提升;头文件仅保留 `cmath`/`cstdint`。

### 4. 源文件格式统一(序号 4)

与 main 同步,统一修复 Kinematics/PIDController 源文件末尾缺失换行符,消除 POSIX 换行符警告。

### 5. 标量化(序号 5)

原 `motor_param_t` 结构体被拆除,`distance_per_tick_mm_` 变为标量,左右两电机共用同一标定值。原因:两轮差速机器人左右轮机械参数一致,无需独立标定。

### 6. 静态采样基线(序号 6)

`update_motor_speed` 内部以方法内 `static` 变量维持采样基线,与 test03/04 逐行同构:

```cpp
static uint64_t last_update_time = 0;
static int64_t  last_ticks[2]    = {0, 0};
static bool     is_first_run     = true;
```

`is_first_run` 首次进入仅建立基线、不提前 return,保证首轮控制周期 `dt` 正常;`dt == 0` 时跳过速度计算并保持上次速度。

### 7. 指针捆绑传参(序号 7)

- 命名区分"电机转速"(`motor_speeds`)与"车体速度"(`body_velocities`),避免语义混淆。
- 正解/逆解参数名镜像统一,一眼可看出数据流向。
- `update_motor_speed` 接收 `const int32_t ticks[2]` 而非 encoder 对象,原因有二:一是 `Esp32PcntEncoder::getTicks()` 非 const 与常量传参冲突;二是避免算法层反向依赖硬件层,保持纯算法。

### 8. 角度归一化重写(序号 11)

原实现缺陷:判断条件用 `angle`、修改却用 `output_angle`,仅在调用点两参传同一变量时才正确;且单次 `+/- 2*PI` 无法保证收敛。新实现:

```cpp
angle = std::fmod(angle + PI_F, 2.0f * PI_F);
if (angle < 0.0f) angle += 2.0f * PI_F;
angle -= PI_F;
```

先平移 PI 取模再平移回来,任意多次旋转均可收敛到 [-PI, PI]。

### 9. 时间差窄化(序号 12)

`dt` 类型由 `uint32_t` 改为 `uint64_t`。ESP32 `millis()` 为 32 位毫秒计数,但里程计/采样逻辑未来可能使用 64 位时间源,避免溢出导致速度计算错误。

### 10. 除零防御与 const 化(序号 13-15)

- 正解角速度计算前先判断 `wheel_distance_ > 0.0f`,未设置轮间距时输出 0。
- 所有只读接口(如 `get_motor_speed`)加 `const` 修饰,常量对象可用。
- `get_odom()` 双版本重载:可写版本供内部/调试修改,const 版本供只读访问。

## 提交链总览

```text
b5e153e  单位换算魔法数 1000 提取为 MS_TO_S
  -> 6aea787  先转换后运算, 消除冗余 cast, 引入 dt==0 防御
  -> cb81db2  PI 宏 -> PI_F, 头文件依赖收缩
  -> 5af66c5  源文件末尾换行符修复 (与 main 同步)
  -> 8fb5b5d  标量化 + 静态三件套
  -> 4ed6329  指针捆绑传参
  -> 3434957  docs 注释同步
  -> 8fecf73  --no-ff 合并 kinematics_param
  -> 7855dbd  预清理(注释空白对齐)
  -> 1039347  TransAngleInPI 重写
  -> bf0e2c5  dt 窄化为 uint64_t
  -> 604ee51  get_odom const 重载
  -> 1e1efdb  除零防御
  -> 430b96a  get_motor_speed const 化
  -> 1d737bd  代码格式化
```

## 遗留事项

- `main.cpp` 与 `test05`-`test07` 调用点未适配新接口,待统一修改。
- 本便签后续优化记录需同步追加。
