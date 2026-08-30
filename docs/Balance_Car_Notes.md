# 两轮自平衡车笔记

在鱼香 ROS 差速底盘（`fishbot_motion_control`）基础上加装 MPU6050，拆下从动轮，整车退化为一级倒立摆，通过电机闭环控制实现两轮自平衡。

本文档按控制环组织，遵循「先控制理论数学模型，再实际代码工程实现」的顺序书写。当前已完成直立环（`test10_upright`）与速度环（`test11_speed`），转向环留空待补。

## 直立环

直立环是两轮自平衡的基础环，负责抵消重力矩使车体在竖直方向稳定。因为倒立摆天然不稳定，且不施加控制时偏离竖直的角度会指数发散，所以直立环必须快速响应。

### 1. 控制理论

#### 1.1 PD 控制回路

小车直立需要一个快速响应，并且倒立摆模型中小车受到地球重力影响几乎不可能产生稳态误差：因为小车只要没有达到机械中值就会倾倒，所以这里直接省略掉积分控制。下图是 PD 直立环的控制回路。

```text
                ┌───────┐
 期望值(机械中值)─┼─ 中值 ─┼──► PD控制器 ──► 平衡小车 ──► 角度, 角速度反馈
                └───────┘
```

- 期望值：机械中值 \(\theta_0\)，即车体能保持平衡时的倾角。
- 被控量：陀螺仪返回的实际角度 \(\theta\) 与角速度 \(\omega\)。
- 控制器：比例 + 微分（无积分）。

**机械中值**：因为受小车组装等移位影响，小车的真实平衡位置可能不是 0 度，需要自行测试小车平衡时的倾角。此角度才是我们期望的角度。因为只有小车在机械中值时才能保持平衡实现直立，所以 \(\theta_0\) 必须由人工实测获得。

#### 1.2 标准 PID 公式

通过控制回路写出如下公式，标准 PID 为：

$$
u_k = K_p \cdot e_k + K_i \cdot \sum_{j=0}^{k} e_j + K_d \cdot (e_k - e_{k-1})
$$

#### 1.3 化简为 PD

直立环只取比例控制和微分控制，因此公式简化为：

$$
u_k = K_p \cdot e_k + K_d \cdot (e_k - e_{k-1})
$$

#### 1.4 误差与微分项

定义误差为机械中值与陀螺仪实际角度之差：

$$
e_k = \theta_0 - \theta \qquad (\text{误差值} = \text{机械中值} - \text{陀螺仪返回实际角度值})
$$

求误差差分：

$$
e_k - e_{k-1}
$$

为什么可以用角速度表示？因为角速度的定义是单位时间的角度变化，且机械中值恒定不变。代入公式（\(\theta\) 记为当前时刻 \(\theta_k\)，\(\theta'\) 记为上一时刻 \(\theta_{k-1}\)）：

$$
e_k = \theta_0 - \theta_k
$$

$$
e_{k-1} = \theta_0 - \theta_{k-1}
$$

$$
e_k - e_{k-1} = (\theta_0 - \theta_k) - (\theta_0 - \theta_{k-1}) = \theta_{k-1} - \theta_k = -(\theta_k - \theta_{k-1})
$$

于是误差差分与「角速度」是**相反数**关系：

$$
e_k - e_{k-1} = -\omega \qquad (\omega:\text{陀螺仪返回实际角速度})
$$

**为何直接采用角速度**：PD 控制算法是离散的，每间隔固定时间计算一次。虽然这里计算出的并不是实际角速度（因为计算间隔不是 1 秒），但 \(K_d\) 系数最后是要去调整的，和实际角速度是成比例的。为了减少程序的运算量，这里就直接采用陀螺仪输出的角速度 \(\omega\)；与之相对，误差差分取 \(-\omega\)（负号将并入 \(K_d\)）。

#### 1.5 最终直立环公式

将上面的公式整合（注意微分项由误差差分推导出的是 \(-\omega\)，故 \(K_d\) 项系数为负）得到实际的 PD 公式：

$$
\mathrm{PWM} = K_p \cdot (\theta_0 - \theta) - K_d \cdot \omega
$$

#### 1.6 变量与符号约定

先约定坐标方向：车体前进方向为 \(x\) 负方向，故「前倾」时 MPU6050 `getAngleY` 读出正值。为统一「前倾为正」的控制坐标系，在代码读取处对 \(\theta\)、\(\omega\) 各取负号（`theta = -getAngleY()`、`omega = -getGyroY()`），即下表 \(\theta\)/\(\omega\) 均为**控制坐标**而非传感器原始读数。前提：正 PWM 驱动两轮向车头（前进）方向转动；若实测反向，应反接电机而不是取负（见 2.5）。

| 符号         | 代码变量         | 含义                         | 来源                       |
| ------------ | ---------------- | ---------------------------- | -------------------------- |
| \(\theta_0\) | `zero_pitch_deg` | 机械中值（期望倾角）         | 人工实测（`'c'` 标定）     |
| \(\theta\)   | `theta`          | 控制俯仰角（前倾为正）       | `theta = -mpu.getAngleY()` |
| \(\omega\)   | `omega`          | 控制角速度（前倾方向为正）   | `omega = -mpu.getGyroY()`  |
| \(K_p\)      | `BALANCE_KP`     | 比例增益，单位 `PWM/deg`     | 待实测整定                 |
| \(K_d\)      | `BALANCE_KD`     | 微分增益，单位 `PWM/(deg/s)` | 待实测整定                 |

公式中的 \(\theta\) 和 \(\omega\) 为控制坐标（为取负后的陀螺仪读数），\(\theta_0\) 由我们手动测得。注意 \(K_d\) 项系数为负（见 1.4 推导），与标准 PID 中「误差差分 \(e_k - e_{k-1}\)」方向一致——因为该差分等于 \(-\omega\)。

代码中该 D 项由 `PIDController::update_pwm_upright` 实现：其内部取 `d_error = -rate`（`rate` 即 \(\omega\)），代入 \(u_k = K_p \cdot e_k + K_d \cdot d\_error\) 恰得 \(K_p \cdot (\theta_0 - \theta) - K_d \cdot \omega\)。

### 2. 固件工程实现

固件为 `src/tests/test10_upright`，仅实现直立环 PD。核心控制算法严格按 1.5 的公式逐符号复现，见 2.1。

#### 2.1 核心算法

直立环 PD 直接交由 `PIDController` 库实现（外部微分变体），不再手写，也无额外封装函数。完整调用（源码见 `src/tests/test10_upright/main.cpp`）：

```cpp
// setup: 配置 P/I/D 增益与输出限幅 (直立环为纯 PD, 库层强制无 I 项, BALANCE_KI 仅占位)
balance_pid.update_pid(BALANCE_KP, BALANCE_KI, BALANCE_KD);
balance_pid.output_limit(BALANCE_PWM_LIMIT);

// 每 5ms 控制周期: 读 IMU 后直接调用库计算直立环输出
const float theta = -mpu.getAngleY();  // 控制俯仰角(前倾为正): 前进方向为 -X, 读取处取负
const float omega = -mpu.getGyroY();   // 控制角速度(前倾方向为正)

const float inputs[2] = { theta, omega };               // [角度, 角速度]
// update_pwm_upright: 目标角度直接入参 (此处为机械中值 theta_0; 串级时改为 速度环输出 + theta_0)
const int16_t pwm_balance = balance_pid.update_pwm_upright(zero_pitch_deg, inputs); // = Kp*(theta_0 - theta) - Kd*omega
```

- `zero_pitch_deg` 即符号 \(\theta_0\)（机械中值），初始取自 `config.h` 的 `BALANCE_ZERO_PITCH_DEG`，可由串口 `'c'` 在线标定。
- `theta`、`omega` 在读取处取负，统一「前倾为正」的控制坐标（见 1.6 方向约定），使被测角速度经 `update_pwm_upright` 内部 `d_error = -rate` 后恰好得到 \(-K_d \cdot \omega\)。
- `update_pwm_upright` 内部取 `d_error = -rate`（`rate` 即取负后的 \(\omega\)），输出 \(K_p \cdot (\theta_0 - \theta) - K_d \cdot \omega\)，与 1.5 一致；输出限幅到 `±output_limit_` 并四舍五入取整，均封装于库内。
- 起控进入 `kRunning` 前调用 `balance_pid.reset()`，清零上一拍的误差差分/积分，避免停车或标定期间的残留影响首次输出。
- **极性校验先于调参（人工串口观察）**：手扶车体前倾，轮子应向车头方向追；若反向，对调 `config.h` 中该电机 `PIN_A/PIN_B` 定义。可从串口状态行 `theta` 变化趋势与 `pwm` 符号人工判断方向（已移除自检命令，见 2.3）。

#### 2.2 引脚与节拍

| 项目     | 值          | 说明                                                  |
| -------- | ----------- | ----------------------------------------------------- |
| I2C SDA  | GPIO10      | 从空闲集合 {3,46,9,10,11,12} 避开 strapping 3/46 选取 |
| I2C SCL  | GPIO9       | 同上                                                  |
| 控制节拍 | 5ms (200Hz) | FreeRTOS 任务 `vTaskDelayUntil` 固定周期，钉 core1    |
| 串口     | 115200      | 命令输入 + 状态文本输出（纯英文）                     |

#### 2.3 串口命令

| 命令 | 功能             | 说明                                      |
| ---- | ---------------- | ----------------------------------------- |
| `s`  | 启停武装切换     | 武装后姿态进入中值窗口才真正起控          |
| `c`  | 机械中值在线标定 | 扶直静止后发送，取 0.2s 平均 `theta` 生效 |

#### 2.4 状态机与安全

- 上电默认 `kIdle` 停止，消除"平放上电误起控"问题（见 2.5）；
- 起控条件：已武装 且 \(|\theta - \theta_0| < 8^\circ\)；
- 倒地保护：\(|\theta - \theta_0| > 45^\circ\) 自动解除武装并停机，扶正后可重新起控。

#### 2.5 调试记录

##### "平放一直前进"根因与修复

根因：MPU6050_light 的角度基准是上电校准时刻的姿态——平放上电，平放即恒读 0°，旧版 \(|\theta|<8^\circ\) 起控条件被永久满足，一上电就闭环；而 `ZERO_PITCH=0` 未标定真实直立中值 \(\theta_0\)，P 项恒有偏差输出，单向漂移狂奔；前进加速度的反作用力矩使车身后仰，松手即倒。

修复：上电默认停止，串口命令显式控制启停（`'s'`），并在起控前复位控制状态。

##### 推荐调试流程

1. 平放静置上电，等校准完成提示；
2. 手扶车体前倾观察串口 `theta` 变号趋势，确认轮子向车头追（极性人工校验，若反向对调 `PIN_A/PIN_B`）；
3. 手扶车体大致直立，发 `c`，看 `[CALIB] zero_pitch=xx.xx`；
4. 发 `s` 武装，松手观察；串口每 100ms 输出一行状态文本：
   `state=RUN theta=1.23 omega=-0.45 pwm=12`。

## 速度环

速度环负责控制小车前后运动速度。目标：给定期望速度（编码器反馈 + PI 控制），通过调整直立环的平衡点角度来驱动小车，而不能直接调 PWM。串级 PID 结构下，速度环为**外环**，其输出作为直立环（内环）的期望角度输入。控制理论见下，固件实现见 4。

### 3. 控制理论

#### 3.1 PI 控制回路

速度调节过程中希望速度变化平缓且连续，因此舍去微分控制（D 项），以防高频振动现象产生。速度环采用 **PI 控制器**，与直立环构成串级 PID：外环为速度环，速度环的输出作为内环（直立环）的输入，内环直接作用到驱动器；这里外环的输出值表示**期望的角度值**。

信号流向如下：

1. 给定**期望速度** \(v_{set}\)（目标编码器速度）；
2. 与**编码器反馈速度** \(v\) 相减得误差 \(e_k = v_{set} - v\)；
3. 速度环 PI 控制器输出 `output`（期望角度增量）；
4. 将 `output` 叠加到机械中值 \(\theta_0\) 上，作为直立环 PD 控制器的输入（期望角度）；
5. 直立环输出 PWM 直接作用到驱动器。

**为何速度反馈直接用编码器数值**：速度的定义是单位时间内物体的位移。物理世界的 m/s 单位与编码器数值呈比例关系，为计算简便，直接采用编码器数值在单位时间内的变化量表示速度。又因为 PI 计算公式是按固定周期计算的，且编码器在固定时间内读取后直接清零，因此直接读取编码器数值就可以表示速度。

#### 3.2 PI 控制回路公式推导

通过控制回路写出如下公式，标准 PID：

$$
u_k = K_p \cdot e_k + K_i \cdot \sum_{j=0}^{k} e_j + K_d \cdot (e_k - e_{k-1})
$$

速度环只取比例控制与积分控制（舍去微分，见 3.1），因此公式简化为：

$$
u_k = K_p' \cdot e_k + K_i' \cdot \sum_{j=0}^{k} e_j
$$

定义速度误差为期望速度与实际速度之差：

$$
e_k = v_{set} - v \qquad (\text{误差值} = \text{期望速度} - \text{实际速度})
$$

将上面的公式整合得到实际的 PI 公式（速度环）：

$$
\mathrm{output} = K_p' \cdot (v_{set} - v) + K_i' \cdot \sum_{j=0}^{k} e_j
$$

#### 3.3 串级控制公式推导

将速度环与直立环串联，推导完整控制公式。

① 直立环：

$$
\mathrm{PWM} = K_p \cdot (\theta_0 - \theta) - K_d \cdot \omega
$$

② 速度环（见 3.2）：

$$
\mathrm{output} = K_p' \cdot (v_{set} - v) + K_i' \cdot \sum_{j=0}^{k} e_j
$$

③ 将 \(\mathrm{output} + \theta_0\) 作为直立环的输入代入①：由于速度环输出的期望角度是基于机械中值基础的，因此需要加上 \(\theta_0\) 之后再带入——直立环的输入就是机械中值，因此用 \(\mathrm{output} + \theta_0\) 替换 \(\theta_0\)：

$$
\mathrm{PWM} = K_p \cdot \left( (\mathrm{output} + \theta_0) - \theta \right) - K_d \cdot \omega
$$

④ 将上式展开并代回速度环，得到双环完整公式：

$$
\mathrm{PWM} = K_p \cdot \left[ K_p' \cdot (v_{set} - v) + K_i' \cdot \sum_{j=0}^{k} e_j \right] + K_p \cdot \theta_0 - K_p \cdot \theta - K_d \cdot \omega
$$

展开后的绿色部分正是速度环项，红色部分正是直立环项：

$$
\mathrm{PWM} = \underbrace{K_p \cdot K_p' \cdot (v_{set} - v) + K_p \cdot K_i' \cdot \sum_{j=0}^{k} e_j}_{\color{green}{\text{速度环项}}} + \underbrace{K_p \cdot \theta_0 - K_p \cdot \theta - K_d \cdot \omega}_{\color{red}\text{直立环项}}
$$

因此这个双环 PID 的控制代码，相当于把两个环的输出结果求和后传入电机驱动器。

### 4. 固件工程实现

速度环固件为 `src/tests/test11_speed`，在 `test10_upright` 直立环基础上新增编码器测速与速度环 PI，构成串级。核心控制算法严格按 3.3 的公式逐符号复现，见 4.1。

#### 4.1 核心算法

速度环输出经 `PIDController::update_pwm_speed` 计算（PI 变体，无 D 项），再与机械中值叠加作为直立环目标角度，交由 `update_pwm_upright`（纯 PD）。完整调用（源码见 `src/tests/test11_speed/main.cpp`）：

```cpp
// setup: 配置速度环 PI (外环, 无 D 项) 与直立环 PD (内环, 库层强制纯 PD)
speed_pid.update_pid(SPEED_KP, SPEED_KI, 0.0f);
speed_pid.output_limit(SPEED_OUTPUT_LIMIT);
balance_pid.update_pid(BALANCE_KP, BALANCE_KI, BALANCE_KD);
balance_pid.output_limit(BALANCE_PWM_LIMIT);

// 每 5ms 控制周期: 读 IMU + 编码器测速 (两轮平均, mm/s)
const float theta = -mpu.getAngleY();       // 控制俯仰角(前倾为正)
const float omega = -mpu.getGyroY();        // 控制角速度(前倾方向为正)
const float speed_mm_s = measure_speed_mm_s(); // 差值法同 test03, 单位 mm/s

// 速度环 (外环, PI): output = Kp'*(v_set - v) + Ki'*Σe, 输出为期望角度增量 (deg)
const int16_t speed_output = speed_pid.update_pwm_speed(target_speed_mm_s, speed_mm_s);

// 串级嵌套: 直立环目标角度 = 速度环输出 + 机械中值 theta_0 (docs 3.3 公式 ③)
const float target_angle = static_cast<float>(speed_output) + zero_pitch_deg;

// 直立环 (内环, PD): = Kp*(target_angle - theta) - Kd*omega
const float inputs[2] = { theta, omega }; // [角度, 角速度]
const int16_t pwm_balance = balance_pid.update_pwm_upright(target_angle, inputs);
```

- 测速实现 `measure_speed_mm_s()`：每控制周期读两路编码器 tick 差值，乘以单脉冲距离并除以时间差得 mm/s（与 `test03_speed_trans` 同法），左右轮平均作为车体前进速度 \(v\)。
- `target_speed_mm_s` 即符号 \(v_{set}\)（期望速度，mm/s），初始取自 `config.h` 的 `SPEED_SETPOINT_MM_S`，串口 `'w'`/`'x'` 按 `SPEED_STEP_MM_S` 步进调整，`'v'` 回显。
- `update_pwm_speed` 输出经四舍五入取整为 `int16_t`，故期望角度增量分辨率为 1°；`SPEED_OUTPUT_LIMIT` 限制目标角偏离 \(\theta_0\) 的幅度，防止外环积分饱和时目标角过大而失衡。
- 起控进入 `kRunning` 前同时 `reset()` 速度环与直立环（清积分/差分状态），避免停车期间的积分残留。
- 两环均为 `PIDController` 独立实例，方法类内互不调用，串级嵌套在 `control_step` 调用方实现。

#### 4.2 串口命令

| 命令 | 功能             | 说明                                            |
| ---- | ---------------- | ----------------------------------------------- |
| `s`  | 启停武装切换     | 同 test10                                       |
| `c`  | 机械中值在线标定 | 同 test10                                       |
| `w`  | 目标速度 +步进   | 每按一次 `target_speed_mm_s += SPEED_STEP_MM_S` |
| `x`  | 目标速度 -步进   | 每按一次 `target_speed_mm_s -= SPEED_STEP_MM_S` |
| `v`  | 回显目标速度     | 打印当前 `target_speed_mm_s`                    |

#### 4.3 状态机与安全

与 test10 一致：上电默认 `kIdle`，`'s'` 武装后姿态进入中值窗口起控；速度环在 `kRunning` 内恒生效（目标速度默认 0，即先验证纯直立，再 `'w'` 提速）。倒地保护 `|\theta - \theta_0| > 45^\circ` 自动停机。

#### 4.4 调参指南

1. 先在 `SPEED_SETPOINT_MM_S = 0` 下验证直立环（此时速度环无扰动，行为同 test10）；
2. 发 `'w'` 给正速度，观察车体是否前倾加速并稳定在目标速度附近；若振荡，减小 `SPEED_KP`；若速度收敛过慢或存在稳态偏差，增大 `SPEED_KI`；
3. 编码器方向校验：前进时串口 `speed` 应为正；若为负，反接编码器 `PIN_A/PIN_B`（同电机极性约定）。

## 转向环

转向环负责控制小车转向。目标：给定期望角速度（gyro_z 或 yaw 误差 + P/PD），通过左右轮差速量实现转向。此环尚未实现，留空待补。
