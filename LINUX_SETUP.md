# PIO 工作区 C/C++ 工具链配置文档

本文档供另一台机器上的 agent 离线执行，在 Ubuntu 24.04 工作区为
PlatformIO + ESP32-S3 + micro-ROS 项目配置 C/C++ 开发环境。当前工作区
（`fishbot_motion_control`）即本模板的本体，仓库内已入库 `.clangd`、
`.clang-format` 两份规范文件，`.vscode/` 按决策不入库、以本文档传递。

规范参考：`/home/changli/Documents/C_project/LINUX_SETUP.md`（桌面 gcc 学习
场景）。本文档针对嵌入式交叉编译场景改写，两场景差异见"与 C_project 模板的
差异"章节。

## 背景与目标

嵌入式侧采用"PIO 交叉编译 + LLVM 开发表层 + gdb 调试"的组合：

- 编译：`xtensa-esp32s3-elf-g++` 8.4.0（PlatformIO espressif32 6.13.0 自带）
- 智能提示：clangd 21，格式化：clang-format 21
- 调试：PlatformIO 的 `platformio-debug`（gdb），保留 PIO 原生链路
- cpptools 扩展保留（PIO IDE 硬依赖）但禁用其 IntelliSense，由 clangd 接管

> 版本说明：LLVM 工具链为 21.1.8（clangd / clang-format），PIO Core 6.1.19，
> espressif32 平台 6.13.0。升级任一版本时需同步核对本文档相关位置。
>
> 维护指引：更换开发机时，重点核对三处路径：工具链路径
> `~/.platformio/packages/toolchain-xtensa-esp32s3/bin`、clangd 路径
> `/usr/bin/clangd`、compile_commands.json 的生成方式（见步骤四）。

## 工具链纵览：PIO 交叉编译 + LLVM 开发表层 + gdb 调试

当前工作区采用三层组合，各层职责独立、互不干扰。这是本模板的核心架构决策。

### 三层结构

| 层      | 工具                          | 版本         | 职责                                |
| ------- | ----------------------------- | ------------ | ----------------------------------- |
| 编译层  | xtensa-esp32s3-elf-g++ (gcc)  | 8.4.0        | 唯一生产编译路径，产出固件          |
| 开发表层 | clangd / clang-format         | 21.1.8       | 智能提示、格式化                    |
| 调试层  | platformio-debug (gdb)        | 随 PIO 包    | F5 调试，加载 firmware.elf          |

### 为什么这样组合

- 编译层必须用 PIO 自带的 xtensa 交叉编译器：ESP32-S3 是 Xtensa LX7 架构，
  Arduino framework 与 micro-ROS 预编译库都以该工具链为编译底线，不可替换。
- 开发表层选 clangd 而非 cpptools：clangd 是语言服务器的事实标准，诊断更
  友好、内存占用更低；且 cpptools 的 IntelliSense 无法正确处理"交叉编译器
  内置宏 + 多环境 libdeps"的解析（见"关键差异"）。
- 调试层保留 PIO 的 gdb 而非 CodeLLDB：CodeLLDB 内嵌 LLDB 不支持 Xtensa
  架构，嵌入式调试只能走 OpenOCD + gdb，即 PIO 原生 `platformio-debug`。

### 编译层：xtensa-esp32s3-elf-g++ 8.4.0

优点：

- PIO espressif32 平台自动安装管理，零手工配置
- 与 Arduino framework、micro-ROS 预编译库、ESP-IDF 生态全面兼容
- 各环境（共 12 个）由 PIO 统一隔离构建

缺点：

- 仅支持 Xtensa 目标，不能在本机直接运行产物（需烧录到板卡）
- 诊断输出冗长，错误定位依赖人工阅读
- 无内置静态分析通道（且本项目不引入 clang-tidy，原因见决策摘要）

### 开发表层：clangd 21.1.8

优点：

- clangd 为 VS Code 提供补全、诊断、悬停、跳转，替代 cpptools IntelliSense
- 依赖 `compile_commands.json` 承载完整编译命令，天然支持交叉编译器
- clang-format 与 clangd 同属 LLVM，格式规则稳定

缺点：

- 必须先生成并维护 `compile_commands.json`，切换激活环境后需重新生成
- clang 前端（x86 宿主）不认识 xtensa 专属编译 flag，需在 `.clangd` 中过滤
- 交叉编译器内置宏（如 `__XTENSA__`）依赖 `--query-driver` 授权查询

### 调试层：platformio-debug（gdb）

- 调试适配器由 PIO 扩展提供，自动接线 OpenOCD + gdb + 烧录
- `launch.json` 由 PIO 自动生成（`type: platformio-debug`），勿手工修改
- CodeLLDB 在本项目不参与：Xtensa 无 LLDB 支持，装扩展也无用

## 与 C_project 模板的差异

| 项目          | C_project（桌面）              | 本项目（PIO + micro-ROS）                    |
| ------------- | ------------------------------ | -------------------------------------------- |
| 编译器        | 系统 gcc 13.3                  | PIO xtensa-esp32s3-elf-g++ 8.4.0             |
| IntelliSense 输入 | `.clangd` 写 `Compiler: gcc`  | 必须用 compile_commands.json（12 环境合并）  |
| `.clangd` 的 Compiler 字段 | 有（依赖 PATH） | 必须删除（会覆盖 compile_commands 的绝对路径） |
| 调试层        | CodeLLDB (lldb)                | platformio-debug (gdb)，CodeLLDB 不适用      |
| cpptools      | 不安装                          | 必须保留（PIO IDE 硬依赖），仅禁用 IntelliSense |
| clang-tidy    | 启用                            | 不启用（micro-ROS/Arduino 头海量宏会刷屏）   |
| `.vscode/`    | 不入库，文档传递               | 同左                                        |

## 决策摘要

| 项目            | 决策                              | 说明                                      |
| --------------- | --------------------------------- | ----------------------------------------- |
| 编译器          | xtensa-esp32s3-elf-g++ 8.4.0      | espressif32 6.13.0 自带                   |
| LLVM 来源       | 系统 apt                          | clangd / clang-format 21.1.8              |
| 静态检查        | 不启用 clang-tidy                 | 第三方库宏海量，收益低                    |
| 调试器          | platformio-debug (gdb)            | Xtensa 架构无 LLDB 支持                   |
| C_Cpp 扩展      | 保留但禁用 IntelliSense           | PIO IDE 硬依赖，不可卸载                  |
| IntelliSense 输入 | compile_commands.json           | 12 环境合并去重，不入库                   |
| 格式化          | clang-format（LLVM 变体）         | `.clang-format` 已入库                    |
| 版本注册        | update-alternatives               | clangd / clang-format 无版本号命令        |

## 前置检查

1. Ubuntu 24.04 x86_64，VS Code 已安装
2. `clangd --version`、`clang-format --version` 可用（21.x，若缺失参考
   C_project 模板的步骤二/三安装并注册 update-alternatives）
3. PlatformIO Core 已安装且 `pio --version` 可用（本项目为 6.1.19）
4. 项目已至少完整编译过一次（`pio run -e esp32-s3-devkitc-1`），确保各环境
   `.pio/libdeps` 依赖已就位
5. 工具链存在：`~/.platformio/packages/toolchain-xtensa-esp32s3/bin/`
   （含 `xtensa-esp32s3-elf-g++` 与 `xtensa-esp32s3-elf-gdb`）

## 步骤一：确认系统 LLVM 工具

```bash
clangd --version
clang-format --version
```

若 `clangd` / `clang-format` 未注册到 PATH，参照 C_project 模板步骤二、三
安装 `clangd-21 clang-format-21` 并注册 update-alternatives。本项目不需要
`clang-tidy`。

## 步骤二：VS Code 扩展

1. 安装 clangd 扩展（`llvm-vs-code-extensions.vscode-clangd`），路径由工作区
   settings.json 指定为 `/usr/bin/clangd`
2. 安装 PlatformIO IDE 扩展（`platformio.platformio-ide`）——它硬依赖
   cpptools，因此 **cpptools 扩展必须保留，不能卸载**
3. 不安装 CodeLLDB（Xtensa 无 LLDB 支持，无实际用途）；不安装
   `ms-vscode.cpptools-extension-pack` / themes

## 步骤三：写入工作区配置文件

在项目根目录创建或覆盖以下文件。`.vscode/` 整目录已在 `.gitignore` 中，
以下 settings.json 改动只在本机生效、不入库。

### 文件 1：根目录 `.clangd`

```yaml
CompileFlags:
  Remove:
    - -mlongcalls
    - -fno-tree-switch-conversion
    - -fstrict-volatile-bitfields
  Add:
    - -Wno-unknown-warning-option
    - -Wno-unused-command-line-argument
```

要点：

- **不能写 `Compiler:` 字段**。compile_commands.json 中编译器已是绝对路径，
  写 `Compiler: xtensa-esp32s3-elf-g++` 会覆盖绝对路径，导致 clangd 去 PATH
  查找相对名失败、系统头提取失败（`uint32_t` 等全报 unknown）。
- `Remove` 三项是 xtensa 专属 flag：`-mlongcalls` / `-fno-tree-switch-conversion`
  （clang 前端不认识）、`-fstrict-volatile-bitfields`（仅 ARM target 支持），
  不过滤会产生 4 条告警噪音。
- `Add` 两项抑制其余未知 flag 的告警。

### 文件 2：根目录 `.clang-format`

```yaml
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 100
ReflowComments: Never
BreakBeforeBraces: Attach
PointerAlignment: Left
ReferenceAlignment: Pointer
BinPackArguments: false
AlignAfterOpenBracket: BlockIndent
AllowAllArgumentsOnNextLine: false
NamespaceIndentation: None
```

纯通用格式规则，与编译器无关，可从 C_project 直接复制。

### 文件 3：`.vscode/settings.json`

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/jazzy/lib/python3.12/site-packages"
  ],
  "python.analysis.extraPaths": ["/opt/ros/jazzy/lib/python3.12/site-packages"],
  "clangd.path": "/usr/bin/clangd",
  "clangd.arguments": [
    "--query-driver=**/xtensa-esp32s3-elf-*",
    "--background-index",
    "--completion-style=detailed",
    "--header-insertion=never"
  ],
  "C_Cpp.intelliSenseEngine": "disabled",
  "[c]": {
    "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
    "editor.formatOnSave": true
  },
  "[cpp]": {
    "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
    "editor.formatOnSave": true
  }
}
```

要点：

- `--query-driver=**/xtensa-esp32s3-elf-*`：授权 clangd 信任并调用交叉编译器
  查询内置宏（`__XTENSA__` 等）与系统头，缺失则解析不完整。
- `C_Cpp.intelliSenseEngine: disabled`：解决"PIO 捆绑 cpptools"——扩展保留
  （满足 PIO IDE 依赖），但其 IntelliSense 关闭，补全/跳转/诊断由 clangd
  接管。PIO 的编译/上传/串口/调试不依赖该引擎，不受影响。
- 不启用 `--clang-tidy`（决策见摘要）。
- 修改后需 `Ctrl+Shift+P` 重载窗口使配置生效。

### 文件 4：`.gitignore` 追加

```text
.cache/
compile_commands.json
```

- `compile_commands.json` 是本机生成物（2MB+，含绝对路径），不入库。
- `.cache/` 是 clangd 在 `--background-index` 模式下生成的磁盘索引缓存
  （`~/.cache/clangd/index/*.idx`，项目内为 `.cache/clangd/index/`），存放
  各头文件/源文件的预解析符号索引，用于加速跨文件跳转与补全。可安全删除，
  clangd 会自动重建（首次重建时补全略慢）。
- `.pio/`、`.vscode/` 原本已忽略。

## 步骤四：生成 compile_commands.json（12 环境合并）

clangd 解析的头文件散布在 Arduino framework、`.pio/libdeps`（各环境独立）、
micro-ROS 预编译库、工具链内置头中，必须由 compile_commands.json 承载完整
编译命令。`pio run -t compiledb` 每次只产出当前激活环境且覆盖根目录文件，
因此循环生成再合并去重。

```bash
cd <项目根目录>
pio=~/.platformio/penv/bin/pio
mkdir -p .pio/ccdbs
find .pio/ccdbs -name '*.json' -delete
for env in esp32-s3-devkitc-1 example01_helloworld example02_LED \
           example03_Ultrasound example04_IMU test01_motor test02_encoder \
           test03_speed_trans test04_PID test05_Kinematics test06_wifi \
           test07_Subscription; do
  $pio run -e "$env" -t compiledb && mv compile_commands.json ".pio/ccdbs/$env.json"
done
export TOOLCHAIN="$HOME/.platformio/packages/toolchain-xtensa-esp32s3/bin"
python3 - <<'EOF'
import json, glob, os
tc = os.environ['TOOLCHAIN']
seen, out = set(), []
for f in sorted(glob.glob('.pio/ccdbs/*.json')):
    for e in json.load(open(f)):
        cmd = e['command']
        if cmd.startswith('xtensa-esp32s3-elf-'):
            e['command'] = f"{tc}/{cmd}"
        if e['file'] not in seen:
            seen.add(e['file']); out.append(e)
json.dump(out, open('compile_commands.json', 'w'), indent=2)
print(f"merged {len(out)} entries from {len(glob.glob('.pio/ccdbs/*.json'))} envs")
EOF
```

要点与坑：

- 各环境 framework 公共源文件会重复出现，须按 `file` 去重，否则 clangd 对
  同一文件多条 command 会冲突。
- compiledb 生成的数据中，用户代码（src/、lib/）的 command 已是绝对路径，
  但 framework 源文件（cores/esp32、libb64 等）是相对名
  `xtensa-esp32s3-elf-`，需用 `startswith` 精确补前缀。
- **不要用 `sed` 做全局替换**做绝对路径化：`xtensa-esp32s3-elf-` 同时存在于
  绝对路径内（`.../bin/xtensa-esp32s3-elf-g++`），sed 链式替换会把绝对路径
  再替换一遍，产生 `bin//home` 双前缀。
- 合并后校验：全部 command 以绝对路径开头、无 `bin//home`、无相对名残留。
- 每次增删环境或切换激活环境后重新执行本步骤。

## 步骤五：验证清单

1. 终端 `clangd --version`、`clang-format --version` 通过
2. `pio run -e esp32-s3-devkitc-1` 编译通过（确认未破坏编译链路）
3. 打开 `src/main.cpp`，`#include <micro_ros_platformio.h>` / `<rcl/rcl.h>`
   不再标红，`uint32_t` 等类型正常识别
4. 打开 `src/examples/example04_IMU/main.cpp`，`#include <MPU6050_light.h>`
   不报错（跨环境索引正常）
5. 定义跳转、补全正常；`Ctrl+S` 保存触发 clangd 格式化
6. F5 调试仍走 `platformio-debug`（gdb）正常启动
7. 命令行抽查（可选）：`clangd --check=src/main.cpp --query-driver='**/xtensa-esp32s3-elf-*'`
   应无 `pp_file_not_found` / `unknown_typename` 类错误

## 常见问题排查

| 现象                                   | 处理                                                             |
| -------------------------------------- | ---------------------------------------------------------------- |
| clangd 报 driver not found in PATH     | `.clangd` 误写 `Compiler: xtensa-esp32s3-elf-g++`，删除该字段    |
| `uint32_t` 等类型全报 unknown           | compile_commands.json 缺失或编译器为相对名，重跑步骤四           |
| 报 -mlongcalls / -fstrict-volatile-bitfields 告警 | 确认 `.clangd` 的 Remove 三项在生效，改后重载窗口         |
| command 出现 bin//home 双前缀           | 误用 sed 全局替换所致，改用步骤四的 Python 脚本重建              |
| 切换激活环境后解析失准                 | 各环境 libdeps 独立，重跑步骤四重新生成 compile_commands.json    |
| cpptools 与 clangd 同时弹诊断          | 确认 `C_Cpp.intelliSenseEngine: disabled`，重载窗口             |
| 调试无法用 CodeLLDB                    | Xtensa 无 LLDB 支持，改用 PIO 的 platformio-debug (gdb)          |
| 头文件可解析但格式化不生效             | 确认 `[c]`/`[cpp]` 的 defaultFormatter 为 clangd 扩展 ID         |
| 跳转/补全异常（索引陈旧）             | 删除 `.cache/` 让 clangd 全新重建索引，改后重载窗口              |

## 附录：本机已装环境记录

| 项                      | 值                                                               |
| ----------------------- | ---------------------------------------------------------------- |
| 系统                    | Ubuntu 24.04.2                                                   |
| PlatformIO Core         | 6.1.19（`~/.platformio/penv/bin/pio`）                           |
| espressif32 平台        | 6.13.0                                                           |
| 交叉编译器              | xtensa-esp32s3-elf-g++ (crosstool-NG esp-2021r2-patch5) 8.4.0    |
| 工具链路径              | ~/.platformio/packages/toolchain-xtensa-esp32s3/bin/             |
| clangd / clang-format   | 21.1.8（/usr/bin，update-alternatives 注册）                     |
| 环境数量                | 12（1 主 + 4 example + 7 test）                                  |
| compile_commands.json   | 91 条，合并去重后全为绝对路径，不入库                            |
| clangd 索引缓存         | `.cache/clangd/index/`（约 2.7MB，可删自动重建，不入库）         |
