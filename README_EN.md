# YuxiangROS-PIO-learning

> Companion repository of the book *ROS 2 Robot Development: from Beginner to
> Practice* (by Sang Xin / fishros). This is the ESP32-S3 motion-control firmware
> for the real robot (Chapter 9): it subscribes to `/cmd_vel` via micro-ROS,
> drives the motors through inverse kinematics and a PID velocity loop, and
> publishes `/odom`.
> Highlight: the book directly modifies the previous section's code instead of
> presenting an integrated project; this repo slices its per-section code blocks
> into 12 independently buildable firmwares (examples / tests / main) for
> side-by-side reading and verification.
> The repo name "PIO" stands for PlatformIO (firmware side), complementary to the
> host-side main repository
> [YuXiangROS-jazzy-learning](https://github.com/HoseaChern/YuXiangROS-jazzy-learning).

[中文版](./README.md)

## Table of Contents

- [YuxiangROS-PIO-learning](#yuxiangros-pio-learning)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Correspondence with the Book](#correspondence-with-the-book)
  - [Hardware and Pinout](#hardware-and-pinout)
  - [platformio.ini Design](#platformioini-design)
  - [Directory Structure](#directory-structure)
  - [Dependencies](#dependencies)
  - [Build and Upload](#build-and-upload)
  - [Running and Integration](#running-and-integration)
  - [Optimizations](#optimizations)
  - [Development Environment](#development-environment)
    - [Generating compile\_commands.json (12 Envs Merged)](#generating-compile_commandsjson-12-envs-merged)
  - [Acknowledgements and References](#acknowledgements-and-references)
  - [License](#license)

## Overview

The firmware runs on an ESP32-S3 (PlatformIO + Arduino framework) and talks to
the host (ROS 2 Jazzy) over WiFi + micro-ROS:

1. subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`);
2. inverse kinematics converts the body velocity into target wheel speeds
   (pure-algorithm library `Kinematics`);
3. PID velocity loop outputs PWM to the motors (pure-algorithm library
   `PIDController`);
4. odometry is integrated from encoder readings and `/odom`
   (`nav_msgs/msg/Odometry`) is published at 50 ms.

```text
/cmd_vel (Twist) -> inverse kinematics -> target wheel speeds -> PID -> PWM
-> motors + encoders -> odometry -> /odom
```

## Correspondence with the Book

Chapter 9 of the book progresses through MCU development basics (9.2),
control-system implementation (9.3), and micro-ROS integration (9.4). **The book
does not present an integrated PlatformIO project: its firmware code appears as
per-section code blocks, and each section directly modifies the previous
section's code.** This repo slices those code blocks into independently
buildable projects, one-to-one:

| Book section (code block)                  | This repo's slice project                          | What is verified                         |
| ------------------------------------------ | -------------------------------------------------- | ---------------------------------------- |
| 9.2.2 first Hello World project            | `examples/example01_helloworld`                    | serial Hello World                       |
| 9.2.3 blinking an LED with code            | `examples/example02_LED`                           | GPIO output, LED blink                   |
| 9.2.4 ultrasonic ranging                   | `examples/example03_Ultrasound`                    | ultrasonic sensor reading                |
| 9.2.5 driving an IMU with a lib            | `examples/example04_IMU`                           | MPU6050 attitude estimation              |
| 9.3.1 driving multiple motors              | `tests/test01_motor`                               | MCPWM motor driving                      |
| 9.3.2 motor speed measurement & conversion | `tests/test02_encoder`, `tests/test03_speed_trans` | encoder reading, speed conversion        |
| 9.3.3 PID speed control                    | `tests/test04_PID`                                 | PID velocity loop                        |
| 9.3.4 forward/inverse kinematics           | `tests/test05_Kinematics`                          | inverse kinematics + PID                 |
| 9.3.5 odometry computation                 | main firmware `src/main.cpp` (odometry part)       | odometry integration                     |
| 9.4.1 the first node                       | `tests/test06_wifi`                                | micro-ROS WiFi connection                |
| 9.4.2 subscribing to control the robot     | `tests/test07_Subscription`                        | `/cmd_vel` subscription + motion control |
| 9.4.3 publishing the odometry topic        | main firmware `src/main.cpp`                       | `/odom` publishing + full integration    |

> Note: 9.2.1 (platform introduction) involves no code; 9.3.2 covers both "speed
> measurement" and "speed conversion", hence two slices; the main firmware is the
> convergence of 9.3.5 and 9.4.3 (embedded odometry + publishing `/odom`).

## Hardware and Pinout

| Device    | Left (Motor/Encoder 0) | Right (Motor/Encoder 1) |
| --------- | ---------------------- | ----------------------- |
| Motor PWM | GPIO 4 / 5             | GPIO 7 / 6              |
| Encoder   | GPIO 15 / 16           | GPIO 18 / 17            |

| Item            | Configuration                                                 |
| --------------- | ------------------------------------------------------------- |
| MCU             | ESP32-S3-DevKitC-1 (Xtensa LX7, Arduino framework)            |
| Motor driver    | `Esp32McpwmMotor` (MCPWM)                                     |
| Encoder reading | `Esp32PcntEncoder` (PCNT pulse counting)                      |
| Communication   | micro-ROS over WiFi (UDP), default Agent `192.168.2.120:8888` |
| Control period  | 10 ms main loop, 50 ms odometry publishing                    |

> Hardware note: the book uses an Adafruit Feather board; this repo uses an
> ESP32-S3-DevKitC-1 instead. The firmware is decoupled from the board, so
> porting only involves the `board` setting and pins in `platformio.ini` — proof
> of the design's flexibility.

## platformio.ini Design

`platformio.ini` is the core configuration: 12 environments (1 main + 4 examples + 7 tests), each example/test only compiles its own `main.cpp`, independent of the main firmware. Key points:

- **`build_src_filter` isolation**: the main firmware uses `+<*> -<examples>
  -<tests>`; each example/test keeps only its own directory. Otherwise the
  multiple `setup()/loop()` in `src/` cause duplicate-symbol link errors.
- **Per-environment `lib_deps`**: no `lib_deps` in the common section (there is
  no common dependency across environments); each environment installs only what
  it needs, so `lib_ignore` is never required.
- **micro-ROS declared only where used** (main, test06/07): its
  `extra_script.py` build hook runs unconditionally in any environment where it
  is installed (injects macros, links the prebuilt `libmicroros`), so it must not
  be installed into unrelated environments.
- **IntelliSense fallback include**: `MPU6050_light` is only installed in the
  example04 environment; a common-section `-I` points at its header so the IDE
  can resolve it under any active environment (harmless for compilation).
- **Credential separation**: WiFi credentials live in
  `lib/Secrets/secrets.h`; the repo keeps only the `secrets.example.h` template.

```ini
[env]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

[env:esp32-s3-devkitc-1]                 ; main firmware
build_src_filter = +<*> -<examples> -<tests>
board_microros_transport = wifi
lib_deps = Esp32McpwmMotor, Esp32PcntEncoder, micro_ros_platformio (fishros mirror)

[env:example01_helloworld]               ; example: compiles its own dir only
build_src_filter = +<examples/example01_helloworld>
```

## Directory Structure

```text
YuxiangROS-PIO-learning/
├── include/                     # project headers (reserved)
├── lib/                         # private libraries
│   ├── Kinematics/              # two-wheel differential kinematics (fwd/inv + odom), pure algorithm
│   ├── PIDController/           # positional PID, pure algorithm
│   ├── SemanticEnums/           # semantic enums (MotorID / VelocityID, etc.)
│   └── Secrets/                 # credential template (secrets.example.h)
├── src/
│   ├── main.cpp                 # main firmware: micro-ROS + motion control
│   ├── examples/                # 4 example firmwares (example01~04)
│   └── tests/                   # 7 test firmwares (test01~07)
├── docs/                        # study notes (About_PlatformIO, CLI usage)
├── .clangd / .clang-format / .clang-tidy   # C/C++ toolchain conventions
└── platformio.ini               # 12-environment configuration
```

## Dependencies

| Library              | Purpose                 | Source                                                              | Used by                     |
| -------------------- | ----------------------- | ------------------------------------------------------------------- | --------------------------- |
| Esp32McpwmMotor      | MCPWM motor driver      | [fishros](https://github.com/fishros/Esp32McpwmMotor)               | main, test01/03/04/05/06/07 |
| Esp32PcntEncoder     | PCNT encoder reading    | [fishros](https://github.com/fishros/Esp32PcntEncoder)              | main, test02/03/04/05/06/07 |
| micro_ros_platformio | micro-ROS support       | [fishros](https://github.com/fishros/micro_ros_platformio) (mirror) | main, test06/07             |
| MPU6050_light        | IMU attitude estimation | [rfetick](https://github.com/rfetick/MPU6050_light)                 | example04                   |

> Why the fishros prebuilt mirror: the official
> [micro-ROS/micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio)
> only lists the generic `esp32dev` board (ESP32-S3 is not covered), and its
> build compiles the whole micro-ROS stack from source (cmake + meta-build, slow
> and error-prone on first build) — that is why the prebuilt mirror is used.

## Build and Upload

```bash
# prepare credentials (copy the template and fill in the WiFi SSID/password)
cp lib/Secrets/secrets.example.h lib/Secrets/secrets.h

# build / upload the main firmware
pio run -e esp32-s3-devkitc-1
pio run -e esp32-s3-devkitc-1 -t upload

# serial monitor / build and upload an example or test (e.g. test01_motor)
pio device monitor -b 115200
pio run -e test01_motor -t upload
```

| Type    | Environment          | Description                                        |
| ------- | -------------------- | -------------------------------------------------- |
| Main    | esp32-s3-devkitc-1   | motion control + micro-ROS (`/cmd_vel` -> `/odom`) |
| Example | example01_helloworld | Hello World                                        |
| Example | example02_LED        | LED blink                                          |
| Example | example03_Ultrasound | ultrasonic ranging                                 |
| Example | example04_IMU        | MPU6050 attitude estimation                        |
| Test    | test01_motor         | motor driver test                                  |
| Test    | test02_encoder       | encoder reading and calibration                    |
| Test    | test03_speed_trans   | speed conversion test                              |
| Test    | test04_PID           | PID velocity loop test                             |
| Test    | test05_Kinematics    | inverse kinematics + PID control test              |
| Test    | test06_wifi          | micro-ROS WiFi connection test                     |
| Test    | test07_Subscription  | `/cmd_vel` subscription + motion control test      |

## Running and Integration

1. fill in the WiFi credentials in `lib/Secrets/secrets.h`;
2. set `AGENT_IP_STR` in `src/main.cpp` to the host running the micro-ROS Agent;
3. after flashing, start the micro-ROS Agent on the host:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

4. publish a velocity command to drive the chassis:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
   ```

5. inspect odometry: `ros2 topic echo /odom`

## Optimizations

This firmware is independently implemented on top of the fishros micro-ROS
template. The algorithm libraries (`Kinematics`, `PIDController`) differ from
the original mainly in code quality:

- **No magic numbers, compile-time constants**: `MS_TO_S` (unit conversion 1000),
  `INTEGRAL_SUP_LIMIT` (integral clamp 2500), `PI_F` expressed as `constexpr`;
- **Type discipline**: explicit integer/float widths (`int16_t`/`uint32_t`/
  `float`); `dt` widened from `uint32_t` to `uint64_t` to avoid ms-counter
  overflow; `update_pwm` returns `int16_t` with rounding to avoid a
  systematically low duty cycle at low speed;
- **Semantic enums**: `SemanticEnums` constrain motor/velocity indices instead
  of bare numbers;
- **Const correctness**: read-only methods are `const`; `get_odom()` has mutable
  and const overloads;
- **Defensive programming**: division-by-zero guards (angular velocity outputs 0
  when the wheel distance is `<= 0`), integral and output clamping, angle
  normalization rewritten with `std::fmod` (converges to [-PI, PI] for any
  number of rotations);
- **Interface slimming and dependency narrowing**: `motor_param_t` removed (both
  wheels share identical calibration, so it became a scalar); algorithm-layer
  dependencies narrowed from `<Arduino.h>` to `<cstdint>`/`<cmath>`, portable
  and unit-testable.

> The full 16-item / 9-item optimization lists (with commits) live in each
> library's `docs/README.md`.

## Development Environment

Three layers, each independent: "PIO cross-compilation + LLVM development
surface + gdb debugging".

| Layer         | Tool                         | Responsibility              |
| ------------- | ---------------------------- | --------------------------- |
| Compile layer | xtensa-esp32s3-elf-g++ (gcc) | sole production build path  |
| Dev surface   | clangd / clang-format        | IntelliSense, formatting    |
| Debug layer   | platformio-debug (gdb)       | `pio debug` (OpenOCD + gdb) |

Rationale (differences from common alternatives):

- **Compile layer is not replaceable**: ESP32-S3 uses the Xtensa LX7
  architecture, and the Arduino framework and micro-ROS prebuilt libraries rely
  on PIO's bundled cross-compiler;
- **clangd over cpptools**: cpptools IntelliSense cannot handle "cross-compiler
  built-in macros + multi-environment libdeps"; clangd consumes
  `compile_commands.json` carrying the full compile command;
- **gdb over CodeLLDB**: CodeLLDB's LLDB has no Xtensa support, so embedded
  debugging goes through OpenOCD + gdb.

The repo commits `.clangd`, `.clang-format`, and `.clang-tidy` (`.vscode/` is
not committed).

### Generating compile_commands.json (12 Envs Merged)

`pio run -t compiledb` only emits the currently active environment, so generate
per env and merge with dedup (machine-generated, contains absolute paths, not
committed):

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

Key points:

- shared framework sources repeat across envs and must be deduplicated by
  `file`, otherwise clangd sees conflicting commands for the same file;
- prefix the relative compiler name `xtensa-esp32s3-elf-` with the absolute path
  using `startswith` only; do not use `sed` for global replacement (the name also
  appears inside absolute paths, producing `bin//home` double prefixes);
- `tools/merge_ccdb.py` appends the missing `-I` for the header-only library
  `lib/SemanticEnums` to every command: PIO's `-t compiledb` drops include paths
  of header-only libraries (no `.cpp`), so the real build command carries it but
  the ccdb does not, which makes clangd report `'SemanticEnums.h' file not found`;
- re-run after adding/removing environments.

Common problems:

| Symptom                          | Handling                                                       |
| -------------------------------- | -------------------------------------------------------------- |
| clangd: driver not found in PATH | `.clangd` accidentally has `Compiler:`; remove it              |
| `uint32_t` etc. all unknown      | compile_commands.json missing or relative compiler; regenerate |
| clang-tidy inactive              | confirm `--clang-tidy` and the root `.clang-tidy`              |
| cannot debug with CodeLLDB       | no Xtensa LLDB support; use CLI `pio debug` (OpenOCD + gdb)    |

## Acknowledgements and References

- fishros and *ROS 2 Robot Development: from Beginner to Practice* (by 桑欣 / Sang Xin);
- [fishros/ros2bookcode](https://github.com/fishros/ros2bookcode): the book's companion code repo;
- [fishros/micro_ros_platformio](https://github.com/fishros/micro_ros_platformio):
  micro-ROS firmware template (this config and toolchain are based on it);
- [fishros/fishbot](https://github.com/fishros/fishbot): algorithm ideas for
  kinematics and PID;
- [fishros/Esp32McpwmMotor](https://github.com/fishros/Esp32McpwmMotor),
  [fishros/Esp32PcntEncoder](https://github.com/fishros/Esp32PcntEncoder):
  motor and encoder driver libraries;
- the main repository
  [YuXiangROS-jazzy-learning](https://github.com/HoseaChern/YuXiangROS-jazzy-learning).

## License

The original code in this repository is licensed under
[Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) (see
[LICENSE](./LICENSE)); third-party libraries retain their respective licenses.
