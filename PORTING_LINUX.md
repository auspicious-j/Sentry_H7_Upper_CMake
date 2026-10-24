# Sentry H7 Upper — Linux 原生移植说明

本工程原本是 **STM32H723 + STM32CubeMX + FreeRTOS + HAL** 的嵌入式固件。
现在已改造为可以在 **Ubuntu (x86-64)** 上用系统 GCC 直接编译、直接运行的原生程序。

> 本文档列出所有改动，代码中对应的位置统一用 `[LINUX-PORT]` 注释标记，
> 直接搜索 `LINUX-PORT` 即可定位全部移植点：
>
> ```bash
> grep -rn "LINUX-PORT" --include=*.c --include=*.h --include=*.txt .
> ```

---

## 1. 快速开始

```bash
# 一键编译（默认 Debug）
./build.sh

# 或手动编译
~/.local/tools/cmake-3.31.6-linux-x86_64/bin/cmake -S . -B build/Debug -DCMAKE_BUILD_TYPE=Debug
~/.local/tools/cmake-3.31.6-linux-x86_64/bin/cmake --build build/Debug -j

# 运行
./build/Debug/sentry_upper_linux

# 输出移植层详细日志（线程启动等）
SENTRY_PORT_VERBOSE=1 ./build/Debug/sentry_upper_linux
```

编译产物：`build/Debug/sentry_upper_linux`（约 232 KB，ELF x86-64），
**不再生成 `.elf` / `.bin` 等 STM32 固件**。

### 关于 CMake

本机没有 sudo 权限，无法 `apt install cmake`，因此使用官方免安装二进制包：

| 项目 | 路径 |
|---|---|
| 安装目录 | `/home/speros987/.local/tools/cmake-3.31.6-linux-x86_64/` |
| 可执行文件 | `~/.local/tools/cmake-3.31.6-linux-x86_64/bin/cmake` |
| 版本 | 3.31.6 |

`build.sh` 会自动检测：系统有 `cmake` 就用系统的，否则用上面这个路径；
也可以用 `CMAKE_BIN=/path/to/cmake ./build.sh` 指定。

---

## 2. 已删除的 STM32 库与工程文件

以下内容已从工作区移除（git 历史中仍可找回：`git log` / `git checkout <commit> -- <path>`）：

| 删除项 | 原用途 | 删除原因 |
|---|---|---|
| `Drivers/` | STM32H7 HAL 驱动 + CMSIS 内核头文件 | 面向 Cortex-M7 寄存器，x86 上无法使用，由 `platform/linux/inc` 替代 |
| `Middlewares/Third_Party/FreeRTOS/` | FreeRTOS 内核 + CMSIS-RTOS 封装 | 由 POSIX pthread 替代 |
| `Middlewares/ST/ARM/DSP/` | CMSIS-DSP（`libCMSISDSP.a` 为 ARM 目标码） | 由 `platform/linux/src/dsp_port.c` + libm 替代 |
| `Middlewares/ST/STM32_USB_Device_Library/` | USB CDC 设备栈 | 由 `usbd_cdc_if.h` + `CDC_Transmit_HS()` 空实现替代 |
| `USB_DEVICE/` | CubeMX 生成的 USB 应用层 | 同上 |
| `Core/` | CubeMX 生成的 `main.c`、外设初始化、中断向量、FreeRTOS 配置 | 由 `platform/linux/src/main_linux.c`、`tasks_linux.c` 替代 |
| `MDK-ARM/` | Keil 工程文件 | Linux 下不再使用 |
| `cmake/` | `gcc-arm-none-eabi.cmake` 等交叉编译工具链文件 | 改为原生编译，不再需要 |
| `startup_stm32h723xx.s` | 启动汇编 | x86 有系统 crt，不需要 |
| `STM32H723XG_FLASH.ld` | 链接脚本 | 同上 |
| `Ozone.jdebug`, `Ozone.jdebug.user`, `flash.cfg` | SEGGER 调试/烧录配置 | 不再使用 |
| `.mxproject` | CubeMX 工程状态 | 不再使用 |

> 保留 `Sentry_H7_Upper_CMake.ioc`：它是硬件引脚/外设配置的参考文件，不参与编译。
> 以后若需要回到 STM32，可用 CubeMX 重新生成 `Core/` 与 `Drivers/`。

`.gitignore` 中已有 `build`，编译产物不会进入版本库。

---

## 3. 新增：Linux 移植层 `platform/linux/`

### 3.1 头文件（`platform/linux/inc/`）

这些头文件的名字与原 `Core/Inc` 下的一致，因此业务层 **不需要改 include**，
只是把搜索路径优先级指向了移植层。

| 文件 | 替代对象 | 对应 Linux 技术 |
|---|---|---|
| `stm32h7xx_hal.h` | STM32 HAL 类型/函数声明 | POSIX（`clock_gettime`、`nanosleep`）+ 空实现 |
| `main.h` | CubeMX 生成的 `main.h` | 保留引脚宏，供旧代码编译 |
| `usart.h` / `dma.h` | 串口与 DMA 句柄 | termios / socket 待接入 |
| `fdcan.h` | FDCAN 句柄 | SocketCAN 待接入 |
| `tim.h` / `spi.h` / `gpio.h` | 定时器/SPI/GPIO 句柄 | 无需硬件，空实现 |
| `cmsis_os.h` | CMSIS-RTOS v1 | **pthread** |
| `FreeRTOS.h` / `task.h` | FreeRTOS 基础类型/任务 API | **pthread + POSIX 信号量/互斥量** |
| `arm_math.h`、`dsp/matrix_functions.h`、`dsp/fast_math_functions.h`、`arm_math_types.h` | CMSIS-DSP | **libm + 纯 C 矩阵运算** |
| `usbd_cdc_if.h` | USB CDC 接口 | 空实现（通信暂时停用） |
| `port_linux.h` | 新增 | 移植层内部公共接口（时钟、日志、线程创建） |

### 3.2 源文件（`platform/linux/src/`）

| 文件 | 作用 |
|---|---|
| `hal_port.c` | `HAL_GetTick()`→`clock_gettime(CLOCK_MONOTONIC)`；`HAL_Delay()`→`clock_nanosleep`；FDCAN/UART/TIM/SPI/GPIO/USB 全部空实现 |
| `os_port.c` | `osDelay`/`osThreadSuspend`/`osThreadResume`、`xTaskNotifyGive`/`ulTaskNotifyTake`、`vTaskSuspendAll`/`xTaskResumeAll`、`pvPortMalloc`/`vPortFree` 的 pthread 实现 |
| `dsp_port.c` | `arm_mat_init/add/sub/mult/trans/inverse_f32`、`arm_atan2_f32` 等，用 libm 实现 |
| `bmi088_sim.c` | **模拟 IMU 数据源**，替代 SPI 读取 BMI088，输出正弦变化的陀螺仪/加速度 |
| `tasks_linux.c` | 替代 `Core/Src/freertos.c`，用 pthread 创建 12 个任务线程 |
| `main_linux.c` | 替代 `Core/Src/main.c`，完成启动流程并周期性打印运行状态 |

### 3.3 任务映射

| 原任务 | 线程名 | 周期 | 说明 |
|---|---|---|---|
| `OS_IMUCallback` | `IMU` | 1 ms | 使用模拟 IMU 数据 |
| `OS_LedCallback` | `LED` | 500 ms | WS2812 空实现 |
| `OS_BeepCallback` | `Beep` | 20 ms | PWM 空实现 |
| `OS_MotorCallback` | `Motor` | 1 ms | CAN 发送空实现 |
| `OS_ErrorCallback` | `Error` | — | 启动后阻塞，等待 `osThreadResume` |
| `OS_Board2BoardCallback` | `B2B` | 1 ms | 串口发送空实现 |
| `OS_DetectCallback` | `Detect` | 10 ms | 掉线检测逻辑保留 |
| `OS_RcCallback` | `RC` | 15 ms | 串口接收空实现 |
| `OS_ShooterCallback` | `Shooter` | 10 ms | 业务逻辑保留 |
| `OS_ChassisCallback` | `Chassis` | 2 ms | 业务逻辑保留 |
| `OS_GimbalCallback` | `Gimbal` | 1 ms | 业务逻辑保留 |
| `OS_VisionCallback` | `Vision` | 1 ms | 发送空实现 |

`Judge` / `SuperCap` / `Default` 任务在原工程中即未启用（`DISABLE_JUDGE_TASK` / 空循环），未创建。

---

## 4. 修改过的原有文件（逐条）

| 文件 | 改动 | 原因 |
|---|---|---|
| `BSP/src/bsp_dwt.c` | **重写**：`DWT->CYCCNT` / `CoreDebug` / `SysTick` 改为 `clock_gettime(CLOCK_MONOTONIC)`；`DWT_Delay` 由忙等改为 `nanosleep` | Cortex-M7 内核寄存器在 x86 上不存在 |
| `Application/inc/Chassis.h` | `#include "pid.h"` → `"PID.h"` | Linux 文件系统区分大小写 |
| `Application/inc/Gimbal.h` | `#include "slope.h"` → `"Slope.h"` | 同上 |
| `Application/src/Judge.c` | `#include "cRc.h"` → `"Crc.h"`；`"userfreertos.h"` → `"UserFreertos.h"` | 同上 |
| `math/src/Crc.c` | `#include "cRc.h"` → `"Crc.h"` | 同上 |
| `IMU/Algorithm/inc/kalman_filter.h` | 增加 `#include "string.h"` | 原代码依赖隐式声明的 `memcpy`/`memset`，现代 GCC 会报错 |
| `CMakeLists.txt` | **重写**：原生 GCC 构建，排除 BMI088 硬件驱动，链接 `m` + `pthread` | 不再交叉编译 STM32 固件 |
| `CMakePresets.json` | 删除 arm 工具链预设，改为 `Unix Makefiles` 原生预设 | 同上 |
| `.vscode/c_cpp_properties.json` | `STM32/windows-msvc-x64` → `Linux/linux-gcc-x64` | IDE 索引 |
| `.vscode/settings.json` | 移除 `cube-cmake` / `stm32cube-ide-clangd` 配置，指向本机 CMake | 同上 |
| `.vscode/launch.json` | `cortex-debug`+OpenOCD → `cppdbg`+gdb | 本机调试 |

**未修改**（保持原样编译）的业务文件：`Application/src/*.c`（除 Judge.c 的 include）、
`BSP/src/bsp_can.c`、`BSP/src/bsp_ws2812.c`、`math/`、`UserMiddlewares/`、
`IMU/Algorithm/`（除头文件新增 include）。这些文件通过 HAL/RTOS 兼容层直接编译通过。

`IMU/BMI088/BMI088driver.c`、`IMU/BMI088/BMI088Middleware.c` **保留但未参与编译**，
由 `platform/linux/src/bmi088_sim.c` 提供同名接口。

---

## 5. 通信相关代码的当前状态（按要求暂时停用）

按你的要求「发送接收相关代码先停用，之后改别的通信」，目前所有收发都是**空实现**，
但接口签名保持原样，后续替换实现即可，业务层不用改：

| 通信 | 入口函数 | 当前行为 | 建议的 Linux 替代 |
|---|---|---|---|
| 电机 CAN | `USER_CAN_Send()` → `HAL_FDCAN_AddMessageToTxFifoQ()` | 直接返回 `HAL_OK`，不发送 | SocketCAN（`PF_CAN` + `linux/can.h`），需 `sudo ip link set can0 up type can bitrate 1000000` |
| 电机 CAN 接收 | `HAL_FDCAN_RxFifo0Callback()` | 不会被调用 | SocketCAN `read()` 线程 + 回调 |
| 遥控器 SBUS | `HAL_UARTEx_ReceiveToIdle_DMA()` | 空实现 | termios + `poll()`，或 USB 转串口 |
| 上下板通信 | `B2B_Transmit()` / `B2B_Receive()` | 空实现 | termios / UDP / 共享内存 |
| 裁判系统 | `JUDGE_Init()` / `Judge_Receive()` | 空实现 | termios |
| 视觉收发 | `CDC_Transmit_HS()` | 空实现 | TCP/UDP socket 或 ROS2 话题 |
| 蜂鸣器 | `HAL_TIM_PWM_Start()` | 空实现 | ALSA / console bell |
| WS2812 | `HAL_TIM_PWM_Start_DMA()` | 空实现 | SPI（`ws2812-spi` 驱动）/ GPIO 模拟 |

需要注意的连带影响：

- 由于收不到电机反馈，`DJIMotor_Update()` 不会被调用，电机角度/速度恒为 0，
  PID 输出会饱和到限幅值。这是预期现象，不代表控制逻辑出错。
- 由于收不到遥控器数据，`rcInfo` 恒为 0，底盘处于默认模式，不会进入小陀螺/急停分支。
- IMU 使用模拟数据，姿态解算链路（QuaternionEKF）是真实运行的，
  所以 `[status]` 里的 `yaw/pitch/roll` 会随时间变化。

---

## 6. 已知限制

1. **非硬实时**：Linux 用户态线程存在毫秒级抖动，1 ms 控制环无法达到 MCU 的确定性。
   如需更严格的周期，可考虑 `PREEMPT_RT` 内核 + `SCHED_FIFO`（需要 root）。
2. **优先级**：普通用户无法设置负 nice 值，`NICE_HIGH` / `NICE_REALTIME` 会被内核忽略。
3. **矩阵运算性能**：`arm_mat_*` 为朴素实现，矩阵规模小（≤10×10），够用但不是 SIMD 优化版。
4. **IMU 为模拟数据**：`bmi088_sim.c` 只用于验证链路，不是真实传感器读数。

---

## 7. 回退到 STM32 固件

删除的内容都在 git 历史里，最近一次提交 `76cd059` 包含完整的 STM32 工程：

```bash
git checkout 76cd059 -- Core Drivers Middlewares USB_DEVICE MDK-ARM cmake \
    startup_stm32h723xx.s STM32H723XG_FLASH.ld CMakeLists.txt CMakePresets.json
```

同时需要 `arm-none-eabi-gcc` 工具链（本机目前未安装）。
