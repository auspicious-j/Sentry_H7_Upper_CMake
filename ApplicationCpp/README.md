# ApplicationCpp

C++17 静态插件图框架 + 三张业务图。CubeMX 生成的 C 代码（`Core/`、`Drivers/`、`USB_DEVICE/`、`Middlewares/`）保持原样，本目录只在其上做「搬运 → 决策 → 回写」的重构。

本文只描述当前代码里真实存在的东西。历史上那套「单一组合根 + 演示链路」（`g_robot_debug`、`ROBOT_ENABLE_*`、分支/计划/比较演示）已从构建中移除，见 §9。

## 1. 分工

```text
CubeMX / HAL / FreeRTOS        外设、中断、任务骨架（C）
        ↓ 短临界区整份拷贝
ApplicationCpp/Io              C 全局量 → POD 快照（纯搬运，无业务判断）
        ↓ 端口 / 快照邮箱
ApplicationCpp/Graph           三张插件图，每帧算一次（业务决策）
        ↓ 提交
C 侧全局量 + CAN / UART / USB   由原有 C 代码继续下发，不经过本层
```

- 图入口是 `extern "C"` 的 `AppCpp_<子系统>_Init()` / `AppCpp_<子系统>_ProcessFrame()`，声明在 `Graph/<子系统>/<子系统>_GraphApi.h`。
- 每张图由自己的 FreeRTOS 任务驱动，**没有**统一的控制执行器。
- 跨任务 / 跨中断只有两种边界：`frame_signal<t>`（同帧内、图内）与 `latest_snapshot_mailbox<t>`（跨任务最新值）。

| 路径 | 内容 | 有业务判断 |
| --- | --- | --- |
| `Framework/` | 图调度器与基础设施 | 否 |
| `Graph/{Gimbal,Chassis,Shooter}/` | 三张业务图及其节点 | 是 |
| `Io/Inputs/` | `read_imu` / `read_rc` / `read_judge` / `read_actuator_feedback` | 否 |
| `Io/Vision/` | USB 环形缓冲、消帧解码、视觉快照邮箱 | 否 |
| `Platform/STM32/` | DWT 微秒时钟、profiler、PRIMASK 临界区 | 否 |
| `Comms/` | 遥控器接收、板间通信（原 `UserMiddlewares`） | 是 |
| `Drivers/` | 电机驱动、设备检测（原 `UserMiddlewares`） | 是 |
| `App/` | 各任务入口 `OS_*Callback` + 迁移期保留的旧实现 | 有（旧实现不调用） |

## 2. 任务与周期

`Core/Src/freertos.c` 建 15 个任务，全部声明为 `__weak` 桩；强符号实现分散在 `App/`、`Comms/`、`Drivers/` 以及 C 侧的 `IMU/src/imu_temp_ctrl.c`、`BSP/src/bsp_ws2812.c`。

| 任务 | 优先级 | 栈(字) | 循环 | 实现 | 驱动对象 |
| --- | --- | ---: | --- | --- | --- |
| `Gimbal` | Normal | 1024 | `osDelay(1)` → 1 kHz | `App/GimbalApp.cpp` | Gimbal 图（6 节点） |
| `Chassis` | Normal | 768 | `osDelay(2)` → 500 Hz | `App/ChassisApp.cpp` | Chassis 图（3 节点） |
| `ShooterTask` | Normal | 512 | `osDelay(10)` → 100 Hz | `App/ShooterApp.cpp` | Shooter 图（2 节点） |
| `VisionTask` | Normal | 512 | `osDelay(1)` → 1 kHz | `App/VisionApp.cpp` | `VisionLink_Poll` + 视觉回传 |
| `RS485Task` | High | 512 | `osDelay(1)` → 1 kHz | `Comms/BoardToBoardApp.cpp` | USART2 板间帧发送 |
| `MotorTask` | Normal | 512 | `osDelay(1)` → 1 kHz | `App/MotoApp.cpp` | CAN 电流/MIT 下发 |
| `IMUTask` | Normal | 2048 | `osDelay(1)` → 1 kHz | `IMU/src/imu_temp_ctrl.c` | BMI088 + 四元数 EKF |
| `RcTask` | AboveNormal | 256 | `osDelay(15)` ≈ 66 Hz | `Comms/RcReceiverApp.cpp` | SBUS 遥控器 |
| `JudgeTask` | Normal | 256 | `osDelay(100)` → 10 Hz | `App/JudgeApp.cpp` | 裁判系统 UART |
| `DetectTask` | Normal | 256 | `osDelay(10)` → 100 Hz | `Drivers/DeviceDetectApp.cpp` | 设备在线检测 |
| `BeepTask` | Normal | 256 | `osDelay(20)` → 50 Hz | `App/BeepApp.cpp` | 蜂鸣器 |
| `LEDTask` | BelowNormal | 128 | 500 ms | `BSP/src/bsp_ws2812.c` | WS2812 心跳灯 |
| `ErrorTask` | Realtime | 128 | 急停循环 | `App/UserFreertosApp.cpp` | 关断电机 + 复位 |
| `SuperCapTask` | Normal | 128 | — | **无实现**（`__weak` 空循环） | — |
| `defaultTask` | Normal | 512 | `osDelay(1)` | `freertos.c` | USB Device 初始化后空转 |

两点需要注意：

- 「1 kHz」是名义值。`configTICK_RATE_HZ = 1000`，`osDelay(1)` 实为 1 tick + 调度延迟；四个高频任务与 `Beep/Judge/Detect` 同为 `osPriorityNormal`，靠 tick 轮转分时，周期抖动没有额外约束。
- `Chassis/Gimbal/ShooterTask` 的栈（768/1024/512）是提交 `M5` 为图节点与 `timed_snapshot` 拷贝抬高的。`.ioc` 的 `FREERTOS.Tasks01` 已同步成与上表相同的栈和优先级（15 个任务逐项一致），CubeMX 重新生成不会再回退。**后续不要把这三个栈往回调**：框架节点与快照整份拷贝都跑在这些任务的栈上。

## 3. 三张图

图结构在 `compile()` 中固定：拓扑排序 → 冻结 → 运行期零堆分配。容量由 `Graph/Gimbal/Gimbal_Graph_Config.h` 定义：32 节点 / 64 边 / 4 执行计划。三张图都只声明 1 套计划。

### 3.1 Gimbal（ID 1–6）

| ID | 节点 | 职责 |
| ---: | --- | --- |
| 6 | `Vision_Target_Node` | 读视觉原始帧 → 打装甲板 / 打符判定（需 IMU 与发射状态）→ 发布端口 + `vision_target_mailbox` |
| 1 | `Gimbal_Input_Node` | 采集 IMU / 遥控 / 裁判 / 执行器反馈；短临界区读 `chassis.pattern`、`chassis.rotate.mode`、pitch 限幅 |
| 2 | `Gimbal_Target_Node` | 决策：AI 视觉跟踪 / 扫描（装甲板匀速 + 正弦、前哨、符）/ 摇杆 / 摩擦轮边沿切换 |
| 3 | `Gimbal_State_Node` | 角度状态更新（逐行对应旧 `Gimbal_UpdataAngle`）；`left==2` 时 base 目标 +45°（加弹）；回写 `gimbal.*` 测量量 |
| 4 | `Gimbal_Control_Node` | 小 yaw 串级、大 yaw 外环（取负 /1000）、pitch 位置环 + 重力前馈；小陀螺时改大 yaw 积分参数 |
| 5 | `Gimbal_Output_Node` | 提交 `gimbal.*_targetAngle` 与 `shooter.fricOpen/command/idToTrigger` |

依赖边：`Vision_Target → Input → Target → State → Control → Output`。
`Target` 读的是 `State` 的上一帧结果（`State` 在其之后执行），与旧 C 实现「先决策、后 `Gimbal_UpdataAngle`」一致。

### 3.2 Chassis（ID 11–13）

| ID | 节点 | 职责 |
| ---: | --- | --- |
| 11 | `Chassis_Input_Node` | 遥控 / 执行器反馈 / 视觉 AI 量（`linear_x`、`linear_y`、`angular_z`、`spin_mode`、`align_mode`） |
| 12 | `Chassis_Mode_Node` | 对应旧 `Chassis_ModeControl`：左拨杆切小陀螺/跟随，右拨杆切 AI/手动，AI 下再由视觉 `spin_mode` 覆盖 |
| 13 | `Chassis_Kinematics_Node` | 对应旧 `Task_Chassis_Callback`：斜坡、云台相对角换算、旋转控制、四舵轮逆解与最短路径翻转，写 `chassis.motors[].target*` |

### 3.3 Shooter（ID 21–22）

| ID | 节点 | 职责 |
| ---: | --- | --- |
| 21 | `Shooter_Input_Node` | 裁判数据、执行器反馈、当前 `shooter.command/idToTrigger/fricOpen` 快照 |
| 22 | `Shooter_Logic_Node` | 弹速自适应、摩擦轮目标、堵转反转判定、热量限制、拨弹到位记录 |

三张图共同点：节点只做「读快照 / 算 / 发布」，不直接发 CAN、不碰 DMA、不跨图调用。

## 4. 视觉链路

```text
USB CDC 中断（OTG_HS，优先级 5）
  └ VisionLink_OnUsbReceive()   只 memcpy 进 SPSC 环形缓冲（1024 B）
      空间不足则整包丢弃并计数；__DMB 后推进写索引；vTaskNotifyGiveFromISR 唤醒 VisionTask
        ↓
VisionTask（1 kHz）
  └ VisionLink_Poll()           64 B 分块取出 → 消帧器（跨 USB 包组帧）
      校验 0xA5 帧头 / 0x5A 帧尾 → decode → vision_frame_mailbox().publish()
        ↓
Gimbal 图 · Vision_Target_Node
      仅在发布序号变化时重算（其余帧保持上次结果，等价于旧的「每来一帧视觉数据算一次」）
      → 端口给 Gimbal_Input_Node；同时 publish 到 vision_target_mailbox()
        ↓
VisionTask · Vision_DataTransmit()   取 is_valid 作为回传帧的 see_enemy
Chassis_Input_Node / B2B_Transmit    直接读原始帧快照（AI 速度、哨兵自主指令）
```

- 帧格式见 [视觉通信协议](../doc/视觉通信协议.md)：接收 88 B（`0xA5`…`0x5A`），发送 49 B（`0x5A`…`0xA5`）。无 CRC，只有帧头/帧尾。
- 邮箱语义是「最新值覆盖写」而非队列：消费者可能跳过中间帧，但一定拿到当前最新值；`sequence` 用来判断有没有新帧。
- TTL：`k_vision_link_ttl_us = 100000`（100 ms）。`evaluate_snapshot(copy, now_us, ttl)` 是纯函数，必须在 `copy_to()` 之后、用同一份副本判断。

## 5. 输入与跨域同步

`Io/Inputs/RobotInputs.cpp` 把 C 侧全局量整份拷成 POD 快照，临界区内只做赋值、不调 HAL、不阻塞：

| 函数 | 读的全局量 | TTL 常量 |
| --- | --- | --- |
| `read_imu` | `INS.*` | `k_imu_ttl_us = 20000` |
| `read_rc` | `rcInfo.*` | `k_rc_ttl_us = 100000` |
| `read_judge` | `USER_JudgeData.*`、`JUDGE_IsValid()` | `k_judge_ttl_us = 100000` |
| `read_actuator_feedback` | `shooter.*Motor`、`gimbal.*Motor`、`chassis.motors[]`、`FEEDBACK`、`STOPFLAG` | — |

写方向（图 → C 全局）目前仍是隐式的，不出现在图依赖上：

| 写者 | 写的量 | 读方 |
| --- | --- | --- |
| `Gimbal_State_Node` | `gimbal.{top_yaw,base_yaw,pitch}` 的角度与 `totalAngle` | `Vision_DataTransmit` |
| `Gimbal_Control_Node` | `gimbal.*.imuPID.output`、`gimbal.base_yaw.imuPID.outer.output` | `MotoApp`（CAN）、`B2B_Transmit`（25–28 字节） |
| `Gimbal_Output_Node` | `gimbal.*_targetAngle`、`shooter.fricOpen/command/idToTrigger` | Shooter 图、`MotoApp` |
| `Chassis_Kinematics_Node` | `chassis.motors[].targetTurnAngle/targetDriveSpeed` | `B2B_Transmit`（1–24 字节） |
| `Shooter_Logic_Node` | `shooter.fricMotor[].targetSpeed`、`triggerMotor.targetAngle`、`command` | `MotoApp`（CAN） |

这些「图 → 全局 → 另一任务」的边没有帧号或版本号校验，正确性依赖各任务周期的相对快慢。要收紧，得把读方也改成从端口/邮箱取。

## 6. 板间通信（USART2，2 Mbaud，1 kHz）

上板 → 下板 64 B（帧头 `0xAA`、帧尾 `0xFE`）：

| 字节 | 内容 |
| --- | --- |
| 1–16 | 四个舵电机角度误差 `targetTurnAngle - now_angle`，float×4 |
| 17–24 | 四个驱动轮目标转速（RPM），int16×4 |
| 25–28 | 大 yaw 目标速度 `-imuPID.outer.output / 1000`，float |
| 29–35 | `USER_SentryCmd_t` 哨兵自主指令（取自视觉帧快照） |
| 36–61 | 当前未写入，接收方不应解释 |
| 62 | `STOPFLAG` |
| 63 | 帧尾 `0xFE` |

下板 → 上板 64 B（帧头 `0xAB`、帧尾 `0xFD`）：1–16 舵角、17–24 轮速、25–28 大 yaw 角度、29–58 裁判数据、62 `FEEDBACK`。

接收链路：DMA（`DMA_NORMAL`）+ IDLE 中断 → `HAL_UARTEx_RxEventCallback` 调 `B2B_Receive()`，并在 `USART2_IRQHandler` 末尾无条件重新 arm。

已知脆弱点（记录现状，未修改）：发送复用同一个 `usart2TxBuf` 调 `HAL_UART_Transmit_DMA`，未检查上一次传输是否完成；接收在中断里直接读 `usart2RxBuf[0]/[63]`，不看长度。

## 7. 框架能力与未启用项

`Framework/PluginGraph.cpp` 已实现：

- 节点注册、依赖边、拓扑排序、编译后冻结；
- 多执行计划（`ROBOT_MAX_EXECUTION_PLANS = 4`），每套计划独立排序；
- 条件分支 `branch_selector` + `set_branch`，汇合边 `add_branch_dependency`；
- 生命周期通知 `on_enter / on_exit / on_plan_changed`（默认空实现，旧节点无需改动）；
- 帧级 profiler（`Platform/STM32/Stm32Profiler`）。

但三张图只用到了单计划 + 无条件分支：`set_plan_count(1U)`，而 `set_branch` / `add_plan_dependency` / `add_branch_dependency` / `request_plan` / `branch_merge_node` 在业务代码里一次都没被调用。这些能力属于「已实现、未接线」，不要当成在用。

## 8. 构建

两个独立工程，源文件清单人工保持一致：同一批 31 个 `.cpp`，且都排除 §9 里标注「不编译」的三个文件。

- **Keil MDK（首选）**：`MDK-ARM/Sentry_H7_Upper_CMake.uvprojx`，ARMCLANG V6.24。输出目录 `MDK-ARM/Sentry_H7_Upper_CMake/` 已加入 `.gitignore`。
- **CMake + Ninja**：需 `arm-none-eabi-gcc` 在 `PATH`（见 `cmake/gcc-arm-none-eabi.cmake`），`cmake --preset Debug`，产物在 `build/Debug/`（已忽略）。
- **clangd**：读 `build/Debug/compile_commands.json`（见 `.clangd`），因此改代码前需先用 CMake 配置过一次。

最近一次 Keil 全量重建：`0 Error(s), 0 Warning(s)`；固件 `Code=104196  RO-data=2656  RW-data=1664  ZI-data=165320`。

## 9. 已弃用 / 死代码

| 对象 | 状态 | 说明 |
| --- | --- | --- |
| `RobotApplication.{hpp,cpp}` | **不编译** | 原「单一组合根」；CMake 用 `list(REMOVE_ITEM)` 排除，Keil 工程也未收录 |
| `app_cpp_entry.{h,cpp}` | **不编译** | 原 C↔C++ 三函数桥接；`main.c` 已不再调用 `AppCpp_Initialize` |
| `ControlExecutorTask.cpp` | **不编译** | 原单控制执行器（一个 1 kHz 任务驱动整图） |
| `Framework/ComparisonTypes.hpp`、`Framework/OutputComparison.hpp` | 零引用 | 通用结果对比接口，无任何包含者 |
| `Framework/BranchMergeNode.hpp` | 零引用 | `branch_merge_node` 模板从未实例化 |
| `SuperCapTask` / `OS_SuperCapCallback` | 空实现 | 仍是 `__weak` 桩 |
| `App/*App.cpp` 的 `Task_Gimbal_Callback`、`Task_Chassis_Callback`、`Task_Shooter_Callback` | 保留、不调用 | 迁移对照用；`vision_receive` / `vision_target` 全局只为它们保留定义 |

注意：`RobotApplication` 不在任何编译单元里，所以它注释中提到的 `g_robot_debug` 与 `ROBOT_ENABLE_*` 宏**都不存在于当前构建**。它 `#include` 的 `Gimbal_Graph_Config.h` 实际只定义三个容量宏（`ROBOT_MAX_PLUGIN_NODES` / `ROBOT_MAX_PLUGIN_EDGES` / `ROBOT_MAX_EXECUTION_PLANS`），没有裁剪开关。

## 10. 待办

1. 高频图任务的优先级与周期约束未定义，抖动未评估（见 §2）。
2. 「图 → C 全局 → 另一任务」的隐式边缺少帧号校验（见 §5）。
3. `B2B_Transmit` 的 DMA 缓冲复用未做忙检查（见 §6）。
4. §9 的死代码是否清理待定；`BranchMergeNode` / `OutputComparison` 要么接线要么删除，否则容易误判能力边界。
5. `Framework/PluginGraph.hpp` 第 46 行注释说容量宏来自 `Config/robot_build_config.h`，该路径不存在（实际在 `Graph/Gimbal/Gimbal_Graph_Config.h`），属注释错引，待随源码修订一并更正。
