# ApplicationCpp

This directory contains user-owned C++17 application and plugin framework code.

CubeMX-generated C files remain under `Core/`, `Drivers/`, `USB_DEVICE/`, and
`Middlewares/`. The C++ layer enters through the small `extern "C"` bridge in
`app_cpp_entry.h`; it does not replace `main.c` or `freertos.c`.

## 底盘反馈观察阶段（2026-10-09）

当前链路：旧 `B2B_Receive()` → C 快照 → `LegacyChassisFeedbackSource` →
`ChassisFeedbackNode` → `ChassisObserverNode` → `g_robot_debug.chassis`。
底盘反馈来自 USART2 下板，之前的 CAN MotorFeedbackNode 是云台/发射机构反馈。

- 不改底盘算法、板间发送、CubeMX 或旧电机输出。
- 两个节点按显式依赖在同一执行器运行；端口不用于异步直接共享。
- C 侧复制快照时短暂屏蔽中断并恢复原 PRIMASK；初始化时间戳 0 不作为无数据标记。
- 仅继承旧接收器的帧头/帧尾校验，未新增 CRC、长度校验或 DMA 缓冲管理。
- 帧率保持 1 kHz；接收计数只随真实板间帧增长。原始反馈不额外滤波，保持值及年龄可见。
- `ROBOT_ENABLE_CHASSIS_OBSERVER=0` 禁用本阶段观察对象与记录；
  `ROBOT_CHASSIS_FEEDBACK_TIMEOUT_MS` 控制观测超时，默认 100 ms。
- `online` 表示下板帧是否超时，不是单个电机的在线判据，也不触发急停。

### Keil 实车观测（用户执行）

重新载入工程、编译、下载后，在 Watch 展开 `g_robot_debug`：

| 字段 | 预期 |
| --- | --- |
| `graph_node_count` / `graph_edge_count` | 不启用分支演示时：开启底盘观察为 5 / 1；关闭为 3 / 0 |
| `initialized` / `running` / `init_error` | 1 / 1 / 0 |
| `chassis.frame_id` | 随插件帧增长 |
| `chassis.receive_count` | 仅下板有效格式帧到达时增长 |
| `chassis.received` | 从未收到为 0，首次收到后为 1 |
| `chassis.age_ms` | 最近下板样本的年龄（received=1 时有效） |
| `chassis.online` | 已接收且年龄不超过阈值时为 1 |
| `chassis.steering_deg[0..3]` | 对照旧 `chassis.motors[i].TurnAngle`，未扣零偏 |
| `chassis.drive_rpm[0..3]` | 对照旧 `chassis.motors[i].now_Speed` |
| `chassis.lower_feedback` | 对照旧 `FEEDBACK` 原值 |

断流后保留最后角度/轮速，online 变 0；恢复反馈后重新变 1。
暂停调试时可能停在快照复制中途，多个 Watch 字段不保证同时刷新。
用户已反馈本阶段 Keil 编译通过；实际周期与实车行为待后续统一烧录验证。


## 后续范围修订（2026-10-09）

后续以框架和薄适配层为主，保留团队原有业务算法，不继续重写底盘解算。
预定义路线的条件选择已进入本次实现；多套预验证计划切换仍是后续目标。
ROS 2 通信桥接与 ONNX/RKNN 推理后端留作上位机扩展，不成为 STM32 依赖。

代码按板块交付用户 Keil 编译，旁路能力就绪后统一烧录观察；新框架暂不接管旧输出。
详细决策见 `doc/插件化架构设计讨论-01.md`。


## 条件分支板块（2026-10-09，用户已确认 Keil 编译通过）

本阶段增加固定 DAG 内的条件执行，代码只产生算术演示数据，不调用电机、CAN、UART 或旧业务函数。

```text
公共输入 10 ─┬─ 路线 A（父节点条件 0）：加 1 → 乘 2 ─┐
             └─ 路线 B（父节点条件 1）：减 1 → 乘 3 ─┴─ 汇合 → Debug
```

路线 A/B 都是普通 PluginNode，内部各有两个普通节点和一张图。根执行计划统一展开它们；父节点的分支条件自动向后代传递，不递归重复执行业务。

### Watch 使用

保持 `ROBOT_USE_PLUGIN_GRAPH=0`；它仍是旧输出接管的预留开关，本例不发送硬件命令。
`ROBOT_ENABLE_BRANCH_DEMO` 控制演示对象组装，默认 1。正常初始化为 `initialized=1`、`init_error=0`。

| 底盘观察 / 分支演示 | 节点 / 边 |
| --- | --- |
| 1 / 1（当前默认） | 14 / 7 |
| 0 / 1 | 12 / 6 |
| 1 / 0 | 5 / 1 |
| 0 / 0 | 3 / 0 |

在 Watch 中修改 `g_robot_debug.branch.requested_route`：

- 0：下一帧选 A，`output_value=22`。
- 1：下一帧选 B，`output_value=27`。
- 其他值：保留上一合法路线，`rejected_frames` 在每个请求非法的帧递增。

| 字段（均位于 g_robot_debug.branch） | 含义 |
| --- | --- |
| `requested_route` | 唯一演示输入字段，写 0 或 1 |
| `active_route` | 本帧实际选择 |
| `generation` / `switch_count` | 路线实际改变时递增，不因重复写同一路线递增 |
| `frame_id` / `output_frame_id` | 有效汇合结果应属于当前帧 |
| `output_valid` / `output_value` | 有效为 1；无效结果置 0 且禁止当作控制量 |
| `source.execution_count` | 公共输入每帧执行一次 |
| `merge.execution_count` | 正常时每帧汇合一次 |
| `routes[0].first/second.execution_count` | A 两级节点执行次数 |
| `routes[1].first/second.execution_count` | B 两级节点执行次数 |
| `routes[i].first/second.branch_skip_count` | 未选中的路线每级分别计数 |
| `routes[i].first/second.blocked_count` | 前置条件或运行状态阻塞，不是正常条件跳过 |
| `routes[i].valid` / `value` | 选中路线为本帧有效结果；另一条无效 |

`frame_state` 对应：0=Pending、1=Executed、2=SkippedBranch、3=SkippedState、4=Blocked、5=NoData、6=Faulted。
暂停在 Debug 复制中途时字段可能暂时来自不同更新位置，建议在 `updateBranchDebug()` 完成后观察一致结果。

### 框架接口和约束

- `BranchSelector(count, initial)` 固定路线集合；`request(route)` 只由所属执行器线程调用，不是跨任务原子接口。
- `graph.setBranch(node, selector, route)` 只设置该图直接节点；嵌套图可继续设置自己的条件，所有祖先条件相与。子图展开后立即冻结，后续 setBranch/add/addDependency 返回 Frozen；应在 compose 内完成局部组装。
- `graph.addDependency(before, after)` 是普通依赖；端点可在尚未展开的子图中，是否存在延后到 compile 检查。
- `graph.addBranchDependency(end, merge)` 只允许忽略 SkippedBranch，不能忽略 Faulted、NoData 或未启动节点。汇合至少需要一个真实完成的分支输入。
- `BranchMergeNode<T,N>` 转发 selector 所选输入，检查输入有效且帧号一致。端口绑定和依赖边必须都声明。
- 分支节点的 `onSkipped(context)` 应使自身输出失效；默认回调为空，以保持旧节点兼容。现有旧端口的保持值语义不变，不能将此改动当作所有旧端口自动防陈旧数据的保证。
- 所有节点启动时配置一次；路线切换不自动重新 start/stop，不清空业务内部状态。激活/退出状态重置留待下一板块明确实现。
- 当前一个执行域、一个 root graph 管理这些对象；禁止同一选择器/节点被多个并行根执行器同时驱动。
- 当前固定展开图仍必须整体无环。A→B 与 B→A 两种顺序的多计划切换尚未实现。
- 未添加 Linux/ROS/模型运行库，不使用运行期堆分配；编译和实际 1 kHz 时序由用户验证。

验证记录：2026-10-09 用户反馈条件分支板块 Keil 编译通过。路线切换、时序与实车行为仍待统一烧录验证。
