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
预定义路线、预验证执行计划切换、生命周期通知和最新值邮箱均已有实现，原始板块已收到用户编译通过反馈。
ROS 2 通信桥接与 ONNX/RKNN 推理后端留作上位机扩展，不成为 STM32 依赖。

最新用户约定：不再逐板块等待用户 Keil 编译；助手连续推进并自行做可用检查，最后由用户统一烧录观察。旧源码和输出链路保持原样。
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

下表仅适用于关闭执行计划演示（`ROBOT_ENABLE_PLAN_DEMO=0`）时：

| 底盘观察 / 分支演示 | 节点 / 边 |
| --- | --- |
| 1 / 1 | 14 / 7 |
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
- 所有节点启动时配置一次；路线切换不自动重新 start/stop。已提供 onEnter/onExit/onPlanChanged，业务状态是否重置由节点自己的回调决定。
- 当前一个执行域、一个 root graph 管理这些对象；禁止同一选择器/节点被多个并行根执行器同时驱动。
- 每套执行计划必须单独无环；新增多计划能力允许 A→B 与 B→A 属于不同计划，详见下节。
- 未添加 Linux/ROS/模型运行库，不使用运行期堆分配；编译和实际 1 kHz 时序由用户验证。

验证记录：2026-10-09 用户反馈条件分支板块 Keil 编译通过。路线切换、时序与实车行为仍待统一烧录验证。


## 多执行计划切换板块（2026-10-09，用户已确认编译通过）

本板块改变执行顺序，不改变数据连接或调用业务算法。新增无硬件探针 A/B，复用同一批实例：

```text
计划 0：A → B
计划 1：B → A
```

公共依赖在所有计划生效；计划专属依赖分别参与各自拓扑排序。
虽然两条专属边的合并图成环，每个单独计划无环，仍是合法配置。
任何计划包含真实环或引用不存在的节点，整张图都不能启动，不会偷偷退回计划 0 运行。

### 组装 API

```cpp
// 根组装层声明两套计划；子图使用相同的全局计划编号。
graph.setPlanCount(2U);
// 组装期注册节点后声明专属顺序边。
graph.addPlanDependency(a, b, 0U);
graph.addPlanDependency(b, a, 1U);
// compile 检查全部计划；仅所属执行任务可以提交切换请求。
graph.requestPlan(1U);
```

示例省略状态码处理，实际代码检查每次组装结果。
`requestPlan()` 仅在所有计划编译成功后接受合法编号，下一帧开始锁存。
同一帧内提出的后续请求留给下一帧；非法编号保留原请求。
`addDependency()` 和 `addBranchDependency()` 保持全计划有效；需反转的边必须声明为专属边。
编译失败的对象保留错误，不支持修补部分展开结果后原地重试，应在重新启动/重新构造时修正组装配置。

### Watch 观测

配置开关 `ROBOT_ENABLE_PLAN_DEMO=1` 默认开启；`ROBOT_MAX_EXECUTION_PLANS=4` 预留四套固定存储，本例实际声明两套。
在上一板块默认 14 节点/7 边基础上，新增 3 节点/2 条声明边：当前应为 **17 节点、9 条声明边、2 个计划**。
每个计划实际使用其中 8 条依赖，不同计划的边不能同时用于本帧。
关闭计划演示后节点/边恢复上一板块，图默认只编译计划 0。

| 字段（g_robot_debug.plan 下） | 预期 |
| --- | --- |
| `requested_plan` | 可写 0 或 1，其他值拒绝 |
| `active_plan` | 本帧采用计划 |
| `plan_count` | 默认 2 |
| `failed_plan` | 默认 65535 表示没有具体计划编译失败；结合 init_error 查看 |
| `switch_count` / `generation` | 仅实际计划改变时递增 |
| `rejected_frames` | 每个非法请求帧递增，保留上一合法计划 |
| `frame_id` | 当前帧号 |
| `a.execution_count` / `b.execution_count` | 正常运行时每帧各递增一次，切换不清零 |
| `a.frame_order` / `b.frame_order` | 计划0为A小于B；计划1为B小于A |
| `a.frame_state` / `b.frame_state` | 正常应为1（Executed） |

`frame_order` 是整个根图本帧的实际 process 调用序号，不保证从 A=0 开始；65535 表示该节点本帧未执行。
本演示是执行顺序探针，不读取对方的旧输出，也不依赖共享可变业务状态。
上一板块的 `g_robot_debug.branch.requested_route` 仍独立控制 A/B 算术分支，与 `plan.requested_plan` 是两个不同控制项。

### 当前边界

- 两套计划包含同一批节点，分支选择依然可跳过不选中的节点。
- 节点 configure/start 固定按计划0调用一次，stop 按计划0逆序；运行期切换不调用这些函数、不重置 PID/滤波或其他状态。生命周期扩展用 onEnter/onExit/onPlanChanged，默认为空，业务节点自行决定是否重置状态。
- 数据端口不自动重连；如两种顺序需要不同数据来源，调用者必须显式适配且检查帧号，不能用同帧环互相等待。
- 当前仍是单根、单执行任务；请求不是线程安全 API，跨任务入口需要平台交接。
- 计划切换通知已由后续生命周期板块提供；异步推理取消、运行时任意改图尚未实现。
- 原始板块用户已确认 Keil 编译通过，资源占用和实际时序仍待实车验证。

2026-10-09 验证更新：用户确认多执行计划板块 Keil 编译通过；运行顺序切换与实际时序待统一烧录验证。


## 节点路径生命周期通知板块（2026-10-09，用户已确认编译通过）

本板块为所有节点增加可选生命周期通知，服务于路径切换、PID/滤波状态策略和未来模型上下文：

```text
帧开始锁存路线/计划
    ↓
计算本帧激活集合
    ↓
旧激活节点逆序 onExit
    ↓
新激活节点顺序 onEnter
继续激活节点（仅计划变化）onPlanChanged
    ↓
执行 process
```

输入暂时没有新数据、一次 `NoNewData` 或条件分支汇合阻塞，不会导致节点反复退出/进入；激活表示路径和节点状态可运行，不代表输入一定齐备。

- 新增节点默认回调为空，旧业务节点无需修改。
- 选择分支/计划切换不会自动调用 configure/start/stop。
- 演示算术节点在 onEnter 清零本次激活执行计数，在 onExit 使输出失效；框架累计执行次数不清零。
- 回调中不能重配图、阻塞或修改其他节点生命周期；请求下一路线/计划只影响后续帧。
- stopAll 只允许帧外调用：先通知当前激活节点退出，再按计划0逆序 stop，重复 stop 不重复 exit。
- 回调自报故障不会回滚已发出的进入通知；当帧不再执行，下一帧按激活状态发出退出。

### Watch 新字段

位于 `g_robot_debug.branch.routes[i].first/second`：

```text
enter_count
exit_count
plan_change_count
active
```

`first_runs_since_enter` 和 `second_runs_since_enter` 是演示节点每次 onEnter 清零的业务计数；它们与框架累计 `execution_count` 分开。

2026-10-09 验证更新：用户确认生命周期通知板块 Keil 编译通过；通知行为与时序待后续统一实车验证。


## 异步最新值邮箱板块（2026-10-09，原始板块用户已确认编译通过）

本阶段为未来 ROS 2 订阅、视觉结果和模型推理结果预留“异步最新值”边界，同时只在 STM32 上使用一个无硬件演示：

```text
SnapshotProducerNode（每10帧发布）
        ↓ 最新值邮箱 + STM32 短临界区
SnapshotConsumerNode（每帧读取）
        ↓
g_robot_debug.snapshot
```

它和 `FrameSignal<T>` 的语义不同：`FrameSignal` 用于同一个插件图执行帧内的同帧数据；邮箱用于外部回调/任务与图任务之间的最新样本交接。邮箱不会排队所有历史消息，也不会因为读取刷新样本序号或采样时间。

### Watch 观测

`ROBOT_ENABLE_SNAPSHOT_DEMO=1` 时，默认节点数从上一阶段的 17 增加到 20，边从 9 增加到 10。`g_robot_debug.snapshot.publish_enabled` 可写：

```text
1：生产节点每10帧发布一个新样本
0：暂停发布；消费者继续每帧读取最新旧样本
```

| 字段 | 含义 |
| --- | --- |
| `frame_id` | 最近消费者帧 |
| `sequence` | 只在生产者发布时递增；暂停后保持 |
| `read_count` | 消费节点读取次数 |
| `age_us` | 当前时钟减原始采样时刻 |
| `status` | 0=Empty，1=Fresh，2=Stale，3=ClockMismatch |
| `publish_enabled` | Watch 发布开关 |

启动后第一次发布前状态为 Empty；发布后按 TTL=5000 us 判断 Fresh/Stale。暂停发布后 sequence 不再增长、age_us 继续增长，超过 TTL 后 status=Stale。读取旧值不会把它当作新样本。

STM32 后端只在复制固定快照时保存/禁用/恢复 PRIMASK，并使用内存屏障；不保护 DMA、NMI、HardFault 或多核访问。真实跨任务接口仍需由平台调用者选定单一同步后端和时钟域。

该板块不实现 ROS 2、ONNX、RKNN、事件队列或模型请求取消，也不修改现有业务和硬件输出。

2026-10-09 20:11 用户已确认原始邮箱板块编译通过。后续修正中，消费者只调用一次 `copyTo()`，然后在取得当前时间后调用 `evaluateSnapshot(snapshot, now, ttl)`；序号、数值与有效期属于同一份稳定副本。兼容的 `mailbox.status()` 仍可用于只关心状态的调用者，不能把它与另一次读取的数值拼成同一份样本。

## 通用数值对比与性能修正（2026-10-09）

新增 `Framework/OutputComparison.hpp`，用于比较两份已经计算完的通用数值结果。只有输入序号、采样时间、状态版本及计算步数都相同，才计算 `candidate - reference` 和最大绝对差。缺失、未对齐、非法容差和非有限值与真正的结果差异分开表示。

接口是纯函数，不调用业务函数、不借用对方控制状态，也不发送硬件命令。当前已接入 g_robot_debug.comparison 的合成输入演示，尚未接入真实控制输出，不能据此宣称已获得两套完整程序的电机输出对比。包含该头文件的新翻译单元必须使用 `-fno-fast-math`；ARMClang 默认的有限数假设会破坏 NaN/Inf 诊断，因此头文件对此模式明确报错，旧业务编译选项未改动。

`PluginGraph` 已恢复在实际 `process()` 调用前后执行 `beginNode/endNode`。条件跳过、依赖阻塞不产生节点执行耗时；生命周期回调不属于该节点 `process` 耗时。帧级统计范围保持原样，包含图调度与 Debug 汇总。当前 `last_node_duration_us/max_node_duration_us` 仍是所有节点共用的最近值/最大值，不是逐节点统计表。

`checks/FrameworkContractChecks.cpp` 使用 ARMClang 编译期断言验证快照 TTL、时间回拨、比较身份、容差及非有限/溢出边界；不进入固件、不引入主机仿真。受影响的新框架翻译单元也完成目标编译器语法检查。以上不是固件全量链接或实车通过的结论。

接口用法、检查命令与交付边界见 [通用快照与结果对比接口](../doc/通用快照与结果对比接口-2026-10-09.md)。

## 独立状态算术比较演示（2026-10-09）

ROBOT_ENABLE_COMPARISON_DEMO=1 时新增 ID 100 容器、101 输入、102 参考、103 候选、104 比较，共 5 个节点和 4 条公共依赖。默认整图为 **25 节点、14 条声明边、2 个计划**，每计划使用 13 条边。之前各阶段的节点数是历史记录。

两个累计器独占各自状态，只读取同一个每帧生成的整数输入，不读取或调用旧业务。调试入口为 g_robot_debug.comparison：

- 正常：status=4（Equal），difference_valid=1，差值为 0。
- candidate_bias=3：status=5（Different），difference[0]=3；清零偏差后恢复。
- pause_candidate=1：保留旧候选结果，status=1（Unaligned）；恢复不补算漏帧，需共同重置对齐。
- reset_candidate_request 改为不同数值：只重置候选，epoch 不同；reset_all_request 改值：分别重置两实例。
- reset_all_request 同帧优先；共同重置时 pause_candidate 应为 0，偏差为 0 才预期 Equal。
- input_value / candidate_bias 合法范围 [-1000,1000]，非法值保留上次参数，applied_* 和 rejected_frames 可观察。
- 重置不清除历史计数与 historical_max_abs_difference；Missing/Unaligned 的零差值没有比较意义。

只有 ComparisonDemoNode.cpp 包含严格浮点比较算法，CMake/Keil 都为该文件设置 -fno-fast-math。ComparisonTypes.hpp 只定义数据，使应用和其他头文件保持原编译选项。

本次 29 条目标编译期断言及受影响应用文件语法检查通过；独立只读审查无待修项。没有运行 UV4 全量构建、完整固件链接或本版本上板实测。逐项 Watch 步骤、当前等待事项及后续计划见 [通用快照与结果对比接口](../doc/通用快照与结果对比接口-2026-10-09.md)。