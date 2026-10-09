# STM32 底盘反馈观察链路实施计划

日期：2026-10-09。用户已授权继续渐进迁移；编译及实车验证由用户执行。

**目标：** 将 USART2 下板反馈接入强类型端口，形成有依赖边的底盘观察链路。
**架构：** 旧 C 接收器发布固定快照；STM32 数据源通过 C ABI 读取；纯 C++ 节点按同帧依赖执行；应用层复制诊断结果到 C 链接调试快照。

## 实施范围

- [x] 新增 Platform/STM32/ChassisFeedbackBridge.h，在 UserMiddlewares/src/USER_B2B.c 实现快照发布与读取。沿用原帧头/帧尾判定；记录首次接收标志、接收计数与 HAL tick，读端保存并恢复 PRIMASK，仅在拷贝时屏蔽中断。
- [x] 新增 ChassisData.hpp、IChassisFeedbackSource.hpp 和 STM32 LegacyChassisFeedbackSource。保留舵角 degree 与轮速原始 rpm，不将云台 CAN 数据当作底盘反馈。
- [x] 新增 ChassisFeedbackNode 和 ChassisObserverNode；发布反馈并计算接收超时，使用输入绑定与 addDependency，不创建新任务。
- [x] RobotApplication 注册节点并接线，追加 g_robot_debug.chassis，记录角度、轮速、帧号、接收次数、年龄、接收标志、链路在线、下板 FEEDBACK 原值。
- [x] 共享配置头设置底盘观察开关及 100 ms 观测阈值；只控制观察对象及记录，不改变旧收发。
- [x] 保留当前 Keil 设置，补全本阶段 .cpp 条目及前阶段缺失的 MotorOfflineNode.cpp。CMake 已递归收集用户 .cpp，无需改生成配置。
- [x] 静态核对链接、类型、顺序、工程路径及 diff；更新文档和 Watch 说明。编译烧录及实际时序由用户验证。

## 数据语义

首次接收前 received=false；tick=0 也是合法接收时间。每 1 ms 发布不代表每 1 ms 收到了新数据，接收次数和年龄独立记录。online 只表示下板帧在线，不表示四个电机各自在线。继承旧协议校验能力，不新增 CRC、不改 DMA、CubeMX、PID 或发送。端口仅在同一个执行器中按依赖顺序使用。

## 用户实车观测

启用后预期节点数 5、依赖边数 1。正常回传时接收计数增长；无数据时 received=false；超过 100 ms 未更新时 online=false，旧样本仍保留。四个角度和轮速对照旧 chassis.motors。关闭观察宏后预期 3 个节点、0 条边，旧业务继续工作。


## 静态检查结果

2026-10-09：原板间解码与发送内容与本次修改前基线一致；Keil XML 可解析，所有源路径存在，新增三个 C++ 文件及 MotorOfflineNode.cpp 均只注册一次且类型为 8；目标选项与本次修改前一致。检查了节点无 HAL/FreeRTOS 依赖、输入绑定与执行依赖建立在图冻结前、调试数据在图执行后复制。git diff --check 通过。

未执行主机测试、固件编译或烧录；用户实车验收仍待完成。
