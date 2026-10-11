// ============================================================================
// robot_application.hpp —— 机器人应用层“组合根”（Composition Root）的声明。
//
// 角色定位：
//   1) 组合根是整棵“插件图”的唯一装配点：由它负责创建节点对象、把节点
//      注册进 plugin_graph（图）、声明节点之间的依赖，并把输入/输出端口连起来；
//   2) 它同时是“每帧入口”：FreeRTOS 控制任务每次被唤醒时调用一次
//      process_frame()，由它向图索取一帧的执行结果；
//   3) 它把各节点的运行结果整理成 g_robot_debug 调试快照，供 Ozone/Watch
//      观察；业务节点本身不直接读写这些调试全局量。
//
// 关键概念：
//   * 节点（plugin_node）：最小执行单元，拥有输入端口、输出端口和 process()；
//   * 图（plugin_graph）：节点容器 + 执行计划，负责拓扑排序与逐帧调度；
//   * 执行计划（ExecutionPlan）：预先编译好的节点执行顺序，可在运行期切换；
//   * 快照（Snapshot）：可跨任务安全传递的只读结果副本（异步邮箱）。
//
// 说明：本文件中大量代码由 ROBOT_ENABLE_xxx 宏控制是否启用，各演示链路
//       （分支、计划、快照、比较、底盘观察）的具体作用见对应成员注释。
// ============================================================================
#ifndef robot_application_hpp
#define robot_application_hpp

// —— 框架核心：帧上下文、图容器、节点基类 ——
#include "Framework/FrameContext.hpp"
#include "Framework/PluginGraph.hpp"
#include "Framework/PluginNode.hpp"
// —— 电机链路：旧电机反馈适配 + 反馈超时离线检测 ——
#include "Nodes/Motor/MotorFeedbackNode.hpp"
#include "Nodes/Motor/MotorOfflineNode.hpp"
// —— 编译期裁剪开关（ROBOT_ENABLE_xxx 等宏定义）——
#include "robot_build_config.h"
#if ROBOT_ENABLE_COMPARISON_DEMO
// 比较演示链路：验证两路独立状态在帧边界上的一致性。
#include "Nodes/Diagnostics/ComparisonDemoNode.hpp"
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
// 快照演示链路：验证跨任务的异步邮箱发布/读取。
#include "Nodes/Diagnostics/SnapshotDemoNode.hpp"
#endif
#if ROBOT_ENABLE_PLAN_DEMO
// 执行计划演示链路：验证图可预存多套执行顺序并在运行期切换。
#include "Nodes/Diagnostics/ExecutionPlanDemoNode.hpp"
static_assert(ROBOT_MAX_EXECUTION_PLANS >= 2U, "Plan demo needs two plans"); // 编译期强制：计划演示至少需要 2 套计划。
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
// 条件分支演示链路：验证运行期的分支选择与汇合。
#include "Nodes/Diagnostics/BranchDemoNode.hpp"
#endif
#if ROBOT_ENABLE_CHASSIS_OBSERVER
// 底盘观察链路：把下板板间反馈适配成节点信号，只读观察、不参与控制输出。
#include "Nodes/Chassis/ChassisFeedbackNode.hpp" // 发布板间反馈的节点。
#include "Nodes/Chassis/ChassisObserverNode.hpp" // 汇总反馈并判断在线状态的观察节点。
#include "../Platform/STM32/LegacyChassisFeedbackSource.hpp" // 包装旧 C 反馈源的适配器。
#endif
// —— 平台抽象接口与 STM32 具体实现（时钟、性能统计）——
#include "Platform/IClock.hpp"
#include "Platform/IProfiler.hpp"
#include "../Platform/STM32/Stm32Clock.hpp"
#include "../Platform/STM32/Stm32Profiler.hpp"

// 应用层命名空间：组合根及其内部节点均处于此作用域。
namespace robot::application {

// robot_application —— 组合根（Composition Root）。
// 职责：
//   1) 持有并组装整棵插件图：创建/登记节点、声明依赖、连接端口；
//   2) 在 initialize() 中按“设置 → 注册 → 编译 → 配置 → 启动”的顺序启动图；
//   3) 在 process_frame() 中作为每帧唯一入口向图索取一帧结果，并把结果同步到调试快照。
// 用法：构造实例后先调用 initialize()；成功后再由控制任务周期调用 process_frame()。
class robot_application {
public:
    // 构造函数：初始化平台服务与各节点成员，并绑定静态心跳节点。
    // 注意：只做成员构造/指针绑定，不做图组装（组装在 initialize() 内完成）。
    robot_application();

    // initialize() —— 组装并启动插件图。
    // 返回值：plugin_status::ok 表示全部成功；否则返回失败阶段对应的状态码。
    // 调用时机：调度启动前调用一次；不可重复调用（重复调用返回 invalid_state）。
    robot::framework::plugin_status initialize();
    // process_frame() —— 每帧入口，处理一帧插件业务。
    // 返回值：本帧图的汇总处理结果（ok/fault 等）。
    // 调用时机：FreeRTOS 控制任务每次被唤醒时调用一次（当前为 1 kHz 基础节拍）。
    robot::framework::process_result process_frame();
    // 查询应用是否已初始化成功（initialize() 返回 ok 后为 true）。
    bool is_initialized() const { return is_initialized_; }

private:
    // 前置声明：心跳节点定义在 .cpp 中（唯一不依赖硬件的常驻节点）。
    class heartbeat_node;
#if ROBOT_ENABLE_COMPARISON_DEMO
    // 帧结束后把比较演示的观察结果复制到调试快照；
    // 只做只读复制，并保留 Watch 中的请求字段（input_value 等）供外部修改。
    void update_comparison_debug();
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    // 帧结束后把当前执行计划与 A/B 探针的执行统计复制到调试快照；
    // 不覆盖用户在 Watch 里写入的 requested_id。
    void update_plan_debug();
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    // 本帧执行结束后把分支选择器状态与各路节点统计复制到调试快照；
    // 保留用户写入的 requested_id 请求值。
    void update_branch_debug();
#endif
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    // 把底盘观察节点的结果复制到 C 调试符号（g_robot_debug.chassis）；
    // 业务节点不直接引用全局调试对象，保持解耦。
    void update_chassis_debug();
#endif

    // —— 图与节点成员 ——
    // 组装心跳、电机状态与底盘反馈观察节点；所有对象都是成员/静态对象，
    // 运行期零 new，初始化顺序严格由成员声明顺序（即构造顺序）决定。
    // 根插件图：所有节点注册到这里，由它负责编译与逐帧调度。
    robot::framework::plugin_graph graph_{};
    // 心跳节点指针；实际对象是 .cpp 中的函数内静态变量，不参与运行期分配。
    heartbeat_node* heartbeat_{nullptr};
    robot::motor::motor_feedback_node motor_feedback_{}; // 旧电机反馈 → 新端口体系的适配节点。
    robot::motor::motor_offline_node motor_offline_{}; // 电机反馈超时检测：判据为长时间收不到反馈。
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    // —— 底盘板间反馈观察链路（只读）——
    robot::platform::stm32::legacy_chassis_feedback_source chassis_source_{}; // 包装旧 C 反馈源的适配器；必须先于引用它的两个节点构造（声明顺序即构造顺序）。
    robot::chassis::chassis_feedback_node chassis_feedback_{30U, chassis_source_}; // 节点 ID 30：从适配器取板间反馈并发布为信号。
    robot::chassis::chassis_observer_node chassis_observer_{31U, ROBOT_CHASSIS_FEEDBACK_TIMEOUT_MS}; // 节点 ID 31：汇总反馈并按超时阈值判断下板是否在线。
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    robot::diagnostics::branch_demo_node branch_demo_{}; // 条件分支演示图：无硬件输出，验证运行期分支选择与汇合。
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    robot::diagnostics::execution_plan_demo_node plan_demo_{}; // 执行计划演示图：无硬件输出，验证多套预编译执行顺序的切换。
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    // 帧结束后把快照邮箱的序号/年龄/状态复制到调试快照。
    void update_snapshot_debug();
#endif
    // STM32 单调时钟：为帧时间戳、快照年龄和性能统计提供统一时基。
    robot::platform::stm32::stm32_clock clock_{};
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    robot::diagnostics::snapshot_demo_node snapshot_demo_; // 异步快照演示图：通过邮箱跨任务发布/读取快照（构造时注入 clock_）。
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    robot::diagnostics::comparison_demo_node comparison_demo_{}; // 比较演示图：对两路独立状态做算术比对，仅产生调试数据。
#endif
    // STM32 性能统计器：记录每帧耗时（构造时注入 clock_）。
    robot::platform::stm32::stm32_profiler profiler_;
    // 当前帧上下文：承载 frame_id、时间戳等跨节点共享的本帧信息。
    robot::framework::frame_context frame_{};
    // 应用是否已完成启动（initialize() 成功即为 true）。
    bool is_initialized_{false};
};

} // namespace robot::application

#endif /* robot_application_hpp */
