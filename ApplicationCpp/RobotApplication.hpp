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

// —— 编译期裁剪开关（ROBOT_ENABLE_xxx 等宏定义）——
#include "robot_build_config.h"

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


    // —— 图与节点成员 ——
    // 组装心跳、电机状态与底盘反馈观察节点；所有对象都是成员/静态对象，
    // 运行期零 new，初始化顺序严格由成员声明顺序（即构造顺序）决定。
    // 根插件图：所有节点注册到这里，由它负责编译与逐帧调度。
    robot::framework::plugin_graph graph_{};
    // 心跳节点指针；实际对象是 .cpp 中的函数内静态变量，不参与运行期分配。
    heartbeat_node* heartbeat_{nullptr};
    // STM32 单调时钟：为帧时间戳、快照年龄和性能统计提供统一时基。
    robot::platform::stm32::stm32_clock clock_{};
    // STM32 性能统计器：记录每帧耗时（构造时注入 clock_）。
    robot::platform::stm32::stm32_profiler profiler_;
    // 当前帧上下文：承载 frame_id、时间戳等跨节点共享的本帧信息。
    robot::framework::frame_context frame_{};
    // 应用是否已完成启动（initialize() 成功即为 true）。
    bool is_initialized_{false};
};

} // namespace robot::application

#endif /* robot_application_hpp */
