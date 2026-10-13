// ============================================================================
// robot_application.cpp —— 组合根（Composition Root）的实现。
//
// 本文件实现 robot_application 的四个职责：
//   1) 构造函数：初始化平台服务与节点成员，并绑定静态心跳节点；
//   2) initialize()：按“设置计划数 → 注册节点 → 声明依赖 → 编译 →
//      配置 → 启动”的顺序组装并启动整棵插件图；
//   3) process_frame()：每帧入口，先读取 Watch 输入，再驱动图处理一帧，
//      最后把结果同步到调试快照；
//   4) update_xxx_debug()：把各演示链路的运行结果复制到 g_robot_debug。
//
// 注意：这里不实现任何控制算法，只做“把零件装起来 + 每帧推一下”。
// ============================================================================
#include "RobotApplication.hpp"

// 应用层命名空间：组合根及其内部节点实现均处于此作用域。
namespace robot::application {

// heartbeat_node —— 心跳节点（骨架阶段的占位节点）。
// 用途：不产生任何硬件输出，仅用于验证调度链路、计时与调试快照是否正常。
// 它是图中的一个“常驻活动节点”，保证每帧都有节点被执行，便于判断图是否在运行。
class robot_application::heartbeat_node final : public robot::framework::plugin_node {
public:
    heartbeat_node() : plugin_node(1U) {} // 节点 ID 固定为 1。

    // 每帧回调。
    // 参数：当前帧上下文（此处不使用；基础节拍为 1 kHz）。
    // 返回值：恒为 ok，表示本节点无故障。
    robot::framework::process_result process(robot::framework::frame_context&) override
    {
        return robot::framework::process_result::ok;
    }
};

// 构造函数：只做“成员初始化 + 心跳节点绑定”，不做图组装。
// 设计原则：所有节点对象都是成员/静态对象，运行期绝不 new/delete，
//           因此初始化顺序完全由成员声明顺序（即下方初始化列表顺序）决定。
robot_application::robot_application()
    : clock_() // 先建时钟：后面的快照演示与性能统计都要用到它。
    , profiler_(clock_) // 性能统计器需要时钟来测量每帧耗时。
{
    // 心跳节点使用函数内静态存储期：常驻且只构造一次。
    // 用指针保存是因为它与本对象生命周期无关，避免在头文件中暴露其定义。
    static heartbeat_node heartbeat_node;
    heartbeat_ = &heartbeat_node; // 登记指针，供 initialize() 注册到图中。
}

// initialize() —— 组装并启动插件图。
// 阶段顺序（全程用 status 串联，任一阶段失败即短路返回）：
//   1) 图级设置（计划数）；
//   2) 注册节点（add）；
//   3) 声明节点依赖（add_dependency）并连接端口（bind）；
//   4) 编译（compile：拓扑排序 / 生成执行计划）；
//   5) 配置（configure_all）与启动（start_all）；
//   6) 把图规模、出错状态等回填到调试快照。
robot::framework::plugin_status robot_application::initialize()
{
    // 保护
    if (is_initialized_)
    {
        return robot::framework::plugin_status::invalid_state;
    }

    // 准备status
    robot::framework::plugin_status status = robot::framework::plugin_status::ok;

    // —— 阶段 2：注册节点 ——
    // 先注册心跳节点（保证图非空），再注册只读的电机反馈链路。
    
    // —— 阶段 4~5：编译 → 配置 → 启动 ——
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.compile(); // 展开依赖、拓扑排序、生成执行计划。
    }
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.configure_all(); // 对每个节点做启动前配置。
    }
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.start_all(); // 让所有节点进入运行态。
    }

    // —— 阶段 6：回填调试快照（无论成败都记录，便于定位问题）——

    // 只有全程 ok 才把本对象标记为已初始化。
    is_initialized_ = (status == robot::framework::plugin_status::ok);
    return status;
}

// process_frame() —— 每帧唯一入口，由 FreeRTOS 控制任务周期性调用。
// 一帧流程：
//   1) 读取 Watch/调试输入的“请求值”（分支、计划、快照开关、比较控制）；
//   2) 推进帧号与时间戳，开启性能统计；
//   3) 驱动整张图处理本帧；
//   4) 把各链路结果复制到调试快照；
//   5) 收尾计时并记录本帧汇总结果。
robot::framework::process_result robot_application::process_frame()
{
    // 未初始化则视为故障帧直接返回，避免操作尚未组装的图。
    if (!is_initialized_)
    {
        return robot::framework::process_result::fault;
    }

    // —— 推进本帧上下文并计时 ——
    frame_.frame_id++; // 帧号自增，供各节点判断数据是否属于本帧。

    // 性能测试开始
    frame_.timestamp_us = clock_.now_us();
    profiler_.begin_frame(frame_.frame_id, frame_.timestamp_us);

    // 根图启动
    const robot::framework::process_result result = graph_.process_frame(frame_, &profiler_); // result：本帧图的汇总处理结果。
    
    // 性能测试结束
    frame_.timestamp_us = clock_.now_us();
    profiler_.end_frame(frame_.timestamp_us); 

    return result;
}

} // namespace robot::application
