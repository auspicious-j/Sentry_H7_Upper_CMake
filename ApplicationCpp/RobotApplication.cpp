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

#include "Platform/DebugSnapshot.hpp" // g_robot_debug 调试快照的声明。

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
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    , snapshot_demo_(clock_) // 快照演示节点需要时钟来打时间戳。
#endif
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
    // 幂等保护：已初始化过则直接拒绝，避免重复组装图。
    if (is_initialized_)
    {
        return robot::framework::plugin_status::invalid_state;
    }

    // status —— 贯穿整个组装流程的状态码。
    // 采用“链式短路”：只有当前状态仍为 ok 时，才继续下一步装配。
    robot::framework::plugin_status status = robot::framework::plugin_status::ok;
#if ROBOT_ENABLE_PLAN_DEMO
    status = graph_.set_plan_count(2U); // 计划演示需要预存 2 套执行计划。
#endif
    // —— 阶段 2：注册节点 ——
    // 先注册心跳节点（保证图非空），再注册只读的电机反馈链路。
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(*heartbeat_); // 节点 ID 1：心跳。
    }
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(motor_feedback_); // 旧电机反馈 → 新端口适配。
    }
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(motor_offline_); // 电机反馈超时检测。
    }
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    // —— 底盘观察链路：先注册发布节点，再注册观察节点，并建立二者依赖 ——
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(chassis_feedback_); // 发布板间反馈。
    }
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(chassis_observer_); // 观察并判断在线状态。
    }
    if (status == robot::framework::plugin_status::ok)
    {
        chassis_observer_.input().bind(chassis_feedback_.signal()); // 把发布节点的输出端口连到观察节点的输入端口。
        status = graph_.add_dependency(chassis_feedback_, chassis_observer_); // 声明“观察依赖发布”，参与拓扑排序。
    }
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(branch_demo_); // 条件分支演示图。
    }
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(plan_demo_); // 执行计划演示图。
    }
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(snapshot_demo_); // 异步快照演示图。
    }
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    if (status == robot::framework::plugin_status::ok)
    {
        status = graph_.add(comparison_demo_); // 比较演示图。
    }
#endif
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
    g_robot_debug.graph_node_count = graph_.node_count(); // 图内节点总数。
    g_robot_debug.graph_edge_count = graph_.edge_count(); // 图内依赖边总数。
    g_robot_debug.init_error = static_cast<uint32_t>(status); // 失败时对应阶段的状态码。
    g_robot_debug.plan.plan_count = graph_.plan_count(); // 已登记的计划数。
    g_robot_debug.plan.failed_id = graph_.failed_plan(); // 编译失败的探针计划。
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    g_robot_debug.snapshot.publish_enabled = 1U; // 快照演示默认开启发布。
#endif
    g_robot_debug.is_initialized = (status == robot::framework::plugin_status::ok) ? 1U : 0U; // 是否初始化成功。
    g_robot_debug.running = (status == robot::framework::plugin_status::ok) ? 1U : 0U; // 是否处于运行态。

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

#if ROBOT_ENABLE_BRANCH_DEMO
    // 读取本帧的分支路由请求（0=A、1=B）。
    // 注意：每帧只读一次；运行中在 Watch 修改的值要到下一帧才生效。
    const uint32_t requested_branch_id = g_robot_debug.branch.requested_id;
    if (!branch_demo_.request(requested_branch_id))
    {
        ++g_robot_debug.branch.rejected_frames; // 请求被拒绝（如取值越界）时计数。
    }
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    const uint32_t requested_plan_id = g_robot_debug.plan.requested_id; // 每帧只采样一次计划请求。
    if (!graph_.request_plan(requested_plan_id))
    {
        ++g_robot_debug.plan.rejected_frames; // 计划切换请求被拒绝时计数。
    }
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    snapshot_demo_.sync_publish_enabled(g_robot_debug.snapshot.publish_enabled != 0U); // 把开关同步到演示节点（只在帧边界同步）。
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    // 把 Watch 中的比较控制项打包成一个控制块，交给演示节点应用。
    // 注意：每个字段只读一次；若在调试器里同时改多个字段，请先暂停再改，避免读到半新半旧的值。
    robot::diagnostics::comparison_demo_controls comparison_controls;
    comparison_controls.input_value = g_robot_debug.comparison.input_value;
    comparison_controls.candidate_bias = g_robot_debug.comparison.candidate_bias;
    comparison_controls.is_pause_candidate = g_robot_debug.comparison.is_pause_candidate != 0U;
    comparison_controls.reset_all_request = g_robot_debug.comparison.reset_all_request;
    comparison_controls.reset_candidate_request = g_robot_debug.comparison.reset_candidate_request;
    comparison_demo_.apply_controls(comparison_controls);
#endif
    // —— 推进本帧上下文并计时 ——
    frame_.frame_id++; // 帧号自增，供各节点判断数据是否属于本帧。
    frame_.timestamp_us = clock_.now_us(); // 记录本帧起始时间戳。
    profiler_.begin_frame(frame_.frame_id, frame_.timestamp_us); // 开始本帧耗时统计。
    // 驱动整张图处理本帧（内部按执行计划调度各节点）。
    const robot::framework::process_result result = graph_.process_frame(frame_, &profiler_); // result：本帧图的汇总处理结果。
    // —— 帧后同步：把各链路结果复制到调试快照 ——
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    update_chassis_debug();
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    update_branch_debug();
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    update_plan_debug();
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    update_snapshot_debug();
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    update_comparison_debug();
#endif
    frame_.timestamp_us = clock_.now_us(); // 记录本帧结束时间戳。
    profiler_.end_frame(frame_.timestamp_us); // 结束本帧耗时统计。
    g_robot_debug.last_result = static_cast<uint8_t>(result); // 把本帧汇总结果写入调试快照。
    return result;
}

#if ROBOT_ENABLE_CHASSIS_OBSERVER
// update_chassis_debug() —— 把底盘观察节点的本帧结果复制到 C 调试符号。
// 调用时机：每帧图执行完毕、在 process_frame() 内调用。
// 数据流：chassis_observer_ 的输出信号 → g_robot_debug.chassis。
void robot_application::update_chassis_debug()
{
    const auto& signal = chassis_observer_.signal(); // 观察节点输出信号（仅当前执行任务访问，无需加锁）。
    if (!signal.is_valid())
    {
        return; // 本帧无有效输出（如尚未收到下板帧），保持调试快照原值。
    }
    const auto& observation = signal.read(); // 取出本帧的观察结果。
    // 逐轮组复制转向角与驱动转速。
    for (uint8_t index = 0U; index < robot::chassis::k_wheel_count; ++index)
    { // index：调试数组的轮组下标。
        g_robot_debug.chassis.steering_deg[index] = observation.feedback.steering_deg[index];
        g_robot_debug.chassis.drive_rpm[index] = observation.feedback.drive_rpm[index];
    }
    g_robot_debug.chassis.receive_count = observation.feedback.receive_count; // 累计接收帧数。
    g_robot_debug.chassis.age_ms = observation.feedback.age_ms; // 距最近一帧的年龄（ms）。
    g_robot_debug.chassis.is_received = observation.feedback.is_received ? 1U : 0U; // 是否收到过反馈。
    g_robot_debug.chassis.is_online = observation.is_online ? 1U : 0U; // 下板是否在线。
    g_robot_debug.chassis.is_lower_feedback = observation.feedback.is_lower_feedback; // 下板原始反馈值。
    g_robot_debug.chassis.frame_id = observation.frame_id; // 该观察结果所属的帧号。
}
#endif

#if ROBOT_ENABLE_BRANCH_DEMO
// update_branch_debug() —— 把分支演示的选择器状态与节点执行统计复制给 Watch。
// 调用时机：本帧图执行结束后、在 process_frame() 内调用。
// 注意：本函数只写“统计/输出”字段，保留 requested_id 请求值供外部修改。
void robot_application::update_branch_debug()
{
    // copy_stats：从某个节点取执行记录，写入调试快照中对应的 destination。
    const auto copy_stats = [this](volatile robot::platform::branch_node_debug_snapshot& destination,
                                   const robot::framework::plugin_node& node) {
        const auto* stats = graph_.execution_stats(node); // 展开图中该节点的统计地址。
        if (stats != nullptr)
        {
            destination.execution_count = stats->execution_count;
            destination.branch_skip_count = stats->branch_skip_count;
            destination.blocked_count = stats->blocked_count;
            destination.frame_state = static_cast<uint8_t>(stats->frame_state);
            destination.frame_order = stats->frame_order;
            destination.enter_count = stats->enter_count;
            destination.exit_count = stats->exit_count;
            destination.plan_change_count = stats->plan_change_count;
            destination.is_active = stats->is_active ? 1U : 0U;
        }
    };
    const auto& selector = branch_demo_.selector(); // 分支选择器：本帧锁存的选择状态。
    const auto& merged = branch_demo_.merge().signal(); // 汇合节点的结果信号。
    const bool is_valid = merged.is_valid() && merged.frame_id() == frame_.frame_id; // 有效性判据：信号有效且属于本帧（拒绝旧帧输出）。
    g_robot_debug.branch.active_id = selector.active_branch_id(); // 当前生效的分支路由。
    g_robot_debug.branch.switch_count = selector.switch_count(); // 累计切换次数。
    g_robot_debug.branch.frame_id = frame_.frame_id; // 本帧帧号。
    g_robot_debug.branch.input_value = branch_demo_.source().signal().is_valid() // 源节点输入值；无有效输入时记 0。
        ? branch_demo_.source().signal().read() : 0.0f;
    g_robot_debug.branch.is_output_valid = is_valid ? 1U : 0U; // 汇合输出是否有效。
    g_robot_debug.branch.output_value = is_valid ? merged.read() : 0.0f; // 汇合输出值。
    g_robot_debug.branch.output_frame_id = is_valid ? merged.frame_id() : 0U; // 汇合输出所属帧号。
    copy_stats(g_robot_debug.branch.source, branch_demo_.source()); // 源节点统计。
    copy_stats(g_robot_debug.branch.merge, branch_demo_.merge()); // 汇合节点统计。
    // branch_index：0=A、1=B，逐个复制两条分支编号的统计与输出。
    for (uint16_t branch_index = 0U; branch_index < 2U; ++branch_index)
    {
        auto& branch_id = branch_demo_.branch(branch_index); // 本次查询的分支编号容器（含首/末两个节点）。
        const auto& signal = branch_id.second().signal(); // 分支编号末端（second）节点的信号。
        const bool is_branch_valid = signal.is_valid() && signal.frame_id() == frame_.frame_id; // 该分支编号输出是否为本帧有效结果。
        copy_stats(g_robot_debug.branch.branches[branch_index].first, branch_id.first()); // 分支编号首节点统计。
        copy_stats(g_robot_debug.branch.branches[branch_index].second, branch_id.second()); // 分支编号末节点统计。
        g_robot_debug.branch.branches[branch_index].first_runs_since_enter = branch_id.first().runs_since_enter(); // 首节点自进入以来的运行次数。
        g_robot_debug.branch.branches[branch_index].second_runs_since_enter = branch_id.second().runs_since_enter(); // 末节点自进入以来的运行次数。
        g_robot_debug.branch.branches[branch_index].is_valid = is_branch_valid ? 1U : 0U; // 分支编号输出是否有效。
        g_robot_debug.branch.branches[branch_index].value = is_branch_valid ? signal.read() : 0.0f; // 分支编号输出值。
    }
}
#endif

#if ROBOT_ENABLE_PLAN_DEMO
// update_plan_debug() —— 记录当前执行计划及本帧 A/B 探针的顺序。
// 调用时机：本帧图执行结束后、在 process_frame() 内调用。
// 注意：不覆盖用户在 Watch 里写入的 requested_id（请求字段只读不改）。
void robot_application::update_plan_debug()
{
    // copy_stats：读取某个节点的统计，复制给调试快照中的 destination。
    const auto copy_stats = [this](volatile robot::platform::branch_node_debug_snapshot& destination,
                                  const robot::framework::plugin_node& node) {
        const auto* stats = graph_.execution_stats(node); // 该节点的执行记录。
        if (stats != nullptr)
        {
            destination.execution_count = stats->execution_count;
            destination.branch_skip_count = stats->branch_skip_count;
            destination.blocked_count = stats->blocked_count;
            destination.frame_state = static_cast<uint8_t>(stats->frame_state);
            destination.frame_order = stats->frame_order;
            destination.enter_count = stats->enter_count;
            destination.exit_count = stats->exit_count;
            destination.plan_change_count = stats->plan_change_count;
            destination.is_active = stats->is_active ? 1U : 0U;
        }
    };
    g_robot_debug.plan.active_id = graph_.active_plan(); // 当前生效的计划编号。
    g_robot_debug.plan.plan_count = graph_.plan_count(); // 已登记的计划总数。
    g_robot_debug.plan.failed_id = graph_.failed_plan(); // 编译失败的探针计划。
    g_robot_debug.plan.switch_count = graph_.plan_switch_count(); // 累计计划切换次数。
    g_robot_debug.plan.frame_id = frame_.frame_id; // 本帧帧号。
    copy_stats(g_robot_debug.plan.a, plan_demo_.a()); // A 探针统计。
    copy_stats(g_robot_debug.plan.b, plan_demo_.b()); // B 探针统计。
}
#endif

#if ROBOT_ENABLE_SNAPSHOT_DEMO
// update_snapshot_debug() —— 把异步快照邮箱的序号、年龄和状态复制到 Watch。
// 调用时机：本帧图执行结束后、在 process_frame() 内调用。
// 说明：这些字段反映“发布线程”写入的最新快照；read_count 反映读取侧累计读取次数。
void robot_application::update_snapshot_debug()
{
    g_robot_debug.snapshot.frame_id = frame_.frame_id; // 本帧帧号。
    g_robot_debug.snapshot.sequence = snapshot_demo_.sequence(); // 快照序号（每发布一次递增）。
    g_robot_debug.snapshot.read_count = snapshot_demo_.read_count(); // 累计读取次数。
    g_robot_debug.snapshot.age_us = snapshot_demo_.age_us(); // 快照年龄（距发布已过去的微秒数）。
    g_robot_debug.snapshot.status = static_cast<uint8_t>(snapshot_demo_.status()); // 邮箱状态（空闲/已发布/已读取等）。
    g_robot_debug.snapshot.publish_enabled = snapshot_demo_.is_publish_enabled() ? 1U : 0U; // 回读当前的发布开关。
}
#endif

#if ROBOT_ENABLE_COMPARISON_DEMO
// update_comparison_debug() —— 把比较演示的观察结果复制到 Watch。
// 调用时机：本帧图执行结束后、在 process_frame() 内调用。
// 注意：只做单向复制，绝不根据任何一路结果反向修改计算状态。
void robot_application::update_comparison_debug()
{
    const auto& observation = comparison_demo_.observation(); // 本帧的比较观察结果（快照）。
    auto& debug = g_robot_debug.comparison; // 目标调试快照引用。
    // copy_sample：把一个比较采样点复制到调试快照的对应位置。
    const auto copy_sample = [](volatile robot::platform::comparison_sample_debug_snapshot& destination,
                                const robot::diagnostics::demo_comparison_sample& sample) {
        destination.is_present = sample.is_present ? 1U : 0U; // 该采样点是否存在。
        destination.input_sequence = sample.key.input_sequence; // 输入序号（用于对齐两路采样）。
        destination.sampled_at_us = sample.key.sampled_at_us; // 采样时间戳（微秒）。
        destination.state_epoch = sample.key.state_epoch; // 状态版本号。
        destination.step = sample.key.step; // 采样步序。
        for (uint8_t i = 0U; i < 2U; ++i)
        {
            destination.value[i] = sample.value[i];
        } // 复制两维采样值。
    };
    copy_sample(debug.reference, observation.reference); // 基准路采样。
    copy_sample(debug.candidate, observation.candidate); // 候选路采样。
    debug.frame_id = observation.frame_id; // 本帧帧号。
    debug.applied_input_value = comparison_demo_.applied_input_value(); // 实际生效的输入值。
    debug.applied_candidate_bias = comparison_demo_.applied_candidate_bias(); // 实际生效的候选偏置。
    debug.status = static_cast<uint8_t>(observation.result.status); // 比较结果状态。
    debug.is_difference_valid = (observation.result.status == robot::framework::comparison_status::equal // 仅当结果为“相等/不等”时差值才有效。
        || observation.result.status == robot::framework::comparison_status::different) ? 1U : 0U;
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        debug.difference[i] = observation.result.difference[i];
    } // 逐维差值。
    debug.max_abs_difference = observation.result.max_abs_difference; // 本帧最大绝对差值。
    debug.historical_max_abs_difference = observation.max_abs_difference; // 历史最大绝对差值。
    debug.equal_count = observation.equal_count; // 累计“相等”帧数。
    debug.different_count = observation.different_count; // 累计“不等”帧数。
    debug.unaligned_count = observation.unaligned_count; // 累计未对齐采样数。
    debug.invalid_count = observation.invalid_count; // 累计无效采样数。
    debug.rejected_frames = comparison_demo_.rejected_frames(); // 累计被拒绝的控制请求帧数。
}
#endif

} // namespace robot::application
