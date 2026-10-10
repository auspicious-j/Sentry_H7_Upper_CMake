#include "RobotApplication.hpp"

#include "Platform/DebugSnapshot.hpp"

namespace robot::application {

// 骨架阶段的无硬件节点，用来验证调度、计时和调试快照。
class RobotApplication::HeartbeatNode final : public robot::framework::PluginNode {
public:
    HeartbeatNode() : PluginNode(1U) {}

    // context：当前 1 kHz 帧上下文。
    robot::framework::ProcessResult process(robot::framework::FrameContext&) override
    {
        return robot::framework::ProcessResult::Ok;
    }
};

// 成员对象静态存在，不在运行期 new。
RobotApplication::RobotApplication()
    : clock_()
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    , snapshot_demo_(clock_)
#endif
    , profiler_(clock_)
{
    // heartbeat_node：静态心跳节点，不产生硬件输出。
    static HeartbeatNode heartbeat_node;
    heartbeat_ = &heartbeat_node;
}

// 启动阶段依次完成注册、展开、验证、拓扑排序、配置和启动。
robot::framework::PluginStatus RobotApplication::initialize()
{
    if (initialized_) {
        return robot::framework::PluginStatus::InvalidState;
    }

    // status：图组装和启动状态。
    // 先注册心跳节点，再注册只读电机反馈节点。
    robot::framework::PluginStatus status = robot::framework::PluginStatus::Ok;
#if ROBOT_ENABLE_PLAN_DEMO
    status = graph_.setPlanCount(2U);
#endif
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(*heartbeat_);
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(motor_feedback_);
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(motor_offline_);
    }
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(chassis_feedback_);
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(chassis_observer_);
    }
    if (status == robot::framework::PluginStatus::Ok) {
        chassis_observer_.input().bind(chassis_feedback_.signal());
        status = graph_.addDependency(chassis_feedback_, chassis_observer_);
    }
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(branch_demo_);
    }
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(plan_demo_);
    }
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(snapshot_demo_);
    }
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.add(comparison_demo_);
    }
#endif
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.compile();
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.configureAll();
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = graph_.startAll();
    }

    g_robot_debug.graph_node_count = graph_.nodeCount();
    g_robot_debug.graph_edge_count = graph_.edgeCount();
    g_robot_debug.init_error = static_cast<uint32_t>(status);
    g_robot_debug.plan.plan_count = graph_.planCount();
    g_robot_debug.plan.failed_plan = graph_.failedPlan();
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    g_robot_debug.snapshot.publish_enabled = 1U;
#endif
    g_robot_debug.initialized = (status == robot::framework::PluginStatus::Ok) ? 1U : 0U;
    g_robot_debug.running = (status == robot::framework::PluginStatus::Ok) ? 1U : 0U;

    initialized_ = (status == robot::framework::PluginStatus::Ok);
    return status;
}

// 每次被 FreeRTOS 控制任务唤醒时执行一帧插件图。
robot::framework::ProcessResult RobotApplication::processFrame()
{
    if (!initialized_) {
        return robot::framework::ProcessResult::Fault;
    }

#if ROBOT_ENABLE_BRANCH_DEMO
    // requested_route：每帧只读一次 Watch 输入，运行中修改在下帧生效。
    const uint32_t requested_route = g_robot_debug.branch.requested_route;
    if (!branch_demo_.request(requested_route)) {
        ++g_robot_debug.branch.rejected_frames;
    }
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    const uint32_t requested_plan = g_robot_debug.plan.requested_plan; // 每帧只采样一次请求。
    if (!graph_.requestPlan(requested_plan)) {
        ++g_robot_debug.plan.rejected_frames;
    }
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    snapshot_demo_.syncPublishEnabled(g_robot_debug.snapshot.publish_enabled != 0U); // 只在帧边界同步。
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    // 每个请求字段只读一次；跨字段同时修改时应先暂停调试器。
    robot::diagnostics::ComparisonDemoControls comparison_controls;
    comparison_controls.input_value = g_robot_debug.comparison.input_value;
    comparison_controls.candidate_bias = g_robot_debug.comparison.candidate_bias;
    comparison_controls.pause_candidate = g_robot_debug.comparison.pause_candidate != 0U;
    comparison_controls.reset_all_request = g_robot_debug.comparison.reset_all_request;
    comparison_controls.reset_candidate_request = g_robot_debug.comparison.reset_candidate_request;
    comparison_demo_.applyControls(comparison_controls);
#endif
    frame_.frame_id++;
    frame_.timestamp_us = clock_.nowUs();
    profiler_.beginFrame(frame_.frame_id, frame_.timestamp_us);
    // result：本帧插件图汇总结果。
    const robot::framework::ProcessResult result = graph_.processFrame(frame_, &profiler_);
#if ROBOT_ENABLE_CHASSIS_OBSERVER
    updateChassisDebug();
#endif
#if ROBOT_ENABLE_BRANCH_DEMO
    updateBranchDebug();
#endif
#if ROBOT_ENABLE_PLAN_DEMO
    updatePlanDebug();
#endif
#if ROBOT_ENABLE_SNAPSHOT_DEMO
    updateSnapshotDebug();
#endif
#if ROBOT_ENABLE_COMPARISON_DEMO
    updateComparisonDebug();
#endif
    frame_.timestamp_us = clock_.nowUs();
    profiler_.endFrame(frame_.timestamp_us);
    g_robot_debug.last_result = static_cast<uint8_t>(result);
    return result;
}

#if ROBOT_ENABLE_CHASSIS_OBSERVER
// 将底盘节点结果复制到 g_robot_debug.chassis。
void RobotApplication::updateChassisDebug()
{
    const auto& signal = chassis_observer_.signal(); // 节点输出，仅当前执行任务访问。
    if (!signal.valid()) {
        return;
    }
    const auto& observation = signal.read(); // 本帧观察结果。
    // index：调试数组轮组下标。
    for (uint8_t index = 0U; index < robot::chassis::kWheelCount; ++index) {
        g_robot_debug.chassis.steering_deg[index] = observation.feedback.steering_deg[index];
        g_robot_debug.chassis.drive_rpm[index] = observation.feedback.drive_rpm[index];
    }
    g_robot_debug.chassis.receive_count = observation.feedback.receive_count;
    g_robot_debug.chassis.age_ms = observation.feedback.age_ms;
    g_robot_debug.chassis.received = observation.feedback.received ? 1U : 0U;
    g_robot_debug.chassis.online = observation.online ? 1U : 0U;
    g_robot_debug.chassis.lower_feedback = observation.feedback.lower_feedback;
    g_robot_debug.chassis.frame_id = observation.frame_id;
}
#endif

#if ROBOT_ENABLE_BRANCH_DEMO
// 将节点执行统计复制给 Watch，保留 requested_route 供外部修改。
void RobotApplication::updateBranchDebug()
{
    // copy_stats：从 node 取执行记录，写入 destination。
    const auto copy_stats = [this](volatile robot::platform::BranchNodeDebugSnapshot& destination,
                                   const robot::framework::PluginNode& node) {
        const auto* stats = graph_.executionStats(node); // 展开图中的统计地址。
        if (stats != nullptr) {
            destination.execution_count = stats->execution_count;
            destination.branch_skip_count = stats->branch_skip_count;
            destination.blocked_count = stats->blocked_count;
            destination.frame_state = static_cast<uint8_t>(stats->frame_state);
            destination.frame_order = stats->frame_order;
            destination.enter_count = stats->enter_count;
            destination.exit_count = stats->exit_count;
            destination.plan_change_count = stats->plan_change_count;
            destination.active = stats->active ? 1U : 0U;
        }
    };
    const auto& selector = branch_demo_.selector(); // 本帧锁存选择状态。
    const auto& merged = branch_demo_.merge().signal(); // 汇合结果信号。
    const bool valid = merged.valid() && merged.frameId() == frame_.frame_id; // 拒绝旧帧输出。
    g_robot_debug.branch.active_route = selector.activeRoute();
    g_robot_debug.branch.generation = selector.generation();
    g_robot_debug.branch.switch_count = selector.switchCount();
    g_robot_debug.branch.frame_id = frame_.frame_id;
    g_robot_debug.branch.input_value = branch_demo_.source().signal().valid()
        ? branch_demo_.source().signal().read() : 0.0f;
    g_robot_debug.branch.output_valid = valid ? 1U : 0U;
    g_robot_debug.branch.output_value = valid ? merged.read() : 0.0f;
    g_robot_debug.branch.output_frame_id = valid ? merged.frameId() : 0U;
    copy_stats(g_robot_debug.branch.source, branch_demo_.source());
    copy_stats(g_robot_debug.branch.merge, branch_demo_.merge());
    // route_index：0=A、1=B。
    for (uint16_t route_index = 0U; route_index < 2U; ++route_index) {
        auto& route = branch_demo_.route(route_index); // 本次查询的路线容器。
        const auto& signal = route.second().signal(); // 路线末端信号。
        const bool route_valid = signal.valid() && signal.frameId() == frame_.frame_id; // 本帧有效性。
        copy_stats(g_robot_debug.branch.routes[route_index].first, route.first());
        copy_stats(g_robot_debug.branch.routes[route_index].second, route.second());
        g_robot_debug.branch.routes[route_index].first_runs_since_enter = route.first().runsSinceEnter();
        g_robot_debug.branch.routes[route_index].second_runs_since_enter = route.second().runsSinceEnter();
        g_robot_debug.branch.routes[route_index].valid = route_valid ? 1U : 0U;
        g_robot_debug.branch.routes[route_index].value = route_valid ? signal.read() : 0.0f;
    }
}
#endif

#if ROBOT_ENABLE_PLAN_DEMO
// 记录当前计划及本帧A/B顺序；不覆盖用户的 requested_plan。
void RobotApplication::updatePlanDebug()
{
    // copy_stats：读取 node 的统计，复制给 destination。
    const auto copy_stats = [this](volatile robot::platform::BranchNodeDebugSnapshot& destination,
                                  const robot::framework::PluginNode& node) {
        const auto* stats = graph_.executionStats(node); // 节点执行记录。
        if (stats != nullptr) {
            destination.execution_count = stats->execution_count;
            destination.branch_skip_count = stats->branch_skip_count;
            destination.blocked_count = stats->blocked_count;
            destination.frame_state = static_cast<uint8_t>(stats->frame_state);
            destination.frame_order = stats->frame_order;
            destination.enter_count = stats->enter_count;
            destination.exit_count = stats->exit_count;
            destination.plan_change_count = stats->plan_change_count;
            destination.active = stats->active ? 1U : 0U;
        }
    };
    g_robot_debug.plan.active_plan = graph_.activePlan();
    g_robot_debug.plan.plan_count = graph_.planCount();
    g_robot_debug.plan.failed_plan = graph_.failedPlan();
    g_robot_debug.plan.switch_count = graph_.planSwitchCount();
    g_robot_debug.plan.generation = graph_.planGeneration();
    g_robot_debug.plan.frame_id = frame_.frame_id;
    copy_stats(g_robot_debug.plan.a, plan_demo_.a());
    copy_stats(g_robot_debug.plan.b, plan_demo_.b());
}
#endif

#if ROBOT_ENABLE_SNAPSHOT_DEMO
// 将异步邮箱的序号、年龄和状态复制到 Watch。
void RobotApplication::updateSnapshotDebug()
{
    g_robot_debug.snapshot.frame_id = frame_.frame_id;
    g_robot_debug.snapshot.sequence = snapshot_demo_.sequence();
    g_robot_debug.snapshot.read_count = snapshot_demo_.readCount();
    g_robot_debug.snapshot.age_us = snapshot_demo_.ageUs();
    g_robot_debug.snapshot.status = static_cast<uint8_t>(snapshot_demo_.status());
    g_robot_debug.snapshot.publish_enabled = snapshot_demo_.publishEnabled() ? 1U : 0U;
}
#endif

#if ROBOT_ENABLE_COMPARISON_DEMO
// 只复制诊断结果，不从任何一路结果反向修改计算状态。
void RobotApplication::updateComparisonDebug()
{
    const auto& observation = comparison_demo_.observation();
    auto& debug = g_robot_debug.comparison;
    const auto copy_sample = [](volatile robot::platform::ComparisonSampleDebugSnapshot& destination,
                                const robot::diagnostics::DemoComparisonSample& sample) {
        destination.present = sample.present ? 1U : 0U;
        destination.input_sequence = sample.key.input_sequence;
        destination.sampled_at_us = sample.key.sampled_at_us;
        destination.state_epoch = sample.key.state_epoch;
        destination.step = sample.key.step;
        for (uint8_t i = 0U; i < 2U; ++i) { destination.value[i] = sample.value[i]; }
    };
    copy_sample(debug.reference, observation.reference);
    copy_sample(debug.candidate, observation.candidate);
    debug.frame_id = observation.frame_id;
    debug.applied_input_value = comparison_demo_.appliedInputValue();
    debug.applied_candidate_bias = comparison_demo_.appliedCandidateBias();
    debug.status = static_cast<uint8_t>(observation.result.status);
    debug.difference_valid = (observation.result.status == robot::framework::ComparisonStatus::Equal
        || observation.result.status == robot::framework::ComparisonStatus::Different) ? 1U : 0U;
    for (uint8_t i = 0U; i < 2U; ++i) { debug.difference[i] = observation.result.difference[i]; }
    debug.max_abs_difference = observation.result.max_abs_difference;
    debug.historical_max_abs_difference = observation.max_abs_difference;
    debug.equal_count = observation.equal_count;
    debug.different_count = observation.different_count;
    debug.unaligned_count = observation.unaligned_count;
    debug.invalid_count = observation.invalid_count;
    debug.rejected_frames = comparison_demo_.rejectedFrames();
}
#endif

} // namespace robot::application
