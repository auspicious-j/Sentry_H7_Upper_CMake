#include "ComparisonDemoNode.hpp"
#include "Framework/OutputComparison.hpp"

namespace robot::diagnostics {
using robot::framework::PluginStatus;
using robot::framework::ProcessResult;
using robot::framework::ComparisonStatus;

// 同帧公共输入；不读取旧业务对象或传感器。
ProcessResult ComparisonInputNode::process(robot::framework::FrameContext& context)
{
    signal_.publish(context.frame_id, {value_, context.frame_id, context.timestamp_us});
    return ProcessResult::Ok;
}

// 启动前要求输入已接线。
PluginStatus ComparisonAccumulatorNode::configure()
{
    return input_.connected() ? PluginNode::configure() : PluginStatus::ConfigurationFault;
}

// 暂停不会补算或复制参考状态；恢复后时间线不一致会由比较器报告。
ProcessResult ComparisonAccumulatorNode::process(robot::framework::FrameContext& context)
{
    if (paused_) { return ProcessResult::OutputHeld; }
    if (!input_.validForFrame(context.frame_id)) {
        signal_.invalidate();
        return ProcessResult::NoNewData;
    }
    const bool advanced = state_.advance(input_.read()); // 同一输入只推进一次。
    signal_.publish(context.frame_id, state_.output(bias_));
    return advanced ? ProcessResult::Ok : ProcessResult::OutputHeld;
}

// 组装层绑定两个结果；端口只读且没有指向生产者状态的指针。
void ComparisonResultNode::bind(const robot::framework::FrameSignal<DemoComparisonSample>& reference,
                                const robot::framework::FrameSignal<DemoComparisonSample>& candidate)
{
    reference_.bind(reference);
    candidate_.bind(candidate);
}

// 缺少任一结果口时拒绝启动。
PluginStatus ComparisonResultNode::configure()
{
    return reference_.connected() && candidate_.connected()
        ? PluginNode::configure() : PluginStatus::ConfigurationFault;
}

// 比较身份使用原始样本键，不能用当前消费帧号掩盖候选暂停。
ProcessResult ComparisonResultNode::process(robot::framework::FrameContext& context)
{
    observation_.frame_id = context.frame_id;
    observation_.reference = reference_.valid() ? reference_.read() : DemoComparisonSample{};
    observation_.candidate = candidate_.valid() ? candidate_.read() : DemoComparisonSample{};
    observation_.result = robot::framework::compareOutputs(observation_.reference, observation_.candidate,
                                                          robot::framework::ComparisonTolerance<2U>{});
    switch (observation_.result.status) {
    case ComparisonStatus::Equal: ++observation_.equal_count; break;
    case ComparisonStatus::Different: ++observation_.different_count; break;
    case ComparisonStatus::Unaligned: ++observation_.unaligned_count; break;
    default: ++observation_.invalid_count; break;
    }
    if ((observation_.result.status == ComparisonStatus::Equal
         || observation_.result.status == ComparisonStatus::Different)
        && observation_.result.max_abs_difference > observation_.max_abs_difference) {
        observation_.max_abs_difference = observation_.result.max_abs_difference;
    }
    return ProcessResult::Ok; // 诊断差异不会传播为图或旧硬件故障。
}

// 调度跳过时清除当前结果，保留累计计数。
void ComparisonResultNode::onSkipped(robot::framework::FrameContext& context)
{
    observation_.frame_id = context.frame_id;
    observation_.reference = {};
    observation_.candidate = {};
    observation_.result = {};
}

// 声明四条公共边，使两个计划都包含该验证链路。
PluginStatus ComparisonDemoNode::compose()
{
    auto& graph = children(); // 演示内部图。
    PluginStatus status = graph.add(source_);
    if (status == PluginStatus::Ok) { status = graph.add(reference_); }
    if (status == PluginStatus::Ok) { status = graph.add(candidate_); }
    if (status == PluginStatus::Ok) { status = graph.add(comparison_); }
    if (status != PluginStatus::Ok) { return status; }
    reference_.bind(source_.signal());
    candidate_.bind(source_.signal());
    comparison_.bind(reference_.signal(), candidate_.signal());
    status = graph.addDependency(source_, reference_);
    if (status == PluginStatus::Ok) { status = graph.addDependency(source_, candidate_); }
    if (status == PluginStatus::Ok) { status = graph.addDependency(reference_, comparison_); }
    if (status == PluginStatus::Ok) { status = graph.addDependency(candidate_, comparison_); }
    return status;
}

// Watch 请求锁存在帧开始；非法算术参数保留上一次合法值。
void ComparisonDemoNode::applyControls(const ComparisonDemoControls& controls)
{
    const bool value_valid = controls.input_value >= -1000 && controls.input_value <= 1000;
    const bool bias_valid = controls.candidate_bias >= -1000 && controls.candidate_bias <= 1000;
    if (value_valid) { source_.setValue(controls.input_value); }
    if (bias_valid) { candidate_.setBias(controls.candidate_bias); }
    if (!value_valid || !bias_valid) { ++rejected_frames_; }
    candidate_.setPaused(controls.pause_candidate);
    const bool reset_all = controls.reset_all_request != last_reset_all_;
    const bool reset_candidate = controls.reset_candidate_request != last_reset_candidate_;
    last_reset_all_ = controls.reset_all_request;
    last_reset_candidate_ = controls.reset_candidate_request;
    if (reset_all || reset_candidate) {
        ++epoch_;
        if (epoch_ == 0U) { ++epoch_; }
        if (reset_all) { reference_.reset(epoch_); }
        candidate_.reset(epoch_);
    }
}

} // namespace robot::diagnostics
