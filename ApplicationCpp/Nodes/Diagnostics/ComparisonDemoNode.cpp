#include "ComparisonDemoNode.hpp"
#include "Framework/OutputComparison.hpp"

namespace robot::diagnostics {
using robot::framework::PluginStatus;
using robot::framework::ProcessResult;
using robot::framework::ComparisonStatus;

/* ============================================================================
 * ComparisonDemoNode.cpp —— 比较演示节点实现（演示/验证用，非功能代码）
 * ----------------------------------------------------------------------------
 * 各节点职责：
 *   - ComparisonInputNode::process           ：每帧发布一次公共输入（唯一写者）。
 *   - ComparisonAccumulatorNode              ：按暂停/输入有效性推进独立状态机并发布结果。
 *   - ComparisonResultNode                   ：只读两路样本并比较，累计对齐/不同/未对齐/无效计数。
 *   - ComparisonDemoNode::compose/applyControls：组装子图并消费 Watch 控制请求。
 * 说明：整个演示只做整数确定性运算与结果比对，用于验证框架的调度与快照是否
 *       保持确定性/可复现性，不涉及任何传感器或控制输出。
 * ========================================================================== */

// 同帧公共输入；不读取旧业务对象或传感器。
ProcessResult ComparisonInputNode::process(robot::framework::FrameContext& context)
{
    signal_.publish(context.frame_id, {value_, context.frame_id, context.timestamp_us}); // 发布 {输入值, 本帧帧号(作为序列号), 本帧时间戳}。
    return ProcessResult::Ok; // 正常生产完成。
}

// 启动前要求输入已接线。
PluginStatus ComparisonAccumulatorNode::configure()
{
    return input_.connected() ? PluginNode::configure() : PluginStatus::ConfigurationFault; // 已连接→走基类默认配置；未连接→配置错误。
}

// 暂停不会补算或复制参考状态；恢复后时间线不一致会由比较器报告。
ProcessResult ComparisonAccumulatorNode::process(robot::framework::FrameContext& context)
{
    if (paused_) { return ProcessResult::OutputHeld; } // 暂停：保持上一次完整结果，不推进、不作废。
    if (!input_.validForFrame(context.frame_id)) { // 本帧没有可用的新输入（端口无数据/未对齐）。
        signal_.invalidate(); // 作废输出，避免下游把陈旧样本当成新结果。
        return ProcessResult::NoNewData; // 上报"无新数据"。
    }
    const bool advanced = state_.advance(input_.read()); // 消费输入并推进状态；重复输入返回 false。
    signal_.publish(context.frame_id, state_.output(bias_)); // 发布当前状态导出值（叠加 bias）。
    return advanced ? ProcessResult::Ok : ProcessResult::OutputHeld; // 真推进→Ok；重复输入→维持旧结果(OutputHeld)。
}

// 组装层绑定两个结果；端口只读且没有指向生产者状态的指针。
void ComparisonResultNode::bind(const robot::framework::FrameSignal<DemoComparisonSample>& reference,
                                const robot::framework::FrameSignal<DemoComparisonSample>& candidate)
{
    reference_.bind(reference); // 绑定参考结果信号。
    candidate_.bind(candidate); // 绑定候选结果信号。
}

// 缺少任一结果口时拒绝启动。
PluginStatus ComparisonResultNode::configure()
{
    return reference_.connected() && candidate_.connected() // 两个结果口都必须已连接。
        ? PluginNode::configure() : PluginStatus::ConfigurationFault; // 是→基类默认配置；否→配置错误。
}

// 比较身份使用原始样本键，不能用当前消费帧号掩盖候选暂停。
ProcessResult ComparisonResultNode::process(robot::framework::FrameContext& context)
{
    observation_.frame_id = context.frame_id; // 记录本帧帧号。
    observation_.reference = reference_.valid() ? reference_.read() : DemoComparisonSample{}; // 读参考；端口无效则用空样本（present=false）。
    observation_.candidate = candidate_.valid() ? candidate_.read() : DemoComparisonSample{}; // 读候选；端口无效则用空样本。
    observation_.result = robot::framework::compareOutputs(observation_.reference, observation_.candidate,
                                                          robot::framework::ComparisonTolerance<2U>{}); // 标准比较：判定对齐/相等/不同/无效，并给出最大绝对差与比较键。
    switch (observation_.result.status) { // 按比较状态累计诊断计数。
    case ComparisonStatus::Equal: ++observation_.equal_count; break; // 时间线对齐且数值相等。
    case ComparisonStatus::Different: ++observation_.different_count; break; // 时间线对齐但数值不同（真正的分歧）。
    case ComparisonStatus::Unaligned: ++observation_.unaligned_count; break; // 时间线不一致（如候选暂停读到旧样本）。
    default: ++observation_.invalid_count; break; // 缺失或非有限结果。
    }
    if ((observation_.result.status == ComparisonStatus::Equal // 仅在两路已对齐（Equal 或 Different）时才更新最大差，
         || observation_.result.status == ComparisonStatus::Different)
        && observation_.result.max_abs_difference > observation_.max_abs_difference) {
        observation_.max_abs_difference = observation_.result.max_abs_difference; // 刷新历史最大绝对差。
    }
    return ProcessResult::Ok; // 诊断差异不会传播为图或旧硬件故障。
}

// 调度跳过时清除当前结果，保留累计计数。
void ComparisonResultNode::onSkipped(robot::framework::FrameContext& context)
{
    observation_.frame_id = context.frame_id; // 记录本帧帧号。
    observation_.reference = {}; // 清空参考当帧结果（present=false），不保留陈旧值。
    observation_.candidate = {}; // 清空候选当帧结果。
    observation_.result = {}; // 清空当帧比较结果（累计计数不受影响）。
}

// 声明四条公共边，使两个计划都包含该验证链路。
PluginStatus ComparisonDemoNode::compose()
{
    auto& graph = children(); // 演示内部子图。
    PluginStatus status = graph.add(source_); // 先加入唯一输入生产者。
    if (status == PluginStatus::Ok) { status = graph.add(reference_); } // 加入参考累加器。
    if (status == PluginStatus::Ok) { status = graph.add(candidate_); } // 加入候选累加器。
    if (status == PluginStatus::Ok) { status = graph.add(comparison_); } // 加入比较器。
    if (status != PluginStatus::Ok) { return status; } // 任一添加失败即中止并返回。
    reference_.bind(source_.signal()); // 参考订阅公共输入。
    candidate_.bind(source_.signal()); // 候选订阅同一公共输入。
    comparison_.bind(reference_.signal(), candidate_.signal()); // 比较器订阅两路结果。
    status = graph.addDependency(source_, reference_); // 依赖：输入 → 参考（先生产、后消费）。
    if (status == PluginStatus::Ok) { status = graph.addDependency(source_, candidate_); } // 依赖：输入 → 候选。
    if (status == PluginStatus::Ok) { status = graph.addDependency(reference_, comparison_); } // 依赖：参考 → 比较。
    if (status == PluginStatus::Ok) { status = graph.addDependency(candidate_, comparison_); } // 依赖：候选 → 比较。
    return status; // 返回组装结果。
}

// Watch 请求锁存在帧开始；非法算术参数保留上一次合法值。
void ComparisonDemoNode::applyControls(const ComparisonDemoControls& controls)
{
    const bool value_valid = controls.input_value >= -1000 && controls.input_value <= 1000; // 输入值是否在允许范围内。
    const bool bias_valid = controls.candidate_bias >= -1000 && controls.candidate_bias <= 1000; // 候选偏差是否在允许范围内。
    if (value_valid) { source_.setValue(controls.input_value); } // 合法才更新输入；非法则保留旧值。
    if (bias_valid) { candidate_.setBias(controls.candidate_bias); } // 合法才更新偏差；非法则保留旧值。
    if (!value_valid || !bias_valid) { ++rejected_frames_; } // 任一非法：仅累计拒绝计数，不改计算状态。
    candidate_.setPaused(controls.pause_candidate); // 暂停开关无条件生效。
    const bool reset_all = controls.reset_all_request != last_reset_all_; // 共同重置请求按"数值变化"检测（边沿触发）。
    const bool reset_candidate = controls.reset_candidate_request != last_reset_candidate_; // 单侧重置请求按"数值变化"检测。
    last_reset_all_ = controls.reset_all_request; // 记录本次请求值，供下一帧做边沿比较。
    last_reset_candidate_ = controls.reset_candidate_request; // 同上。
    if (reset_all || reset_candidate) { // 存在任一重置请求。
        ++epoch_; // 分配新的状态版本（使重置后身份区别于重置前）。
        if (epoch_ == 0U) { ++epoch_; } // epoch=0 保留为"未初始化"含义，回绕时跳过。
        if (reset_all) { reference_.reset(epoch_); } // 共同重置：参考也一起重置。
        candidate_.reset(epoch_); // 单侧或共同：候选总是被重置。
    }
}

} // namespace robot::diagnostics
