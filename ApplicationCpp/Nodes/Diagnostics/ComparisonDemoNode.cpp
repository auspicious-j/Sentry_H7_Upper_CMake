#include "ComparisonDemoNode.hpp"
#include "Framework/OutputComparison.hpp"

namespace robot::diagnostics {
using robot::framework::plugin_status;
using robot::framework::process_result;
using robot::framework::comparison_status;

/* ============================================================================
 * comparison_demo_node.cpp —— 比较演示节点实现（演示/验证用，非功能代码）
 * ----------------------------------------------------------------------------
 * 各节点职责：
 *   - comparison_input_node::process           ：每帧发布一次公共输入（唯一写者）。
 *   - comparison_accumulator_node              ：按暂停/输入有效性推进独立状态机并发布结果。
 *   - comparison_result_node                   ：只读两路样本并比较，累计对齐/不同/未对齐/无效计数。
 *   - comparison_demo_node::compose/apply_controls：组装子图并消费 Watch 控制请求。
 * 说明：整个演示只做整数确定性运算与结果比对，用于验证框架的调度与快照是否
 *       保持确定性/可复现性，不涉及任何传感器或控制输出。
 * ========================================================================== */

// 同帧公共输入；不读取旧业务对象或传感器。
process_result comparison_input_node::process(robot::framework::frame_context& context)
{
    signal_.publish(context.frame_id, {value_, context.frame_id, context.timestamp_us}); // 发布 {输入值, 本帧帧号(作为序列号), 本帧时间戳}。
    return process_result::ok; // 正常生产完成。
}

// 启动前要求输入已接线。
plugin_status comparison_accumulator_node::configure()
{
    return input_.is_connected() ? plugin_node::configure() : plugin_status::configuration_fault; // 已连接→走基类默认配置；未连接→配置错误。
}

// 暂停不会补算或复制参考状态；恢复后时间线不一致会由比较器报告。
process_result comparison_accumulator_node::process(robot::framework::frame_context& context)
{
    if (is_paused_)
    {
        return process_result::output_held;
    } // 暂停：保持上一次完整结果，不推进、不作废。
    if (!input_.is_valid_for_frame(context.frame_id))
    { // 本帧没有可用的新输入（端口无数据/未对齐）。
        signal_.invalidate(); // 作废输出，避免下游把陈旧样本当成新结果。
        return process_result::no_new_data; // 上报"无新数据"。
    }
    const bool is_advanced = state_.advance(input_.read()); // 消费输入并推进状态；重复输入返回 false。
    signal_.publish(context.frame_id, state_.output(bias_)); // 发布当前状态导出值（叠加 bias）。
    return is_advanced ? process_result::ok : process_result::output_held; // 真推进→ok；重复输入→维持旧结果(output_held)。
}

// 组装层绑定两个结果；端口只读且没有指向生产者状态的指针。
void comparison_result_node::bind(const robot::framework::frame_signal<demo_comparison_sample>& reference,
                                const robot::framework::frame_signal<demo_comparison_sample>& candidate)
{
    reference_.bind(reference); // 绑定参考结果信号。
    candidate_.bind(candidate); // 绑定候选结果信号。
}

// 缺少任一结果口时拒绝启动。
plugin_status comparison_result_node::configure()
{
    return reference_.is_connected() && candidate_.is_connected() // 两个结果口都必须已连接。
        ? plugin_node::configure() : plugin_status::configuration_fault; // 是→基类默认配置；否→配置错误。
}

// 比较身份使用原始样本键，不能用当前消费帧号掩盖候选暂停。
process_result comparison_result_node::process(robot::framework::frame_context& context)
{
    observation_.frame_id = context.frame_id; // 记录本帧帧号。
    observation_.reference = reference_.is_valid() ? reference_.read() : demo_comparison_sample{}; // 读参考；端口无效则用空样本（is_present=false）。
    observation_.candidate = candidate_.is_valid() ? candidate_.read() : demo_comparison_sample{}; // 读候选；端口无效则用空样本。
    observation_.result = robot::framework::compare_outputs(observation_.reference, observation_.candidate,
                                                          robot::framework::comparison_tolerance<2U>{}); // 标准比较：判定对齐/相等/不同/无效，并给出最大绝对差与比较键。
    switch (observation_.result.status)
    { // 按比较状态累计诊断计数。
    case comparison_status::equal: ++observation_.equal_count; break; // 时间线对齐且数值相等。
    case comparison_status::different: ++observation_.different_count; break; // 时间线对齐但数值不同（真正的分歧）。
    case comparison_status::unaligned: ++observation_.unaligned_count; break; // 时间线不一致（如候选暂停读到旧样本）。
    default: ++observation_.invalid_count; break; // 缺失或非有限结果。
    }
    if ((observation_.result.status == comparison_status::equal // 仅在两路已对齐（equal 或 different）时才更新最大差，
         || observation_.result.status == comparison_status::different)
        && observation_.result.max_abs_difference > observation_.max_abs_difference) {
        observation_.max_abs_difference = observation_.result.max_abs_difference; // 刷新历史最大绝对差。
    }
    return process_result::ok; // 诊断差异不会传播为图或旧硬件故障。
}

// 调度跳过时清除当前结果，保留累计计数。
void comparison_result_node::on_skipped(robot::framework::frame_context& context)
{
    observation_.frame_id = context.frame_id; // 记录本帧帧号。
    observation_.reference = {}; // 清空参考当帧结果（is_present=false），不保留陈旧值。
    observation_.candidate = {}; // 清空候选当帧结果。
    observation_.result = {}; // 清空当帧比较结果（累计计数不受影响）。
}

// 声明四条公共边，使两个计划都包含该验证链路。
plugin_status comparison_demo_node::compose()
{
    auto& graph = children(); // 演示内部子图。
    plugin_status status = graph.add(source_); // 先加入唯一输入生产者。
    if (status == plugin_status::ok)
    {
        status = graph.add(reference_);
    } // 加入参考累加器。
    if (status == plugin_status::ok)
    {
        status = graph.add(candidate_);
    } // 加入候选累加器。
    if (status == plugin_status::ok)
    {
        status = graph.add(comparison_);
    } // 加入比较器。
    if (status != plugin_status::ok)
    {
        return status;
    } // 任一添加失败即中止并返回。
    reference_.bind(source_.signal()); // 参考订阅公共输入。
    candidate_.bind(source_.signal()); // 候选订阅同一公共输入。
    comparison_.bind(reference_.signal(), candidate_.signal()); // 比较器订阅两路结果。
    status = graph.add_dependency(source_, reference_); // 依赖：输入 → 参考（先生产、后消费）。
    if (status == plugin_status::ok)
    {
        status = graph.add_dependency(source_, candidate_);
    } // 依赖：输入 → 候选。
    if (status == plugin_status::ok)
    {
        status = graph.add_dependency(reference_, comparison_);
    } // 依赖：参考 → 比较。
    if (status == plugin_status::ok)
    {
        status = graph.add_dependency(candidate_, comparison_);
    } // 依赖：候选 → 比较。
    return status; // 返回组装结果。
}

// Watch 请求锁存在帧开始；非法算术参数保留上一次合法值。
void comparison_demo_node::apply_controls(const comparison_demo_controls& controls)
{
    const bool is_value_valid = controls.input_value >= -1000 && controls.input_value <= 1000; // 输入值是否在允许范围内。
    const bool is_bias_valid = controls.candidate_bias >= -1000 && controls.candidate_bias <= 1000; // 候选偏差是否在允许范围内。
    if (is_value_valid)
    {
        source_.set_value(controls.input_value);
    } // 合法才更新输入；非法则保留旧值。
    if (is_bias_valid)
    {
        candidate_.set_bias(controls.candidate_bias);
    } // 合法才更新偏差；非法则保留旧值。
    if (!is_value_valid || !is_bias_valid)
    {
        ++rejected_frames_;
    } // 任一非法：仅累计拒绝计数，不改计算状态。
    candidate_.set_paused(controls.is_pause_candidate); // 暂停开关无条件生效。
    const bool is_reset_all = controls.reset_all_request != last_reset_all_; // 共同重置请求按"数值变化"检测（边沿触发）。
    const bool is_reset_candidate = controls.reset_candidate_request != last_reset_candidate_; // 单侧重置请求按"数值变化"检测。
    last_reset_all_ = controls.reset_all_request; // 记录本次请求值，供下一帧做边沿比较。
    last_reset_candidate_ = controls.reset_candidate_request; // 同上。
    if (is_reset_all || is_reset_candidate)
    { // 存在任一重置请求。
        ++epoch_; // 分配新的状态版本（使重置后身份区别于重置前）。
        if (epoch_ == 0U)
        {
            ++epoch_;
        } // epoch=0 保留为"未初始化"含义，回绕时跳过。
        if (is_reset_all)
        {
            reference_.reset(epoch_);
        } // 共同重置：参考也一起重置。
        candidate_.reset(epoch_); // 单侧或共同：候选总是被重置。
    }
}

} // namespace robot::diagnostics
