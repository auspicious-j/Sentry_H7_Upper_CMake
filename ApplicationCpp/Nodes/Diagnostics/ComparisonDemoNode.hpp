#ifndef ROBOT_COMPARISON_DEMO_NODE_HPP
#define ROBOT_COMPARISON_DEMO_NODE_HPP

#include "ComparisonDemoState.hpp"
#include "Framework/PluginNode.hpp"
#include "Framework/Ports.hpp"

namespace robot::diagnostics {

using DemoComparisonSample = robot::framework::ComparisonSample<2U>;

// 应用每帧一次性采样的 Watch 请求，只用于算术演示。
struct ComparisonDemoControls {
    int32_t input_value{7}; // 公共整数输入，范围 -1000..1000。
    int32_t candidate_bias{0}; // 候选输出偏差，范围 -1000..1000。
    uint32_t reset_all_request{0U}; // 数值变化时请求共同重置。
    uint32_t reset_candidate_request{0U}; // 数值变化时请求单侧重置。
    bool pause_candidate{false}; // 暂停候选状态推进。
};

// 一个比较帧以及累计诊断；缺失/未对齐不视为数值不同。
struct ComparisonDemoObservation {
    DemoComparisonSample reference{}; // 参考实例的完整结果。
    DemoComparisonSample candidate{}; // 候选实例的完整结果。
    robot::framework::ComparisonResult<2U> result{}; // 当前帧比较结果。
    uint32_t frame_id{0U}; // 最近比较帧。
    uint32_t equal_count{0U}; // 对齐且相等的帧数。
    uint32_t different_count{0U}; // 对齐但不同的帧数。
    uint32_t unaligned_count{0U}; // 输入/状态时间线不一致的帧数。
    uint32_t invalid_count{0U}; // 缺失或非有限结果的帧数。
    double max_abs_difference{0.0}; // 历史已对齐结果的最大绝对差。
};

// 每帧只生产一次输入，供两个状态独立的节点只读。
class ComparisonInputNode final : public robot::framework::PluginNode {
public:
    ComparisonInputNode() : PluginNode(101U) {} // 固定演示 ID。
    void setValue(int32_t value) { value_ = value; } // 设置下一帧输入。
    int32_t value() const { return value_; } // 最近合法输入。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    void onSkipped(robot::framework::FrameContext&) override { signal_.invalidate(); } // 不残留有效输出。
    const robot::framework::FrameSignal<ComparisonDemoInput>& signal() const { return signal_; }
private:
    int32_t value_{7}; // 公共测试输入。
    robot::framework::FrameSignal<ComparisonDemoInput> signal_{}; // 唯一写者的固定存储。
};

// 相同类的两个实例持有不同的 IndependentAccumulator 对象。
class ComparisonAccumulatorNode final : public robot::framework::PluginNode {
public:
    explicit ComparisonAccumulatorNode(robot::framework::PluginId id) : PluginNode(id) {}
    void bind(const robot::framework::FrameSignal<ComparisonDemoInput>& source) { input_.bind(source); }
    void setPaused(bool paused) { paused_ = paused; } // 暂停时保留最后完整结果。
    void setBias(int32_t bias) { bias_ = bias; } // 只用于输出偏差注入。
    int32_t bias() const { return bias_; } // 最近合法偏差。
    void reset(uint32_t epoch) { state_.reset(epoch); signal_.invalidate(); } // 不触碰其他实例。
    robot::framework::PluginStatus configure() override;
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    void onSkipped(robot::framework::FrameContext&) override { signal_.invalidate(); }
    const robot::framework::FrameSignal<DemoComparisonSample>& signal() const { return signal_; }
private:
    IndependentAccumulator state_{}; // 本节点独占的状态，完全独立于另一节点。
    robot::framework::InputPort<ComparisonDemoInput> input_{}; // 公共只读输入。
    robot::framework::FrameSignal<DemoComparisonSample> signal_{}; // 本节点独占结果。
    int32_t bias_{0}; // 偏差不进入 state_。
    bool paused_{false}; // 是否保持旧结果。
};

// 只读两个计算结果，容许读到暂停一路的旧样本并明确标为未对齐。
class ComparisonResultNode final : public robot::framework::PluginNode {
public:
    ComparisonResultNode() : PluginNode(104U) {}
    void bind(const robot::framework::FrameSignal<DemoComparisonSample>& reference,
              const robot::framework::FrameSignal<DemoComparisonSample>& candidate);
    robot::framework::PluginStatus configure() override;
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    void onSkipped(robot::framework::FrameContext& context) override;
    const ComparisonDemoObservation& observation() const { return observation_; }
private:
    robot::framework::InputPort<DemoComparisonSample> reference_{}; // 参考只读口。
    robot::framework::InputPort<DemoComparisonSample> candidate_{}; // 候选只读口。
    ComparisonDemoObservation observation_{}; // 由比较节点独占更新。
};

// 容器只组装与接收请求，内部节点统一由根执行计划执行。
class ComparisonDemoNode final : public robot::framework::PluginNode {
public:
    ComparisonDemoNode() : PluginNode(100U) {}
    robot::framework::PluginStatus compose() override;
    robot::framework::ProcessResult process(robot::framework::FrameContext&) override
    { return robot::framework::ProcessResult::Ok; }
    void applyControls(const ComparisonDemoControls& controls); // 仅执行器任务在帧边界调用。
    const ComparisonDemoObservation& observation() const { return comparison_.observation(); }
    uint32_t rejectedFrames() const { return rejected_frames_; } // 非法参数请求帧数。
    int32_t appliedInputValue() const { return source_.value(); }
    int32_t appliedCandidateBias() const { return candidate_.bias(); }
private:
    ComparisonInputNode source_{}; // 唯一公共输入生产者。
    ComparisonAccumulatorNode reference_{102U}; // 参考实例。
    ComparisonAccumulatorNode candidate_{103U}; // 候选实例。
    ComparisonResultNode comparison_{}; // 无反馈作用的比较节点。
    uint32_t last_reset_all_{0U}; // 已消费共同重置请求。
    uint32_t last_reset_candidate_{0U}; // 已消费单侧重置请求。
    uint32_t epoch_{1U}; // 演示状态版本分配器，回绕时跳过 0。
    uint32_t rejected_frames_{0U}; // 非法参数诊断，不改变计算状态。
};

} // namespace robot::diagnostics
#endif // ROBOT_COMPARISON_DEMO_NODE_HPP
