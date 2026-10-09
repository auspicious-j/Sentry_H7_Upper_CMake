#ifndef ROBOT_BRANCH_MERGE_NODE_HPP
#define ROBOT_BRANCH_MERGE_NODE_HPP

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "BranchSelector.hpp"

namespace robot::framework {

// T：共同输出类型；RouteCount：固定路线数。
template <typename T, uint16_t RouteCount>
class BranchMergeNode final : public PluginNode {
public:
    // id：节点编号；selector：与上游共用的选择器。
    BranchMergeNode(PluginId id, BranchSelector& selector) : PluginNode(id), selector_(selector)
    {
        output_.bind(signal_);
    }
    // route/source：组装期绑定路线末端输出。
    bool bindInput(uint16_t route, const FrameSignal<T>& source)
    {
        if (state() != PluginState::Created || route >= RouteCount) { return false; }
        inputs_[route].bind(source);
        return true;
    }
    // 检查路线数量及输入绑定。
    PluginStatus configure() override
    {
        if (!selector_.configured() || selector_.routeCount() != RouteCount) {
            return PluginStatus::ConfigurationFault;
        }
        // route：待检查输入下标。
        for (uint16_t route = 0U; route < RouteCount; ++route) {
            if (!inputs_[route].connected()) { return PluginStatus::ConfigurationFault; }
        }
        return PluginNode::configure();
    }
    // context：当前帧；只转发选中路线的本帧结果。
    ProcessResult process(FrameContext& context) override
    {
        signal_.invalidate();
        const uint16_t route = selector_.activeRoute(); // 本帧锁存路线。
        if (route >= RouteCount || !inputs_[route].validForFrame(context.frame_id)) {
            return ProcessResult::NoNewData;
        }
        return output_.publish(context.frame_id, inputs_[route].read())
            ? ProcessResult::Ok : ProcessResult::Fault;
    }
    // context：跳过帧；禁止残留上次有效输出。
    void onSkipped(FrameContext& context) override
    {
        (void)context;
        signal_.invalidate();
    }
    // 返回只读汇合信号。
    const FrameSignal<T>& signal() const { return signal_; }
private:
    static_assert(RouteCount > 0U, "At least one branch input is required");
    BranchSelector& selector_; // 外部选择器。
    InputPort<T> inputs_[RouteCount]{}; // 各路线输入。
    FrameSignal<T> signal_{}; // 结果存储。
    OutputPort<T> output_{}; // 唯一结果写端。
};

} // namespace robot::framework
#endif // ROBOT_BRANCH_MERGE_NODE_HPP
