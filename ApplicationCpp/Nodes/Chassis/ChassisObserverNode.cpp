#include "ChassisObserverNode.hpp"

namespace robot::chassis {

// 保存 id 和 timeout_ms，输入由组装层接线。
ChassisObserverNode::ChassisObserverNode(robot::framework::PluginId id, uint32_t timeout_ms)
    : PluginNode(id), timeout_ms_(timeout_ms)
{
    output_.bind(signal_);
}

// 暴露输入绑定口。
robot::framework::InputPort<ChassisFeedback>& ChassisObserverNode::input()
{
    return input_;
}

// 提供下游只读结果。
const robot::framework::FrameSignal<ChassisObservation>& ChassisObserverNode::signal() const
{
    return signal_;
}

// context：当前帧；无反馈与已超时均是观察结果，不触发旧控制故障。
robot::framework::ProcessResult ChassisObserverNode::process(robot::framework::FrameContext& context)
{
    ChassisObservation observation{}; // 本帧默认离线结果。
    observation.frame_id = context.frame_id;
    if (input_.valid() && input_.frameId() == context.frame_id) {
        observation.feedback = input_.read();
        observation.online = observation.feedback.received
            && observation.feedback.age_ms <= timeout_ms_;
    }
    // 每帧发布，避免前次在线结果在缺少上游时残留。
    return output_.publish(context.frame_id, observation)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

} // namespace robot::chassis
