#include "MotorOfflineNode.hpp"

extern "C" {
#include "bsp_can.h"
}

namespace robot::motor {

MotorOfflineNode::MotorOfflineNode(uint32_t timeout_ms)
    : PluginNode(21U), timeout_ms_(timeout_ms)
{
    output_.bind(signal_);
}

robot::framework::OutputPort<MotorOnlineFrame>& MotorOfflineNode::output()
{
    return output_;
}

const robot::framework::FrameSignal<MotorOnlineFrame>& MotorOfflineNode::signal() const
{
    return signal_;
}

robot::framework::ProcessResult MotorOfflineNode::process(robot::framework::FrameContext& context)
{
    MotorOnlineFrame online{}; // 当前帧的在线状态。
    for (uint8_t index = 0U; index < kMotorChannelCount; ++index) {
        online.online[index] = BSP_MotorFeedbackIsOnline(index, timeout_ms_);
    }

    if (!output_.publish(context.frame_id, online)) {
        return robot::framework::ProcessResult::Fault;
    }
    return robot::framework::ProcessResult::Ok;
}

} // namespace robot::motor
