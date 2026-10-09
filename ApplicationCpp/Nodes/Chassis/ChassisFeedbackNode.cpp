#include "ChassisFeedbackNode.hpp"

namespace robot::chassis {

// 绑定 source；id 不依赖插件嵌套层级。
ChassisFeedbackNode::ChassisFeedbackNode(robot::framework::PluginId id,
                                       IChassisFeedbackSource& source)
    : PluginNode(id), source_(source)
{
    output_.bind(signal_);
}

// 提供下游只读视图。
const robot::framework::FrameSignal<ChassisFeedback>& ChassisFeedbackNode::signal() const
{
    return signal_;
}

// context：当前帧；保持数据时仍保留真实接收计数和年龄。
robot::framework::ProcessResult ChassisFeedbackNode::process(robot::framework::FrameContext& context)
{
    const ChassisFeedback feedback = source_.read(); // 本帧稳定输入。
    return output_.publish(context.frame_id, feedback)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

} // namespace robot::chassis
