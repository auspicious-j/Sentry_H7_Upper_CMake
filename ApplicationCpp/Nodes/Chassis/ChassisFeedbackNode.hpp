#ifndef ROBOT_CHASSIS_FEEDBACK_NODE_HPP
#define ROBOT_CHASSIS_FEEDBACK_NODE_HPP

#include "Framework/PluginNode.hpp"
#include "Framework/Ports.hpp"
#include "IChassisFeedbackSource.hpp"

namespace robot::chassis {

// 发布下板反馈；数据源由组装层注入。
class ChassisFeedbackNode final : public robot::framework::PluginNode {
public:
    // id：图内唯一编号；source：生命周期长于本节点的数据源。
    ChassisFeedbackNode(robot::framework::PluginId id, IChassisFeedbackSource& source);
    // 返回只读输出信号，供下游绑定。
    const robot::framework::FrameSignal<ChassisFeedback>& signal() const;
    // context：本次插件帧；每帧更新快照年龄。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;

private:
    IChassisFeedbackSource& source_; // 外部持有的反馈源。
    robot::framework::FrameSignal<ChassisFeedback> signal_{}; // 固定信号存储。
    robot::framework::OutputPort<ChassisFeedback> output_{}; // 唯一写入端。
};

} // namespace robot::chassis
#endif // ROBOT_CHASSIS_FEEDBACK_NODE_HPP
