#ifndef ROBOT_CHASSIS_OBSERVER_NODE_HPP
#define ROBOT_CHASSIS_OBSERVER_NODE_HPP

#include "Framework/PluginNode.hpp"
#include "Framework/Ports.hpp"
#include "ChassisData.hpp"

namespace robot::chassis {

// 底盘迁移第一步：消费新端口并判断链路在线，不控制轮子。
class ChassisObserverNode final : public robot::framework::PluginNode {
public:
    // id：唯一编号；timeout_ms：板间反馈超时阈值。
    ChassisObserverNode(robot::framework::PluginId id, uint32_t timeout_ms);
    // 返回输入端口，由组装层绑定反馈源。
    robot::framework::InputPort<ChassisFeedback>& input();
    // 返回只读观察结果。
    const robot::framework::FrameSignal<ChassisObservation>& signal() const;
    // context：当前帧；要求上游已提交同帧快照。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;

private:
    uint32_t timeout_ms_; // 观测超时阈值，毫秒。
    robot::framework::InputPort<ChassisFeedback> input_{}; // 下板反馈输入。
    robot::framework::FrameSignal<ChassisObservation> signal_{}; // 观察结果存储。
    robot::framework::OutputPort<ChassisObservation> output_{}; // 观察结果写端。
};

} // namespace robot::chassis
#endif // ROBOT_CHASSIS_OBSERVER_NODE_HPP
