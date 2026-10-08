#ifndef ROBOT_MOTOR_OFFLINE_NODE_HPP
#define ROBOT_MOTOR_OFFLINE_NODE_HPP

#include "../../Framework/PluginNode.hpp"
#include "../../Framework/Ports.hpp"
#include "MotorData.hpp"

namespace robot::motor {

// 根据 BSP 接收时间戳发布电机在线状态的节点。
class MotorOfflineNode final : public robot::framework::PluginNode {
public:
    explicit MotorOfflineNode(uint32_t timeout_ms = 100U);

    // 返回在线状态输出端口。
    robot::framework::OutputPort<MotorOnlineFrame>& output();

    // 返回在线状态信号存储。
    const robot::framework::FrameSignal<MotorOnlineFrame>& signal() const;

    // 每帧检查所有电机反馈是否超时。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;

private:
    uint32_t timeout_ms_{100U}; // 反馈超时时间。
    robot::framework::FrameSignal<MotorOnlineFrame> signal_{}; // 在线状态信号。
    robot::framework::OutputPort<MotorOnlineFrame> output_{}; // 在线状态输出端口。
};

} // namespace robot::motor

#endif /* ROBOT_MOTOR_OFFLINE_NODE_HPP */
