#ifndef ROBOT_MOTOR_FEEDBACK_NODE_HPP
#define ROBOT_MOTOR_FEEDBACK_NODE_HPP

#include "../../Framework/PluginNode.hpp"
#include "../../Framework/Ports.hpp"

extern "C" {
#include "USER_Moto.h"
}
#include "MotorData.hpp"

namespace robot::motor {

// 将旧 C 电机对象转换成统一 MotorFeedbackFrame 的观察节点。
class MotorFeedbackNode final : public robot::framework::PluginNode {
public:
    MotorFeedbackNode();

    // 返回反馈输出端口，供后续底盘、云台节点连接。
    robot::framework::OutputPort<MotorFeedbackFrame>& output();

    // 返回内部信号存储，供组装器绑定输入端口。
    const robot::framework::FrameSignal<MotorFeedbackFrame>& signal() const;

    // 每帧读取旧电机对象并发布统一反馈。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;

private:
    // 读取一个 DJI 电机并写入统一反馈结构。
    static MotorFeedback convertDji(uint16_t id, const DJI_Motor_t& motor);

    // 读取一个 DM 电机并写入统一反馈结构。
    static MotorFeedback convertDm(uint16_t id, const DM_motor_t& motor);

    robot::framework::FrameSignal<MotorFeedbackFrame> signal_{}; // 反馈信号存储。
    robot::framework::OutputPort<MotorFeedbackFrame> output_{}; // 反馈输出端口。
};

} // namespace robot::motor

#endif /* ROBOT_MOTOR_FEEDBACK_NODE_HPP */
