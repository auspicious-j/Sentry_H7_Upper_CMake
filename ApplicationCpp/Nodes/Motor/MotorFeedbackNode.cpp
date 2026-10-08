#include "MotorFeedbackNode.hpp"

extern "C" {
#include "Gimbal.h"
#include "Shooter.h"
}

namespace robot::motor {

namespace {
constexpr uint16_t kFrictionMotor0Id = 0x201U; // 左摩擦轮 ID。
constexpr uint16_t kFrictionMotor1Id = 0x202U; // 右摩擦轮 ID。
constexpr uint16_t kTriggerMotorId = 0x203U; // 拨弹电机 ID。
constexpr uint16_t kTopYawMotorId = 0x205U; // 小 yaw 电机 ID。
constexpr uint16_t kPitchMotorId = 0x001U; // DM pitch 电机 ID。
}

MotorFeedbackNode::MotorFeedbackNode()
    : PluginNode(20U)
{
    output_.bind(signal_);
}

robot::framework::OutputPort<MotorFeedbackFrame>& MotorFeedbackNode::output()
{
    return output_;
}

const robot::framework::FrameSignal<MotorFeedbackFrame>& MotorFeedbackNode::signal() const
{
    return signal_;
}

MotorFeedback MotorFeedbackNode::convertDji(uint16_t id, const DJI_Motor_t& motor)
{
    MotorFeedback feedback{}; // 当前 DJI 电机的统一反馈。
    feedback.id = id;
    feedback.angle = motor.angle;
    feedback.speed = motor.speed;
    feedback.torque = motor.torque;
    feedback.temperature = motor.temp;
    feedback.online = true;
    return feedback;
}

MotorFeedback MotorFeedbackNode::convertDm(uint16_t id, const DM_motor_t& motor)
{
    MotorFeedback feedback{}; // 当前 DM 电机的统一反馈。
    feedback.id = id;
    feedback.angle = static_cast<int16_t>(motor.para.p_int);
    feedback.speed = static_cast<int16_t>(motor.para.v_int);
    feedback.torque = static_cast<int16_t>(motor.para.t_int);
    feedback.temperature = static_cast<int8_t>(motor.para.Tmos);
    feedback.online = true;
    return feedback;
}

robot::framework::ProcessResult MotorFeedbackNode::process(robot::framework::FrameContext& context)
{
    MotorFeedbackFrame frame{}; // 当前帧的统一反馈快照。
    frame.motor[0] = convertDji(kFrictionMotor0Id, shooter.fricMotor[0]);
    frame.motor[1] = convertDji(kFrictionMotor1Id, shooter.fricMotor[1]);
    frame.motor[2] = convertDji(kTriggerMotorId, shooter.triggerMotor);
    frame.motor[3] = convertDji(kTopYawMotorId, gimbal.top_yawMotor);
    frame.motor[4] = convertDm(kPitchMotorId, gimbal.pitchMotor);

    if (!output_.publish(context.frame_id, frame)) {
        return robot::framework::ProcessResult::Fault;
    }
    return robot::framework::ProcessResult::Ok;
}

} // namespace robot::motor
