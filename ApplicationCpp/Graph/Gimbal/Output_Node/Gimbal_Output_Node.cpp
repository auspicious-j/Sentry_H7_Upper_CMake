#include "Gimbal_Output_Node.hpp"

extern "C"
{
#include "Gimbal.h"
#include "Shooter.h"
}

namespace robot::Gimbal {

robot::framework::process_result Gimbal_Output_Node::process(robot::framework::frame_context& context)
{
    if (state_ == nullptr)
    {
        return robot::framework::process_result::fault;
    }
    if ((state_->frame_id() != context.frame_id) || !state_->is_valid())
    {
        return robot::framework::process_result::no_new_data;
    }

    const gimbal_state& state = state_->read();

    // C 接口提交：目标角与发射请求。
    gimbal.base_yaw.targetAngle = state.base_yaw_target_deg;
    gimbal.top_yaw.targetAngle = state.top_yaw_target_deg;
    gimbal.pitch.targetAngle = state.pitch_target_deg;

    shooter.fricOpen = state.fric_open;
    shooter.command = state.shooter_command;
    shooter.idToTrigger = state.id_to_trigger;

    return robot::framework::process_result::ok;
}

} // namespace robot::Gimbal
