#include "Gimbal_Output_Node.hpp"

extern "C"
{
    #include "Shooter.h"
}

namespace robot::Gimbal 
{

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

        // 只提交「跨任务」的发射请求。
        // 目标角不再写回 gimbal.*.targetAngle：全工程没有任何读者——控制节点直接
        // 从 gimbal_state 端口取目标值，不需要再经过 C 全局量转一手。
        shooter.fric_open = state.fric_open;
        shooter.shooter_state = static_cast<Shooter_State_t>(state.shooter_command);
        shooter.id_to_trigger = state.id_to_trigger;

        return robot::framework::process_result::ok;
    }

} // namespace robot::Gimbal
