#include "Gimbal_State_Node.hpp"

#include "GimbalStateMailbox.hpp"

extern "C"
{
    #include "Chassis.h"
    #include "Gimbal.h"
    #include "degree.h"
}

namespace robot::Gimbal 
{

    robot::framework::process_result Gimbal_State_Node::process(robot::framework::frame_context& context)
    {
        // 保护
        if ((inputs_ == nullptr) || (targets_ == nullptr))
        {
            return robot::framework::process_result::fault;
        }
        if ((inputs_->frame_id() != context.frame_id) || !inputs_->is_valid() ||
            (targets_->frame_id() != context.frame_id) || !targets_->is_valid())
        {
            output_.invalidate();
            return robot::framework::process_result::no_new_data;
        }

        // 复制
        const gimbal_inputs& inputs = inputs_->read();
        const gimbal_targets& targets = targets_->read();

        

        const float top_angle_deg = inputs.imu.yaw_deg;     // IMU.yaw
        const float top_gyro_z = inputs.imu.gyro_z;         // gyro[2]
        const float pitch_angle_deg = inputs.imu.pitch_deg; // IMU.pitch

        top_total_deg_ += ModularDegreeTowards(top_angle_deg - last_top_angle_deg_, 0.0f);
        const float top_target_deg = ModularDegreeTowards(targets.top_yaw_target_deg, top_total_deg_);
        last_top_angle_deg_ = top_angle_deg;
        base_total_deg_ = top_total_deg_ - (static_cast<float>(inputs.feedback.top_yaw_motor_angle) - TOP_YAW_OFFSET) / 8192.0f * 360.0f;// 从小yaw换算大yaw角度

        float base_target_deg = targets.base_yaw_target_deg;
        if ((inputs.chassis_pattern == Chassis_AI) && (inputs.rc.left == 2U))
        {
            base_target_deg += 45.0f; // 加弹动作
        }

        // 发布
        gimbal_state state{};
        state.top_yaw_total_deg = top_total_deg_;
        state.base_yaw_total_deg = base_total_deg_;
        state.pitch_angle_deg = pitch_angle_deg;
        state.top_gyro_z = top_gyro_z;

        state.base_yaw_target_deg = base_target_deg;
        state.top_yaw_target_deg = top_target_deg;
        state.pitch_target_deg = targets.pitch_target_deg;

        state.fric_open = targets.fric_open;
        state.shooter_command = targets.shooter_command;
        state.id_to_trigger = targets.id_to_trigger;
        state.sampled_at_us = context.timestamp_us;

        // 测量量只通过邮箱交给 Vision 任务，不再写 C 侧 gimbal.*。
        gimbal_state_mailbox().publish(state, context.timestamp_us);

        output_.publish(context.frame_id, state);
        return robot::framework::process_result::ok;
    }

} // namespace robot::Gimbal
