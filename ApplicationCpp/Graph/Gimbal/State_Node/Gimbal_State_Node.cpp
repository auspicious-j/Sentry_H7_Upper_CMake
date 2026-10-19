#include "Gimbal_State_Node.hpp"

extern "C"
{
#include "Chassis.h"
#include "Gimbal.h"
#include "degree.h"
}

namespace robot::Gimbal {

robot::framework::process_result Gimbal_State_Node::process(robot::framework::frame_context& context)
{
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

    const gimbal_inputs& inputs = inputs_->read();
    const gimbal_targets& targets = targets_->read();

    // —— 以下计算逐行对应旧 Gimbal_UpdataAngle() ——
    const float top_angle_deg = inputs.imu.yaw_deg;
    const float top_gyro_z = inputs.imu.gyro_z;
    const float base_angle_deg =
        top_angle_deg - (static_cast<float>(inputs.feedback.top_yaw_angle) - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
    const float pitch_angle_deg = inputs.imu.pitch_deg;
    const float pitch_gyro_x = -inputs.imu.gyro_x;

    top_total_deg_ += ModularDegreeTowards(top_angle_deg - last_top_angle_deg_, 0.0f);
    const float top_target_deg = ModularDegreeTowards(targets.top_yaw_target_deg, top_total_deg_);
    last_top_angle_deg_ = top_angle_deg;
    base_total_deg_ =
        top_total_deg_ - (static_cast<float>(inputs.feedback.top_yaw_angle) - TOP_YAW_OFFSET) / 8192.0f * 360.0f;

    float base_target_deg = targets.base_yaw_target_deg;
    if ((inputs.chassis_pattern == Chassis_AI) && (inputs.rc.left == 2U))
    {
        base_target_deg += 45.0f; // 加弹动作
    }

    // 写回 C 侧测量量：Vision_DataTransmit 还在读 top_yaw.totalAngle 等字段。
    gimbal.top_yaw.angle = top_angle_deg;
    gimbal.top_yaw.gyro = top_gyro_z;
    gimbal.top_yaw.totalAngle = top_total_deg_;
    gimbal.top_yaw.lastAngle = last_top_angle_deg_;
    gimbal.base_yaw.angle = base_angle_deg;
    gimbal.base_yaw.totalAngle = base_total_deg_;
    gimbal.pitch.angle = pitch_angle_deg;
    gimbal.pitch.gyro = pitch_gyro_x;

    gimbal_state state{};
    state.top_yaw_angle_deg = top_angle_deg;
    state.top_yaw_total_deg = top_total_deg_;
    state.base_yaw_total_deg = base_total_deg_;
    state.pitch_angle_deg = pitch_angle_deg;
    state.top_gyro_z = top_gyro_z;
    state.pitch_gyro_x = pitch_gyro_x;

    state.base_yaw_target_deg = base_target_deg;
    state.top_yaw_target_deg = top_target_deg;
    state.pitch_target_deg = targets.pitch_target_deg;

    state.fric_open = targets.fric_open;
    state.shooter_command = targets.shooter_command;
    state.id_to_trigger = targets.id_to_trigger;
    state.vision_control_active = targets.vision_control_active;
    state.sampled_at_us = context.timestamp_us;

    output_.publish(context.frame_id, state);
    return robot::framework::process_result::ok;
}

} // namespace robot::Gimbal
