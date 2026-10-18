#include "Gimbal_Control_Node.hpp"

#include <cmath>

extern "C"
{
#include "Chassis.h"
#include "Gimbal.h"
#include "PID.h"
}

namespace robot::Gimbal {

namespace {
constexpr float k_pi = 3.14159265358979f;
} // namespace

robot::framework::process_result Gimbal_Control_Node::process(robot::framework::frame_context& context)
{
    if ((inputs_ == nullptr) || (state_ == nullptr))
    {
        return robot::framework::process_result::fault;
    }
    if ((inputs_->frame_id() != context.frame_id) || !inputs_->is_valid() ||
        (state_->frame_id() != context.frame_id) || !state_->is_valid())
    {
        output_.invalidate();
        return robot::framework::process_result::no_new_data;
    }

    const gimbal_inputs& inputs = inputs_->read();
    const gimbal_state& state = state_->read();

    // —— 以下计算逐行对应旧 Task_Gimbal_Callback 的 Control_Node 区域 ——
    if (inputs.chassis_rotate_mode == ChassisMode_Spin)
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 6000;
        gimbal.base_yaw.imuPID.outer.ki = 0.5f;
    }
    else
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 1000;
        gimbal.base_yaw.imuPID.outer.ki = 0.1f;
    }

    // 小 yaw 串级 PID。
    DEPID_CascadeCalc(&gimbal.top_yaw.imuPID, state.top_yaw_target_deg, state.top_yaw_total_deg, state.top_gyro_z);
    gimbal.top_yaw.imuPID.output = gimbal.top_yaw.imuPID.output;

    // 大 yaw 外环（输出单位换算为 B2B 下发的速度指令）。
    PID_SingleCalc(&gimbal.base_yaw.imuPID.outer, state.base_yaw_target_deg, state.base_yaw_total_deg);
    gimbal.base_yaw.imuPID.outer.output = -gimbal.base_yaw.imuPID.outer.output / 1000.0f;

    // pitch 位置环 + 重力前馈。
    // 注意：这里与旧实现保持一致（写 imuPID.output，而不是 outer.output）。
    PID_SingleCalc(&gimbal.pitch.imuPID.outer, state.pitch_target_deg, state.pitch_angle_deg);
    gimbal.pitch.imuPID.output = -gimbal.pitch.imuPID.output / 1000.0f -
        PITCH_MASS * MASS_G * PITCH_R * std::cos(state.pitch_angle_deg * k_pi / 180.0f);

    gimbal_control control{};
    control.top_yaw_current = gimbal.top_yaw.imuPID.output;
    control.pitch_mit_torque = gimbal.pitch.imuPID.output;
    control.base_yaw_speed_cmd = gimbal.base_yaw.imuPID.outer.output;
    control.sampled_at_us = context.timestamp_us;

    output_.publish(context.frame_id, control);
    return robot::framework::process_result::ok;
}

} // namespace robot::Gimbal
