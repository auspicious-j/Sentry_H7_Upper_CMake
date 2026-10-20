#include "Gimbal_Control_Node.hpp"

#include <cmath>

#include "GimbalControlMailbox.hpp"

extern "C"
{
    #include "Chassis.h"
    #include "Gimbal.h"
}

namespace robot::Gimbal 
{

    namespace 
    {
        constexpr float k_pi = 3.14159265358979f;
    } // namespace

    robot::framework::plugin_status Gimbal_Control_Node::configure()
    {
        // PID 参数逐项照搬旧 Gimbal_InitPID()，数值一个不动。
        PID_Init(&top_yaw_pid_.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300
        DEPID_Init(&top_yaw_pid_.deOuter, 150, 0.8, 300, 100, 30000, 0.7);

        PID_Init(&base_yaw_pid_.inner, 5.8, 0.02, 3.5, 1000, 7000);
        DEPID_Init(&base_yaw_pid_.deOuter, 37, 0.03, 173, 700, 2000, 0.45);
        PID_Init(&base_yaw_pid_.outer, 300, 0.5, 5000, 1000, 15000);

        PID_Init(&pitch_pid_.inner, 6, 0, 0.4, 500, 7000);
        DEPID_Init(&pitch_pid_.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50
        PID_Init(&pitch_pid_.outer, -0.8, -0.001, -8, 1.3, 10);

        return robot::framework::plugin_node::configure();
    }

    robot::framework::process_result Gimbal_Control_Node::process(robot::framework::frame_context& context)
    {
        // 保护
        if ((inputs_ == nullptr) || (state_ == nullptr))
        {
            return robot::framework::process_result::fault;
        }
        if ((inputs_->frame_id() != context.frame_id) || !inputs_->is_valid() ||
            (state_->frame_id() != context.frame_id) || !state_->is_valid())
        {
            return robot::framework::process_result::no_new_data;
        }

        // 复制
        const gimbal_inputs& inputs = inputs_->read();
        const gimbal_state& state = state_->read();


        // 小陀螺模式放大积分，与旧实现一致。
        if (inputs.chassis_rotate_mode == ChassisMode_Spin)
        {
            base_yaw_pid_.outer.maxIntegral = 6000;
            base_yaw_pid_.outer.ki = 0.5f;
        }
        else
        {
            base_yaw_pid_.outer.maxIntegral = 1000;
            base_yaw_pid_.outer.ki = 0.1f;
        }

        // 小 yaw 串级 PID。
        DEPID_CascadeCalc(&top_yaw_pid_, state.top_yaw_target_deg, state.top_yaw_total_deg, state.top_gyro_z);

        // 大 yaw 外环（输出换算为 B2B 下发的速度指令）。
        PID_SingleCalc(&base_yaw_pid_.outer, state.base_yaw_target_deg, state.base_yaw_total_deg);
        base_yaw_pid_.outer.output = -base_yaw_pid_.outer.output / 1000.0f;

        // pitch 位置环 + 重力前馈。
        // 与旧实现保持一致：写 CascadePID.output，而不是 outer.output。
        PID_SingleCalc(&pitch_pid_.outer, state.pitch_target_deg, state.pitch_angle_deg);
        pitch_pid_.output = -pitch_pid_.output / 1000.0f -
            PITCH_MASS * MASS_G * PITCH_R * std::cos(state.pitch_angle_deg * k_pi / 180.0f);


        // 发布
        gimbal_control control{};
        control.top_yaw_current = top_yaw_pid_.output;
        control.pitch_mit_torque = pitch_pid_.output;
        control.pitch_outer_output = pitch_pid_.outer.output;
        control.base_yaw_speed_cmd = base_yaw_pid_.outer.output;
        control.sampled_at_us = context.timestamp_us;

        // 跨任务的唯一交付通道：Motor / RS485 任务从邮箱取最新值。
        gimbal_control_mailbox().publish(control, context.timestamp_us);

        return robot::framework::process_result::ok;
    }

} // namespace robot::Gimbal
