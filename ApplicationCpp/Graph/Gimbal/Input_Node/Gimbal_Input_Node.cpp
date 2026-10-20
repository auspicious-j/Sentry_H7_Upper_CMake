#include "Gimbal_Input_Node.hpp"

#include "ChassisModeMailbox.hpp"
#include "VisionLink.hpp"

extern "C"
{
#include "Shooter.h" // IDLE
}

namespace robot::Gimbal 
{

    robot::framework::process_result Gimbal_Input_Node::process(robot::framework::frame_context& context)
    {
        if (vision_target_ == nullptr)
        {
            return robot::framework::process_result::fault;
        }

        // 视觉目标节点由依赖边保证先执行；但视觉链路可能尚未收到过数据。
        // 与旧实现一致：没有视觉数据时 vision_valid=false，云台仍按遥控器工作。
        robot::io::vision::vision_target_snapshot target{};
        target.id_to_trigger = -1;
        target.shooter_command = IDLE;

        // 未超时且可用
        if ((vision_target_->frame_id() == context.frame_id) && vision_target_->is_valid())
        {
            target = vision_target_->read();
        }

        // 视觉 AI 指令
        robot::framework::timed_snapshot<robot::io::vision::vision_frame_snapshot> vision_copy{};
        robot::io::vision::vision_frame_mailbox().copy_to(vision_copy);

        // 底盘模式：跨任务，从邮箱取最新值（可能是上一个 Chassis 周期算出的）。
        robot::framework::timed_snapshot<robot::Chassis::chassis_mode> chassis_mode_copy{};
        robot::Chassis::chassis_mode_mailbox().copy_to(chassis_mode_copy);

        // region 整理本帧输入并发布
        gimbal_inputs inputs{};
        inputs.imu = robot::io::inputs::read_imu(context.timestamp_us);
        inputs.rc = robot::io::inputs::read_rc(context.timestamp_us);
        inputs.feedback = robot::io::inputs::read_actuator_feedback(context.timestamp_us);

        inputs.vision_valid = target.is_valid;
        inputs.vision_base_yaw_deg = target.base_yaw;
        inputs.vision_yaw_deg = target.yaw;
        inputs.vision_pitch_deg = target.pitch;
        inputs.vision_id_to_trigger = target.id_to_trigger;
        inputs.vision_shooter_command = target.shooter_command;

        inputs.ai_armor_mode = vision_copy.value.armor_mode;
        inputs.ai_look_yaw_deg = vision_copy.value.look_yaw;
        inputs.ai_look_pitch_deg = vision_copy.value.look_pitch;

        inputs.chassis_pattern = chassis_mode_copy.value.pattern;
        inputs.chassis_rotate_mode = chassis_mode_copy.value.rotate_mode;

        inputs.pitch_min_deg = pitch_min_deg_;
        inputs.pitch_max_deg = pitch_max_deg_;

        inputs.sampled_at_us = context.timestamp_us;
        // endregion
        
        // 发布本帧输入
        output_.publish(context.frame_id, inputs);
        return robot::framework::process_result::ok;
    }

} // namespace robot::Gimbal
