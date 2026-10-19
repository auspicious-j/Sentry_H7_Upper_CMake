#include "Gimbal_Input_Node.hpp"

#include "InterruptSnapshotLock.hpp"
#include "VisionLink.hpp"

extern "C"
{
#include "Chassis.h"
#include "Gimbal.h"
#include "Shooter.h"
}

namespace robot::Gimbal {

namespace {

// 迁移期仍需读取的 C 侧状态（底盘模式、云台限幅）由对应任务写入，
// 这里用短临界区做整份拷贝。视觉目标已改由 Vision_Target_Node 端口提供。
robot::platform::stm32::interrupt_snapshot_lock g_legacy_lock{};

} // namespace

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
    if ((vision_target_->frame_id() == context.frame_id) && vision_target_->is_valid())
    {
        target = vision_target_->read();
    }

    // 视觉 AI 指令（与瞄准目标无关，直接取原始帧快照）。
    robot::framework::timed_snapshot<robot::io::vision::vision_frame_snapshot> vision_copy{};
    robot::io::vision::vision_frame_mailbox().copy_to(vision_copy);

    gimbal_inputs inputs{};
    inputs.imu = robot::io::inputs::read_imu(context.timestamp_us);
    inputs.rc = robot::io::inputs::read_rc(context.timestamp_us);
    inputs.feedback = robot::io::inputs::read_actuator_feedback(context.timestamp_us);
    inputs.judge = robot::io::inputs::read_judge(context.timestamp_us);

    inputs.vision_valid = target.is_valid;
    inputs.vision_base_yaw_deg = target.base_yaw;
    inputs.vision_yaw_deg = target.yaw;
    inputs.vision_pitch_deg = target.pitch;
    inputs.vision_id_to_trigger = target.id_to_trigger;
    inputs.vision_shooter_command = target.shooter_command;

    inputs.ai_armor_mode = vision_copy.value.armor_mode;
    inputs.ai_look_yaw_deg = vision_copy.value.look_yaw;
    inputs.ai_look_pitch_deg = vision_copy.value.look_pitch;

    {
        const robot::framework::snapshot_lock_guard guard(g_legacy_lock);
        inputs.chassis_pattern = static_cast<std::uint8_t>(chassis.pattern);
        inputs.chassis_rotate_mode = static_cast<std::uint8_t>(chassis.rotate.mode);

        inputs.pitch_min_deg = gimbal.pitch.pitchMin;
        inputs.pitch_max_deg = gimbal.pitch.pitchMax;
    }

    inputs.sampled_at_us = context.timestamp_us;
    output_.publish(context.frame_id, inputs);
    return robot::framework::process_result::ok;
}

} // namespace robot::Gimbal
