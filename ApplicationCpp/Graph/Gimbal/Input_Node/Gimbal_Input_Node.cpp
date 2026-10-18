#include "Gimbal_Input_Node.hpp"

#include "InterruptSnapshotLock.hpp"

extern "C"
{
#include "Chassis.h"
#include "Gimbal.h"
#include "Judge.h"
#include "Vision.h"
}

namespace robot::Gimbal {

namespace {

// 旧链路写方的临界区：vision_target / USER_SentryCmd 由视觉中断写，
// chassis 由底盘任务写。这里用与 Io/Inputs 相同的方式做整份拷贝。
robot::platform::stm32::interrupt_snapshot_lock g_legacy_lock{};

} // namespace

robot::framework::process_result Gimbal_Input_Node::process(robot::framework::frame_context& context)
{
    gimbal_inputs inputs{};
    inputs.imu = robot::io::inputs::read_imu(context.timestamp_us);
    inputs.rc = robot::io::inputs::read_rc(context.timestamp_us);
    inputs.feedback = robot::io::inputs::read_actuator_feedback(context.timestamp_us);
    inputs.judge = robot::io::inputs::read_judge(context.timestamp_us);

    {
        const robot::framework::snapshot_lock_guard guard(g_legacy_lock);
        inputs.vision_valid = vision_target.vaild;
        inputs.vision_base_yaw_deg = vision_target.base_yaw;
        inputs.vision_yaw_deg = vision_target.yaw;
        inputs.vision_pitch_deg = vision_target.pitch;
        inputs.vision_id_to_trigger = vision_target.idToTrigger;
        inputs.vision_shooter_command = vision_target.shooterCommand;

        inputs.ai_armor_mode = USER_SentryCmd.armor_mode;
        inputs.ai_look_yaw_deg = USER_SentryCmd.look_yaw;
        inputs.ai_look_pitch_deg = USER_SentryCmd.look_pitch;

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
