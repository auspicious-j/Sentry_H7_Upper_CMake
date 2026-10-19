#include "Chassis_Input_Node.hpp"

#include "VisionLink.hpp"

namespace robot::Chassis {

robot::framework::process_result Chassis_Input_Node::process(robot::framework::frame_context& context)
{
    chassis_inputs inputs{};
    inputs.rc = robot::io::inputs::read_rc(context.timestamp_us);

    const robot::io::inputs::actuator_feedback feedback =
        robot::io::inputs::read_actuator_feedback(context.timestamp_us);
    inputs.base_yaw_angle_deg = feedback.base_yaw_angle_deg;
    for (std::uint8_t index = 0U; index < 4U; ++index)
    {
        inputs.wheel_turn_angle_deg[index] = feedback.wheel_turn_angle_deg[index];
    }

    // 视觉 AI 指令：直接取原始帧快照（无新帧时保留上一次的值，与旧全局语义一致）。
    robot::framework::timed_snapshot<robot::io::vision::vision_frame_snapshot> vision_copy{};
    robot::io::vision::vision_frame_mailbox().copy_to(vision_copy);
    inputs.vx = -vision_copy.value.linear_y * 1000.0f;
    inputs.vy = -vision_copy.value.linear_x * 1000.0f;
    inputs.vw = vision_copy.value.angular_z;
    inputs.spin_mode = vision_copy.value.spin_mode;
    inputs.align_mode = vision_copy.value.align_mode;
    inputs.align_yaw_deg = vision_copy.value.align_yaw;

    inputs.sampled_at_us = context.timestamp_us;
    output_.publish(context.frame_id, inputs);
    return robot::framework::process_result::ok;
}

} // namespace robot::Chassis
