#include "Chassis_Input_Node.hpp"

#include "InterruptSnapshotLock.hpp"

extern "C"
{
#include "Chassis.h"
#include "Vision.h"
}

namespace robot::Chassis {

namespace {
robot::platform::stm32::interrupt_snapshot_lock g_legacy_lock{};
} // namespace

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
        inputs.wheel_speed[index] = feedback.wheel_speed[index];
    }

    {
        const robot::framework::snapshot_lock_guard guard(g_legacy_lock);
        inputs.vx = vx;
        inputs.vy = vy;
        inputs.vw = vw;
        inputs.spin_mode = vision_receive.spin_mode;
        inputs.align_mode = vision_receive.align_mode;
        inputs.align_yaw_deg = vision_receive.align_yaw;
    }

    inputs.sampled_at_us = context.timestamp_us;
    output_.publish(context.frame_id, inputs);
    return robot::framework::process_result::ok;
}

} // namespace robot::Chassis
