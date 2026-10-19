#include "Shooter_Input_Node.hpp"

#include "InterruptSnapshotLock.hpp"

extern "C"
{
#include "Shooter.h"
}

namespace robot::Shooter {

namespace {
robot::platform::stm32::interrupt_snapshot_lock g_legacy_lock{};
} // namespace

robot::framework::process_result Shooter_Input_Node::process(robot::framework::frame_context& context)
{
    shooter_inputs inputs{};
    inputs.judge = robot::io::inputs::read_judge(context.timestamp_us);
    inputs.feedback = robot::io::inputs::read_actuator_feedback(context.timestamp_us);

    {
        const robot::framework::snapshot_lock_guard guard(g_legacy_lock);
        inputs.command_request = shooter.command;
        inputs.id_to_trigger_request = shooter.idToTrigger;
        inputs.fric_open = shooter.fricOpen;
    }

    inputs.sampled_at_us = context.timestamp_us;
    output_.publish(context.frame_id, inputs);
    return robot::framework::process_result::ok;
}

} // namespace robot::Shooter
