#include "Chassis_Mode_Node.hpp"

extern "C"
{
#include "Chassis.h"
#include "Vision.h"
}

namespace robot::Chassis {

robot::framework::process_result Chassis_Mode_Node::process(robot::framework::frame_context& context)
{
    if (inputs_ == nullptr)
    {
        return robot::framework::process_result::fault;
    }
    if ((inputs_->frame_id() != context.frame_id) || !inputs_->is_valid())
    {
        output_.invalidate();
        return robot::framework::process_result::no_new_data;
    }

    const chassis_inputs& inputs = inputs_->read();

    // —— 逐行对应旧 Chassis_ModeControl() ——
    // 左拨杆：下 → 小陀螺，上（3）→ 跟随。
    if ((chassis.rotate.mode != ChassisMode_Spin) && (inputs.rc.left == 2U))
    {
        chassis.rotate.mode = ChassisMode_Spin;
    }
    else if ((chassis.rotate.mode == ChassisMode_Spin) && (inputs.rc.left == 3U))
    {
        chassis.rotate.mode = ChassisMode_Follow;
    }

    // 右拨杆：3 → AI，1 → 手动；其它保持原值。
    switch (inputs.rc.right)
    {
    case 3:
        chassis.pattern = Chassis_AI;
        break;
    case 1:
        chassis.pattern = Chassis_Control;
        break;
    default:
        break;
    }

    // AI 模式下由视觉决定旋转模式（ModeControl 的中间结果）。
    // 注意：旧实现这里用的是“反”的映射（spin_mode==0 → Spin），它会被
    // Chassis_UpdateSlope() 用来选择 maxVx；随后 Kinematics_Node 会在
    // UpdateSlope 之后再做一次最终覆盖（spin_mode==0 → Follow）。
    if (chassis.pattern == Chassis_AI)
    {
        chassis.rotate.mode = (inputs.spin_mode == 0U) ? ChassisMode_Spin : ChassisMode_Follow;
    }

    chassis_mode mode{};
    mode.pattern = static_cast<std::uint8_t>(chassis.pattern);
    mode.rotate_mode = static_cast<std::uint8_t>(chassis.rotate.mode);
    mode.sampled_at_us = context.timestamp_us;

    output_.publish(context.frame_id, mode);
    return robot::framework::process_result::ok;
}

} // namespace robot::Chassis
