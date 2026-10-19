#include "Chassis_Mode_Node.hpp"

#include "ChassisModeMailbox.hpp"

extern "C"
{
#include "Chassis.h"
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

    // 模式归本节点所有：跨帧状态保存在成员里，不再写 C 侧 chassis.pattern / rotate.mode。
    // 左拨杆：下 → 小陀螺，上（3）→ 跟随。
    if ((rotate_mode_ != static_cast<std::uint8_t>(ChassisMode_Spin)) && (inputs.rc.left == 2U))
    {
        rotate_mode_ = static_cast<std::uint8_t>(ChassisMode_Spin);
    }
    else if ((rotate_mode_ == static_cast<std::uint8_t>(ChassisMode_Spin)) && (inputs.rc.left == 3U))
    {
        rotate_mode_ = static_cast<std::uint8_t>(ChassisMode_Follow);
    }

    // 右拨杆：3 → AI，1 → 手动；其它保持原值。
    switch (inputs.rc.right)
    {
    case 3:
        pattern_ = static_cast<std::uint8_t>(Chassis_AI);
        break;
    case 1:
        pattern_ = static_cast<std::uint8_t>(Chassis_Control);
        break;
    default:
        break;
    }

    // AI 模式下由视觉决定旋转模式（ModeControl 的中间结果）。
    // 注意：旧实现这里用的是“反”的映射（spin_mode==0 → Spin），它会被
    // Kinematics_Node 的 update_slope() 用来选择 maxVx；随后 Kinematics_Node
    // 会在 update_slope 之后再做一次最终覆盖（spin_mode==0 → Follow）。
    if (pattern_ == static_cast<std::uint8_t>(Chassis_AI))
    {
        rotate_mode_ = (inputs.spin_mode == 0U)
            ? static_cast<std::uint8_t>(ChassisMode_Spin)
            : static_cast<std::uint8_t>(ChassisMode_Follow);
    }

    chassis_mode mode{};
    mode.pattern = pattern_;
    mode.rotate_mode = rotate_mode_;
    mode.sampled_at_us = context.timestamp_us;

    output_.publish(context.frame_id, mode);

    // 跨任务交付：Gimbal 任务从邮箱取底盘模式。
    chassis_mode_mailbox().publish(mode, context.timestamp_us);

    return robot::framework::process_result::ok;
}

} // namespace robot::Chassis
