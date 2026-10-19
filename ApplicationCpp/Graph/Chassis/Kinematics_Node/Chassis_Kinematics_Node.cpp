#include "Chassis_Kinematics_Node.hpp"

#include <cmath>

#include "ChassisOutputMailbox.hpp"

extern "C"
{
#include "Chassis.h" // WHEELSPEED_MAX
#include "Gimbal.h"  // INIT_YAW_ANGLE
#include "degree.h"  // ModularDegreeTowards
}

namespace robot::Chassis {

namespace {
constexpr float k_pi = 3.14159265358979f;

float clamp_to(float value, float min_value, float max_value)
{
    if (value <= min_value)
    {
        return min_value;
    }
    if (value >= max_value)
    {
        return max_value;
    }
    return value;
}
} // namespace

robot::framework::plugin_status Chassis_Kinematics_Node::configure()
{
    // 底盘尺寸（原 Chassis_Init）
    info_.wheelbase = 320;
    info_.wheeltrack = 320;
    info_.wheelRadius = 115;
    info_.offsetX = 0;
    info_.offsetY = 0;

    init_angle_ = INIT_YAW_ANGLE;

    // 舵电机零点标定（单位°，正常加减 30° 的倍数）
    turn_offset_[0] = -304.57f - 30.0f + 180.0f;
    turn_offset_[1] = -44.78f + 30.0f + 45.0f + 180.0f;
    turn_offset_[2] = -119.49f + 30.0f + 90.0f + 15.0f + 180.0f;
    turn_offset_[3] = -6.5f + 90.0f - 15.0f + 180.0f;

    Slope_Init(&x_slope_, 40, 0);
    Slope_Init(&y_slope_, 40, 0);
    Slope_Init(&spin_slope_, 0.1, 0);

    PID_Init(&rotate_pid_, 0.25f, 0, 6, 2, 15);
    PID_SetDeadzone(&rotate_pid_, 0.1f);

    return robot::framework::plugin_node::configure();
}

void Chassis_Kinematics_Node::spin_speed_update(ChassisMode mode)
{
    max_vw_ = 3.25f;
    if (max_vw_ <= 0.5f)
    {
        max_vw_ = 0.5f;
    }

    if (mode == ChassisMode_Spin)
    {
        max_vx_ = 2159.6f;
        if (max_vx_ <= 0)
            max_vx_ = 0;
        max_vy_ = max_vx_;
    }
    else
    {
        max_vx_ = 5500;
        max_vy_ = 5500;
    }
}

void Chassis_Kinematics_Node::update_slope(ChassisMode mode)
{
    Slope_NextVal(&x_slope_);
    Slope_NextVal(&y_slope_);
    Slope_NextVal(&spin_slope_);
    spin_speed_update(mode);

    const float rotate_ratio = (info_.wheelbase + info_.wheeltrack) / 4.0f;
    max_vx_ = WHEELSPEED_MAX / 60.0f / (268.f / 17.f) * 2.0f * k_pi * info_.wheelRadius;
    max_vy_ = max_vx_;
    max_vw_ = WHEELSPEED_MAX / rotate_ratio / 60.0f / (268.f / 17.f) * 2.0f * k_pi *
        info_.wheelRadius * 1.0f / 1.414f;
}

robot::framework::process_result Chassis_Kinematics_Node::process(robot::framework::frame_context& context)
{
    if ((inputs_ == nullptr) || (mode_ == nullptr))
    {
        return robot::framework::process_result::fault;
    }
    if ((inputs_->frame_id() != context.frame_id) || !inputs_->is_valid() ||
        (mode_->frame_id() != context.frame_id) || !mode_->is_valid())
    {
        output_.invalidate();
        return robot::framework::process_result::no_new_data;
    }

    const chassis_inputs& inputs = inputs_->read();
    const chassis_mode& mode = mode_->read();
    const bool is_ai = (mode.pattern == Chassis_AI);

    // 模式由 Mode_Node 决定、经端口传入。AI 分支会在 update_slope 之后覆盖它，
    // 因此这里先锁存"覆盖前"的值，再算出本帧最终模式（与旧实现顺序一致）。
    const ChassisMode mode_before = static_cast<ChassisMode>(mode.rotate_mode);
    ChassisMode final_mode = mode_before;

    // —— 以下计算逐行对应旧 Task_Chassis_Callback() ——
    init_angle_ = ModularDegreeTowards(INIT_YAW_ANGLE, 0.0f);
    now_angle_ = inputs.base_yaw_angle_deg;
    relative_angle_ = now_angle_ - init_angle_;

    if (is_ai)
    {
        Slope_SetTarget(&x_slope_, 1.5f * inputs.vx);
        Slope_SetTarget(&y_slope_, 1.5f * inputs.vy);
        const float gimbal_angle_sin = std::sin(-relative_angle_ * k_pi / 180.0f);
        const float gimbal_angle_cos = std::cos(-relative_angle_ * k_pi / 180.0f);
        vx_ = -(Slope_GetVal(&x_slope_) * gimbal_angle_cos +
                Slope_GetVal(&y_slope_) * gimbal_angle_sin);
        vy_ = (-Slope_GetVal(&x_slope_) * gimbal_angle_sin +
               Slope_GetVal(&y_slope_) * gimbal_angle_cos);
        update_slope(mode_before);

        // 旧实现：AI 模式下在 update_slope 之后按视觉 spin_mode 覆盖旋转模式。
        final_mode = (inputs.spin_mode == 0U) ? ChassisMode_Follow : ChassisMode_Spin;
    }
    else
    {
        Slope_SetTarget(&x_slope_, -(float)inputs.rc.ch3 * max_vx_ / 660.0f);
        Slope_SetTarget(&y_slope_, (float)inputs.rc.ch4 * max_vy_ / 660.0f);
        const float gimbal_angle_sin = std::sin(-relative_angle_ * k_pi / 180.0f);
        const float gimbal_angle_cos = std::cos(-relative_angle_ * k_pi / 180.0f);
        vx_ = -(Slope_GetVal(&x_slope_) * gimbal_angle_cos +
                Slope_GetVal(&y_slope_) * gimbal_angle_sin);
        vy_ = (-Slope_GetVal(&x_slope_) * gimbal_angle_sin +
               Slope_GetVal(&y_slope_) * gimbal_angle_cos);
        update_slope(mode_before);
    }

    const bool is_spin = (final_mode == ChassisMode_Spin);

    // 旋转控制。
    if (!is_spin) // 跟随模式
    {
        Slope_SetTarget(&spin_slope_, 0);
        relative_angle_ = ModularDegreeTowards(relative_angle_, 0.0f);

        if (!is_ai)
        {
            PID_SingleCalc(&rotate_pid_, 0, -relative_angle_);
            vw_ = rotate_pid_.output + spin_slope_.value;
            vw_ = clamp_to(vw_, -max_vw_, max_vw_);
        }
        else
        {
            if (inputs.align_mode == 1U)
            {
                const float target = -inputs.align_yaw_deg;
                const float feedback = ModularDegreeTowards(-relative_angle_, target);
                PID_SingleCalc(&rotate_pid_, target, feedback);
                vw_ = rotate_pid_.output + spin_slope_.value;
                vw_ = clamp_to(vw_, -max_vw_, max_vw_);
            }
            else
            {
                vw_ = inputs.vw;
                vw_ = clamp_to(vw_, -max_vw_, max_vw_);
            }
        }
    }
    else // 小陀螺模式
    {
        vw_ = spin_slope_.value;
        float ratio = 1.0f;

        if (!is_ai)
        {
            if (std::fabs(Slope_GetVal(&x_slope_)) / max_vx_ +
                    std::fabs(Slope_GetVal(&y_slope_)) / max_vy_ > 0.05f)
            {
                ratio = 0.6f;
            }
            else
            {
                ratio = 1.0f;
            }
        }
        else
        {
            ratio = 0.5f;
        }

        Slope_SetTarget(&spin_slope_, max_vw_ * ratio);
    }

    // 当前舵角（反馈来自 B2B 快照），只在本节点内使用。
    float wheel_now_angle[4];
    for (std::uint8_t index = 0U; index < 4U; ++index)
    {
        wheel_now_angle[index] =
            ModularDegreeTowards(inputs.wheel_turn_angle_deg[index] - turn_offset_[index], 0.0f);
    }

    // 四轮解算。
    float rotate_ratio[4];
    rotate_ratio[0] = 1.414f * (info_.wheelbase + info_.wheeltrack) / 4.0f -
        info_.offsetY * std::fabs(spin_slope_.value) / max_vw_ +
        info_.offsetX * std::fabs(spin_slope_.value) / max_vw_;
    rotate_ratio[1] = 1.414f * (info_.wheelbase + info_.wheeltrack) / 4.0f -
        info_.offsetY * std::fabs(spin_slope_.value) / max_vw_ -
        info_.offsetX * std::fabs(spin_slope_.value) / max_vw_;
    rotate_ratio[2] = 1.414f * (info_.wheelbase + info_.wheeltrack) / 4.0f +
        info_.offsetY * std::fabs(spin_slope_.value) / max_vw_ +
        info_.offsetX * std::fabs(spin_slope_.value) / max_vw_;
    rotate_ratio[3] = 1.414f * (info_.wheelbase + info_.wheeltrack) / 4.0f +
        info_.offsetY * std::fabs(spin_slope_.value) / max_vw_ -
        info_.offsetX * std::fabs(spin_slope_.value) / max_vw_;

    float wheel_vx[4];
    float wheel_vy[4];
    wheel_vx[0] = vx_ + vw_ * rotate_ratio[1] / 1.414f;
    wheel_vy[0] = vy_ + vw_ * rotate_ratio[1] / 1.414f;
    wheel_vx[1] = vx_ + vw_ * rotate_ratio[2] / 1.414f;
    wheel_vy[1] = vy_ - vw_ * rotate_ratio[2] / 1.414f;
    wheel_vx[2] = vx_ - vw_ * rotate_ratio[0] / 1.414f;
    wheel_vy[2] = vy_ + vw_ * rotate_ratio[0] / 1.414f;
    wheel_vx[3] = vx_ - vw_ * rotate_ratio[3] / 1.414f;
    wheel_vy[3] = vy_ - vw_ * rotate_ratio[3] / 1.414f;

    chassis_output output{};
    for (std::uint8_t index = 0U; index < 4U; ++index)
    {
        float target_angle;
        std::int32_t wheel_rpm = -std::hypot(wheel_vx[index], wheel_vy[index]) * 60 /
            (2 * k_pi * info_.wheelRadius) * (268.0f / 17.0f);

        if (wheel_rpm != 0)
        {
            target_angle = std::atan2(-wheel_vy[index], wheel_vx[index]) / k_pi * 180;
        }
        else
        {
            static const float default_angle[4] = {135.0f, 45.0f, 45.0f, 135.0f};
            target_angle = default_angle[index];
        }

        target_angle = ModularDegreeTowards(target_angle, wheel_now_angle[index]);

        if (target_angle - wheel_now_angle[index] >= 90)
        {
            target_angle -= 180.0f;
            wheel_rpm = -wheel_rpm;
        }
        else if (target_angle - wheel_now_angle[index] < -90)
        {
            target_angle += 180.0f;
            wheel_rpm = -wheel_rpm;
        }

        output.target_turn_angle_deg[index] = target_angle;
        output.target_drive_speed[index] = wheel_rpm;
        output.now_turn_angle_deg[index] = wheel_now_angle[index];
    }

    output.sampled_at_us = context.timestamp_us;

    // 跨任务交付：RS485 任务从邮箱取四舵轮目标值与当前舵角。
    chassis_output_mailbox().publish(output, context.timestamp_us);

    output_.publish(context.frame_id, output);
    return robot::framework::process_result::ok;
}

} // namespace robot::Chassis
