#include "Chassis_Kinematics_Node.hpp"

#include <cmath>

extern "C"
{
#include "Chassis.h"
#include "Gimbal.h"
#include "PID.h"
#include "Slope.h"
#include "degree.h"
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

    // —— 以下计算逐行对应旧 Task_Chassis_Callback() ——
    chassis.rotate.InitAngle = ModularDegreeTowards(INIT_YAW_ANGLE, 0.0f);
    chassis.rotate.nowAngle = inputs.base_yaw_angle_deg;
    chassis.rotate.relativeAngle = chassis.rotate.nowAngle - chassis.rotate.InitAngle;

    if (is_ai)
    {
        Slope_SetTarget(&chassis.move.xSlope, 1.5f * inputs.vx);
        Slope_SetTarget(&chassis.move.ySlope, 1.5f * inputs.vy);
        const float gimbal_angle_sin = std::sin(-chassis.rotate.relativeAngle * k_pi / 180.0f);
        const float gimbal_angle_cos = std::cos(-chassis.rotate.relativeAngle * k_pi / 180.0f);
        chassis.move.vx = -(Slope_GetVal(&chassis.move.xSlope) * gimbal_angle_cos +
                            Slope_GetVal(&chassis.move.ySlope) * gimbal_angle_sin);
        chassis.move.vy = (-Slope_GetVal(&chassis.move.xSlope) * gimbal_angle_sin +
                           Slope_GetVal(&chassis.move.ySlope) * gimbal_angle_cos);
        Chassis_UpdateSlope();

        // 旧实现：AI 模式下在 UpdateSlope 之后按视觉 spin_mode 覆盖旋转模式。
        chassis.rotate.mode = (inputs.spin_mode == 0U) ? ChassisMode_Follow : ChassisMode_Spin;
    }
    else
    {
        Slope_SetTarget(&chassis.move.xSlope, -(float)inputs.rc.ch3 * chassis.move.maxVx / 660.0f);
        Slope_SetTarget(&chassis.move.ySlope, (float)inputs.rc.ch4 * chassis.move.maxVy / 660.0f);
        const float gimbal_angle_sin = std::sin(-chassis.rotate.relativeAngle * k_pi / 180.0f);
        const float gimbal_angle_cos = std::cos(-chassis.rotate.relativeAngle * k_pi / 180.0f);
        chassis.move.vx = -(Slope_GetVal(&chassis.move.xSlope) * gimbal_angle_cos +
                            Slope_GetVal(&chassis.move.ySlope) * gimbal_angle_sin);
        chassis.move.vy = (-Slope_GetVal(&chassis.move.xSlope) * gimbal_angle_sin +
                           Slope_GetVal(&chassis.move.ySlope) * gimbal_angle_cos);
        Chassis_UpdateSlope();
    }

    const bool is_spin = (chassis.rotate.mode == ChassisMode_Spin);

    // 旋转控制。
    if (!is_spin) // 跟随模式
    {
        Slope_SetTarget(&chassis.move.spinSlope, 0);
        chassis.rotate.relativeAngle = ModularDegreeTowards(chassis.rotate.relativeAngle, 0.0f);

        if (!is_ai)
        {
            PID_SingleCalc(&chassis.rotate.pid, 0, -chassis.rotate.relativeAngle);
            chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
            chassis.move.vw = clamp_to(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
        }
        else
        {
            if (inputs.align_mode == 1U)
            {
                const float target = -inputs.align_yaw_deg;
                const float feedback = ModularDegreeTowards(-chassis.rotate.relativeAngle, target);
                PID_SingleCalc(&chassis.rotate.pid, target, feedback);
                chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
                chassis.move.vw = clamp_to(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
            }
            else
            {
                chassis.move.vw = inputs.vw;
                chassis.move.vw = clamp_to(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
            }
        }
    }
    else // 小陀螺模式
    {
        chassis.move.vw = chassis.move.spinSlope.value;
        float ratio = 1.0f;

        if (!is_ai)
        {
            if (std::fabs(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx +
                    std::fabs(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
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

        Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
    }

    // 当前舵角（反馈来自 B2B 快照）。
    for (std::uint8_t index = 0U; index < 4U; ++index)
    {
        chassis.motors[index].now_angle =
            ModularDegreeTowards(inputs.wheel_turn_angle_deg[index] - chassis.motors[index].TurnOffset, 0.0f);
    }

    // 四轮解算。
    float rotate_ratio[4];
    rotate_ratio[0] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f -
        chassis.info.offsetY * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw +
        chassis.info.offsetX * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    rotate_ratio[1] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f -
        chassis.info.offsetY * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw -
        chassis.info.offsetX * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    rotate_ratio[2] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f +
        chassis.info.offsetY * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw +
        chassis.info.offsetX * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    rotate_ratio[3] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f +
        chassis.info.offsetY * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw -
        chassis.info.offsetX * std::fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;

    float wheel_vx[4];
    float wheel_vy[4];
    wheel_vx[0] = chassis.move.vx + chassis.move.vw * rotate_ratio[1] / 1.414f;
    wheel_vy[0] = chassis.move.vy + chassis.move.vw * rotate_ratio[1] / 1.414f;
    wheel_vx[1] = chassis.move.vx + chassis.move.vw * rotate_ratio[2] / 1.414f;
    wheel_vy[1] = chassis.move.vy - chassis.move.vw * rotate_ratio[2] / 1.414f;
    wheel_vx[2] = chassis.move.vx - chassis.move.vw * rotate_ratio[0] / 1.414f;
    wheel_vy[2] = chassis.move.vy + chassis.move.vw * rotate_ratio[0] / 1.414f;
    wheel_vx[3] = chassis.move.vx - chassis.move.vw * rotate_ratio[3] / 1.414f;
    wheel_vy[3] = chassis.move.vy - chassis.move.vw * rotate_ratio[3] / 1.414f;

    chassis_output output{};
    for (std::uint8_t index = 0U; index < 4U; ++index)
    {
        float target_angle;
        std::int32_t wheel_rpm = -std::hypot(wheel_vx[index], wheel_vy[index]) * 60 / (2 * k_pi * chassis.info.wheelRadius) *
            (268.0f / 17.0f);

        if (wheel_rpm != 0)
        {
            target_angle = std::atan2(-wheel_vy[index], wheel_vx[index]) / k_pi * 180;
        }
        else
        {
            static const float default_angle[4] = {135.0f, 45.0f, 45.0f, 135.0f};
            target_angle = default_angle[index];
        }

        target_angle = ModularDegreeTowards(target_angle, chassis.motors[index].now_angle);

        if (target_angle - chassis.motors[index].now_angle >= 90)
        {
            target_angle -= 180.0f;
            wheel_rpm = -wheel_rpm;
        }
        else if (target_angle - chassis.motors[index].now_angle < -90)
        {
            target_angle += 180.0f;
            wheel_rpm = -wheel_rpm;
        }

        chassis.motors[index].targetTurnAngle = target_angle;
        chassis.motors[index].targetDriveSpeed = wheel_rpm;

        output.target_turn_angle_deg[index] = target_angle;
        output.target_drive_speed[index] = wheel_rpm;
    }

    output.vx = chassis.move.vx;
    output.vy = chassis.move.vy;
    output.vw = chassis.move.vw;
    output.sampled_at_us = context.timestamp_us;

    output_.publish(context.frame_id, output);
    return robot::framework::process_result::ok;
}

} // namespace robot::Chassis
