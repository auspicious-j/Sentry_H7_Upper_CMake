#include "Shooter_Logic_Node.hpp"

extern "C"
{
#include "Beep.h"
#include "Shooter.h"
#include "USER_Moto.h"
}

namespace robot::Shooter {

robot::framework::process_result Shooter_Logic_Node::process(robot::framework::frame_context& context)
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

    const shooter_inputs& inputs = inputs_->read();
    const std::uint32_t now_ms = HAL_GetTick();

    // —— 以下逻辑逐行对应旧 Task_Shooter_Callback() ——
    // 1) 弹速自适应。
    if (inputs.judge.initial_speed > 10.0f)
    {
        shooter.bullet_speed = inputs.judge.initial_speed;
    }

    if (shooter.bullet_speed != shooter.last_bullet_speed)
    {
        if (shooter.bullet_speed > 22.5f)
        {
            ++over_speed_;
            if (over_speed_ >= 2U)
            {
                over_speed_ = 0U;
                shooter.fricSpd -= 60;
                down_speed_ = 0U;
            }
        }
        else if (shooter.bullet_speed < 21.5f)
        {
            ++down_speed_;
            if (down_speed_ >= 4U)
            {
                down_speed_ = 0U;
                shooter.fricSpd += 50;
            }
        }
    }
    shooter.last_bullet_speed = shooter.bullet_speed;

    // 2) 摩擦轮目标速度。
    if (inputs.fric_open)
    {
        shooter.fricMotor[0].targetSpeed = -shooter.fricSpd;
        shooter.fricMotor[1].targetSpeed = shooter.fricSpd;
    }
    else
    {
        shooter.fricMotor[0].targetSpeed = 0;
        shooter.fricMotor[1].targetSpeed = 0;
    }

    // 3) 堵转检测（累计角度与目标角度相差超过 10°）。
    const std::int32_t angle_error = inputs.feedback.trigger_total_angle - shooter.triggerMotor.targetAngle;
    if (((angle_error > MOTOR_M3508_DGR2CODE(10)) || (angle_error < -MOTOR_M3508_DGR2CODE(10))) &&
        (shooter.command != TRIGGER_REVERSE))
    {
        ++shooter.block.judgeCnt;
        if (shooter.block.judgeCnt > 100)
        {
            shooter.block.judgeCnt = 0;
            shooter.command = TRIGGER_REVERSE;
        }
    }
    else
    {
        shooter.block.judgeCnt = 0;
    }

    // 4) 热量限制（与旧实现一致：恒返回 true，只更新发射间隔）。
    const bool heat_ok = update_heat_limit(inputs.judge);

    // 5) 拨弹到位后记录符命中时刻。
    const bool target_reached =
        (shooter.triggerMotor.targetAngle - inputs.feedback.trigger_total_angle) < MOTOR_M3508_DGR2CODE(9);
    if (target_reached && (shooter.triggeringId >= 0))
    {
        shooter.hitAtTick[shooter.triggeringId] = HAL_GetTick() + shooter.triggeringHitTick;
        shooter.triggeringId = -1;
    }

    // 6) 拨弹状态机（原来的 osDelay 改为时间戳等待）。
    switch (shooter.command)
    {
    case TRIGGER:
        if (shooter.fricOpen && (!inputs.judge.is_valid || heat_ok))
        {
            if (target_reached && (now_ms >= trigger_block_until_ms_))
            {
                shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 1 / 9.0 * 1);
                shooter.triggeringId = shooter.idToTrigger;
                shooter.idToTrigger = -1;
                shooter.command = IDLE;
                trigger_block_until_ms_ = now_ms + t_ms_;
            }
        }
        else
        {
            shooter.idToTrigger = -1;
            shooter.command = IDLE;
        }
        break;

    case TRIGGER_CONTINUE:
        if (shooter.fricOpen && (!inputs.judge.is_valid || heat_ok))
        {
            if (target_reached && (now_ms >= trigger_block_until_ms_))
            {
                shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 1 / 9.0 * 2);
                trigger_block_until_ms_ = now_ms + t_ms_;
            }
        }
        else
        {
            shooter.command = IDLE;
        }
        break;

    case TRIGGER_REVERSE:
        if (!reverse_waiting_)
        {
            Note notes[3] = {
                {T_M1, D_Sixteenth},
                {T_M1, D_Sixteenth},
                {T_M1, D_Sixteenth},
            };
            Beep_PlayNotes(notes, 3);
            shooter.triggerMotor.targetAngle -= MOTOR_M3508_DGR2CODE(360 * 1.5 / 9.0 * 2);
            reverse_waiting_ = true;
            reverse_until_ms_ = now_ms + 500U;
        }
        else if (now_ms >= reverse_until_ms_)
        {
            shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360 * 0.5 / 9.0 * 2);
            shooter.command = IDLE;
            reverse_waiting_ = false;
        }
        break;

    default:
        break;
    }

    shooter_output output{};
    output.fric_target_speed[0] = shooter.fricMotor[0].targetSpeed;
    output.fric_target_speed[1] = shooter.fricMotor[1].targetSpeed;
    output.trigger_target_angle = shooter.triggerMotor.targetAngle;
    output.command = shooter.command;
    output.triggering_id = shooter.triggeringId;
    output.sampled_at_us = context.timestamp_us;
    output_.publish(context.frame_id, output);

    return robot::framework::process_result::ok;
}

bool Shooter_Logic_Node::update_heat_limit(const robot::io::inputs::judge_snapshot& judge)
{
    const std::int16_t remain_heat =
        static_cast<std::int16_t>(judge.barrel_heat_limit) - static_cast<std::int16_t>(judge.barrel_heat_now);
    const float cooling = static_cast<float>(judge.barrel_cooling_value);

    if (remain_heat < 20)
    {
        // 旧实现此分支只改 k，不更新 t；保持 t 不变。
    }
    else if (remain_heat <= 35)
    {
        t_ms_ = static_cast<std::uint32_t>(1.0f * 1000.0f * 10.0f / cooling * 1.01f);
    }
    else if (remain_heat <= 100)
    {
        t_ms_ = static_cast<std::uint32_t>(
            0.5f * 1000.0f * 10.0f * (-static_cast<float>(remain_heat) + 100.0f) / (100.0f - 30.0f) / cooling);
    }
    else
    {
        t_ms_ = 0U;
    }
    return true;
}

} // namespace robot::Shooter
