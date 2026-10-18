#include "Gimbal_Target_Node.hpp"

#include <cmath>

extern "C"
{
#include "Chassis.h"
#include "Shooter.h"
#include "Vision.h"
#include "degree.h"
#include "PID.h"
}

namespace robot::Gimbal {

namespace {

constexpr float k_pi = 3.14159265358979f;
constexpr float k_top_yaw_limit_deg = 45.0f;
constexpr float k_scan_yaw_speed_dps = 60.0f;
constexpr float k_scan_pitch_freq_hz = 2.0f;
constexpr float k_scan_pitch_min_deg = -15.0f;
constexpr float k_scan_pitch_max_deg = 5.0f;
constexpr float k_scan_pitch_amp_deg = (k_scan_pitch_max_deg - k_scan_pitch_min_deg) * 0.5f;

} // namespace

robot::framework::process_result Gimbal_Target_Node::process(robot::framework::frame_context& context)
{
    if ((inputs_ == nullptr) || (state_ == nullptr))
    {
        return robot::framework::process_result::fault;
    }
    if ((inputs_->frame_id() != context.frame_id) || !inputs_->is_valid())
    {
        output_.invalidate();
        return robot::framework::process_result::no_new_data;
    }

    const gimbal_inputs& inputs = inputs_->read();
    // 注意：State_Node 在本节点之后运行，所以这里读到的是上一帧的角度状态，
    // 与旧 C 实现（先决策、后 Gimbal_UpdataAngle）完全一致。
    const gimbal_state& previous_state = state_->read();
    const float dt_s = static_cast<float>(context.period_us) * 1e-6f;

    bool vision_control_active = false;
    const bool attack_allowed = (inputs.rc.left == 1U);
    const bool vision_forced = (inputs.rc.wheel < -400);

    if ((inputs.chassis_pattern == Chassis_AI) && attack_allowed)
    {
        vision_control_active = apply_vision_control(inputs, previous_state);
        if (vision_control_active)
        {
            fric_open_ = true;
        }
        else
        {
            apply_scan(inputs, dt_s);
            fric_open_ = false;
        }
    }
    else
    {
        if (vision_forced)
        {
            vision_control_active = apply_vision_control(inputs, previous_state);
            if (!vision_control_active)
            {
                apply_rocker(inputs);
            }
        }
        else
        {
            apply_rocker(inputs);
            // Shooter_RockerControl：攻击开关按下即持续拨弹，否则停。
            shooter_command_ = attack_allowed ? TRIGGER_CONTINUE : IDLE;
        }
        apply_fric_toggle(inputs);
    }

    gimbal_targets targets{};
    targets.base_yaw_target_deg = base_target_deg_;
    targets.top_yaw_target_deg = top_target_deg_;
    targets.pitch_target_deg = pitch_target_deg_;
    targets.fric_open = fric_open_;
    targets.shooter_command = shooter_command_;
    targets.id_to_trigger = id_to_trigger_;
    targets.vision_control_active = vision_control_active;
    targets.sampled_at_us = context.timestamp_us;

    output_.publish(context.frame_id, targets);
    return robot::framework::process_result::ok;
}

bool Gimbal_Target_Node::apply_vision_control(const gimbal_inputs& inputs, const gimbal_state& previous_state)
{
    if (!inputs.vision_valid)
    {
        shooter_command_ = IDLE;
        return false;
    }

    // 大 yaw 直接瞄准。
    base_target_deg_ = ModularDegreeTowards(inputs.vision_base_yaw_deg, base_target_deg_);

    // 小 yaw：目标相对角度叠加在 base 解缠目标上，并限制在 ±45°。
    const float top_target_unwrap = base_target_deg_ + (inputs.vision_yaw_deg - inputs.vision_base_yaw_deg);
    float delta = top_target_unwrap - previous_state.base_yaw_total_deg;
    delta = clampf(delta, -k_top_yaw_limit_deg, k_top_yaw_limit_deg);
    top_target_deg_ = previous_state.base_yaw_total_deg + delta;

    // pitch 直接跟踪并限幅。
    pitch_target_deg_ = inputs.vision_pitch_deg;
    pitch_target_deg_ = clampf(pitch_target_deg_, inputs.pitch_min_deg, inputs.pitch_max_deg);

    id_to_trigger_ = inputs.vision_id_to_trigger;
    shooter_command_ = inputs.vision_shooter_command;
    return true;
}

void Gimbal_Target_Node::apply_scan(const gimbal_inputs& inputs, float dt_s)
{
    switch (inputs.ai_armor_mode)
    {
    case VISION_AI_ARMOR:
        // yaw 匀速扫描，pitch 正弦摆动（与旧实现相同的参数）。
        top_target_deg_ += k_scan_yaw_speed_dps * dt_s;
        base_target_deg_ = top_target_deg_;

        pitch_phase_ += 2.0f * k_pi * k_scan_pitch_freq_hz * dt_s;
        if (pitch_phase_ > 2.0f * k_pi)
        {
            pitch_phase_ -= 2.0f * k_pi;
        }
        pitch_target_deg_ += k_scan_pitch_amp_deg * (2.0f * k_pi * k_scan_pitch_freq_hz) * std::cos(pitch_phase_) * dt_s;
        pitch_target_deg_ = clampf(pitch_target_deg_, k_scan_pitch_min_deg, k_scan_pitch_max_deg);
        break;

    case VISION_AI_OUTPOST:
        top_target_deg_ = inputs.ai_look_yaw_deg;
        base_target_deg_ = inputs.ai_look_yaw_deg;
        pitch_target_deg_ = inputs.ai_look_pitch_deg + 7.5f;
        break;

    case VISION_AI_RUNE:
        top_target_deg_ = inputs.ai_look_yaw_deg;
        base_target_deg_ = inputs.ai_look_yaw_deg;
        pitch_target_deg_ = inputs.ai_look_pitch_deg;
        break;

    default:
        break;
    }
}

void Gimbal_Target_Node::apply_rocker(const gimbal_inputs& inputs)
{
    top_target_deg_ -= static_cast<float>(inputs.rc.ch1) * 0.3f / 660.0f;
    base_target_deg_ = top_target_deg_;
    pitch_target_deg_ += static_cast<float>(inputs.rc.ch2) * 0.35f / 660.0f;
    pitch_target_deg_ = clampf(pitch_target_deg_, inputs.pitch_min_deg, inputs.pitch_max_deg);
}

void Gimbal_Target_Node::apply_fric_toggle(const gimbal_inputs& inputs)
{
    // 拨轮向上越过 400 视为一次“切换摩擦轮”动作（沿用旧实现的边沿判断）。
    if ((inputs.rc.wheel > 400) && (last_wheel_ <= 400))
    {
        fric_open_ = !fric_open_;
    }
    last_wheel_ = inputs.rc.wheel;
}

} // namespace robot::Gimbal
