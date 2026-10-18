/* ============================================================================
 * 文件：RobotInputs.cpp
 * 作用：RobotInputs.hpp 的实现——在任务上下文里把 C 侧全局量整份拷贝成快照。
 * 线程安全：
 *   - 所有 C 全局量的写方可能位于中断（CAN/B2B）或其它任务（IMU/RC）；
 *   - 本文件用 interrupt_snapshot_lock（保存/恢复 PRIMASK 的短临界区）保护
 *     “整份拷贝”，保证读到的是一致的一组值，而不是写了一半的中间态；
 *   - 临界区内只做赋值，不做计算、不调用 HAL、不阻塞。
 * ========================================================================== */
#include "RobotInputs.hpp"

#include "InterruptSnapshotLock.hpp"

extern "C"
{
#include "Chassis.h"
#include "Gimbal.h"
#include "Judge.h"
#include "Shooter.h"
#include "USER_B2B.h"
#include "USER_RC.h"
#include "imu_temp_ctrl.h"
}

namespace robot::io::inputs {

namespace {

// 所有输入快照共用一个短临界区后端：单核上保护拷贝即可。
robot::platform::stm32::interrupt_snapshot_lock g_input_lock{};

} // namespace

imu_snapshot read_imu(std::uint64_t sampled_at_us)
{
    imu_snapshot snapshot{};
    {
        const robot::framework::snapshot_lock_guard guard(g_input_lock);
        snapshot.yaw_deg = INS.yaw;
        snapshot.pitch_deg = INS.pitch;
        snapshot.roll_deg = INS.roll;
        snapshot.yaw_total_deg = INS.YawTotalAngle;
        snapshot.gyro_x = INS.gyro[0];
        snapshot.gyro_y = INS.gyro[1];
        snapshot.gyro_z = INS.gyro[2];
    }
    snapshot.sampled_at_us = sampled_at_us;
    return snapshot;
}

rc_snapshot read_rc(std::uint64_t sampled_at_us)
{
    rc_snapshot snapshot{};
    {
        const robot::framework::snapshot_lock_guard guard(g_input_lock);
        snapshot.ch1 = rcInfo.ch1;
        snapshot.ch2 = rcInfo.ch2;
        snapshot.ch3 = rcInfo.ch3;
        snapshot.ch4 = rcInfo.ch4;
        snapshot.wheel = rcInfo.wheel;
        snapshot.left = rcInfo.left;
        snapshot.right = rcInfo.right;
        snapshot.left_last = rcInfo.left_last;
        snapshot.right_last = rcInfo.right_last;
        snapshot.lost = rcInfo.lost;
        snapshot.failsafe = rcInfo.failsafe;
    }
    snapshot.sampled_at_us = sampled_at_us;
    return snapshot;
}

judge_snapshot read_judge(std::uint64_t sampled_at_us)
{
    judge_snapshot snapshot{};
    {
        const robot::framework::snapshot_lock_guard guard(g_input_lock);
        snapshot.game_progress = USER_JudgeData.game_progress;
        snapshot.remain_time_s = USER_JudgeData.remain_time;
        snapshot.current_hp = USER_JudgeData.current_hp;
        snapshot.projectile_allowance = USER_JudgeData.projectile;
        snapshot.sentry_info = USER_JudgeData.sentry_info;
        snapshot.sentry_info_2 = USER_JudgeData.sentry_info_2;
        snapshot.ally_outpost_hp = USER_JudgeData.ally_outpost_hp;
        snapshot.ally_base_hp = USER_JudgeData.ally_base_hp;
        snapshot.barrel_cooling_value = USER_JudgeData.shooter_barrel_cooling_value;
        snapshot.barrel_heat_limit = USER_JudgeData.shooter_barrel_heat_limit;
        snapshot.barrel_heat_now = USER_JudgeData.shooter_id1_17mm_cooling_heat;
        snapshot.initial_speed = USER_JudgeData.initial_speed;
        snapshot.power_gimbal_output = USER_JudgeData.power_management_gimbal_output;
        snapshot.power_chassis_output = USER_JudgeData.power_management_chassis_output;
        snapshot.power_shooter_output = USER_JudgeData.power_management_shooter_output;
        snapshot.self_color = USER_JudgeData.self_color;
        snapshot.is_valid = JUDGE_IsValid();
    }
    snapshot.sampled_at_us = sampled_at_us;
    return snapshot;
}

actuator_feedback read_actuator_feedback(std::uint64_t sampled_at_us)
{
    actuator_feedback snapshot{};
    {
        const robot::framework::snapshot_lock_guard guard(g_input_lock);

        snapshot.fric_speed[0] = shooter.fricMotor[0].speed;
        snapshot.fric_speed[1] = shooter.fricMotor[1].speed;
        snapshot.trigger_angle = shooter.triggerMotor.angle;
        snapshot.trigger_speed = shooter.triggerMotor.speed;

        snapshot.top_yaw_angle = gimbal.top_yawMotor.angle;
        snapshot.top_yaw_speed = gimbal.top_yawMotor.speed;
        snapshot.pitch_pos_rad = gimbal.pitchMotor.para.pos;
        snapshot.pitch_vel_rad_s = gimbal.pitchMotor.para.vel;
        snapshot.pitch_state = static_cast<std::int8_t>(gimbal.pitchMotor.para.state);

        for (std::uint8_t index = 0U; index < 4U; ++index)
        {
            snapshot.wheel_turn_angle_deg[index] = chassis.motors[index].TurnAngle;
            snapshot.wheel_speed[index] = chassis.motors[index].now_Speed;
        }
        snapshot.base_yaw_angle_deg = gimbal.base_yawMotor.nowAngle;
        snapshot.b2b_feedback = FEEDBACK;
        snapshot.stop_flag = STOPFLAG;
    }
    snapshot.sampled_at_us = sampled_at_us;
    return snapshot;
}

} // namespace robot::io::inputs
