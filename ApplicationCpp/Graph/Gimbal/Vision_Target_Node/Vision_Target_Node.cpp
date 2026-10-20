/* ============================================================================
 * 文件：Vision_Target_Node.cpp
 * 作用：视觉目标判定节点（旧 Vision_UseArmor / Vision_UseRune 的节点化迁移）。
 *
 * 数据流：
 *   vision_frame_mailbox() --copy--> 本节点 --publish--> frame_signal / 目标邮箱
 *
 * 与旧实现的行为对应关系：
 *   - 旧实现每收到一帧视觉数据就在 USB 中断里重算一次 vision_target；
 *   - 本节点用“发布序号”检测新帧，只有新帧到达时才重算，其余帧保持上次结果，
 *     因此消费者看到的更新节奏与旧实现一致；
 *   - 旧的 shooter.triggeringHitTick 写入由本节点在识别到“正在触发的符”时提交。
 * ========================================================================== */
#include "Vision_Target_Node.hpp"

#include <cfloat>
#include <cmath>
#include <cstdint>

#include "InterruptSnapshotLock.hpp"
#include "RobotInputs.hpp"
#include "VisionLink.hpp"

extern "C"
{
    #include "Shooter.h"
    #include "stm32h7xx_hal.h"
}

namespace robot::Gimbal 
{

    namespace 
    {

        constexpr float k_pi = 3.14159265358979f;

        // Shooter包的一部分快照
        struct shooter_vision_state
        {
            std::int8_t triggering_id;      // 正在触发的符 id，-1 表示无
            std::uint32_t hit_at_tick[5];   // 每个符的命中延时（ms），shooter.hitAtTick 的快照
            std::uint32_t now_ms;           // 当前时刻（ms），shooter.hitAtTick 的快照
        };

        robot::platform::stm32::interrupt_snapshot_lock g_shooter_lock{};

        // 读取发射器状态的快照
        shooter_vision_state read_shooter_vision_state()
        {
            shooter_vision_state state{};
            {
                const robot::framework::snapshot_lock_guard guard(g_shooter_lock);//上锁
                state.triggering_id = shooter.triggeringId;
                for (std::uint32_t index = 0U; index < 5U; ++index)
                {
                    state.hit_at_tick[index] = shooter.hitAtTick[index];
                }
            }
            state.now_ms = HAL_GetTick();
            return state;
        }

        // 旧 Vision_UseRune 在识别到“正在触发的符”时会写 shooter.triggeringHitTick，
        // 这里由本节点作为唯一写者提交。
        void commit_triggering_hit_tick(std::uint32_t hit_tick)
        {
            const robot::framework::snapshot_lock_guard guard(g_shooter_lock);
            shooter.triggeringHitTick = hit_tick;
        }

        // 搬运目标值并判断在不在攻击圆锥内
        robot::io::vision::vision_target_snapshot decode_armor(
            const robot::io::vision::vision_frame_snapshot& frame,
            float imu_yaw_deg)
        {
            robot::io::vision::vision_target_snapshot target{};
            target.is_valid = true;
            target.base_yaw = frame.base_yaw;
            target.yaw = frame.yaw;
            target.pitch = frame.pitch;
            target.id_to_trigger = -1;

            const float a = imu_yaw_deg / 180.0f * k_pi - frame.yaw / 180.0f * k_pi;
            const float b = frame.armor_incident_yaw;
            const float distance = frame.armor_distance;
            const float hit_radius = frame.armor_hit_radius;
            const float radius_cos_b = hit_radius * std::cos(b);
            const float radius_sin_b = hit_radius * std::sin(b);
            const float tan_a = std::tan(a);

            // 如果目标在圆锥内，则发射器继续触发，否则空闲
            if ((tan_a < radius_cos_b / (distance - radius_sin_b)) && (tan_a > -radius_cos_b / (distance + radius_sin_b)))
            {
                target.shooter_command = TRIGGER_CONTINUE;
            }
            else
            {
                target.shooter_command = IDLE;
            }

            return target;
        }

        // 打符判定
        robot::io::vision::vision_target_snapshot decode_rune(
            const robot::io::vision::vision_frame_snapshot& frame,
            float imu_yaw_deg,
            float imu_pitch_deg,
            const shooter_vision_state& shooter_state,
            bool& has_hit_tick,
            std::uint32_t& hit_tick)
        {
            has_hit_tick = false;
            hit_tick = 0U;

            std::int32_t index = -1; // 当前最优的符下标。
            bool is_triggering = false;
            float min_angle_distance = FLT_MAX;

            for (std::int32_t i = 0; i < 2; ++i)
            {
                const std::int8_t rune_id = frame.rune_id[i];

                // 有符
                if (rune_id != -1)
                {
                    if (shooter_state.triggering_id == rune_id)
                    {
                        hit_tick = static_cast<std::uint32_t>(frame.rune_hit_time[i] * 1000.0f);
                        has_hit_tick = true;
                        index = i;
                        is_triggering = true;
                        break;
                    }

                    if ((shooter_state.triggering_id == -1) &&
                        (shooter_state.now_ms > shooter_state.hit_at_tick[rune_id]))
                    {
                        const float angle_distance = std::hypot(
                            imu_yaw_deg - frame.rune_yaw[i],
                            imu_pitch_deg - frame.rune_pitch[i]);

                        if (angle_distance < min_angle_distance)
                        {
                            index = i;
                            is_triggering = false;
                            min_angle_distance = angle_distance;
                        }
                    }
                }
            }

            robot::io::vision::vision_target_snapshot target{};
            target.id_to_trigger = -1;

            if (index == -1)
            {
                target.is_valid = false;
                target.shooter_command = IDLE;
                return target;
            }

            target.is_valid = true;
            target.base_yaw = frame.base_yaw;
            target.yaw = frame.rune_yaw[index];
            target.pitch = frame.rune_pitch[index];

            if (is_triggering)
            {
                target.shooter_command = IDLE;
                return target;
            }

            const float a = imu_yaw_deg / 180.0f * k_pi - frame.rune_yaw[index] / 180.0f * k_pi;
            const float b = frame.rune_incident_yaw[index];
            const float distance = frame.rune_distance[index];
            const float hit_radius = frame.rune_hit_radius;
            const float range = distance / std::cos(a - b);
            const float y = range * std::sin(a);
            const float x = range * std::cos(b);

            const float height_error =
                x * std::tan(imu_pitch_deg / 180.0f * k_pi) -
                distance * std::tan(frame.rune_pitch[index] / 180.0f * k_pi);

            if (std::hypot(y, height_error) <= hit_radius)
            {
                target.id_to_trigger = frame.rune_id[index];
                target.shooter_command = TRIGGER;
            }
            else
            {
                target.shooter_command = IDLE;
            }

            return target;
        }

        // 按模式分派
        robot::io::vision::vision_target_snapshot decode_vision_target( const robot::io::vision::vision_frame_snapshot& frame,
                                                                        std::uint64_t timestamp_us)
        {
            robot::io::vision::vision_target_snapshot target{};// 解析出的目标动作
            target.is_valid = false;
            target.id_to_trigger = -1;
            target.shooter_command = IDLE;

            // 装甲板
            if (frame.raw_mode == robot::io::vision::k_mode_armor)
            {
                const robot::io::inputs::imu_snapshot imu = robot::io::inputs::read_imu(timestamp_us);
                target = decode_armor(frame, imu.yaw_deg);
            }
            // 符
            else if (frame.raw_mode == robot::io::vision::k_mode_rune)
            {
                const robot::io::inputs::imu_snapshot imu = robot::io::inputs::read_imu(timestamp_us);
                const shooter_vision_state shooter_state = read_shooter_vision_state();

                bool has_hit_tick = false;
                std::uint32_t hit_tick = 0U;
                target = decode_rune(frame, imu.yaw_deg, imu.pitch_deg, shooter_state, has_hit_tick, hit_tick);

                if (has_hit_tick)
                {
                    commit_triggering_hit_tick(hit_tick);
                    target.triggering_hit_tick = hit_tick;
                }
            }

            return target;
        }

    } // namespace

    // 邮箱里读取然后解码成target_action_，按帧发布到端口和跨任务邮箱
    robot::framework::process_result Vision_Target_Node::process(robot::framework::frame_context& context)
    {
        robot::framework::timed_snapshot<robot::io::vision::vision_frame_snapshot> vision_mailbox_copy_{}; // 复制视觉邮箱内容
        robot::io::vision::vision_frame_mailbox().copy_to(vision_mailbox_copy_);

        // 无视觉目标
        if (!vision_mailbox_copy_.is_present)
        {
            target_action_ = robot::io::vision::vision_target_snapshot{};
            target_action_.id_to_trigger = -1;
            target_action_.shooter_command = IDLE;
            is_first_frame_ = false;

            // 发布
            output_.publish(context.frame_id, target_action_);
            robot::io::vision::vision_target_mailbox().publish(target_action_, context.timestamp_us);
            return robot::framework::process_result::ok;
        }

        // 第一帧或者收到新帧
        if (is_first_frame_ || (vision_mailbox_copy_.sequence != last_sequence_))
        {
            last_sequence_ = vision_mailbox_copy_.sequence;
            target_action_ = decode_vision_target(vision_mailbox_copy_.value, context.timestamp_us);
            is_first_frame_ = false;
        }

        output_.publish(context.frame_id, target_action_);
        robot::io::vision::vision_target_mailbox().publish(target_action_, context.timestamp_us);

        return robot::framework::process_result::ok;
    }

} // namespace robot::Gimbal
