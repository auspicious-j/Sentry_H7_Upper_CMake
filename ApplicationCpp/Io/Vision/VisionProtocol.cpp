/* ============================================================================
 * 文件：VisionProtocol.cpp
 * 作用：视觉协议层的实现——消帧状态机与领域解码，全部为纯逻辑。
 * 注意：本文件不得 include 任何硬件/OS 头（见 VisionProtocol.hpp 的约束）。
 * ========================================================================== */
#include "VisionProtocol.hpp"

#include <cstring>

namespace robot::io::vision 
{

    vision_frame_snapshot decode_vision_frame(const vision_receive_wire& wire)
    {
        // 先整份拷贝 AI 指令字段：这些字段与模式无关，任何一帧都带。
        vision_frame_snapshot snapshot{};
        snapshot.has_target = false;
        snapshot.raw_mode = wire.mode;
        snapshot.id_to_trigger = -1;
        snapshot.force_shoot = (wire.force_shoot != 0U);
        snapshot.base_yaw = 0.0F;
        snapshot.yaw = 0.0F;
        snapshot.pitch = 0.0F;

        // 打装甲板判定字段。
        snapshot.armor_incident_yaw = 0.0F;
        snapshot.armor_distance = 0.0F;
        snapshot.armor_hit_radius = 0.0F;

        // 打符判定字段：数组整体清零，id 显式置 -1。
        for (int i = 0; i < 2; ++i)
        {
            snapshot.rune_yaw[i] = 0.0F;
            snapshot.rune_pitch[i] = 0.0F;
            snapshot.rune_incident_yaw[i] = 0.0F;
            snapshot.rune_distance[i] = 0.0F;
            snapshot.rune_hit_time[i] = 0.0F;
            snapshot.rune_id[i] = -1;
        }
        snapshot.rune_hit_radius = 0.0F;

        snapshot.linear_x = wire.linear_x;
        snapshot.linear_y = wire.linear_y;
        snapshot.angular_z = wire.angular_z;
        snapshot.align_yaw = wire.align_yaw;
        snapshot.look_yaw = wire.look_yaw;
        snapshot.look_pitch = wire.look_pitch;
        snapshot.spin_mode = wire.spin_mode;
        snapshot.sentry_mode = wire.sentry_mode;
        snapshot.armor_mode = wire.armor_mode;
        snapshot.align_mode = wire.align_mode;
        snapshot.energy_activation = wire.energy_activation;
        snapshot.buy_life = wire.buy_life;
        snapshot.remote_buy_blood = wire.remote_buy_blood;
        snapshot.remote_buy_bullet = wire.remote_buy_bullet;
        snapshot.buy_projectile = wire.buy_projectile;

        switch (wire.mode)
        {
        case k_mode_armor:
            // 旧逻辑（Vision_UseArmor）：收到装甲板帧即视为有目标。
            snapshot.has_target = true;
            snapshot.base_yaw = wire.armor.base_yaw;
            snapshot.yaw = wire.armor.yaw;
            snapshot.pitch = wire.armor.pitch;
            snapshot.armor_incident_yaw = wire.armor.incident_yaw;
            snapshot.armor_distance = wire.armor.distance;
            snapshot.armor_hit_radius = wire.armor.hit_radius;
            break;

        case k_mode_rune:
            // 旧逻辑会结合 shooter 触发状态与 IMU 角度挑目标并判定触发；这些
            // 输入不属于协议层。这里保留全部原始字段，实际判定由
            // Vision_Target_Node 完成（与旧 Vision_UseRune 逐行等价）。
            snapshot.rune_hit_radius = wire.rune.hit_radius;
            for (int i = 0; i < 2; ++i)
            {
                snapshot.rune_yaw[i] = wire.rune.yaw[i];
                snapshot.rune_pitch[i] = wire.rune.pitch[i];
                snapshot.rune_incident_yaw[i] = wire.rune.incident_yaw[i];
                snapshot.rune_distance[i] = wire.rune.distance[i];
                snapshot.rune_hit_time[i] = wire.rune.hit_time[i];
                snapshot.rune_id[i] = wire.rune.id[i];

                if (wire.rune.id[i] != -1)
                {
                    snapshot.has_target = true;
                    snapshot.base_yaw = wire.rune.base_yaw;
                    snapshot.yaw = wire.rune.yaw[i];
                    snapshot.pitch = wire.rune.pitch[i];
                    snapshot.id_to_trigger = wire.rune.id[i];
                    break;
                }
            }
            break;

        default:
            // 无效模式：has_target 保持 false。
            break;
        }

        return snapshot;
    }

    bool vision_frame_parser::feed(const std::uint8_t* data, std::uint32_t length)
    {
        if (data == nullptr)
        {
            return false;
        }

        bool decoded = false;

        while (length > 0U)
        {
            // 未锁定帧头：逐字节扫描，直到遇到 0xA5。
            if ((fill_index_ < 0) && (*data == k_frame_header))
            {
                fill_index_ = 0;
            }

            if (fill_index_ >= 0)
            {
                // 已锁定帧头：尽量把剩余字节填进帧缓冲。
                const std::uint32_t capacity = static_cast<std::uint32_t>(sizeof(vision_receive_wire));
                const std::uint32_t offset = static_cast<std::uint32_t>(fill_index_);
                const std::uint32_t rest = capacity - offset;
                const std::uint32_t size = (rest < length) ? rest : length;

                std::memcpy(reinterpret_cast<std::uint8_t*>(&wire_) + offset, data, size);
                fill_index_ += static_cast<std::int32_t>(size);

                if (static_cast<std::uint32_t>(fill_index_) == capacity)
                {
                    fill_index_ = -1;

                    if (wire_.end_frame == k_frame_end)
                    {
                        last_frame_ = decode_vision_frame(wire_);
                        ++frame_count_;
                        decoded = true;
                    }
                }

                data += size;
                length -= size;
            }
            else
            {
                // 帧外垃圾字节：丢弃，继续找帧头。
                ++data;
                --length;
            }
        }

        return decoded;
    }

} // namespace robot::io::vision
