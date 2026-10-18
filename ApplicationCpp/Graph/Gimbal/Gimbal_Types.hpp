/* ============================================================================
 * 文件：Gimbal_Types.hpp
 * 作用：Gimbal 图内部各节点之间传递的 POD 数据结构（帧信号的值类型）。
 * 约定：
 *   - 全部 trivially copyable，可直接放进 frame_signal；
 *   - 角度统一为“度”，与旧 C 侧 gimbal 结构体保持一致；
 *   - sampled_at_us 为本帧时间戳，便于下游做新鲜度判断。
 * ========================================================================== */
#ifndef robot_gimbal_types_hpp
#define robot_gimbal_types_hpp

#include <cstdint>

#include "RobotInputs.hpp"

namespace robot::Gimbal {

// 一帧内 Gimbal 所需的全部输入（由 Input_Node 采集）。
struct gimbal_inputs {
    robot::io::inputs::imu_snapshot imu;
    robot::io::inputs::rc_snapshot rc;
    robot::io::inputs::actuator_feedback feedback;
    robot::io::inputs::judge_snapshot judge;

    // 视觉目标（迁移期读 C 侧 vision_target；后续切换为 Io/Vision 快照）。
    bool vision_valid;
    float vision_base_yaw_deg;
    float vision_yaw_deg;
    float vision_pitch_deg;
    std::int8_t vision_id_to_trigger;
    std::uint8_t vision_shooter_command;

    // 视觉 AI 指令（迁移期读 C 侧 USER_SentryCmd）。
    std::uint8_t ai_armor_mode;
    float ai_look_yaw_deg;
    float ai_look_pitch_deg;

    // 底盘状态（迁移期读 C 侧 chassis）。
    std::uint8_t chassis_pattern;
    std::uint8_t chassis_rotate_mode;

    // 目标角限幅（由 Gimbal_Init 写入 gimbal.pitch，每帧读取）。
    float pitch_min_deg;
    float pitch_max_deg;

    std::uint64_t sampled_at_us;
};

// Target_Node 的决策结果（尚未做角度解缠）。
struct gimbal_targets {
    float base_yaw_target_deg;
    float top_yaw_target_deg;
    float pitch_target_deg;
    bool fric_open;
    std::uint8_t shooter_command;
    std::int8_t id_to_trigger;
    bool vision_control_active;
    std::uint64_t sampled_at_us;
};

// State_Node 的输出：本帧测量状态 + 解缠/加弹修正后的最终目标。
struct gimbal_state {
    // 测量量（对应旧 Gimbal_UpdataAngle 写入的字段）。
    float top_yaw_angle_deg;
    float top_yaw_total_deg;
    float base_yaw_total_deg;
    float pitch_angle_deg;
    float top_gyro_z;
    float pitch_gyro_x;

    // 最终目标角（已做 ModularDegreeTowards 解缠，base 可能含 +45 加弹修正）。
    float base_yaw_target_deg;
    float top_yaw_target_deg;
    float pitch_target_deg;

    // 发射请求透传（Output_Node 写回 C 侧 shooter）。
    bool fric_open;
    std::uint8_t shooter_command;
    std::int8_t id_to_trigger;
    bool vision_control_active;

    std::uint64_t sampled_at_us;
};

// Control_Node 的输出：本帧算出的执行机构指令。
struct gimbal_control {
    float top_yaw_current;    // 小 yaw 电流（CAN 0x1FF）。
    float pitch_mit_torque;   // pitch MIT 力矩前馈输出。
    float base_yaw_speed_cmd; // 大 yaw 速度指令（B2B 发送）。
    std::uint64_t sampled_at_us;
};

// 小工具：与 C 侧 LIMIT 宏语义一致的三值钳位。
inline float clampf(float value, float min_value, float max_value)
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

} // namespace robot::Gimbal

#endif // robot_gimbal_types_hpp
