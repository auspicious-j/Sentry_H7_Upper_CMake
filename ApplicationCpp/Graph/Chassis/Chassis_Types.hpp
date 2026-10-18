/* ============================================================================
 * 文件：Chassis_Types.hpp
 * 作用：底盘图内部节点之间传递的 POD 结构。
 * 说明：底盘的持续状态（斜坡、模式、电机偏置等）仍保存在 C 侧 chassis 结构体，
 *       本图节点按“单一写者”原则分段维护它；帧信号只承载跨节点传递的输入/决策。
 * ========================================================================== */
#ifndef robot_chassis_types_hpp
#define robot_chassis_types_hpp

#include <cstdint>

#include "RobotInputs.hpp"

namespace robot::Chassis {

struct chassis_inputs {
    robot::io::inputs::rc_snapshot rc;

    // 视觉 AI 指令（迁移期读 C 侧 vx/vy/vw 与 vision_receive）。
    float vx;
    float vy;
    float vw;
    std::uint8_t spin_mode;
    std::uint8_t align_mode;
    float align_yaw_deg;

    // 下板反馈（B2B 中断写入 chassis/gimbal 的字段）。
    float base_yaw_angle_deg;
    float wheel_turn_angle_deg[4];
    std::int16_t wheel_speed[4];

    std::uint64_t sampled_at_us;
};

struct chassis_mode {
    std::uint8_t pattern;     // Chassis_Control / Chassis_AI
    std::uint8_t rotate_mode; // ChassisMode_Follow / ChassisMode_Spin
    std::uint64_t sampled_at_us;
};

// 运动学输出快照（同时已写入 C 侧 chassis.motors）。
struct chassis_output {
    float target_turn_angle_deg[4];
    std::int32_t target_drive_speed[4];
    float vx;
    float vy;
    float vw;
    std::uint64_t sampled_at_us;
};

} // namespace robot::Chassis

#endif // robot_chassis_types_hpp
