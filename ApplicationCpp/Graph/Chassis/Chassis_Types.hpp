/* ============================================================================
 * 文件：Chassis_Types.hpp
 * 作用：底盘图内部节点之间传递的 POD 结构。
 * 说明：底盘的持续状态（几何、零点、斜坡、PID、速度）已全部归
 *       Chassis_Kinematics_Node 私有；C 侧 chassis 结构体只剩 B2B 中断写入的
 *       硬件反馈。帧信号承载图内输入/决策，chassis_output 承载跨任务输出。
 * ========================================================================== */
#ifndef robot_chassis_types_hpp
#define robot_chassis_types_hpp

#include <cstdint>

#include "RobotInputs.hpp"

namespace robot::Chassis {

struct chassis_inputs {
    robot::io::inputs::rc_snapshot rc;

    // 视觉 AI 指令（取自原始视觉帧快照 vision_frame_snapshot）。
    float vx;
    float vy;
    float vw;
    std::uint8_t spin_mode;
    std::uint8_t align_mode;
    float align_yaw_deg;

    // 下板反馈（B2B 中断写入 chassis/gimbal 的字段）。
    float base_yaw_angle_deg;
    float wheel_turn_angle_deg[4];

    std::uint64_t sampled_at_us;
};

struct chassis_mode {
    std::uint8_t pattern;     // Chassis_Control / Chassis_AI
    std::uint8_t rotate_mode; // ChassisMode_Follow / ChassisMode_Spin
    std::uint64_t sampled_at_us;
};

// 运动学输出快照：跨任务交给 RS485 任务组 B2B 帧，不再写 C 侧 chassis.motors[]。
struct chassis_output {
    float target_turn_angle_deg[4];
    std::int32_t target_drive_speed[4];
    float now_turn_angle_deg[4]; // 本帧舵角反馈（B2B 帧发的是 target - now）。
    std::uint64_t sampled_at_us;
};

} // namespace robot::Chassis

#endif // robot_chassis_types_hpp
