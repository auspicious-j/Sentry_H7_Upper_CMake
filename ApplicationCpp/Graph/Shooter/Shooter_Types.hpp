/* ============================================================================
 * 文件：Shooter_Types.hpp
 * 作用：发射图内部传递的 POD 结构。
 * ========================================================================== */
#ifndef robot_shooter_types_hpp
#define robot_shooter_types_hpp

#include <cstdint>

#include "RobotInputs.hpp"

namespace robot::Shooter {

struct shooter_inputs {
    robot::io::inputs::judge_snapshot judge;
    robot::io::inputs::actuator_feedback feedback;

    // 来自 Gimbal 的发射请求（迁移期读 C 侧 shooter 全局）。
    std::uint8_t command_request;
    std::int8_t id_to_trigger_request;
    bool fric_open;

    std::uint64_t sampled_at_us;
};

// 逻辑节点处理后的目标（同时已写入 C 侧 shooter 全局）。
struct shooter_output {
    std::int16_t fric_target_speed[2];
    std::int32_t trigger_target_angle;
    std::uint8_t command;
    std::int8_t triggering_id;
    std::uint64_t sampled_at_us;
};

} // namespace robot::Shooter

#endif // robot_shooter_types_hpp
