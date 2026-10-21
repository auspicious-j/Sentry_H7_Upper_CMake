/* ============================================================================
 * 文件：Shooter_Types.hpp
 * 作用：发射图内部传递的 POD 结构。
 * ========================================================================== */
#ifndef robot_shooter_types_hpp
#define robot_shooter_types_hpp

#include <cstdint>

#include "RobotInputs.hpp"
#include "Shooter.h"

namespace robot::Shooter 
{

    struct shooter_inputs 
    {
        robot::io::inputs::judge_snapshot judge;                    // 来自裁判系统的快照。
        robot::io::inputs::actuator_feedback downboard_feedback;    // 来自电机与下板的反馈快照。

        // 来自 Gimbal 的发射请求（迁移期读 C 侧 shooter 全局）。
        Shooter_State_t shooter_state;  // 拨弹状态机指令
        std::int8_t id_to_trigger;      // 视觉传来的符id，我这边打完之后回答视觉这个id的符可以屏蔽了
        bool fric_open;                 // 摩擦轮开启标志  0开1关

        std::uint64_t timestamp_us;    // 本帧时间戳，单位微秒(us)
    };

    // 逻辑节点处理后的目标；跨任务通过 shooter_output_mailbox() 交给 Motor 任务。
    struct shooter_output 
    {
        std::int16_t fric_target_speed[2];
        std::int32_t trigger_target_total_angle;
        std::uint64_t timestamp_us;
    };

} // namespace robot::Shooter

#endif // robot_shooter_types_hpp
