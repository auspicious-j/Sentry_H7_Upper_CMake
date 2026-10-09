#include "LegacyChassisFeedbackSource.hpp"
#include "ChassisFeedbackBridge.h"

namespace robot::platform::stm32 {

// 从 C 接收快照转换为有明确单位的插件协议。
robot::chassis::ChassisFeedback LegacyChassisFeedbackSource::read() const
{
    RobotChassisFeedbackSnapshot snapshot{}; // C 侧完整反馈。
    robot::chassis::ChassisFeedback feedback{}; // 输出的类型化反馈。
    static_assert(robot::chassis::kWheelCount == ROBOT_CHASSIS_FEEDBACK_WHEELS,
                  "Chassis wheel count mismatch");
    RobotChassisFeedback_Read(&snapshot);
    // index：轮组下标，保持旧协议次序。
    for (uint8_t index = 0U; index < robot::chassis::kWheelCount; ++index) {
        feedback.steering_deg[index] = snapshot.steering_deg[index];
        feedback.drive_rpm[index] = snapshot.drive_rpm[index];
    }
    feedback.receive_count = snapshot.receive_count;
    feedback.received = snapshot.received != 0U;
    feedback.lower_feedback = snapshot.lower_feedback;
    // 无符号差支持 tick 自然回绕；不把重复读取当作新反馈。
    feedback.age_ms = feedback.received
        ? snapshot.sampled_tick_ms - snapshot.received_tick_ms : 0U;
    return feedback;
}

} // namespace robot::platform::stm32
