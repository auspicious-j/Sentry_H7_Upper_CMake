#ifndef ROBOT_LEGACY_CHASSIS_FEEDBACK_SOURCE_HPP
#define ROBOT_LEGACY_CHASSIS_FEEDBACK_SOURCE_HPP

#include "Nodes/Chassis/IChassisFeedbackSource.hpp"

namespace robot::platform::stm32 {

// 旧板间反馈的 STM32 适配器，业务节点不引用旧 chassis 全局对象。
class LegacyChassisFeedbackSource final : public robot::chassis::IChassisFeedbackSource {
public:
    // 读取旧接收器发布的稳定快照。
    robot::chassis::ChassisFeedback read() const override;
};

} // namespace robot::platform::stm32
#endif // ROBOT_LEGACY_CHASSIS_FEEDBACK_SOURCE_HPP
