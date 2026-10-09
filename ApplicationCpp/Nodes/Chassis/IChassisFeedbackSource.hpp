#ifndef ROBOT_CHASSIS_FEEDBACK_SOURCE_HPP
#define ROBOT_CHASSIS_FEEDBACK_SOURCE_HPP

#include "ChassisData.hpp"

namespace robot::chassis {

// 隔离底盘节点与 C 全局对象、硬件接收器。
class IChassisFeedbackSource {
public:
    // 支持经接口析构。
    virtual ~IChassisFeedbackSource() = default;
    // 返回一份稳定快照；从未接收时 received=false。
    virtual ChassisFeedback read() const = 0;
};

} // namespace robot::chassis
#endif // ROBOT_CHASSIS_FEEDBACK_SOURCE_HPP
