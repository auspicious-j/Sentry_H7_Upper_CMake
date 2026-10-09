#ifndef ROBOT_CHASSIS_DATA_HPP
#define ROBOT_CHASSIS_DATA_HPP

#include <cstdint>

namespace robot::chassis {

constexpr uint8_t kWheelCount = 4U; // 保持旧底盘轮组顺序。

// 来自下板的最新反馈，不与云台 CAN 电机协议混用。
struct ChassisFeedback {
    float steering_deg[kWheelCount]{}; // 单圈舵角，度，未扣零偏。
    int16_t drive_rpm[kWheelCount]{}; // 原始轮电机转速，不是车体速度。
    uint32_t receive_count{0U}; // 真实接收次数，不是插件帧号。
    uint32_t age_ms{0U}; // 最近帧年龄，仅 received=true 时有意义。
    bool received{false}; // 是否收到过板间反馈。
    uint8_t lower_feedback{0U}; // 下板使能反馈原值。
};

// 底盘观察结果，仅供诊断，不产生电机命令。
struct ChassisObservation {
    ChassisFeedback feedback{}; // 本次消费的板间快照。
    uint32_t frame_id{0U}; // 消费该快照的插件帧号。
    bool online{false}; // 板间链路是否未超时。
};

} // namespace robot::chassis
#endif // ROBOT_CHASSIS_DATA_HPP
