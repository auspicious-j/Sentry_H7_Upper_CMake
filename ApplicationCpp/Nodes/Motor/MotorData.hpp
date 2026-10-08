#ifndef ROBOT_MOTOR_DATA_HPP
#define ROBOT_MOTOR_DATA_HPP

#include <cstdint>

namespace robot::motor {

constexpr uint8_t kMotorChannelCount = 8U; // 单帧最多支持的电机通道数。

// 一个电机的统一反馈数据，不暴露 DJI/DM 协议细节。
struct MotorFeedback {
    uint16_t id{0U}; // 电机协议 ID。
    int16_t angle{0}; // 编码器角度或协议原始角度。
    int16_t speed{0}; // 转速或协议原始速度。
    int16_t torque{0}; // 转矩或电流反馈。
    int8_t temperature{0}; // 电机温度。
    bool online{false}; // 本周期是否在线。
};

// 一组电机反馈，供上层节点读取。
struct MotorFeedbackFrame {
    MotorFeedback motor[kMotorChannelCount]{}; // 固定容量反馈数组。
};

// 一组电机在线状态。
struct MotorOnlineFrame {
    bool online[kMotorChannelCount]{}; // 固定容量在线状态数组。
};

// 一个电机的统一控制命令。
struct MotorCommand {
    uint16_t id{0U}; // 电机协议 ID。
    int16_t current{0}; // 目标电流或等价输出量。
    bool enable{false}; // 是否允许输出。
};

// 一组电机控制命令，暂时只作为端口协议，不直接发送 CAN。
struct MotorCommandFrame {
    MotorCommand motor[kMotorChannelCount]{}; // 固定容量命令数组。
};

} // namespace robot::motor

#endif /* ROBOT_MOTOR_DATA_HPP */
