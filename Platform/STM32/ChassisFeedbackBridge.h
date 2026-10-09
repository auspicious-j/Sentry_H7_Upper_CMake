#ifndef ROBOT_CHASSIS_FEEDBACK_BRIDGE_H
#define ROBOT_CHASSIS_FEEDBACK_BRIDGE_H

#include <stdint.h>

#define ROBOT_CHASSIS_FEEDBACK_WHEELS 4U // 下板四个轮组。

#ifdef __cplusplus
extern "C" {
#endif

// C/C++ 共享的板间反馈快照，不含 HAL 类型。
typedef struct {
    float steering_deg[ROBOT_CHASSIS_FEEDBACK_WHEELS]; // 原始单圈舵角，度。
    int16_t drive_rpm[ROBOT_CHASSIS_FEEDBACK_WHEELS]; // 下板反馈原始轮电机转速。
    uint32_t received_tick_ms; // 最近接收时的 HAL tick。
    uint32_t sampled_tick_ms; // 本次读取时的 HAL tick。
    uint32_t receive_count; // 已接收帧数，允许自然回绕。
    uint8_t received; // 是否至少收到过一帧。
    uint8_t lower_feedback; // 下板 FEEDBACK 原值，不代表单电机在线。
} RobotChassisFeedbackSnapshot;

// 向 destination 复制完整快照；空指针时不操作，只供任务调用。
void RobotChassisFeedback_Read(RobotChassisFeedbackSnapshot* destination);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CHASSIS_FEEDBACK_BRIDGE_H
