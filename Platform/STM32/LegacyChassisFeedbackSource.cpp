// =============================================================================
// LegacyChassisFeedbackSource.cpp —— 适配器实现：C 快照 -> C++ 类型化反馈
//
// 单一职责：只做"搬运与换算"，不做解算、滤波或状态保持，不产生任何电机命令。
//
// 数据流：
//   下板 -> 板间串口接收回调(USER_B2B.c) 解析报文
//        -> 写入 static volatile 全局 chassis_feedback_snapshot
//        -> RobotChassisFeedback_Read() 在短临界区内复制出稳定快照，并顺带记录
//           本次采样的 HAL tick（sampled_tick_ms）
//        -> 本文件 read() 逐字段搬进 robot::chassis::ChassisFeedback
//        -> 上层底盘插件节点消费该反馈
//
// 为什么需要适配器：旧代码的数据是 C 全局变量、单位为原始量；插件框架要求的是
// 纯 C++、带明确单位、与硬件/全局态解耦的接口。本类正是二者之间的桥。
// =============================================================================
#include "LegacyChassisFeedbackSource.hpp"
#include "ChassisFeedbackBridge.h"

namespace robot::platform::stm32 {

// 从 C 侧的稳定快照转换为带明确单位的插件协议结构。
// 参数：无。
// 返回：robot::chassis::ChassisFeedback，本帧的完整反馈。
robot::chassis::ChassisFeedback LegacyChassisFeedbackSource::read() const
{
    RobotChassisFeedbackSnapshot snapshot{}; // C 侧完整反馈，由 Bridge 函数填充。
    robot::chassis::ChassisFeedback feedback{}; // 输出的类型化反馈，逐字段赋值后返回。
    // 编译期校验：C 宏与 C++ 常量必须一致，否则轮组数组会越界/错位，直接编译报错。
    static_assert(robot::chassis::kWheelCount == ROBOT_CHASSIS_FEEDBACK_WHEELS,
                  "Chassis wheel count mismatch");
    // 从 C 全局快照复制出一份稳定样本（内部含短临界区，避免读到撕裂数据）。
    RobotChassisFeedback_Read(&snapshot);
    // 逐轮搬运舵角与转速；index：轮组下标，保持旧协议次序（0~3）。
    for (uint8_t index = 0U; index < robot::chassis::kWheelCount; ++index) {
        feedback.steering_deg[index] = snapshot.steering_deg[index];
        feedback.drive_rpm[index] = snapshot.drive_rpm[index];
    }
    feedback.receive_count = snapshot.receive_count; // 累计接收帧数（真实接收次数，非插件帧号）。
    feedback.received = snapshot.received != 0U; // C 的 uint8_t 标志转 C++ bool。
    feedback.lower_feedback = snapshot.lower_feedback; // 下板使能反馈原值（不代表单电机在线）。
    // age_ms：本帧年龄 = 采样时刻 - 接收时刻。
    // - 两个 tick 均为无符号，相减在 32 位回绕时仍能得到正确的小间隔；
    // - 从未收到过帧时年龄无意义，直接置 0；
    // - 重复读取同一帧不会产生新反馈，故年龄会随读取时刻增大，属预期行为。
    feedback.age_ms = feedback.received
        ? snapshot.sampled_tick_ms - snapshot.received_tick_ms : 0U;
    return feedback;
}

} // namespace robot::platform::stm32
