#ifndef ROBOT_DEBUG_SNAPSHOT_HPP
#define ROBOT_DEBUG_SNAPSHOT_HPP

#include <cstdint>

namespace robot::platform {

// Keil Watch 可直接观察的固定调试快照；业务状态仍封装在节点对象内。
struct RobotDebugSnapshot {
    volatile uint32_t frame_id; // 当前帧号。
    volatile uint32_t last_frame_duration_us; // 最近帧耗时。
    volatile uint32_t max_frame_duration_us; // 最大帧耗时。
    volatile uint32_t last_node_duration_us; // 最近节点耗时。
    volatile uint32_t max_node_duration_us; // 最大节点耗时。
    volatile uint32_t graph_node_count; // 图节点数。
    volatile uint32_t graph_edge_count; // 图边数。
    volatile uint32_t init_error; // 初始化错误码。
    volatile uint32_t fault_flags; // 故障标志。
    volatile uint8_t initialized; // 初始化完成标志。
    volatile uint8_t running; // 运行标志。
    volatile uint8_t last_result; // 最近处理结果。
    volatile uint8_t reserved; // 保留字节。
};

} // namespace robot::platform

// 使用 C 链接名，方便 Keil Watch 直接通过 g_robot_debug 查找。
#ifdef __cplusplus
extern "C" {
#endif
extern volatile robot::platform::RobotDebugSnapshot g_robot_debug;
#ifdef __cplusplus
}
#endif

#endif /* ROBOT_DEBUG_SNAPSHOT_HPP */
