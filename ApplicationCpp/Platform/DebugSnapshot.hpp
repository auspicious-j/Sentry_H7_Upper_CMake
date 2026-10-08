#ifndef ROBOT_DEBUG_SNAPSHOT_HPP
#define ROBOT_DEBUG_SNAPSHOT_HPP

#include <cstdint>

namespace robot::platform {

// Keil Watch 可直接观察的固定调试快照；业务状态仍封装在节点对象内。
struct RobotDebugSnapshot {
    volatile uint32_t frame_id;
    volatile uint32_t last_frame_duration_us;
    volatile uint32_t max_frame_duration_us;
    volatile uint32_t last_node_duration_us;
    volatile uint32_t max_node_duration_us;
    volatile uint32_t graph_node_count;
    volatile uint32_t graph_edge_count;
    volatile uint32_t init_error;
    volatile uint32_t fault_flags;
    volatile uint8_t initialized;
    volatile uint8_t running;
    volatile uint8_t last_result;
    volatile uint8_t reserved;
};

// volatile 防止调试观察和中断/任务更新被编译器优化掉。
extern volatile RobotDebugSnapshot g_robot_debug;

} // namespace robot::platform

#endif /* ROBOT_DEBUG_SNAPSHOT_HPP */
