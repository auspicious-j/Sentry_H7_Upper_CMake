#ifndef ROBOT_DEBUG_SNAPSHOT_HPP
#define ROBOT_DEBUG_SNAPSHOT_HPP

#include <cstdint>

namespace robot::platform {

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

extern volatile RobotDebugSnapshot g_robot_debug;

} // namespace robot::platform

#endif /* ROBOT_DEBUG_SNAPSHOT_HPP */
