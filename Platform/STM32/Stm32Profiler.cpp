#include "Stm32Profiler.hpp"

#include "../../ApplicationCpp/Platform/DebugSnapshot.hpp"

namespace robot::platform::stm32 {

// 帧级统计使用执行器传入的帧起始时间。
void Stm32Profiler::beginFrame(uint32_t frame_id, uint64_t timestamp_us)
{
    frame_start_us_ = timestamp_us;
    robot::platform::g_robot_debug.frame_id = frame_id;
}

// 节点单独读取当前时钟，测量真实节点耗时。
void Stm32Profiler::beginNode(robot::framework::PluginId id)
{
    current_node_id_ = id;
    node_start_us_ = clock_.nowUs();
}

// 节点结束时更新最近一次和历史最大耗时。
void Stm32Profiler::endNode()
{
    // timestamp_us：节点结束时的当前时间。
    const uint64_t timestamp_us = clock_.nowUs();
    if (current_node_id_ == robot::framework::kInvalidPluginId || timestamp_us < node_start_us_) {
        return;
    }
    // duration：本次节点或帧的耗时。
    const uint32_t duration = static_cast<uint32_t>(timestamp_us - node_start_us_);
    robot::platform::g_robot_debug.last_node_duration_us = duration;
    if (duration > robot::platform::g_robot_debug.max_node_duration_us) {
        robot::platform::g_robot_debug.max_node_duration_us = duration;
    }
    current_node_id_ = robot::framework::kInvalidPluginId;
}

// 帧结束时更新整帧耗时和最大耗时。
void Stm32Profiler::endFrame(uint64_t timestamp_us)
{
    if (timestamp_us < frame_start_us_) {
        return;
    }
    const uint32_t duration = static_cast<uint32_t>(timestamp_us - frame_start_us_);
    robot::platform::g_robot_debug.last_frame_duration_us = duration;
    if (duration > robot::platform::g_robot_debug.max_frame_duration_us) {
        robot::platform::g_robot_debug.max_frame_duration_us = duration;
    }
}

} // namespace robot::platform::stm32
