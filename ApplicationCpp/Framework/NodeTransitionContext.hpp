#ifndef ROBOT_NODE_TRANSITION_CONTEXT_HPP
#define ROBOT_NODE_TRANSITION_CONTEXT_HPP

#include <cstdint>

namespace robot::framework {

// 帧边界生命周期通知；此时不能假设本帧输入已经生产。
struct NodeTransitionContext {
    uint32_t frame_id{0U}; // 当前帧号；stopAll 使用最近一帧。
    uint64_t timestamp_us{0U}; // 帧开始时间，微秒。
    uint16_t previous_plan{0U}; // 切换前计划。
    uint16_t current_plan{0U}; // 本帧锁存计划。
    uint32_t plan_generation{0U}; // 当前计划版本。
    bool stopping{false}; // 是否由整图 stopAll 触发退出。
};

} // namespace robot::framework
#endif // ROBOT_NODE_TRANSITION_CONTEXT_HPP
