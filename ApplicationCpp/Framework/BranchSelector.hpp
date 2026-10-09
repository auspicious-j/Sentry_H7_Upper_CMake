#ifndef ROBOT_BRANCH_SELECTOR_HPP
#define ROBOT_BRANCH_SELECTOR_HPP

#include "PluginTypes.hpp"

namespace robot::framework {

class PluginGraph;

// 固定路线的选择器；只能由所属执行器任务访问，不负责跨线程同步。
class BranchSelector {
public:
    // route_count：路线数量；initial_route：初始路线，编号从 0 开始。
    explicit BranchSelector(uint16_t route_count, uint16_t initial_route = 0U)
        : route_count_(route_count), requested_(initial_route), active_(initial_route) {}

    // route：下一帧请求；非法编号保留原选择。
    bool request(uint32_t route)
    {
        if (!configured() || route >= route_count_) {
            return false;
        }
        requested_ = static_cast<uint16_t>(route);
        return true;
    }

    // 检查构造参数是否有效。
    bool configured() const { return route_count_ != 0U && requested_ < route_count_ && active_ < route_count_; }
    // route：待检查编号。
    bool contains(uint16_t route) const { return route < route_count_; }
    // 返回固定路线数量。
    uint16_t routeCount() const { return route_count_; }
    // 返回本帧锁存的路线。
    uint16_t activeRoute() const { return active_; }
    // 返回实际发生的路线切换次数。
    uint32_t switchCount() const { return switch_count_; }
    // 返回路径版本，只有路线改变时递增。
    uint32_t generation() const { return generation_; }

private:
    friend class PluginGraph;
    // 帧开始时由执行器调用一次，不在节点运行中修改 active_。
    void latch()
    {
        if (active_ != requested_) {
            active_ = requested_;
            ++switch_count_;
            ++generation_;
        }
    }

    uint16_t route_count_; // 可选路线数。
    uint16_t requested_; // 等待下一帧生效的路线。
    uint16_t active_; // 本帧使用的路线。
    uint32_t switch_count_{0U}; // 累积切换次数，允许回绕。
    uint32_t generation_{0U}; // 路径版本，允许回绕。
};

// 单节点条件；父节点条件在执行时一并检查。
struct BranchCondition {
    BranchSelector* selector{nullptr}; // 外部静态选择器，空表示无附加条件。
    uint16_t route{0U}; // 此节点所属路线。
    int16_t parent{-1}; // 展开图中父节点下标，-1 表示根节点。
};

// 每帧调度状态；条件跳过与节点自身故障分开记录。
enum class NodeFrameState : uint8_t {
    Pending = 0, // 本帧尚未处理。
    Executed, // 本帧正常、降级或保持输出。
    SkippedBranch, // 条件未选中。
    SkippedState, // 节点未运行或已禁用。
    Blocked, // 本帧前置输入未满足。
    NoData, // 节点运行但未提供新结果。
    Faulted // 本帧处理故障。
};

// 固定容量执行统计，计数允许自然回绕。
struct NodeExecutionStats {
    uint32_t execution_count{0U}; // process 调用次数。
    uint32_t branch_skip_count{0U}; // 条件跳过次数。
    uint32_t blocked_count{0U}; // 依赖或运行状态阻塞次数。
    NodeFrameState frame_state{NodeFrameState::Pending}; // 最近帧调度状态。
};

} // namespace robot::framework
#endif // ROBOT_BRANCH_SELECTOR_HPP
