#ifndef ROBOT_BRANCH_SELECTOR_HPP
#define ROBOT_BRANCH_SELECTOR_HPP

#include "PluginTypes.hpp"

/*
 * BranchSelector.hpp —— 分支选择器（selector）及相关调度类型。
 *
 * 术语速览：
 *   分支(branch)：一条可被独立选中或跳过的执行路径。
 *   路线(route) ：分支编号，从 0 开始；用于在汇合节点里定位对应输入端口。
 *   选择器(selector)：持有“当前生效路线”的对象；同一个选择器会被上游分支与
 *                     下游汇合节点共享，从而保证上下游对“本帧走哪条分支”判断一致。
 *   汇合(merge)：把多条分支的输出收拢为单一输出，只转发本帧被选中分支的结果。
 *
 * 本文件不提供任何跨线程同步：所有成员只允许由所属执行器任务（同一任务上下文）访问。
 */

namespace robot::framework {

class PluginGraph;

/*
 * BranchSelector —— 固定路线的选择器。
 *
 * 用途：分别记录“请求路线”与“生效路线”，并在每个帧边界把请求锁存为生效，
 *       供上游分支（判断是否执行）与下游汇合节点（判断读哪一路输入）共同查询。
 *
 * 线程模型：非线程安全；只能由所属执行器任务访问，不负责跨线程同步。
 *
 * 生命周期：构造 → 每帧由执行器调用 latch() 锁存 → request() 提新请求 → 查询。
 */
class BranchSelector {
public:
    /*
     * 构造。
     * 参数：
     *   route_count   路线总数（应大于 0，否则 configured() 恒为 false）。
     *   initial_route 初始路线编号，从 0 开始，默认 0。
     * 副作用：requested_ 与 active_ 均被初始化为 initial_route。
     */
    explicit BranchSelector(uint16_t route_count, uint16_t initial_route = 0U)
        : route_count_(route_count), requested_(initial_route), active_(initial_route) {}

    /*
     * request —— 提出“下一帧希望切换到”的路线。
     * 参数 route：目标路线编号（用 uint32_t 便于与外部无符号量直接比较）。
     * 返回：true 表示请求已受理并写入 requested_；false 表示未受理。
     * 副作用：受理时改写 requested_；非法编号或选择器未配置时保留原选择（不改动任何成员）。
     * 调用时机：任意时刻均可（通常在帧间提出），真正切换发生在下一次 latch()。
     */
    bool request(uint32_t route)
    {
        if (!configured() || route >= route_count_) {
            return false;
        }
        requested_ = static_cast<uint16_t>(route);
        return true;
    }

    // configured —— 判断构造参数是否有效：路线数非 0，且请求路线与生效路线都在范围内。
    bool configured() const { return route_count_ != 0U && requested_ < route_count_ && active_ < route_count_; }
    // contains —— 判断编号 route 是否为合法路线（即 route 小于路线总数）。
    bool contains(uint16_t route) const { return route < route_count_; }
    // routeCount —— 返回固定路线数量。
    uint16_t routeCount() const { return route_count_; }
    // activeRoute —— 返回本帧锁存的生效路线（上游/下游据此判断本帧走哪条分支）。
    uint16_t activeRoute() const { return active_; }
    // switchCount —— 返回实际发生的路线切换次数（累积值，允许自然回绕）。
    uint32_t switchCount() const { return switch_count_; }
    // generation —— 返回路径版本，仅在路线真正改变时递增（累积值，允许自然回绕）。
    uint32_t generation() const { return generation_; }

private:
    friend class PluginGraph;
    // latch —— 帧开始时由执行器调用一次：把 requested_ 锁存为 active_；
    //          若发生变化则累加切换次数与路径版本。不在节点运行过程中修改 active_。
    void latch()
    {
        if (active_ != requested_) {
            active_ = requested_;
            ++switch_count_;
            ++generation_;
        }
    }

    uint16_t route_count_; // 可选路线总数。
    uint16_t requested_; // 请求路线：等待下一帧 latch() 生效。
    uint16_t active_; // 生效路线：本帧实际使用的路线。
    uint32_t switch_count_{0U}; // 累积切换次数，允许自然回绕（uint32 溢出）。
    uint32_t generation_{0U}; // 路径版本，路线每次改变递增一次，允许自然回绕。
};

/*
 * BranchCondition —— 单个节点的分支条件。
 * 含义：描述某节点属于哪条路线（route），以及它在展开图中的父节点位置（parent）。
 * 说明：父节点的条件会在执行该节点时一并检查，只有父节点所在路线的条件也满足，
 *       本节点才可能被执行。
 */
struct BranchCondition {
    BranchSelector* selector{nullptr}; // 外部静态选择器指针；nullptr 表示该节点无附加条件（不参与路线筛选）。
    uint16_t route{0U}; // 此节点所属路线编号（从 0 开始）。
    int16_t parent{-1}; // 展开图中父节点下标；-1 表示根节点（无父节点）。
};

/*
 * NodeFrameState —— 单帧调度状态。
 * 用途：把“因条件未选中被跳过”与“节点自身故障”分开记录，便于调试与统计。
 * 各取值含义见下方枚举项注释。
 */
enum class NodeFrameState : uint8_t {
    Pending = 0, // 本帧尚未处理。
    Executed, // 本帧正常执行、降级执行或保持输出。
    SkippedBranch, // 因条件未选中而跳过（分支级跳过）。
    SkippedState, // 节点未运行或已被禁用（状态级跳过）。
    Blocked, // 本帧前置输入未满足（依赖阻塞）。
    NoData, // 节点已运行但未提供新结果。
    Faulted // 本帧处理过程中发生故障。
};

/*
 * NodeExecutionStats —— 固定容量执行统计。
 * 用途：记录单个节点的运行计数、跳过计数与最近帧的调度状态。
 * 说明：各计数为 uint32，允许自然回绕；不参与跨线程同步。
 */
struct NodeExecutionStats {
    uint32_t execution_count{0U}; // process 调用次数（累计）。
    uint32_t branch_skip_count{0U}; // 因条件未选中被跳过的累计次数。
    uint32_t blocked_count{0U}; // 因依赖未满足或运行状态阻塞的累计次数。
    NodeFrameState frame_state{NodeFrameState::Pending}; // 最近一帧的调度状态，默认 Pending。
    uint16_t frame_order{0xFFFFU}; // 本帧内 process 调用序号；未执行时为 0xFFFF。
    uint32_t enter_count{0U}; // 累积进入通知次数。
    uint32_t exit_count{0U}; // 累积退出通知次数。
    uint32_t plan_change_count{0U}; // 保持激活状态下，计划发生切换的通知次数。
    bool active{false}; // 帧边界锁存的激活状态；为 true 不代表输入已就绪。
};

} // namespace robot::framework
#endif // ROBOT_BRANCH_SELECTOR_HPP
