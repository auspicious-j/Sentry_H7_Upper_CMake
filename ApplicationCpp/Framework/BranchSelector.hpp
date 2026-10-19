#ifndef robot_branch_selector_hpp
#define robot_branch_selector_hpp

#include "PluginTypes.hpp"

/*
 * branch_selector.hpp —— 分支选择器（selector）及相关调度类型。
 *
 * 术语速览：
 *   分支(branch)：一条可被独立选中或跳过的执行路径。
 *   分支编号(branch_id) ：分支编号，从 0 开始；用于在汇合节点里定位对应输入端口。
 *   选择器(selector)：持有“当前生效分支编号”的对象；同一个选择器会被上游分支与
 *                     下游汇合节点共享，从而保证上下游对“本帧走哪条分支”判断一致。
 *   汇合(merge)：把多条分支的输出收拢为单一输出，只转发本帧被选中分支的结果。
 *
 * 本文件不提供任何跨线程同步：所有成员只允许由所属执行器任务（同一任务上下文）访问。
 */

namespace robot::framework {

class plugin_graph;

/*
 * 分支编号类
 * @ bool request(uint32_t branch_id)
 * @ bool all_is_valid() const 
 * @ bool branch_id_is_valid(uint16_t branch_id) const 
 */
class branch_selector {
public:
    /*
     * 构造。
     * 参数：
     *   branch_count   分支编号总数（应大于 0，否则 all_is_valid() 恒为 false）。
     *   initial_branch_id 初始分支编号编号，从 0 开始，默认 0。
     * 副作用：requested_id_ 与 active_id_ 均被初始化为 initial_branch_id。
     */
    explicit branch_selector(uint16_t branch_count, uint16_t initial_branch_id = 0U)
        : branch_count_(branch_count), requested_id_(initial_branch_id), active_id_(initial_branch_id) 
        {}

    /*
     * @brief 设置下一帧分支编号
     * @param branch_id：目标分支编号
     * @return  true :表示请求已受理并写入 requested_id_
     *          false :表示未受理。
     */
    bool request(uint32_t branch_id)
    {
        // 检查类私有变量和新id合法性
        if (!all_is_valid() || branch_id >= branch_count_)
        {
            return false;
        }

        requested_id_ = static_cast<uint16_t>(branch_id);
        return true;
    }

    // 判断当前branch_count_、requested_id_、active_三参数均合法，返回1
    bool all_is_valid() const 
    { 
        return branch_count_ != 0U && requested_id_ < branch_count_ && active_id_ < branch_count_; 
    }
    // branch_id_is_valid —— 判断编号 branch_id 是否为合法分支编号（即 branch_id 小于分支编号总数）。
    bool branch_id_is_valid(uint16_t branch_id) const 
    { 
        return branch_id < branch_count_; 
    }

    uint16_t branch_count() const { return branch_count_; }
    uint16_t active_branch_id() const { return active_id_; }
    uint32_t switch_count() const { return switch_count_; }

private:
    friend class plugin_graph;
    // latch —— 帧开始时由执行器调用一次：把 requested_id_ 锁存为 active_id_；
    //          若发生变化则累加切换次数。不在节点运行过程中修改 active_id_。
    void latch()
    {
        if (active_id_ != requested_id_)
        {
            active_id_ = requested_id_;
            ++switch_count_;
        }
    }

    uint16_t branch_count_; // 可选分支编号总数。
    uint16_t requested_id_; // 请求分支编号：等待下一帧 latch() 生效。
    uint16_t active_id_; // 本帧分支branch
    uint32_t switch_count_{0U}; // 累积切换次数，允许自然回绕（uint32 溢出）。
};

// 单个节点的所属分支的性质
struct branch_condition 
{
    branch_selector* selector{nullptr}; // 分支的地址
    uint16_t branch_id{0U}; // 在分支里的序号
    int16_t parent{-1}; // 展开图中父节点下标；-1 表示根节点（无父节点）。
};

// 节点此时的运行状态
enum class node_frame_state : uint8_t 
{
    pending = 0, // 本帧尚未处理。
    executed, // 本帧正常执行、降级执行或保持输出。
    skipped_branch, // 因条件未选中而跳过（分支级跳过）。
    skipped_state, // 节点未运行或已被禁用（状态级跳过）。
    blocked, // 本帧前置输入未满足（依赖阻塞）。
    no_data, // 节点已运行但未提供新结果。
    faulted // 本帧处理过程中发生故障。
};

/*
 * node_execution_stats —— 固定容量执行统计。
 * 用途：记录单个节点的运行计数、跳过计数与最近帧的调度状态。
 * 说明：各计数为 uint32，允许自然回绕；不参与跨线程同步。
 */
struct node_execution_stats 
{
    uint32_t execution_count{0U}; // process 调用次数（累计）。
    uint32_t branch_skip_count{0U}; // 因条件未选中被跳过的累计次数。
    uint32_t blocked_count{0U}; // 因依赖未满足或运行状态阻塞的累计次数。
    node_frame_state frame_state{node_frame_state::pending}; // 最近一帧的调度状态，默认 pending。
    uint16_t frame_order{0xFFFFU}; // 本帧内 process 调用序号；未执行时为 0xFFFF。
    uint32_t enter_count{0U}; // 累积进入通知次数。
    uint32_t exit_count{0U}; // 累积退出通知次数。
    uint32_t plan_change_count{0U}; // 保持激活状态下，计划发生切换的通知次数。
    bool is_active{false}; // 帧边界锁存的激活状态；为 true 不代表输入已就绪。
};

} // namespace robot::framework
#endif // robot_branch_selector_hpp
