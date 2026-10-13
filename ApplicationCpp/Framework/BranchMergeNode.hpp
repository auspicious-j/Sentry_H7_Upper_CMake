#ifndef robot_branch_merge_node_hpp
#define robot_branch_merge_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "BranchSelector.hpp"

/*
 * branch_merge_node.hpp —— 分支汇合节点（merge）。
 *
 * 术语速览：
 *   分支(branch)：一条可被独立选中或跳过的执行路径。
 *   分支编号(branch_id) ：分支编号，从 0 开始。
 *   选择器(selector)：决定本帧“生效分支编号”的对象，本节点与上游分支共享同一个选择器。
 *   汇合(merge)：把多条分支末端的输出收拢为单一输出，且只转发本帧被选中分支的结果。
 *
 * 用途：位于多分支子图末端。各上游分支各自产出同类型结果 t，本节点在每一帧
 *       根据共享 selector 的 active_branch_id()，仅把对应分支的本帧结果转发到唯一
 *       输出端口 output_，从而完成“多路合一”。
 *
 * 线程模型：与其余节点一致，process/on_skipped 只由所属执行器任务在帧内调用。
 */

namespace robot::framework {

/*
 * branch_merge_node —— 模板参数：
 *   t          各分支与本节点共用的输出类型（所有分支结果必须同类型）。
 *   branch_count 固定分支编号数量（编译期常量，必须大于 0，见下方 static_assert）。
 * 继承 plugin_node，是插件图中的一个执行节点。
 */
template <typename t, uint16_t branch_count>
class branch_merge_node final : public plugin_node {
public:
    /*
     * 构造。
     * 参数：
     *   id       本节点编号。
     *   selector 与上游分支共享的分支选择器（按引用持有，生命周期由外部保证）。
     * 副作用：把唯一输出端口 output_ 绑定到内部信号 signal_。
     */
    branch_merge_node(plugin_id id, branch_selector& selector) : plugin_node(id), selector_(selector)
    {
        output_.bind(signal_);
    }
    /*
     * bind_input —— 组装期（configure 之前）绑定某条分支编号末端的输出信号。
     * 参数：
     *   branch_id  目标分支编号编号，必须小于 branch_count。
     *   source 该分支编号末端节点提供的输出信号。
     * 返回：true 绑定成功；false 表示节点已不在 created 状态或 branch_id 越界。
     * 副作用：成功后 inputs_[branch_id] 指向 source。
     */
    bool bind_input(uint16_t branch_id, const frame_signal<t>& source)
    {
        if (state() != plugin_state::created || branch_id >= branch_count)
        {
            return false;
        }
        inputs_[branch_id].bind(source);
        return true;
    }
    /*
     * configure —— 组装期校验。
     * 返回：plugin_status::configuration_fault 表示配置错误；否则返回父类 configure() 的结果。
     * 校验内容：选择器已配置且其分支编号数与 branch_count 一致；并且每条分支编号的输入都已连接。
     */
    plugin_status configure() override
    {
        if (!selector_.all_is_valid() || selector_.branch_count() != branch_count)
        {
            return plugin_status::configuration_fault;
        }
        // branch_id：当前正在检查的输入下标（取值 0 .. branch_count-1）。
        for (uint16_t branch_id = 0U; branch_id < branch_count; ++branch_id)
        {
            if (!inputs_[branch_id].is_connected())
            {
                return plugin_status::configuration_fault;
            }
        }
        return plugin_node::configure();
    }
    /*
     * process —— 每帧执行：仅转发本帧被选中分支编号的结果。
     * 参数 context：当前帧上下文（提供 frame_id 等）。
     * 返回：
     *   ok        成功发布选中分支编号的本帧结果；
     *   no_new_data 分支编号越界，或选中分支编号的输入对本帧无效（无新数据）；
     *   fault     发布失败。
     * 副作用：无条件先 invalidate() 失效内部信号；若成功发布则信号被置为有效。
     */
    process_result process(frame_context& context) override
    {
        signal_.invalidate();
        const uint16_t branch_id = selector_.active_branch_id(); // 本帧锁存的生效分支编号。
        if (branch_id >= branch_count || !inputs_[branch_id].is_valid_for_frame(context.frame_id))
        {
            return process_result::no_new_data;
        }
        return output_.publish(context.frame_id, inputs_[branch_id].read())
            ? process_result::ok : process_result::fault;
    }
    /*
     * on_skipped —— 本帧被跳过时调用。
     * 参数 context：当前帧上下文（此处未使用）。
     * 副作用：invalidate() 内部信号，禁止残留上一次有效输出造成“旧值被误读”。
     */
    void on_skipped(frame_context& context) override
    {
        (void)context;
        signal_.invalidate();
    }
    // signal —— 返回只读的汇合信号（供本节点下游或其他观察者读取）。
    const frame_signal<t>& signal() const { return signal_; }
private:
    static_assert(branch_count > 0U, "At least one branch input is required");
    branch_selector& selector_; // 外部共享的分支选择器引用。
    input_port<t> inputs_[branch_count]{}; // 每条分支编号一个输入端口，下标即分支编号编号。
    frame_signal<t> signal_{}; // 汇合结果存储；被 output_ 绑定。
    output_port<t> output_{}; // 唯一结果写端。
};

} // namespace robot::framework
#endif // robot_branch_merge_node_hpp
