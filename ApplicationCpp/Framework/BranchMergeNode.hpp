#ifndef ROBOT_BRANCH_MERGE_NODE_HPP
#define ROBOT_BRANCH_MERGE_NODE_HPP

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "BranchSelector.hpp"

/*
 * BranchMergeNode.hpp —— 分支汇合节点（merge）。
 *
 * 术语速览：
 *   分支(branch)：一条可被独立选中或跳过的执行路径。
 *   路线(route) ：分支编号，从 0 开始。
 *   选择器(selector)：决定本帧“生效路线”的对象，本节点与上游分支共享同一个选择器。
 *   汇合(merge)：把多条分支末端的输出收拢为单一输出，且只转发本帧被选中分支的结果。
 *
 * 用途：位于多分支子图末端。各上游分支各自产出同类型结果 T，本节点在每一帧
 *       根据共享 selector 的 activeRoute()，仅把对应分支的本帧结果转发到唯一
 *       输出端口 output_，从而完成“多路合一”。
 *
 * 线程模型：与其余节点一致，process/onSkipped 只由所属执行器任务在帧内调用。
 */

namespace robot::framework {

/*
 * BranchMergeNode —— 模板参数：
 *   T          各分支与本节点共用的输出类型（所有分支结果必须同类型）。
 *   RouteCount 固定路线数量（编译期常量，必须大于 0，见下方 static_assert）。
 * 继承 PluginNode，是插件图中的一个执行节点。
 */
template <typename T, uint16_t RouteCount>
class BranchMergeNode final : public PluginNode {
public:
    /*
     * 构造。
     * 参数：
     *   id       本节点编号。
     *   selector 与上游分支共享的分支选择器（按引用持有，生命周期由外部保证）。
     * 副作用：把唯一输出端口 output_ 绑定到内部信号 signal_。
     */
    BranchMergeNode(PluginId id, BranchSelector& selector) : PluginNode(id), selector_(selector)
    {
        output_.bind(signal_);
    }
    /*
     * bindInput —— 组装期（configure 之前）绑定某条路线末端的输出信号。
     * 参数：
     *   route  目标路线编号，必须小于 RouteCount。
     *   source 该路线末端节点提供的输出信号。
     * 返回：true 绑定成功；false 表示节点已不在 Created 状态或 route 越界。
     * 副作用：成功后 inputs_[route] 指向 source。
     */
    bool bindInput(uint16_t route, const FrameSignal<T>& source)
    {
        if (state() != PluginState::Created || route >= RouteCount) { return false; }
        inputs_[route].bind(source);
        return true;
    }
    /*
     * configure —— 组装期校验。
     * 返回：PluginStatus::ConfigurationFault 表示配置错误；否则返回父类 configure() 的结果。
     * 校验内容：选择器已配置且其路线数与 RouteCount 一致；并且每条路线的输入都已连接。
     */
    PluginStatus configure() override
    {
        if (!selector_.configured() || selector_.routeCount() != RouteCount) {
            return PluginStatus::ConfigurationFault;
        }
        // route：当前正在检查的输入下标（取值 0 .. RouteCount-1）。
        for (uint16_t route = 0U; route < RouteCount; ++route) {
            if (!inputs_[route].connected()) { return PluginStatus::ConfigurationFault; }
        }
        return PluginNode::configure();
    }
    /*
     * process —— 每帧执行：仅转发本帧被选中路线的结果。
     * 参数 context：当前帧上下文（提供 frame_id 等）。
     * 返回：
     *   Ok        成功发布选中路线的本帧结果；
     *   NoNewData 路线越界，或选中路线的输入对本帧无效（无新数据）；
     *   Fault     发布失败。
     * 副作用：无条件先 invalidate() 失效内部信号；若成功发布则信号被置为有效。
     */
    ProcessResult process(FrameContext& context) override
    {
        signal_.invalidate();
        const uint16_t route = selector_.activeRoute(); // 本帧锁存的生效路线。
        if (route >= RouteCount || !inputs_[route].validForFrame(context.frame_id)) {
            return ProcessResult::NoNewData;
        }
        return output_.publish(context.frame_id, inputs_[route].read())
            ? ProcessResult::Ok : ProcessResult::Fault;
    }
    /*
     * onSkipped —— 本帧被跳过时调用。
     * 参数 context：当前帧上下文（此处未使用）。
     * 副作用：invalidate() 内部信号，禁止残留上一次有效输出造成“旧值被误读”。
     */
    void onSkipped(FrameContext& context) override
    {
        (void)context;
        signal_.invalidate();
    }
    // signal —— 返回只读的汇合信号（供本节点下游或其他观察者读取）。
    const FrameSignal<T>& signal() const { return signal_; }
private:
    static_assert(RouteCount > 0U, "At least one branch input is required");
    BranchSelector& selector_; // 外部共享的分支选择器引用。
    InputPort<T> inputs_[RouteCount]{}; // 每条路线一个输入端口，下标即路线编号。
    FrameSignal<T> signal_{}; // 汇合结果存储；被 output_ 绑定。
    OutputPort<T> output_{}; // 唯一结果写端。
};

} // namespace robot::framework
#endif // ROBOT_BRANCH_MERGE_NODE_HPP
