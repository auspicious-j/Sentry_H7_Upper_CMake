/*
 * ChassisFeedbackNode.cpp —— 底盘反馈节点的实现（请对照 ChassisFeedbackNode.hpp 阅读）。
 *
 * 实现只有三处：构造（绑定输出端口）、signal()（暴露只读视图）、process()（每帧读源
 * 并发布）。数据源在构造时注入，本文件不含任何底盘解算/控制逻辑，只做数据搬运。
 */

#include "ChassisFeedbackNode.hpp"

namespace robot::chassis {

// 构造函数实现：初始化基类与数据源引用，并把输出端口绑定到内部信号存储。
// id     ：转交给基类 PluginNode 的节点编号（全图唯一即可，与插件嵌套层级无关）。
// source ：外部注入的反馈数据源，直接以引用保存（不拷贝、不接管其生命周期）。
// 关键一步：output_.bind(signal_) 让输出端口 output_ 指向内部存储 signal_，
//           这样之后每次 publish 都会写入 signal_，下游才能读到。
ChassisFeedbackNode::ChassisFeedbackNode(robot::framework::PluginId id,
                                       IChassisFeedbackSource& source)
    : PluginNode(id), source_(source)
{
    output_.bind(signal_); // 让输出端口指向本节点的信号存储，建立「发布落点」。
}

// 返回内部信号存储的只读引用，作为下游输入端口的数据来源（只读视图）。
const robot::framework::FrameSignal<ChassisFeedback>& ChassisFeedbackNode::signal() const
{
    return signal_;
}

// 每帧处理：从数据源读取一份稳定快照，并用本帧帧号发布出去。
// 说明：接收计数 receive_count 与数据年龄 age_ms 由数据源内部维护，本节点只搬运，
//       因此即使在「保持」态，接收计数与年龄仍反映数据源的真实值，不会被帧号污染。
// context：当前帧上下文；取其中的 frame_id 作为本次发布的帧号。
// 返回：发布成功（端口已绑定）→ Ok；发布失败（端口未绑定）→ Fault。
robot::framework::ProcessResult ChassisFeedbackNode::process(robot::framework::FrameContext& context)
{
    const ChassisFeedback feedback = source_.read(); // 本帧稳定输入：向数据源索要一份快照。
    // 以本帧帧号发布该快照：成功则本节点本帧输出有效，失败（signal_ 未绑定）则报故障。
    return output_.publish(context.frame_id, feedback)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

} // namespace robot::chassis
