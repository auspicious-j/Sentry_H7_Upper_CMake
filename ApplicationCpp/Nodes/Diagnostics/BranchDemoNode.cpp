#include "BranchDemoNode.hpp"

namespace robot::diagnostics {

// id：公共输入编号。
DemoSourceNode::DemoSourceNode(robot::framework::PluginId id) : PluginNode(id)
{
    output_.bind(signal_);
}

// context：当前帧；所有路线复用同一输入。
robot::framework::ProcessResult DemoSourceNode::process(robot::framework::FrameContext& context)
{
    return output_.publish(context.frame_id, 10.0f)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

// context：跳过帧；不保留上轮有效值。
void DemoSourceNode::onSkipped(robot::framework::FrameContext& context)
{
    (void)context;
    signal_.invalidate();
}

// 返回只读信号。
const robot::framework::FrameSignal<float>& DemoSourceNode::signal() const { return signal_; }

// id/scale/offset：编号和算术系数。
DemoMathNode::DemoMathNode(robot::framework::PluginId id, float scale, float offset)
    : PluginNode(id), scale_(scale), offset_(offset)
{
    output_.bind(signal_);
}

// source：同执行域上游信号。
void DemoMathNode::bind(const robot::framework::FrameSignal<float>& source) { input_.bind(source); }

// context：当前帧；无输入时不沿用旧结果。
robot::framework::ProcessResult DemoMathNode::process(robot::framework::FrameContext& context)
{
    signal_.invalidate();
    if (!input_.validForFrame(context.frame_id)) { return robot::framework::ProcessResult::NoNewData; }
    const float result = input_.read() * scale_ + offset_; // 本次结果。
    return output_.publish(context.frame_id, result)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

// context：跳过帧；未选路线结果失效。
void DemoMathNode::onSkipped(robot::framework::FrameContext& context)
{
    (void)context;
    signal_.invalidate();
}

// 返回只读结果。
const robot::framework::FrameSignal<float>& DemoMathNode::signal() const { return signal_; }

// id：容器编号；offset/scale：两个阶段系数。
DemoRouteNode::DemoRouteNode(robot::framework::PluginId id, float offset, float scale)
    : PluginNode(id), first_(id + 1U, 1.0f, offset), second_(id + 2U, scale, 0.0f) {}

// 子图在启动期组装，运行期由根计划执行。
robot::framework::PluginStatus DemoRouteNode::compose()
{
    robot::framework::PluginStatus status = children().add(first_); // 组装返回值。
    if (status == robot::framework::PluginStatus::Ok) { status = children().add(second_); }
    if (status == robot::framework::PluginStatus::Ok) {
        second_.bind(first_.signal());
        status = children().addDependency(first_, second_);
    }
    return status;
}

// context：当前帧；避免重复执行子图。
robot::framework::ProcessResult DemoRouteNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    return robot::framework::ProcessResult::Ok;
}

// 返回加偏置阶段。
DemoMathNode& DemoRouteNode::first() { return first_; }
// 返回乘系数阶段。
DemoMathNode& DemoRouteNode::second() { return second_; }

// 创建演示根节点。
BranchDemoNode::BranchDemoNode() : PluginNode(40U) {}

// 两条预定义路线，不修改旧业务的输入输出。
robot::framework::PluginStatus BranchDemoNode::compose()
{
    auto& graph = children(); // 演示内部图。
    robot::framework::PluginStatus status = graph.add(source_); // 组装状态。
    if (status == robot::framework::PluginStatus::Ok) { status = graph.add(route_a_); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.add(route_b_); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.add(merge_); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.setBranch(route_a_, selector_, 0U); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.setBranch(route_b_, selector_, 1U); }
    if (status != robot::framework::PluginStatus::Ok) { return status; }
    route_a_.first().bind(source_.signal());
    route_b_.first().bind(source_.signal());
    if (!merge_.bindInput(0U, route_a_.second().signal()) || !merge_.bindInput(1U, route_b_.second().signal())) {
        return robot::framework::PluginStatus::ConfigurationFault;
    }
    // 端点位于稍后展开的子图；compile 统一验证归属。
    status = graph.addDependency(source_, route_a_.first());
    if (status == robot::framework::PluginStatus::Ok) { status = graph.addDependency(source_, route_b_.first()); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.addBranchDependency(route_a_.second(), merge_); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.addBranchDependency(route_b_.second(), merge_); }
    return status;
}

// context：当前帧；根节点不重复执行业务。
robot::framework::ProcessResult BranchDemoNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    return robot::framework::ProcessResult::Ok;
}

// route：下一帧请求。
bool BranchDemoNode::request(uint32_t route) { return selector_.request(route); }
// 返回锁存选择状态。
const robot::framework::BranchSelector& BranchDemoNode::selector() const { return selector_; }
// 返回公共输入节点。
const DemoSourceNode& BranchDemoNode::source() const { return source_; }
// route：0=A、1=B。
DemoRouteNode& BranchDemoNode::route(uint16_t route) { return route == 0U ? route_a_ : route_b_; }
// 返回汇合节点。
const robot::framework::BranchMergeNode<float, 2U>& BranchDemoNode::merge() const { return merge_; }

} // namespace robot::diagnostics
