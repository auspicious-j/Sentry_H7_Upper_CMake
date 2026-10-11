#include "BranchDemoNode.hpp"

/*
 * BranchDemoNode.cpp —— “分支演示”节点集合的实现（纯软件，无硬件输出）。
 *
 * 实现要点：所有节点都不访问硬件。公共输入每帧发布固定 10.0f；两条路线各自做仿射变换；
 *   汇合节点依据共享选择器的生效路线，仅转发对应路线本帧的结果。
 *   各节点的 process 在每次进入时都会先 invalidate() 旧输出，从而保证未生效路线 / 无新数据时
 *   不会把上一帧的旧值误传给下游。
 */

namespace robot::diagnostics {

// 构造：登记编号，并把输出写端绑定到内部信号，使 publish() 可直接落到 signal_。
// id：公共输入编号（演示固定为 41）。
DemoSourceNode::DemoSourceNode(robot::framework::PluginId id) : PluginNode(id)
{
    output_.bind(signal_);
}

// process —— 每帧向输出端口发布固定常量 10.0f，供所有路线复用同一数据源。
// context：当前帧上下文（提供 frame_id）。
// 返回：发布成功为 Ok，失败为 Fault。
robot::framework::ProcessResult DemoSourceNode::process(robot::framework::FrameContext& context)
{
    return output_.publish(context.frame_id, 10.0f)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

// onSkipped —— 本帧被跳过时调用：使信号失效，确保下游不保留上一轮的有效值。
// context：当前帧上下文（此处未使用）。
void DemoSourceNode::onSkipped(robot::framework::FrameContext& context)
{
    (void)context;
    signal_.invalidate();
}

// signal —— 返回只读的公共输入信号。
const robot::framework::FrameSignal<float>& DemoSourceNode::signal() const { return signal_; }

// 构造：登记编号与算术系数，并把输出写端绑定到内部信号。
// id/scale/offset：编号和算术系数（结果 = 输入 * scale + offset）。
DemoMathNode::DemoMathNode(robot::framework::PluginId id, float scale, float offset)
    : PluginNode(id), scale_(scale), offset_(offset)
{
    output_.bind(signal_);
}

// bind —— 组装期把输入端口指向同一执行域的上游信号。
// source：上游节点发布的输出信号。
void DemoMathNode::bind(const robot::framework::FrameSignal<float>& source) { input_.bind(source); }

// onEnter —— 激活（进入）通知：展示节点可自选重置策略，此处只清空“本次激活内”的调用计数，
//   不改动 scale_/offset_ 等算术参数。
// transition：状态迁移上下文（此处未使用）。
void DemoMathNode::onEnter(const robot::framework::NodeTransitionContext& transition)
{
    (void)transition;
    runs_since_enter_ = 0U;
}

// onExit —— 去激活（退出）通知：使输出失效，即使整图随后停止也不残留有效输出。
// transition：状态迁移上下文（此处未使用）。
void DemoMathNode::onExit(const robot::framework::NodeTransitionContext& transition)
{
    (void)transition;
    signal_.invalidate();
}

// runsSinceEnter —— 返回本次激活以来的调用计数；总执行计数由框架独立保存。
uint32_t DemoMathNode::runsSinceEnter() const { return runs_since_enter_; }

// process —— 每帧执行：先递增本次激活计数并失效旧输出；上游无新数据时直接放弃，
//   否则计算 输入 * scale_ + offset_ 并发布。
// context：当前帧上下文。
// 返回：Ok 成功；NoNewData 本帧上游无新数据；Fault 发布失败。
robot::framework::ProcessResult DemoMathNode::process(robot::framework::FrameContext& context)
{
    ++runs_since_enter_;
    signal_.invalidate();
    if (!input_.validForFrame(context.frame_id)) { return robot::framework::ProcessResult::NoNewData; }
    const float result = input_.read() * scale_ + offset_; // 本次结果。
    return output_.publish(context.frame_id, result)
        ? robot::framework::ProcessResult::Ok : robot::framework::ProcessResult::Fault;
}

// onSkipped —— 本帧被跳过时调用：未生效路线的结果失效。
// context：当前帧上下文（此处未使用）。
void DemoMathNode::onSkipped(robot::framework::FrameContext& context)
{
    (void)context;
    signal_.invalidate();
}

// signal —— 返回只读的结果信号。
const robot::framework::FrameSignal<float>& DemoMathNode::signal() const { return signal_; }

// 构造：登记容器编号，并按“先加偏置、后乘系数”的顺序构造两个串行子节点。
// id：容器编号；offset/scale：两个阶段各自的系数（子节点占用 id+1、id+2）。
DemoRouteNode::DemoRouteNode(robot::framework::PluginId id, float offset, float scale)
    : PluginNode(id), first_(id + 1U, 1.0f, offset), second_(id + 2U, scale, 0.0f) {}

// compose —— 启动期组装子图：先加入两个子节点，再让第二阶段读取第一阶段结果并建立串行依赖；
//   运行期由根计划按依赖顺序执行。
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

// process —— 每帧执行：容器不参与运算（避免重复执行子图），直接返回 Ok。
// context：当前帧上下文（此处未使用）。
robot::framework::ProcessResult DemoRouteNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    return robot::framework::ProcessResult::Ok;
}

// first —— 返回加偏置阶段节点。
DemoMathNode& DemoRouteNode::first() { return first_; }
// second —— 返回乘系数阶段节点。
DemoMathNode& DemoRouteNode::second() { return second_; }

// 构造：创建演示根容器，固定编号 40。
BranchDemoNode::BranchDemoNode() : PluginNode(40U) {}

// compose —— 组装演示图：
//   1) 依次加入公共输入、两条路线容器与汇合节点；
//   2) 把两条路线分别挂到共享选择器的 0/1 号分支；
//   3) 把两条路线首节点接到公共输入，末端结果接入汇合节点；
//   4) 补建依赖关系（含分支依赖），使调度顺序正确。
//   两条预定义路线各自独立组装，不修改既有业务的输入输出。
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
    // 下列依赖的端点位于稍后展开的子图内；其归属关系统一由 compile 阶段验证。
    status = graph.addDependency(source_, route_a_.first());
    if (status == robot::framework::PluginStatus::Ok) { status = graph.addDependency(source_, route_b_.first()); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.addBranchDependency(route_a_.second(), merge_); }
    if (status == robot::framework::PluginStatus::Ok) { status = graph.addBranchDependency(route_b_.second(), merge_); }
    return status;
}

// process —— 每帧执行：根节点不重复执行业务（实际算术在子图），直接返回 Ok。
// context：当前帧上下文（此处未使用）。
robot::framework::ProcessResult BranchDemoNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    return robot::framework::ProcessResult::Ok;
}

// request —— 提出下一帧生效路线（0=A、1=B），仅由应用任务在帧间调用；返回是否受理。
bool BranchDemoNode::request(uint32_t route) { return selector_.request(route); }
// selector —— 返回锁存后的选择器状态（只读）。
const robot::framework::BranchSelector& BranchDemoNode::selector() const { return selector_; }
// source —— 返回公共输入节点（只读）。
const DemoSourceNode& BranchDemoNode::source() const { return source_; }
// route —— 按编号返回路线容器：0=A，其他=B。
DemoRouteNode& BranchDemoNode::route(uint16_t route) { return route == 0U ? route_a_ : route_b_; }
// merge —— 返回汇合节点（只读）。
const robot::framework::BranchMergeNode<float, 2U>& BranchDemoNode::merge() const { return merge_; }

} // namespace robot::diagnostics
