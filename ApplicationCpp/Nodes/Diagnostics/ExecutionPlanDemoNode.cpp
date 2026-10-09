#include "ExecutionPlanDemoNode.hpp"

namespace robot::diagnostics {

// 创建无硬件输出的演示根节点。
ExecutionPlanDemoNode::ExecutionPlanDemoNode() : PluginNode(80U) {}

// 计划编号属于根图；根图必须先声明两个计划。
robot::framework::PluginStatus ExecutionPlanDemoNode::compose()
{
    robot::framework::PluginStatus status = children().add(a_); // 当前组装结果。
    if (status == robot::framework::PluginStatus::Ok) { status = children().add(b_); }
    if (status == robot::framework::PluginStatus::Ok) {
        status = children().addPlanDependency(a_, b_, 0U);
    }
    if (status == robot::framework::PluginStatus::Ok) {
        status = children().addPlanDependency(b_, a_, 1U);
    }
    return status;
}

// context：当前帧；容器不重复执行内部探针。
robot::framework::ProcessResult ExecutionPlanDemoNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    return robot::framework::ProcessResult::Ok;
}

// 返回探针A。
const PlanProbeNode& ExecutionPlanDemoNode::a() const { return a_; }
// 返回探针B。
const PlanProbeNode& ExecutionPlanDemoNode::b() const { return b_; }

} // namespace robot::diagnostics
