#ifndef ROBOT_EXECUTION_PLAN_DEMO_NODE_HPP
#define ROBOT_EXECUTION_PLAN_DEMO_NODE_HPP

#include "Framework/PluginNode.hpp"

namespace robot::diagnostics {

// 无硬件操作的顺序探针；次数和执行序号由框架记录。
class PlanProbeNode final : public robot::framework::PluginNode {
public:
    // id：根图中唯一编号。
    explicit PlanProbeNode(robot::framework::PluginId id) : PluginNode(id) {}
    // context：当前帧；探针只证明该节点被执行。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override
    {
        (void)context;
        return robot::framework::ProcessResult::Ok;
    }
};

// 两节点同一份实例，计划0 A→B，计划1 B→A。
class ExecutionPlanDemoNode final : public robot::framework::PluginNode {
public:
    // 创建演示根节点，编号80，内部使用81和82。
    ExecutionPlanDemoNode();
    // 向内部图声明两套计划的顺序边。
    robot::framework::PluginStatus compose() override;
    // context：当前帧；内部节点由根执行器统一执行。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // 返回探针A用于统计。
    const PlanProbeNode& a() const;
    // 返回探针B用于统计。
    const PlanProbeNode& b() const;
private:
    PlanProbeNode a_{81U}; // 顺序探针A。
    PlanProbeNode b_{82U}; // 顺序探针B。
};

} // namespace robot::diagnostics
#endif // ROBOT_EXECUTION_PLAN_DEMO_NODE_HPP
