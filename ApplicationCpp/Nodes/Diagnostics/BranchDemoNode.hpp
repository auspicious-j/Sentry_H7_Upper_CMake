#ifndef ROBOT_BRANCH_DEMO_NODE_HPP
#define ROBOT_BRANCH_DEMO_NODE_HPP

#include "Framework/BranchMergeNode.hpp"

namespace robot::diagnostics {

// 公共输入节点，不访问硬件。
class DemoSourceNode final : public robot::framework::PluginNode {
public:
    // id：图内唯一编号。
    explicit DemoSourceNode(robot::framework::PluginId id);
    // context：当前帧；发布固定输入 10。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // context：跳过帧；使输出失效。
    void onSkipped(robot::framework::FrameContext& context) override;
    // 返回公共输入信号。
    const robot::framework::FrameSignal<float>& signal() const;
private:
    robot::framework::FrameSignal<float> signal_{}; // 信号存储。
    robot::framework::OutputPort<float> output_{}; // 输出写端。
};

// 算术演示节点：input * scale + offset。
class DemoMathNode final : public robot::framework::PluginNode {
public:
    // id：编号；scale/offset：演示系数。
    DemoMathNode(robot::framework::PluginId id, float scale, float offset);
    // source：组装期上游信号。
    void bind(const robot::framework::FrameSignal<float>& source);
    // transition：进入时只重置演示计数，不修改算术参数。
    void onEnter(const robot::framework::NodeTransitionContext& transition) override;
    // transition：退出时使本节点输出失效。
    void onExit(const robot::framework::NodeTransitionContext& transition) override;
    // 返回本次激活期间的 process 调用次数。
    uint32_t runsSinceEnter() const;
    // context：当前帧；拒绝上轮数据。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // context：跳过帧；清除有效性。
    void onSkipped(robot::framework::FrameContext& context) override;
    // 返回输出信号。
    const robot::framework::FrameSignal<float>& signal() const;
private:
    float scale_; // 乘法系数。
    float offset_; // 加法偏置。
    uint32_t runs_since_enter_{0U}; // 本次激活以来调用次数，仅进入时清零。
    robot::framework::InputPort<float> input_{}; // 算术输入。
    robot::framework::FrameSignal<float> signal_{}; // 结果存储。
    robot::framework::OutputPort<float> output_{}; // 结果写端。
};

// 路线内部继续有图；两个子节点继承父节点条件。
class DemoRouteNode final : public robot::framework::PluginNode {
public:
    // id：容器编号；子节点用 id+1/id+2；offset/scale：两级系数。
    DemoRouteNode(robot::framework::PluginId id, float offset, float scale);
    // 注册内部节点和依赖。
    robot::framework::PluginStatus compose() override;
    // context：当前帧；容器不重复调用子节点。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // 返回第一阶段节点。
    DemoMathNode& first();
    // 返回第二阶段节点。
    DemoMathNode& second();
private:
    DemoMathNode first_; // 加偏置阶段。
    DemoMathNode second_; // 乘系数阶段。
};

// 框架分支演示根节点，与业务节点使用同一接口。
class BranchDemoNode final : public robot::framework::PluginNode {
public:
    // 固定编号：40/41/50..52/60..62/70。
    BranchDemoNode();
    // 组装公共输入、两条子图及汇合。
    robot::framework::PluginStatus compose() override;
    // context：当前帧；实际算术由子图执行。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // route：下一帧路线 0=A、1=B，只由应用任务调用。
    bool request(uint32_t route);
    // 返回选择器状态。
    const robot::framework::BranchSelector& selector() const;
    // 返回公共节点用于统计。
    const DemoSourceNode& source() const;
    // route：应用仅传入 0 或 1。
    DemoRouteNode& route(uint16_t route);
    // 返回汇合节点用于观察。
    const robot::framework::BranchMergeNode<float, 2U>& merge() const;
private:
    robot::framework::BranchSelector selector_{2U}; // 默认路线 A。
    DemoSourceNode source_{41U}; // 每帧执行一次的公共输入。
    DemoRouteNode route_a_{50U, 1.0f, 2.0f}; // A=(10+1)*2=22。
    DemoRouteNode route_b_{60U, -1.0f, 3.0f}; // B=(10-1)*3=27。
    robot::framework::BranchMergeNode<float, 2U> merge_{70U, selector_}; // 汇合输出。
};

} // namespace robot::diagnostics
#endif // ROBOT_BRANCH_DEMO_NODE_HPP
