#ifndef ROBOT_BRANCH_DEMO_NODE_HPP
#define ROBOT_BRANCH_DEMO_NODE_HPP

#include "Framework/BranchMergeNode.hpp"

/*
 * BranchDemoNode.hpp —— 插件框架“分支演示”节点集合（纯软件，无任何硬件输出）。
 *
 * 演示目标：用一组不访问硬件的节点，展示框架的分支(branch)/路线(route)/选择器(selector)/汇合(merge)能力。
 *
 * 图结构（两条互斥路线 A/B，由同一个选择器决定本帧哪条激活）：
 *   公共输入 DemoSourceNode(41) 每帧发布常量 10.0f；
 *     ├── 路线 A：DemoRouteNode(50) = [加偏置(51)] -> [乘系数(52)]，即 (10 + 1) * 2 = 22；
 *     └── 路线 B：DemoRouteNode(60) = [加偏置(61)] -> [乘系数(62)]，即 (10 - 1) * 3 = 27；
 *   两条路线末端都接入汇合节点 BranchMergeNode(70)：仅转发本帧被选中路线的结果；
 *   根容器 BranchDemoNode(40) 负责把公共输入、两条子图与汇合节点组装成一张图。
 *
 * 关键概念：
 *   分支(branch)：一条可被独立选中或跳过的执行路径。
 *   路线(route) ：分支编号，从 0 开始；本演示 A=0、B=1。
 *   选择器(selector)：记录“下一帧请求路线”与“本帧生效路线”；上游分支与下游汇合共享同一实例，
 *                     以保证上下游对“本帧走哪条路线”的判断完全一致。
 *   汇合(merge)：把多条分支末端的同类型结果收拢为单一输出，只转发当前生效路线的那一路。
 *
 * 路由/汇合流程：应用任务在帧间调用 request(route) 提出“下一帧走 A 还是 B”；帧边界处选择器把
 *   请求锁存为生效路线；随后仅生效路线上的子节点被执行并产出结果，汇合节点读取该路线结果并
 *   发布到唯一输出端口。未生效路线本帧被跳过，其输出失效（不会残留旧值）。
 */

namespace robot::diagnostics {

// 公共输入节点：两条路线共享的同一数据源；纯软件，不访问硬件。
class DemoSourceNode final : public robot::framework::PluginNode {
public:
    // 构造。参数 id：图内唯一编号（本演示固定为 41）。
    explicit DemoSourceNode(robot::framework::PluginId id);
    // process —— 每帧被调用一次：发布固定常量 10.0f，供两条路线共同消费。
    //   参数 context：当前帧上下文（提供 frame_id 等）。
    //   返回：Ok 发布成功；Fault 发布失败（例如上一帧的有效数据尚未被消费完）。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // onSkipped —— 本帧被跳过时调用：使内部信号失效，避免下游读到上一帧的旧值。
    //   参数 context：当前帧上下文（本节点未使用）。
    void onSkipped(robot::framework::FrameContext& context) override;
    // signal —— 返回本节点发布的公共输入信号（供各路线首节点绑定读取）。
    //   返回：只读的信号引用。
    const robot::framework::FrameSignal<float>& signal() const;
private:
    robot::framework::FrameSignal<float> signal_{}; // 内部信号存储：承载本帧发布值及其有效性/帧号。
    robot::framework::OutputPort<float> output_{}; // 输出写端：已绑定 signal_，对外发布结果。
};

// 算术演示节点：对输入执行一次仿射变换 input * scale + offset，用于观察各路线系数差异。
class DemoMathNode final : public robot::framework::PluginNode {
public:
    // 构造。参数：
    //   id    图内唯一编号。
    //   scale 乘法系数（本节点结果 = 输入 * scale + offset）。
    //   offset 加法偏置。
    DemoMathNode(robot::framework::PluginId id, float scale, float offset);
    // bind —— 组装期绑定上游信号（同执行域内的数据源）。
    //   参数 source：上游节点发布的输出信号。
    void bind(const robot::framework::FrameSignal<float>& source);
    // onEnter —— 节点被激活（进入）时的通知：本演示只把“本次激活内的调用计数”清零，
    //   刻意不修改 scale_/offset_ 等算术参数，用以说明重置策略由节点自行决定。
    //   参数 transition：状态迁移上下文。
    void onEnter(const robot::framework::NodeTransitionContext& transition) override;
    // onExit —— 节点被去激活（退出）时的通知：使本节点输出失效。
    //   即使整图随即停止，也不残留上一帧的有效输出。
    //   参数 transition：状态迁移上下文。
    void onExit(const robot::framework::NodeTransitionContext& transition) override;
    // runsSinceEnter —— 返回“本次激活以来” process 的调用次数（进入时被清零）。
    //   返回：自最近一次 onEnter 起的调用次数（总执行计数另由框架独立保存）。
    uint32_t runsSinceEnter() const;
    // process —— 每帧执行：读取上游并计算 输入 * scale_ + offset_ 后发布。
    //   参数 context：当前帧上下文。
    //   返回：Ok 计算并发布成功；NoNewData 本帧上游无新数据；Fault 发布失败。
    //   注意：进入时先失效旧输出，故不会沿用上轮结果。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // onSkipped —— 本帧被跳过时调用：清空输出有效性（未生效路线不得残留旧结果）。
    //   参数 context：当前帧上下文（本节点未使用）。
    void onSkipped(robot::framework::FrameContext& context) override;
    // signal —— 返回本节点输出信号。
    //   返回：只读的信号引用。
    const robot::framework::FrameSignal<float>& signal() const;
private:
    float scale_; // 乘法系数：随构造参数固定，运行期不改变。
    float offset_; // 加法偏置：随构造参数固定，运行期不改变。
    uint32_t runs_since_enter_{0U}; // 本次激活以来的 process 调用次数；仅在 onEnter 清零（默认 0）。
    robot::framework::InputPort<float> input_{}; // 算术输入端口：由 bind() 指向上游信号。
    robot::framework::FrameSignal<float> signal_{}; // 计算结果存储：含数值与其有效性/帧号。
    robot::framework::OutputPort<float> output_{}; // 结果写端：已绑定 signal_，对外发布。
};

// 路线容器节点：一条路线内部再展开成一张子图（含两个串行阶段）。
// 两个子节点继承父节点的分支条件——只有本路线被选中时它们才会被执行。
class DemoRouteNode final : public robot::framework::PluginNode {
public:
    // 构造。参数：
    //   id     容器编号；内部两个子节点分别占用 id+1、id+2。
    //   offset 第一级（加偏置）阶段使用的偏置系数。
    //   scale  第二级（乘系数）阶段使用的乘法系数。
    DemoRouteNode(robot::framework::PluginId id, float offset, float scale);
    // compose —— 组装期调用：把内部两个子节点及其串行依赖注册进容器子图。
    //   返回：Ok 组装成功；其余为失败原因（见 PluginStatus）。
    robot::framework::PluginStatus compose() override;
    // process —— 每帧执行：容器本身不参与运算（避免重复执行子图），直接返回 Ok。
    //   参数 context：当前帧上下文（本节点未使用）。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // first —— 返回第一阶段（加偏置）节点，供外部绑定上游信号。
    DemoMathNode& first();
    // second —— 返回第二阶段（乘系数）节点，供外部接线到汇合节点。
    DemoMathNode& second();
private:
    DemoMathNode first_; // 第一阶段：加偏置（结果 = 输入 + offset）。
    DemoMathNode second_; // 第二阶段：乘系数（结果 = 第一阶段结果 * scale）。
};

// 分支演示根节点：作为整张演示图的容器，对外与业务节点使用完全相同的接口。
class BranchDemoNode final : public robot::framework::PluginNode {
public:
    // 构造。固定编号布局：
    //   40 = 根容器；41 = 公共输入；50..52 = 路线 A（容器/加偏置/乘系数）；
    //   60..62 = 路线 B；70 = 汇合节点。
    BranchDemoNode();
    // compose —— 组装期调用：注册公共输入、两条子图与汇合节点，设置分支关系并完成数据接线。
    //   返回：Ok 组装成功；其余为失败原因。
    robot::framework::PluginStatus compose() override;
    // process —— 每帧执行：根节点不重复执行业务（实际算术在子图），直接返回 Ok。
    //   参数 context：当前帧上下文（本节点未使用）。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // request —— 提出“下一帧生效的路线”：0=A、1=B；只应由应用任务在帧间调用（非线程安全）。
    //   参数 route：目标路线编号（0 或 1）。
    //   返回：true 请求已受理；false 编号非法或选择器未配置（保留原路线不变）。
    bool request(uint32_t route);
    // selector —— 返回选择器状态（只读），可用于观察当前路线 / 切换次数等。
    const robot::framework::BranchSelector& selector() const;
    // source —— 返回公共输入节点（只读），主要用于统计 / 观察。
    const DemoSourceNode& source() const;
    // route —— 按编号返回对应路线容器：0 返回路线 A，其他值返回路线 B。
    //   参数 route：应用只应传入 0 或 1。
    DemoRouteNode& route(uint16_t route);
    // merge —— 返回汇合节点（只读），用于观察最终汇合输出。
    const robot::framework::BranchMergeNode<float, 2U>& merge() const;
private:
    robot::framework::BranchSelector selector_{2U}; // 选择器：2 条路线，默认生效路线 A（编号 0）。
    DemoSourceNode source_{41U}; // 公共输入：两条路线共享，每帧执行一次。
    DemoRouteNode route_a_{50U, 1.0f, 2.0f}; // 路线 A：先加 1 再乘 2，即 (10 + 1) * 2 = 22。
    DemoRouteNode route_b_{60U, -1.0f, 3.0f}; // 路线 B：先减 1 再乘 3，即 (10 - 1) * 3 = 27。
    robot::framework::BranchMergeNode<float, 2U> merge_{70U, selector_}; // 汇合节点：与两条路线共享 selector_，只转发生效路线的结果。
};

} // namespace robot::diagnostics
#endif // ROBOT_BRANCH_DEMO_NODE_HPP
