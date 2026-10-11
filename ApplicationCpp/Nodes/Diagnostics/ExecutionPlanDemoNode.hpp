#ifndef ROBOT_EXECUTION_PLAN_DEMO_NODE_HPP
#define ROBOT_EXECUTION_PLAN_DEMO_NODE_HPP

/*
 * ============================================================================
 * ExecutionPlanDemoNode.hpp —— 执行计划（plan）演示节点
 * ----------------------------------------------------------------------------
 * 【本文件是什么】
 *   这是一组"诊断/演示"用的插件节点，用来验证框架的"执行计划(plan)"机制：
 *   同一批节点在同一张图里，切换到不同 plan 时会被安排成不同的执行顺序。
 *   它不产生任何硬件输出、不参与机器人控制，只用来观察"顺序是否按计划改变"。
 *
 * 【两个类的关系】
 *   - PlanProbeNode         ：顺序探针。process() 什么都不做，只用来被"数次数/记序号"。
 *   - ExecutionPlanDemoNode ：演示根节点（容器）。内部持有两个探针 A、B，
 *                             并声明两套计划顺序：计划 0 = A 先于 B，计划 1 = B 先于 A。
 *
 * 【关键概念】
 *   - 执行计划(plan)：一套预先编译好的执行顺序。同一批节点可以有多个计划，每帧只使用
 *     其中一个（由 latchPlan() 在帧开始时锁存）。本演示声明了 2 套计划。
 *   - 拓扑顺序：框架对每个计划做拓扑排序，得到"谁先谁后"的线性执行序列；探针的先后
 *     关系就是由"计划专属依赖边"决定的。
 *   - 组装子图：容器节点的 children() 是它的内部子图。compose() 阶段框架会递归展开子图，
 *     把子图里的节点"提升"到根图中，之后由根执行器统一调度（所以容器自己的 process()
 *     不需要、也不会重复执行子节点）。
 *
 * 【编号约定】
 *   根节点 ID = 80；探针 A = 81；探针 B = 82。编号必须在整张根图内唯一。
 * ============================================================================
 */

#include "Framework/PluginNode.hpp"

namespace robot::diagnostics {

// 顺序探针：一个"无硬件、无数据、无副作用"的占位节点。
// 它唯一的用途是让框架记录"本节点被执行了几次、以及在本帧里排第几个执行"
// （执行统计由框架统一维护），据此可判断它相对另一个探针的先后顺序。
class PlanProbeNode final : public robot::framework::PluginNode {
public:
    // 构造函数。
    // 参数 id：探针在根图内的唯一编号（PluginId，16 位无符号整数）。
    explicit PlanProbeNode(robot::framework::PluginId id) : PluginNode(id) {}
    // 每帧都会被框架调用；这里刻意不做任何事，只证明"本节点本帧被执行过"。
    // 参数 context：当前帧上下文（本探针不读取，故显式丢弃）。
    // 返回：固定 Ok，表示处理正常。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override
    {
        (void)context;
        return robot::framework::ProcessResult::Ok;
    }
};

// 执行计划演示的根节点（组合/容器节点）。
// 作用：把两个顺序探针（a_、b_）组装进内部子图，并声明两套"顺序不同"的执行计划：
//   计划 0：A → B（a_ 在 b_ 之前）
//   计划 1：B → A（b_ 在 a_ 之前）
// 它本身不承载业务逻辑，只负责"搭好这张可切换顺序的小图"，供上层通过统计验证。
class ExecutionPlanDemoNode final : public robot::framework::PluginNode {
public:
    // 创建演示根节点：编号固定为 80，无硬件依赖；内部使用 81、82 两个探针编号。
    // 注意：本图使用"计划 0 / 计划 1"，根图必须先用 setPlanCount(2U) 声明两套计划
    //       （由 RobotApplication::initialize() 在注册本节点之前完成）。
    ExecutionPlanDemoNode();
    // 组装回调（框架在组装阶段调用：所有 add 之后、validate/compile 之前，且只调用一次）。
    // 职责：把 a_、b_ 加进内部子图，并为两套计划各声明一条专属顺序边。
    // 返回：Ok 表示两节点与两条计划边全部注册成功，否则返回首个错误状态。
    robot::framework::PluginStatus compose() override;
    // 每帧回调；容器节点不重复执行内部探针（子图已在 compose 阶段展开、由根执行器统一调度）。
    // 参数 context：当前帧上下文（容器不读取，故显式丢弃）。
    // 返回：固定 Ok。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // 返回探针 A 的只读引用，供上层把执行统计复制到调试 Watch（只读，不修改）。
    const PlanProbeNode& a() const;
    // 返回探针 B 的只读引用，供上层把执行统计复制到调试 Watch（只读，不修改）。
    const PlanProbeNode& b() const;
private:
    PlanProbeNode a_{81U}; // 顺序探针 A，编号固定 81；值成员，构造即完成编号赋值。
    PlanProbeNode b_{82U}; // 顺序探针 B，编号固定 82；值成员，生命周期覆盖整张图。
};

} // namespace robot::diagnostics
#endif // ROBOT_EXECUTION_PLAN_DEMO_NODE_HPP
