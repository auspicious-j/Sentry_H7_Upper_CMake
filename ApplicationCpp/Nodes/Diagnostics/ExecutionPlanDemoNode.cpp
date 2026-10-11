#include "ExecutionPlanDemoNode.hpp"

/*
 * execution_plan_demo_node.cpp —— 执行计划演示节点的实现
 * 说明：这里只搭"可切换顺序的两节点小图"，不含任何硬件访问或控制算法。
 *   计划 0：A → B；计划 1：B → A。
 *   切换请求由上层调用 graph_.request_plan() 提交，在下一帧开始时锁存生效；
 *   本文件不主动切换计划，只负责把两套顺序声明好。
 */

namespace robot::diagnostics {

// 构造函数：创建演示用根节点，编号 80；无硬件输出、无外部依赖。
execution_plan_demo_node::execution_plan_demo_node() : plugin_node(80U) {}

// 组装阶段回调：把两个探针和两套计划顺序声明进内部子图。
// 时机：框架在 add(plan_demo_) 之后、compile() 之前调用，整张图只组装这一次。
// 注意：计划编号(0/1)是"根图全局"的编号，子图沿用同一套编号；因此根图必须先
//       set_plan_count(2U) 声明计划总数，本函数里的 0U/1U 才合法。
robot::framework::plugin_status execution_plan_demo_node::compose()
{
    // 先注册探针 A；add() 的返回值即本次组装结果（status 逐级串联检查）。
    robot::framework::plugin_status status = children().add(a_); // 当前组装结果。
    // 若 A 注册成功，再注册探针 B；任一失败即短路后续步骤并原样返回错误。
    if (status == robot::framework::plugin_status::ok)
    {
        status = children().add(b_);
    }
    if (status == robot::framework::plugin_status::ok)
    {
        // 计划 0 专属边：A 先于 B（仅当激活计划为 0 时才参与拓扑排序）。
        status = children().add_plan_dependency(a_, b_, 0U);
    }
    if (status == robot::framework::plugin_status::ok)
    {
        // 计划 1 专属边：B 先于 A（仅当激活计划为 1 时才参与拓扑排序）。
        status = children().add_plan_dependency(b_, a_, 1U);
    }
    // 注意：两条专属边合在一起看会形成 A→B→A 的环，但它们分属不同计划，
    //       任意单个计划内部都是无环的，因此仍是合法配置。
    return status;
}

// 每帧回调：容器节点本身不执行内部探针（a_/b_ 已在 compose 阶段展开为根图节点，
// 由根执行器按"当前计划的拓扑顺序"统一执行），这里只需返回成功。
// 参数 context：当前帧上下文（未使用，故显式丢弃）。
robot::framework::process_result execution_plan_demo_node::process(robot::framework::frame_context& context)
{
    (void)context;
    return robot::framework::process_result::ok;
}

// 返回探针 A 的只读引用；调用方仅用于读取执行统计（不做修改）。
const plan_probe_node& execution_plan_demo_node::a() const { return a_; }
// 返回探针 B 的只读引用；调用方仅用于读取执行统计（不做修改）。
const plan_probe_node& execution_plan_demo_node::b() const { return b_; }

} // namespace robot::diagnostics
