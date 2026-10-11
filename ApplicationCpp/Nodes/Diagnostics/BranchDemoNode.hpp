#ifndef robot_branch_demo_node_hpp
#define robot_branch_demo_node_hpp

#include "Framework/BranchMergeNode.hpp"

/*
 * branch_demo_node.hpp —— 插件框架“分支演示”节点集合（纯软件，无任何硬件输出）。
 *
 * 演示目标：用一组不访问硬件的节点，展示框架的分支(branch)/分支编号(branch_id)/选择器(selector)/汇合(merge)能力。
 *
 * 图结构（两条互斥分支编号 A/B，由同一个选择器决定本帧哪条激活）：
 *   公共输入 demo_source_node(41) 每帧发布常量 10.0f；
 *     ├── 分支编号 A：demo_branch_node(50) = [加偏置(51)] -> [乘系数(52)]，即 (10 + 1) * 2 = 22；
 *     └── 分支编号 B：demo_branch_node(60) = [加偏置(61)] -> [乘系数(62)]，即 (10 - 1) * 3 = 27；
 *   两条分支编号末端都接入汇合节点 branch_merge_node(70)：仅转发本帧被选中分支编号的结果；
 *   根容器 branch_demo_node(40) 负责把公共输入、两条子图与汇合节点组装成一张图。
 *
 * 关键概念：
 *   分支(branch)：一条可被独立选中或跳过的执行路径。
 *   分支编号(branch_id) ：分支编号，从 0 开始；本演示 A=0、B=1。
 *   选择器(selector)：记录“下一帧请求分支编号”与“本帧生效分支编号”；上游分支与下游汇合共享同一实例，
 *                     以保证上下游对“本帧走哪条分支编号”的判断完全一致。
 *   汇合(merge)：把多条分支末端的同类型结果收拢为单一输出，只转发当前生效分支编号的那一路。
 *
 * 路由/汇合流程：应用任务在帧间调用 request(branch_id) 提出“下一帧走 A 还是 B”；帧边界处选择器把
 *   请求锁存为生效分支编号；随后仅生效分支编号上的子节点被执行并产出结果，汇合节点读取该分支编号结果并
 *   发布到唯一输出端口。未生效分支编号本帧被跳过，其输出失效（不会残留旧值）。
 */

namespace robot::diagnostics {

// 公共输入节点：两条分支编号共享的同一数据源；纯软件，不访问硬件。
class demo_source_node final : public robot::framework::plugin_node {
public:
    // 构造。参数 id：图内唯一编号（本演示固定为 41）。
    explicit demo_source_node(robot::framework::plugin_id id);
    // process —— 每帧被调用一次：发布固定常量 10.0f，供两条分支编号共同消费。
    //   参数 context：当前帧上下文（提供 frame_id 等）。
    //   返回：ok 发布成功；fault 发布失败（例如上一帧的有效数据尚未被消费完）。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // on_skipped —— 本帧被跳过时调用：使内部信号失效，避免下游读到上一帧的旧值。
    //   参数 context：当前帧上下文（本节点未使用）。
    void on_skipped(robot::framework::frame_context& context) override;
    // signal —— 返回本节点发布的公共输入信号（供各分支编号首节点绑定读取）。
    //   返回：只读的信号引用。
    const robot::framework::frame_signal<float>& signal() const;
private:
    robot::framework::frame_signal<float> signal_{}; // 内部信号存储：承载本帧发布值及其有效性/帧号。
    robot::framework::output_port<float> output_{}; // 输出写端：已绑定 signal_，对外发布结果。
};

// 算术演示节点：对输入执行一次仿射变换 input * scale + offset，用于观察各分支编号系数差异。
class demo_math_node final : public robot::framework::plugin_node {
public:
    // 构造。参数：
    //   id    图内唯一编号。
    //   scale 乘法系数（本节点结果 = 输入 * scale + offset）。
    //   offset 加法偏置。
    demo_math_node(robot::framework::plugin_id id, float scale, float offset);
    // bind —— 组装期绑定上游信号（同执行域内的数据源）。
    //   参数 source：上游节点发布的输出信号。
    void bind(const robot::framework::frame_signal<float>& source);
    // on_enter —— 节点被激活（进入）时的通知：本演示只把“本次激活内的调用计数”清零，
    //   刻意不修改 scale_/offset_ 等算术参数，用以说明重置策略由节点自行决定。
    //   参数 transition：状态迁移上下文。
    void on_enter(const robot::framework::node_transition_context& transition) override;
    // on_exit —— 节点被去激活（退出）时的通知：使本节点输出失效。
    //   即使整图随即停止，也不残留上一帧的有效输出。
    //   参数 transition：状态迁移上下文。
    void on_exit(const robot::framework::node_transition_context& transition) override;
    // runs_since_enter —— 返回“本次激活以来” process 的调用次数（进入时被清零）。
    //   返回：自最近一次 on_enter 起的调用次数（总执行计数另由框架独立保存）。
    uint32_t runs_since_enter() const;
    // process —— 每帧执行：读取上游并计算 输入 * scale_ + offset_ 后发布。
    //   参数 context：当前帧上下文。
    //   返回：ok 计算并发布成功；no_new_data 本帧上游无新数据；fault 发布失败。
    //   注意：进入时先失效旧输出，故不会沿用上轮结果。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // on_skipped —— 本帧被跳过时调用：清空输出有效性（未生效分支编号不得残留旧结果）。
    //   参数 context：当前帧上下文（本节点未使用）。
    void on_skipped(robot::framework::frame_context& context) override;
    // signal —— 返回本节点输出信号。
    //   返回：只读的信号引用。
    const robot::framework::frame_signal<float>& signal() const;
private:
    float scale_; // 乘法系数：随构造参数固定，运行期不改变。
    float offset_; // 加法偏置：随构造参数固定，运行期不改变。
    uint32_t runs_since_enter_{0U}; // 本次激活以来的 process 调用次数；仅在 on_enter 清零（默认 0）。
    robot::framework::input_port<float> input_{}; // 算术输入端口：由 bind() 指向上游信号。
    robot::framework::frame_signal<float> signal_{}; // 计算结果存储：含数值与其有效性/帧号。
    robot::framework::output_port<float> output_{}; // 结果写端：已绑定 signal_，对外发布。
};

// 分支编号容器节点：一条分支编号内部再展开成一张子图（含两个串行阶段）。
// 两个子节点继承父节点的分支条件——只有本分支编号被选中时它们才会被执行。
class demo_branch_node final : public robot::framework::plugin_node {
public:
    // 构造。参数：
    //   id     容器编号；内部两个子节点分别占用 id+1、id+2。
    //   offset 第一级（加偏置）阶段使用的偏置系数。
    //   scale  第二级（乘系数）阶段使用的乘法系数。
    demo_branch_node(robot::framework::plugin_id id, float offset, float scale);
    // compose —— 组装期调用：把内部两个子节点及其串行依赖注册进容器子图。
    //   返回：ok 组装成功；其余为失败原因（见 plugin_status）。
    robot::framework::plugin_status compose() override;
    // process —— 每帧执行：容器本身不参与运算（避免重复执行子图），直接返回 ok。
    //   参数 context：当前帧上下文（本节点未使用）。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // first —— 返回第一阶段（加偏置）节点，供外部绑定上游信号。
    demo_math_node& first();
    // second —— 返回第二阶段（乘系数）节点，供外部接线到汇合节点。
    demo_math_node& second();
private:
    demo_math_node first_; // 第一阶段：加偏置（结果 = 输入 + offset）。
    demo_math_node second_; // 第二阶段：乘系数（结果 = 第一阶段结果 * scale）。
};

// 分支演示根节点：作为整张演示图的容器，对外与业务节点使用完全相同的接口。
class branch_demo_node final : public robot::framework::plugin_node {
public:
    // 构造。固定编号布局：
    //   40 = 根容器；41 = 公共输入；50..52 = 分支编号 A（容器/加偏置/乘系数）；
    //   60..62 = 分支编号 B；70 = 汇合节点。
    branch_demo_node();
    // compose —— 组装期调用：注册公共输入、两条子图与汇合节点，设置分支关系并完成数据接线。
    //   返回：ok 组装成功；其余为失败原因。
    robot::framework::plugin_status compose() override;
    // process —— 每帧执行：根节点不重复执行业务（实际算术在子图），直接返回 ok。
    //   参数 context：当前帧上下文（本节点未使用）。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // request —— 提出“下一帧生效的分支编号”：0=A、1=B；只应由应用任务在帧间调用（非线程安全）。
    //   参数 branch_id：目标分支编号编号（0 或 1）。
    //   返回：true 请求已受理；false 编号非法或选择器未配置（保留原分支编号不变）。
    bool request(uint32_t branch_id);
    // selector —— 返回选择器状态（只读），可用于观察当前分支编号 / 切换次数等。
    const robot::framework::branch_selector& selector() const;
    // source —— 返回公共输入节点（只读），主要用于统计 / 观察。
    const demo_source_node& source() const;
    // branch_id —— 按编号返回对应分支编号容器：0 返回分支编号 A，其他值返回分支编号 B。
    //   参数 branch_id：应用只应传入 0 或 1。
    demo_branch_node& branch(uint16_t branch_id);
    // merge —— 返回汇合节点（只读），用于观察最终汇合输出。
    const robot::framework::branch_merge_node<float, 2U>& merge() const;
private:
    robot::framework::branch_selector selector_{2U}; // 选择器：2 条分支编号，默认生效分支编号 A（编号 0）。
    demo_source_node source_{41U}; // 公共输入：两条分支编号共享，每帧执行一次。
    demo_branch_node branch_a_{50U, 1.0f, 2.0f}; // 分支编号 A：先加 1 再乘 2，即 (10 + 1) * 2 = 22。
    demo_branch_node branch_b_{60U, -1.0f, 3.0f}; // 分支编号 B：先减 1 再乘 3，即 (10 - 1) * 3 = 27。
    robot::framework::branch_merge_node<float, 2U> merge_{70U, selector_}; // 汇合节点：与两条分支编号共享 selector_，只转发生效分支编号的结果。
};

} // namespace robot::diagnostics
#endif // robot_branch_demo_node_hpp
