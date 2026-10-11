#ifndef robot_plugin_graph_hpp
#define robot_plugin_graph_hpp

/*
 * ============================================================================
 * plugin_graph.hpp —— 插件图（框架的“骨架”与调度器）
 * ----------------------------------------------------------------------------
 * 【这是什么】
 *   plugin_graph 把一堆 plugin_node（插件节点，各自实现一小段控制逻辑）以及它们
 *   之间的先后依赖，组织成一张“有向图”，并负责按正确顺序、在每个 1ms 控制帧里
 *   依次调用它们。机器人控制中，一个节点的输出往往就是下一个节点的输入
 *   （例如：解算 -> 限幅 -> 下发），因此执行顺序必须被严格保证。
 *
 * 【三个阶段】
 *   1) 组装（compose）：注册节点（add）、添加依赖边（add_dependency 等），并递归
 *      展开可嵌套的子图（compose_node）。此阶段图结构可以修改。
 *   2) 编译（compile）：先做合法性检查（validate），再用拓扑排序算出执行顺序
 *      （compile_plan），随后把图“冻结”（is_frozen_ = true），运行期不再改结构。
 *   3) 执行（process_frame）：每一帧按编译好的顺序跑一遍节点，其间完成分支判断、
 *      依赖就绪检查、生命周期（进入/退出/换计划）通知以及统计。
 *
 * 【执行计划 plan】
 *   同一批节点，在不同场景下可能需要不同的执行顺序或依赖集合。“执行计划”就是
 *   一套预先编译好的执行顺序（例如计划 0 = 正常巡逻，计划 1 = 某种特殊行为）。
 *   计划数量由 set_plan_count 声明，最多 ROBOT_MAX_EXECUTION_PLANS 套。每帧开始时
 *   latch_plan() 会把“下一帧请求的计划”锁存为“本帧实际使用的计划”，帧运行中途
 *   不再改变，保证一帧内调度一致。
 *
 * 【依赖边 is_delayed】
 *   若节点间的依赖被标为 is_delayed = true，则表示“它读取的是上一个周期的数据”，
 *   属于跨帧依赖，不参与本周期的拓扑排序（否则会被误判成本周期回环）。
 *
 * 【分支 / 汇合】
 *   某些节点只在特定分支编号下运行，用 branch_selector + set_branch 标记其所属分支编号；
 *   条件不满足时该节点本帧被跳过（分支）。多条分支再次汇聚到同一节点时，用
 *   add_branch_dependency 声明“汇合边”，允许忽略未选中的前驱。
 *
 * 【关键术语】
 *   - 拓扑排序：把有向无环图排成一个线性顺序，保证每条边的“前驱”都排在“后继”之前。
 *   - 冻结：compile() 之后禁止再增删节点/边，使运行期结构固定、无需加锁。
 *   - 汇合：多条分支重新汇聚到同一节点，需要等待或忽略不同来源的数据。
 *   - 锁存：在帧开始时一次性固定某个值（如计划、分支编号），本帧内不再变化。
 *
 * 【内存约束】
 *   节点表、边表、执行顺序全部用静态数组实现（文件末尾有 static_assert 校验），
 *   运行期不做堆分配。容量来自 Config/robot_build_config.h 中的宏：
 *   ROBOT_MAX_PLUGIN_NODES（节点数）、ROBOT_MAX_PLUGIN_EDGES（边数）、
 *   ROBOT_MAX_EXECUTION_PLANS（执行计划数）。
 * ============================================================================
 */

#include <cstddef>
#include <cstdint>

#include "robot_build_config.h"
#include "PluginTypes.hpp"
#include "IProfiler.hpp"
#include "BranchSelector.hpp"
#include "NodeTransitionContext.hpp"

namespace robot::framework {

class plugin_node;
// 标记值：该依赖适用于全部执行计划（0xFFFF 是不会与真实计划编号冲突的哨兵值）。
constexpr uint16_t k_all_execution_plans = 0xFFFFU; // 依赖适用于全部计划。
// 标记值：没有具体失败计划（compile 失败但无法定位到某一套计划时使用）。
constexpr uint16_t k_no_failed_plan = 0xFFFFU; // 无具体失败计划。

// 一条执行依赖边：描述“谁必须在谁之前执行”。
// is_delayed=true 表示该依赖读取的是上一周期的数据（跨帧依赖），不参与本周期拓扑排序。
struct plugin_edge {
    plugin_node* before{nullptr}; // 前驱节点（先执行者）。
    plugin_node* after{nullptr};  // 后继节点（后执行者）。
    bool is_delayed{false}; // 是否跨帧，不参与本帧排序。
    bool is_allow_branch_skip{false}; // 汇合边可忽略条件未选中的前驱。
    uint16_t before_slot{0U}; // compile 缓存的前驱下标。
    uint16_t after_slot{0U}; // compile 缓存的后继下标。
    uint16_t plan_id{k_all_execution_plans}; // 专属计划编号或全部计划。
};

// 固定容量的有向插件图：所有存储都是静态数组，运行期不扩容。
// 组装（add/add_dependency/...）完成后调用 compile() 生成执行顺序并冻结图结构。
class plugin_graph {
public:
    // 注册一个节点到图中。
    plugin_status add(plugin_node& node);
    // 注册节点之间的依赖边：规定 before 必须在 after 之前执行。
    plugin_status add_dependency(plugin_node& before, plugin_node& after, bool is_delayed = false);

    // 注册一条“汇合边”（分支末端汇聚到汇合节点）。
    plugin_status add_branch_dependency(plugin_node& before, plugin_node& after);

    plugin_status set_branch(plugin_node& node, branch_selector& selector, uint16_t branch_id);
    // 查询某节点的执行统计（次数、本帧调度状态等）。
    // node：待查询节点；不属于本图时返回空指针。
    const node_execution_stats* execution_stats(const plugin_node& node) const;

    // 声明本根图共有多少套执行计划。
    // 参数 count：根图全局计划数，应在启动时声明；默认只有计划 0。
    // 返回：ok 成功；若已编译/冻结或超出 ROBOT_MAX_EXECUTION_PLANS 则返回错误状态。
    plugin_status set_plan_count(uint16_t count);
    // 注册一条只属于某个执行计划的依赖边。
    // before/after：依赖端点；plan_id：该边专属的计划编号；is_delayed：是否跨帧。
    plugin_status add_plan_dependency(plugin_node& before, plugin_node& after, uint16_t plan_id, bool is_delayed = false);
    // 请求在下一帧切换到指定计划（本帧计划不变，帧开始才锁存生效）。
    // 参数 plan_id：下一帧要使用的计划编号，只允许由所属执行器任务调用；
    //       非法编号（越界等）会被忽略并保留原请求。
    // 返回：true 表示请求被接受；false 表示编号非法。
    bool request_plan(uint32_t plan_id);
    // 返回已声明的计划数量。
    uint16_t plan_count() const { return plan_count_; }
    // 返回本帧锁存的（实际使用的）计划编号。
    uint16_t active_plan() const { return active_plan_id_; }
    // 返回计划实际发生切换的累计次数（同时充当"计划版本号"）。
    uint32_t plan_switch_count() const { return plan_switch_count_; }
    // 返回编译失败所属的计划编号；值为 0xFFFF（k_no_failed_plan）表示没有定位到具体计划。
    uint16_t failed_plan() const { return failed_plan_id_; }

    // 递归调用各节点的 compose()，完成子图装配。
    // 调用时机：所有 add/add_dependency 之后、validate/compile 之前。
    plugin_status compose();
    // 检查图配置是否合法（节点/边容量、回环、空指针等）。
    // 只读检查，不改变图；compile() 内部会先调用它。
    plugin_status validate() const;
    // 对每套计划做拓扑排序，生成执行顺序，并把图冻结（之后禁止修改结构）。
    // 调用时机：compose() 之后、启动前；冻结后 add* 系列会返回 frozen。
    plugin_status compile();
    // 依次配置全部节点（调用各节点的 configure()，适合加载静态参数）。
    // 调用时机：compile() 冻结图之后、start_all() 之前。
    plugin_status configure_all();
    // 依次启动全部节点（置为运行态）。
    // 调用时机：configure_all() 成功之后。
    plugin_status start_all();
    // 执行当前帧的全部节点，是每 1ms 控制周期的入口。
    // 参数 context：本帧的时间上下文（帧号、周期、时间戳）；profiler：可选的耗时统计器，
    //       默认 nullptr 表示不统计。
    // 返回：本帧整体处理结果（ok/no_new_data/... 见 process_result）。
    process_result process_frame(frame_context& context, i_profiler* profiler = nullptr);
    // 反向（按执行顺序的逆序）停止全部节点。
    // 调用时机：整图停机时，保证先停下游再停上游。
    void stop_all();

    // 查询图是否已冻结（compile 之后为 true，表示结构不可再改）。
    bool is_frozen() const { return is_frozen_; }
    // 查询执行计划是否已生成（compile 成功后为 true）。
    bool is_compiled() const { return is_compiled_; }
    // 返回已注册的节点数量。
    uint16_t node_count() const { return node_count_; }
    // 返回已注册的依赖边数量。
    uint16_t edge_count() const { return edge_count_; }
    // 按下标读取节点（index 从 0 开始，越界返回 nullptr）。
    plugin_node* node_at(uint16_t index) const { return index < node_count_ ? nodes_[index] : nullptr; }
    // 按下标读取依赖边（index 从 0 开始，越界返回 nullptr）。
    const plugin_edge* edge_at(uint16_t index) const { return index < edge_count_ ? &edges_[index] : nullptr; }

private:
    friend class plugin_node;

    // 递归装配单个节点：调用其 compose()，并把其内部声明的子节点/依赖并入本图。
    // node：递归展开的父节点。
    plugin_status compose_node(plugin_node& node);
    // 注册依赖边的内部实现（公有 add* 最终都走这里）。
    // before/after：依赖端点；is_delayed/is_allow_branch_skip：依赖行为（跨帧 / 汇合可跳过）；
    //       plan_id：该边所属计划，默认 k_all_execution_plans（全部计划）。
    plugin_status add_edge(plugin_node& before, plugin_node& after, bool is_delayed, bool is_allow_branch_skip,
                         uint16_t plan_id = k_all_execution_plans);
    // 为某一套执行计划单独做拓扑排序，生成该计划的执行顺序。
    // plan_id：启动时为此计划生成独立拓扑顺序。
    plugin_status compile_plan(uint16_t plan_id);
    // 判断某条依赖边是否属于指定计划（边为“全部计划”或计划号一致时为真）。
    // edge/plan_id：判断依赖是否属于该计划。
    static bool edge_applies(const plugin_edge& edge, uint16_t plan_id);
    // 帧开始统一锁存请求：把 requested_plan_id_ 生效为 active_plan_id_，运行中不改变实际计划。
    void latch_plan();
    // 检查节点自身及其祖先的分支分支编号条件是否均满足（决定本帧是否运行）。
    // slot：节点下标；检查自身及祖先的分支编号条件。
    bool branch_active(uint16_t slot) const;
    // 检查节点自身及其祖先是否都处于可运行状态（未禁用/未故障等）。
    // slot：节点下标；自身与祖先都必须处于可运行状态。
    bool hierarchy_runnable(uint16_t slot) const;
    // 帧边界处理生命周期通知：对状态变化的节点先发退出、再发进入/换计划通知。
    // context/previous_plan_id：帧边界通知，先退出再进入/计划变化。
    void update_transitions(const frame_context& context, uint16_t previous_plan_id);
    // 检查某节点的所有“本帧输入依赖”是否已就绪（前驱均已执行）。
    // slot：节点下标；检查所有本帧输入依赖。
    bool dependencies_ready(uint16_t slot) const;
    // 在节点表中查找节点下标；未找到返回 -1。
    int find_node_index(const plugin_node* node) const;
    // 查找与给定参数语义重叠（重复）的依赖边。
    // before/after/is_delayed/plan_id：查找语义重叠的依赖，返回边下标或 -1。
    int find_overlapping_edge(const plugin_node* before, const plugin_node* after, bool is_delayed, uint16_t plan_id) const;

    // 注册表、边表和编译后的执行顺序均为静态数组，避免运行期堆分配。
    // 节点注册表：按注册先后存放节点指针，容量 ROBOT_MAX_PLUGIN_NODES（=32）。
    plugin_node* nodes_[ROBOT_MAX_PLUGIN_NODES]{};
    // 依赖边表：容量 ROBOT_MAX_PLUGIN_EDGES（=64）。
    plugin_edge edges_[ROBOT_MAX_PLUGIN_EDGES]{};
    // 每套计划独立的执行顺序表，行=计划，列=节点下标。
    // 各计划共用同一批节点对象，这里只记录“跑的顺序”。
    // 容量 [ROBOT_MAX_EXECUTION_PLANS][ROBOT_MAX_PLUGIN_NODES]（=4 × 32）。
    uint16_t execution_slots_[ROBOT_MAX_EXECUTION_PLANS][ROBOT_MAX_PLUGIN_NODES]{};
    branch_condition conditions_[ROBOT_MAX_PLUGIN_NODES]{}; // 各节点及祖先的分支条件。
    node_execution_stats stats_[ROBOT_MAX_PLUGIN_NODES]{}; // 各节点执行统计。
    branch_selector* selectors_[ROBOT_MAX_PLUGIN_NODES]{}; // 本图去重后的选择器。
    uint16_t selector_count_{0U}; // 选择器数量。
    // 已注册节点数量（默认 0）。
    uint16_t node_count_{0U};
    // 已注册边数量（默认 0）。
    uint16_t edge_count_{0U};
    // 已编译的执行节点数量（默认 0），即拓扑排序得到的顺序长度。
    uint16_t execution_count_{0U};
    uint16_t plan_count_{1U}; // 本根图声明的计划数量（默认 1，即只有计划 0）。
    uint16_t requested_plan_id_{0U}; // 下帧待生效的计划（默认 0）。
    uint16_t active_plan_id_{0U}; // 本帧实际计划（默认 0）。
    uint16_t failed_plan_id_{k_no_failed_plan}; // 编译错误定位（默认 k_no_failed_plan，表示无具体失败计划）。
    uint32_t plan_switch_count_{0U}; // 实际切换次数（默认 0，允许自然回绕）。
    bool is_compile_attempted_{false}; // 图是否编译过
    plugin_status compile_status_{plugin_status::invalid_state}; // 保存失败原因，禁止使用部分计划。
    // 是否完成递归组装（默认 false）。
    bool is_composed_{false};
    // 是否生成执行计划（默认 false）。
    bool is_compiled_{false};
    bool is_frame_in_progress_{false}; // 防止回调递归执行本图。
    bool is_lifecycle_in_progress_{false}; // 防止启停或退出通知递归。
    node_transition_context last_transition_{}; // 最近帧上下文，供停止通知使用。
    // 是否禁止继续修改图（compile 后置 true，默认 false）。
    bool is_frozen_{false};
};

// 编译期容量校验：计划数必须为正且不能与 k_all_execution_plans 哨兵值冲突。
static_assert(ROBOT_MAX_EXECUTION_PLANS > 0U && ROBOT_MAX_EXECUTION_PLANS < k_all_execution_plans,
              "Invalid execution plan capacity");
// 节点表容量必须为正。
static_assert(ROBOT_MAX_PLUGIN_NODES > 0U, "Plugin graph needs node storage");
// 边表容量必须为正。
static_assert(ROBOT_MAX_PLUGIN_EDGES > 0U, "Plugin graph needs edge storage");
// 父节点下标用 int16_t 存储，节点数不得超过 32767。
static_assert(ROBOT_MAX_PLUGIN_NODES <= 32767U, "Parent index storage exceeded");
// 边下标用 uint16_t 存储，边数不得超过 65535。
static_assert(ROBOT_MAX_PLUGIN_EDGES <= 65535U, "Edge index storage exceeded");

} // namespace robot::framework

#endif /* robot_plugin_graph_hpp */




