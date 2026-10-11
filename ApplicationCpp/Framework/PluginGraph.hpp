#ifndef ROBOT_PLUGIN_GRAPH_HPP
#define ROBOT_PLUGIN_GRAPH_HPP

/*
 * ============================================================================
 * PluginGraph.hpp —— 插件图（框架的“骨架”与调度器）
 * ----------------------------------------------------------------------------
 * 【这是什么】
 *   PluginGraph 把一堆 PluginNode（插件节点，各自实现一小段控制逻辑）以及它们
 *   之间的先后依赖，组织成一张“有向图”，并负责按正确顺序、在每个 1ms 控制帧里
 *   依次调用它们。机器人控制中，一个节点的输出往往就是下一个节点的输入
 *   （例如：解算 -> 限幅 -> 下发），因此执行顺序必须被严格保证。
 *
 * 【三个阶段】
 *   1) 组装（compose）：注册节点（add）、添加依赖边（addDependency 等），并递归
 *      展开可嵌套的子图（composeNode）。此阶段图结构可以修改。
 *   2) 编译（compile）：先做合法性检查（validate），再用拓扑排序算出执行顺序
 *      （compilePlan），随后把图“冻结”（frozen_ = true），运行期不再改结构。
 *   3) 执行（processFrame）：每一帧按编译好的顺序跑一遍节点，其间完成分支判断、
 *      依赖就绪检查、生命周期（进入/退出/换计划）通知以及统计。
 *
 * 【执行计划 plan】
 *   同一批节点，在不同场景下可能需要不同的执行顺序或依赖集合。“执行计划”就是
 *   一套预先编译好的执行顺序（例如计划 0 = 正常巡逻，计划 1 = 某种特殊行为）。
 *   计划数量由 setPlanCount 声明，最多 ROBOT_MAX_EXECUTION_PLANS 套。每帧开始时
 *   latchPlan() 会把“下一帧请求的计划”锁存为“本帧实际使用的计划”，帧运行中途
 *   不再改变，保证一帧内调度一致。
 *
 * 【依赖边 delayed】
 *   若节点间的依赖被标为 delayed = true，则表示“它读取的是上一个周期的数据”，
 *   属于跨帧依赖，不参与本周期的拓扑排序（否则会被误判成本周期回环）。
 *
 * 【分支 / 汇合】
 *   某些节点只在特定路线下运行，用 BranchSelector + setBranch 标记其所属路线；
 *   条件不满足时该节点本帧被跳过（分支）。多条分支再次汇聚到同一节点时，用
 *   addBranchDependency 声明“汇合边”，允许忽略未选中的前驱。
 *
 * 【关键术语】
 *   - 拓扑排序：把有向无环图排成一个线性顺序，保证每条边的“前驱”都排在“后继”之前。
 *   - 冻结：compile() 之后禁止再增删节点/边，使运行期结构固定、无需加锁。
 *   - 汇合：多条分支重新汇聚到同一节点，需要等待或忽略不同来源的数据。
 *   - 锁存：在帧开始时一次性固定某个值（如计划、路线），本帧内不再变化。
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

class PluginNode;
// 标记值：该依赖适用于全部执行计划（0xFFFF 是不会与真实计划编号冲突的哨兵值）。
constexpr uint16_t kAllExecutionPlans = 0xFFFFU; // 依赖适用于全部计划。
// 标记值：没有具体失败计划（compile 失败但无法定位到某一套计划时使用）。
constexpr uint16_t kNoFailedPlan = 0xFFFFU; // 无具体失败计划。

// 一条执行依赖边：描述“谁必须在谁之前执行”。
// delayed=true 表示该依赖读取的是上一周期的数据（跨帧依赖），不参与本周期拓扑排序。
struct PluginEdge {
    PluginNode* before{nullptr}; // 前驱节点（先执行者）。
    PluginNode* after{nullptr};  // 后继节点（后执行者）。
    bool delayed{false}; // 是否跨帧，不参与本帧排序。
    bool allow_branch_skip{false}; // 汇合边可忽略条件未选中的前驱。
    uint16_t before_slot{0U}; // compile 缓存的前驱下标。
    uint16_t after_slot{0U}; // compile 缓存的后继下标。
    uint16_t plan_id{kAllExecutionPlans}; // 专属计划编号或全部计划。
};

// 固定容量的有向插件图：所有存储都是静态数组，运行期不扩容。
// 组装（add/addDependency/...）完成后调用 compile() 生成执行顺序并冻结图结构。
class PluginGraph {
public:
    // 注册一个节点到图中。
    // 参数 node：要注册的节点对象（引用，生命周期需覆盖整张图）。
    // 返回：Ok 成功；DuplicateNode 重复注册；CapacityExceeded 超出节点容量；
    //       Frozen 图已冻结。
    PluginStatus add(PluginNode& node);
    // 注册节点之间的依赖边：规定 before 必须在 after 之前执行。
    // 参数 before/after：前驱与后继节点；delayed：true 表示跨帧依赖（读取上周期数据），
    //       不参与本周期的拓扑排序。
    // 返回：Ok 成功；MissingNode 端点未注册；DuplicateEdge 边重复；Frozen 图已冻结。
    PluginStatus addDependency(PluginNode& before, PluginNode& after, bool delayed = false);

    // 注册一条“汇合边”（分支末端汇聚到汇合节点）。
    // 与普通依赖的区别：当 before 因分支条件被跳过时，本边允许被忽略，不会卡住 after。
    // before/after：分支末端与汇合节点；只允许忽略条件跳过。
    PluginStatus addBranchDependency(PluginNode& before, PluginNode& after);
    // 给节点设置分支条件（属于哪条路线的哪个分支）。
    // node：本图直接节点；selector/route：所属的路线选择器与路线编号。
    // 该条件会被后代节点继承（子图展开后的子孙同理生效）。
    PluginStatus setBranch(PluginNode& node, BranchSelector& selector, uint16_t route);
    // 查询某节点的执行统计（次数、本帧调度状态等）。
    // node：待查询节点；不属于本图时返回空指针。
    const NodeExecutionStats* executionStats(const PluginNode& node) const;

    // 声明本根图共有多少套执行计划。
    // 参数 count：根图全局计划数，应在启动时声明；默认只有计划 0。
    // 返回：Ok 成功；若已编译/冻结或超出 ROBOT_MAX_EXECUTION_PLANS 则返回错误状态。
    PluginStatus setPlanCount(uint16_t count);
    // 注册一条只属于某个执行计划的依赖边。
    // before/after：依赖端点；plan：该边专属的计划编号；delayed：是否跨帧。
    PluginStatus addPlanDependency(PluginNode& before, PluginNode& after, uint16_t plan, bool delayed = false);
    // 请求在下一帧切换到指定计划（本帧计划不变，帧开始才锁存生效）。
    // 参数 plan：下一帧要使用的计划编号，只允许由所属执行器任务调用；
    //       非法编号（越界等）会被忽略并保留原请求。
    // 返回：true 表示请求被接受；false 表示编号非法。
    bool requestPlan(uint32_t plan);
    // 返回已声明的计划数量。
    uint16_t planCount() const { return plan_count_; }
    // 返回本帧锁存的（实际使用的）计划编号。
    uint16_t activePlan() const { return active_plan_; }
    // 返回计划实际发生切换的累计次数。
    uint32_t planSwitchCount() const { return plan_switch_count_; }
    // 返回计划切换版本号：只有计划真正改变时才自增，重复请求同一计划不递增。
    uint32_t planGeneration() const { return plan_generation_; }
    // 返回编译失败所属的计划编号；值为 0xFFFF（kNoFailedPlan）表示没有定位到具体计划。
    uint16_t failedPlan() const { return failed_plan_; }

    // 递归调用各节点的 compose()，完成子图装配。
    // 调用时机：所有 add/addDependency 之后、validate/compile 之前。
    PluginStatus compose();
    // 检查图配置是否合法（节点/边容量、回环、空指针等）。
    // 只读检查，不改变图；compile() 内部会先调用它。
    PluginStatus validate() const;
    // 对每套计划做拓扑排序，生成执行顺序，并把图冻结（之后禁止修改结构）。
    // 调用时机：compose() 之后、启动前；冻结后 add* 系列会返回 Frozen。
    PluginStatus compile();
    // 依次配置全部节点（调用各节点的 configure()，适合加载静态参数）。
    // 调用时机：compile() 冻结图之后、startAll() 之前。
    PluginStatus configureAll();
    // 依次启动全部节点（置为运行态）。
    // 调用时机：configureAll() 成功之后。
    PluginStatus startAll();
    // 执行当前帧的全部节点，是每 1ms 控制周期的入口。
    // 参数 context：本帧的时间上下文（帧号、周期、时间戳）；profiler：可选的耗时统计器，
    //       默认 nullptr 表示不统计。
    // 返回：本帧整体处理结果（Ok/NoNewData/... 见 ProcessResult）。
    ProcessResult processFrame(FrameContext& context, IProfiler* profiler = nullptr);
    // 反向（按执行顺序的逆序）停止全部节点。
    // 调用时机：整图停机时，保证先停下游再停上游。
    void stopAll();

    // 查询图是否已冻结（compile 之后为 true，表示结构不可再改）。
    bool isFrozen() const { return frozen_; }
    // 查询执行计划是否已生成（compile 成功后为 true）。
    bool isCompiled() const { return compiled_; }
    // 返回已注册的节点数量。
    uint16_t nodeCount() const { return node_count_; }
    // 返回已注册的依赖边数量。
    uint16_t edgeCount() const { return edge_count_; }
    // 按下标读取节点（index 从 0 开始，越界返回 nullptr）。
    PluginNode* nodeAt(uint16_t index) const { return index < node_count_ ? nodes_[index] : nullptr; }
    // 按下标读取依赖边（index 从 0 开始，越界返回 nullptr）。
    const PluginEdge* edgeAt(uint16_t index) const { return index < edge_count_ ? &edges_[index] : nullptr; }

private:
    friend class PluginNode;

    // 递归装配单个节点：调用其 compose()，并把其内部声明的子节点/依赖并入本图。
    // node：递归展开的父节点。
    PluginStatus composeNode(PluginNode& node);
    // 注册依赖边的内部实现（公有 add* 最终都走这里）。
    // before/after：依赖端点；delayed/allow_branch_skip：依赖行为（跨帧 / 汇合可跳过）；
    //       plan：该边所属计划，默认 kAllExecutionPlans（全部计划）。
    PluginStatus addEdge(PluginNode& before, PluginNode& after, bool delayed, bool allow_branch_skip,
                         uint16_t plan = kAllExecutionPlans);
    // 为某一套执行计划单独做拓扑排序，生成该计划的执行顺序。
    // plan：启动时为此计划生成独立拓扑顺序。
    PluginStatus compilePlan(uint16_t plan);
    // 判断某条依赖边是否属于指定计划（边为“全部计划”或计划号一致时为真）。
    // edge/plan：判断依赖是否属于该计划。
    static bool edgeApplies(const PluginEdge& edge, uint16_t plan);
    // 帧开始统一锁存请求：把 requested_plan_ 生效为 active_plan_，运行中不改变实际计划。
    void latchPlan();
    // 检查节点自身及其祖先的分支路线条件是否均满足（决定本帧是否运行）。
    // slot：节点下标；检查自身及祖先的路线条件。
    bool branchActive(uint16_t slot) const;
    // 检查节点自身及其祖先是否都处于可运行状态（未禁用/未故障等）。
    // slot：节点下标；自身与祖先都必须处于可运行状态。
    bool hierarchyRunnable(uint16_t slot) const;
    // 帧边界处理生命周期通知：对状态变化的节点先发退出、再发进入/换计划通知。
    // context/previous_plan：帧边界通知，先退出再进入/计划变化。
    void updateTransitions(const FrameContext& context, uint16_t previous_plan);
    // 检查某节点的所有“本帧输入依赖”是否已就绪（前驱均已执行）。
    // slot：节点下标；检查所有本帧输入依赖。
    bool dependenciesReady(uint16_t slot) const;
    // 在节点表中查找节点下标；未找到返回 -1。
    int findNodeIndex(const PluginNode* node) const;
    // 查找与给定参数语义重叠（重复）的依赖边。
    // before/after/delayed/plan：查找语义重叠的依赖，返回边下标或 -1。
    int findOverlappingEdge(const PluginNode* before, const PluginNode* after, bool delayed, uint16_t plan) const;

    // 注册表、边表和编译后的执行顺序均为静态数组，避免运行期堆分配。
    // 节点注册表：按注册先后存放节点指针，容量 ROBOT_MAX_PLUGIN_NODES（=32）。
    PluginNode* nodes_[ROBOT_MAX_PLUGIN_NODES]{};
    // 依赖边表：容量 ROBOT_MAX_PLUGIN_EDGES（=64）。
    PluginEdge edges_[ROBOT_MAX_PLUGIN_EDGES]{};
    // 每套计划独立的执行顺序表，行=计划，列=节点下标。
    // 各计划共用同一批节点对象，这里只记录“跑的顺序”。
    // 容量 [ROBOT_MAX_EXECUTION_PLANS][ROBOT_MAX_PLUGIN_NODES]（=4 × 32）。
    uint16_t execution_slots_[ROBOT_MAX_EXECUTION_PLANS][ROBOT_MAX_PLUGIN_NODES]{};
    BranchCondition conditions_[ROBOT_MAX_PLUGIN_NODES]{}; // 各节点及祖先的分支条件。
    NodeExecutionStats stats_[ROBOT_MAX_PLUGIN_NODES]{}; // 各节点执行统计。
    BranchSelector* selectors_[ROBOT_MAX_PLUGIN_NODES]{}; // 本图去重后的选择器。
    uint16_t selector_count_{0U}; // 选择器数量。
    // 已注册节点数量（默认 0）。
    uint16_t node_count_{0U};
    // 已注册边数量（默认 0）。
    uint16_t edge_count_{0U};
    // 已编译的执行节点数量（默认 0），即拓扑排序得到的顺序长度。
    uint16_t execution_count_{0U};
    uint16_t plan_count_{1U}; // 本根图声明的计划数量（默认 1，即只有计划 0）。
    uint16_t requested_plan_{0U}; // 下帧待生效的计划（默认 0）。
    uint16_t active_plan_{0U}; // 本帧实际计划（默认 0）。
    uint16_t failed_plan_{kNoFailedPlan}; // 编译错误定位（默认 kNoFailedPlan，表示无具体失败计划）。
    uint32_t plan_switch_count_{0U}; // 实际切换次数（默认 0，允许自然回绕）。
    uint32_t plan_generation_{0U}; // 计划切换版本（默认 0，只有计划真正改变才递增）。
    bool compile_attempted_{false}; // 图是否编译过
    PluginStatus compile_status_{PluginStatus::InvalidState}; // 保存失败原因，禁止使用部分计划。
    // 是否完成递归组装（默认 false）。
    bool composed_{false};
    // 是否生成执行计划（默认 false）。
    bool compiled_{false};
    bool frame_in_progress_{false}; // 防止回调递归执行本图。
    bool lifecycle_in_progress_{false}; // 防止启停或退出通知递归。
    NodeTransitionContext last_transition_{}; // 最近帧上下文，供停止通知使用。
    // 是否禁止继续修改图（compile 后置 true，默认 false）。
    bool frozen_{false};
};

// 编译期容量校验：计划数必须为正且不能与 kAllExecutionPlans 哨兵值冲突。
static_assert(ROBOT_MAX_EXECUTION_PLANS > 0U && ROBOT_MAX_EXECUTION_PLANS < kAllExecutionPlans,
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

#endif /* ROBOT_PLUGIN_GRAPH_HPP */




