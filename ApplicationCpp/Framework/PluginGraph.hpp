#ifndef ROBOT_PLUGIN_GRAPH_HPP
#define ROBOT_PLUGIN_GRAPH_HPP

#include <cstddef>
#include <cstdint>

#include "robot_build_config.h"
#include "PluginTypes.hpp"
#include "IProfiler.hpp"
#include "BranchSelector.hpp"

namespace robot::framework {

class PluginNode;
constexpr uint16_t kAllExecutionPlans = 0xFFFFU; // 依赖适用于全部计划。
constexpr uint16_t kNoFailedPlan = 0xFFFFU; // 无具体失败计划。

// 一条执行依赖边。delayed=true 表示读取上一周期数据，不参与本周期拓扑排序。
struct PluginEdge {
    PluginNode* before{nullptr};
    PluginNode* after{nullptr};
    bool delayed{false}; // 是否跨帧，不参与本帧排序。
    bool allow_branch_skip{false}; // 汇合边可忽略条件未选中的前驱。
    uint16_t before_slot{0U}; // compile 缓存的前驱下标。
    uint16_t after_slot{0U}; // compile 缓存的后继下标。
    uint16_t plan_id{kAllExecutionPlans}; // 专属计划编号或全部计划。
};

// 固定容量的有向插件图。组装完成后 compile() 生成执行顺序并冻结图结构。
class PluginGraph {
public:
    // 注册一个节点。
    PluginStatus add(PluginNode& node);
    // 注册节点之间的依赖边。
    PluginStatus addDependency(PluginNode& before, PluginNode& after, bool delayed = false);

    // before/after：分支末端与汇合节点；只允许忽略条件跳过。
    PluginStatus addBranchDependency(PluginNode& before, PluginNode& after);
    // node：本图直接节点；selector/route：所属路线，后代继承该条件。
    PluginStatus setBranch(PluginNode& node, BranchSelector& selector, uint16_t route);
    // node：待查询节点；不属于本图时返回空指针。
    const NodeExecutionStats* executionStats(const PluginNode& node) const;

    // count：根图全局计划数，启动时声明；默认只有计划 0。
    PluginStatus setPlanCount(uint16_t count);
    // before/after：端点；plan：专属计划；delayed：是否跨帧。
    PluginStatus addPlanDependency(PluginNode& before, PluginNode& after, uint16_t plan, bool delayed = false);
    // plan：下一帧请求，只由所属执行器任务调用；非法值保留原请求。
    bool requestPlan(uint32_t plan);
    // 返回已声明的计划数量。
    uint16_t planCount() const { return plan_count_; }
    // 返回本帧锁存计划。
    uint16_t activePlan() const { return active_plan_; }
    // 返回计划实际切换次数。
    uint32_t planSwitchCount() const { return plan_switch_count_; }
    // 返回计划切换版本，不因重复请求同一计划递增。
    uint32_t planGeneration() const { return plan_generation_; }
    // 返回编译失败计划编号；0xFFFF 表示没有具体计划失败。
    uint16_t failedPlan() const { return failed_plan_; }

    // 递归调用节点的 compose。
    PluginStatus compose();
    // 检查图配置是否合法。
    PluginStatus validate() const;
    // 生成拓扑执行顺序并冻结图。
    PluginStatus compile();
    // 配置全部节点。
    PluginStatus configureAll();
    // 启动全部节点。
    PluginStatus startAll();
    // 执行当前帧的全部节点。
    ProcessResult processFrame(FrameContext& context, IProfiler* profiler = nullptr);
    // 反向停止全部节点。
    void stopAll();

    // 查询图是否冻结。
    bool isFrozen() const { return frozen_; }
    // 查询执行计划是否生成。
    bool isCompiled() const { return compiled_; }
    // 返回节点数量。
    uint16_t nodeCount() const { return node_count_; }
    // 返回边数量。
    uint16_t edgeCount() const { return edge_count_; }
    // 按下标读取节点。
    PluginNode* nodeAt(uint16_t index) const { return index < node_count_ ? nodes_[index] : nullptr; }
    // 按下标读取依赖边。
    const PluginEdge* edgeAt(uint16_t index) const { return index < edge_count_ ? &edges_[index] : nullptr; }

private:
    friend class PluginNode;

    // node：递归展开的父节点。
    PluginStatus composeNode(PluginNode& node);
    // before/after：端点；delayed/allow_branch_skip：依赖行为。
    PluginStatus addEdge(PluginNode& before, PluginNode& after, bool delayed, bool allow_branch_skip,
                         uint16_t plan = kAllExecutionPlans);
    // plan：启动时为此计划生成独立拓扑顺序。
    PluginStatus compilePlan(uint16_t plan);
    // edge/plan：判断依赖是否属于该计划。
    static bool edgeApplies(const PluginEdge& edge, uint16_t plan);
    // 帧开始统一锁存请求，运行中不改变实际计划。
    void latchPlan();
    // slot：节点下标；检查自身及祖先的路线条件。
    bool branchActive(uint16_t slot) const;
    // slot：节点下标；检查所有本帧输入依赖。
    bool dependenciesReady(uint16_t slot) const;
    int findNodeIndex(const PluginNode* node) const;
    // before/after/delayed/plan：查找语义重叠的依赖，返回边下标或 -1。
    int findOverlappingEdge(const PluginNode* before, const PluginNode* after, bool delayed, uint16_t plan) const;

    // 注册表、边表和编译后的执行顺序均为静态数组，避免运行期堆分配。
    // 节点注册表。
    PluginNode* nodes_[ROBOT_MAX_PLUGIN_NODES]{};
    // 依赖边表。
    PluginEdge edges_[ROBOT_MAX_PLUGIN_EDGES]{};
    // 每计划独立存放注册下标，所有计划共用同一批节点对象。
    uint16_t execution_slots_[ROBOT_MAX_EXECUTION_PLANS][ROBOT_MAX_PLUGIN_NODES]{};
    BranchCondition conditions_[ROBOT_MAX_PLUGIN_NODES]{}; // 各节点及祖先的分支条件。
    NodeExecutionStats stats_[ROBOT_MAX_PLUGIN_NODES]{}; // 各节点执行统计。
    BranchSelector* selectors_[ROBOT_MAX_PLUGIN_NODES]{}; // 本图去重后的选择器。
    uint16_t selector_count_{0U}; // 选择器数量。
    // 已注册节点数量。
    uint16_t node_count_{0U};
    // 已注册边数量。
    uint16_t edge_count_{0U};
    // 已编译执行节点数量。
    uint16_t execution_count_{0U};
    uint16_t plan_count_{1U}; // 本根图声明的计划数量。
    uint16_t requested_plan_{0U}; // 下帧待生效的计划。
    uint16_t active_plan_{0U}; // 本帧实际计划。
    uint16_t failed_plan_{kNoFailedPlan}; // 编译错误定位。
    uint32_t plan_switch_count_{0U}; // 实际切换次数。
    uint32_t plan_generation_{0U}; // 计划切换版本。
    bool compile_attempted_{false}; // 本对象是否已经尝试编译。
    PluginStatus compile_status_{PluginStatus::InvalidState}; // 保存失败原因，禁止使用部分计划。
    // 是否完成递归组装。
    bool composed_{false};
    // 是否生成执行计划。
    bool compiled_{false};
    // 是否禁止继续修改图。
    bool frozen_{false};
};

static_assert(ROBOT_MAX_EXECUTION_PLANS > 0U && ROBOT_MAX_EXECUTION_PLANS < kAllExecutionPlans,
              "Invalid execution plan capacity");
static_assert(ROBOT_MAX_PLUGIN_NODES > 0U, "Plugin graph needs node storage");
static_assert(ROBOT_MAX_PLUGIN_EDGES > 0U, "Plugin graph needs edge storage");
static_assert(ROBOT_MAX_PLUGIN_NODES <= 32767U, "Parent index storage exceeded");
static_assert(ROBOT_MAX_PLUGIN_EDGES <= 65535U, "Edge index storage exceeded");

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_GRAPH_HPP */




