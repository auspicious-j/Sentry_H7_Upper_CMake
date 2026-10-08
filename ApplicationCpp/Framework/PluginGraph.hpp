#ifndef ROBOT_PLUGIN_GRAPH_HPP
#define ROBOT_PLUGIN_GRAPH_HPP

#include <cstddef>
#include <cstdint>

#include "robot_build_config.h"
#include "PluginTypes.hpp"
#include "IProfiler.hpp"

namespace robot::framework {

class PluginNode;

// 一条执行依赖边。delayed=true 表示读取上一周期数据，不参与本周期拓扑排序。
struct PluginEdge {
    PluginNode* before{nullptr};
    PluginNode* after{nullptr};
    bool delayed{false};
};

// 固定容量的有向插件图。组装完成后 compile() 生成执行顺序并冻结图结构。
class PluginGraph {
public:
    // 注册一个节点。
    PluginStatus add(PluginNode& node);
    // 注册节点之间的依赖边。
    PluginStatus addDependency(PluginNode& before, PluginNode& after, bool delayed = false);

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

    PluginStatus composeNode(PluginNode& node);
    int findNodeIndex(const PluginNode* node) const;
    bool hasEdge(const PluginNode* before, const PluginNode* after, bool delayed) const;

    // 注册表、边表和编译后的执行顺序均为静态数组，避免运行期堆分配。
    // 节点注册表。
    PluginNode* nodes_[ROBOT_MAX_PLUGIN_NODES]{};
    // 依赖边表。
    PluginEdge edges_[ROBOT_MAX_PLUGIN_EDGES]{};
    // 拓扑排序后的执行顺序。
    PluginNode* execution_order_[ROBOT_MAX_PLUGIN_NODES]{};
    // 已注册节点数量。
    uint16_t node_count_{0U};
    // 已注册边数量。
    uint16_t edge_count_{0U};
    // 已编译执行节点数量。
    uint16_t execution_count_{0U};
    // 是否完成递归组装。
    bool composed_{false};
    // 是否生成执行计划。
    bool compiled_{false};
    // 是否禁止继续修改图。
    bool frozen_{false};
};

static_assert(ROBOT_MAX_PLUGIN_NODES > 0U, "Plugin graph needs node storage");
static_assert(ROBOT_MAX_PLUGIN_EDGES > 0U, "Plugin graph needs edge storage");

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_GRAPH_HPP */




