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
    PluginStatus add(PluginNode& node);
    PluginStatus addDependency(PluginNode& before, PluginNode& after, bool delayed = false);

    PluginStatus compose();
    PluginStatus validate() const;
    PluginStatus compile();
    PluginStatus configureAll();
    PluginStatus startAll();
    ProcessResult processFrame(FrameContext& context, IProfiler* profiler = nullptr);
    void stopAll();

    bool isFrozen() const { return frozen_; }
    bool isCompiled() const { return compiled_; }
    uint16_t nodeCount() const { return node_count_; }
    uint16_t edgeCount() const { return edge_count_; }
    PluginNode* nodeAt(uint16_t index) const { return index < node_count_ ? nodes_[index] : nullptr; }
    const PluginEdge* edgeAt(uint16_t index) const { return index < edge_count_ ? &edges_[index] : nullptr; }

private:
    friend class PluginNode;

    PluginStatus composeNode(PluginNode& node);
    int findNodeIndex(const PluginNode* node) const;
    bool hasEdge(const PluginNode* before, const PluginNode* after, bool delayed) const;

    // 注册表、边表和编译后的执行顺序均为静态数组，避免运行期堆分配。
    PluginNode* nodes_[ROBOT_MAX_PLUGIN_NODES]{};
    PluginEdge edges_[ROBOT_MAX_PLUGIN_EDGES]{};
    PluginNode* execution_order_[ROBOT_MAX_PLUGIN_NODES]{};
    uint16_t node_count_{0U};
    uint16_t edge_count_{0U};
    uint16_t execution_count_{0U};
    bool composed_{false};
    bool compiled_{false};
    bool frozen_{false};
};

static_assert(ROBOT_MAX_PLUGIN_NODES > 0U, "Plugin graph needs node storage");
static_assert(ROBOT_MAX_PLUGIN_EDGES > 0U, "Plugin graph needs edge storage");

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_GRAPH_HPP */




