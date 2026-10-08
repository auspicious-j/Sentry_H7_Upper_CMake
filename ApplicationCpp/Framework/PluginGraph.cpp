#include "PluginGraph.hpp"

#include "PluginNode.hpp"

namespace robot::framework {

// 在线性固定数组中查找节点；容量较小且只在启动期频繁使用。
int PluginGraph::findNodeIndex(const PluginNode* node) const
{
    // index：当前节点或边的数组下标。
    for (uint16_t index = 0U; index < node_count_; ++index) {
        if (nodes_[index] == node) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

// 防止同一依赖边被重复注册。
bool PluginGraph::hasEdge(const PluginNode* before, const PluginNode* after, bool delayed) const
{
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        // edge：当前检查的依赖边。
        const PluginEdge& edge = edges_[index];
        if (edge.before == before && edge.after == after && edge.delayed == delayed) {
            return true;
        }
    }
    return false;
}

// 注册节点。图冻结后禁止修改，保证运行期指针和性能 ID 稳定。
PluginStatus PluginGraph::add(PluginNode& node)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    if (findNodeIndex(&node) >= 0) {
        return PluginStatus::DuplicateNode;
    }
    // index：用于检查已注册节点 ID。
    for (uint16_t index = 0U; index < node_count_; ++index) {
        if (nodes_[index]->id() == node.id()) {
            return PluginStatus::DuplicateNode;
        }
    }
    if (node_count_ >= ROBOT_MAX_PLUGIN_NODES) {
        return PluginStatus::CapacityExceeded;
    }
    nodes_[node_count_] = &node;
    ++node_count_;
    return PluginStatus::Ok;
}

// 注册本周期依赖；延迟边只表达跨周期反馈，不阻塞本周期拓扑排序。
PluginStatus PluginGraph::addDependency(PluginNode& before, PluginNode& after, bool delayed)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    if (findNodeIndex(&before) < 0 || findNodeIndex(&after) < 0) {
        return PluginStatus::MissingNode;
    }
    if (&before == &after && !delayed) {
        return PluginStatus::CycleDetected;
    }
    if (hasEdge(&before, &after, delayed)) {
        return PluginStatus::DuplicateEdge;
    }
    if (edge_count_ >= ROBOT_MAX_PLUGIN_EDGES) {
        return PluginStatus::CapacityExceeded;
    }
    edges_[edge_count_] = PluginEdge{&before, &after, delayed};
    ++edge_count_;
    return PluginStatus::Ok;
}

// 递归展开节点内部图，但所有节点最终进入同一个执行计划。
PluginStatus PluginGraph::composeNode(PluginNode& node)
{
    if (node.composed_) {
        return PluginStatus::Ok;
    }

    node.composed_ = true;
    // status：子图组装结果。
    PluginStatus status = node.compose();
    if (status != PluginStatus::Ok) {
        return status;
    }

    // child_graph：当前节点的内部子图。
    PluginGraph& child_graph = node.children();
    // child_count：内部节点数量。
    const uint16_t child_count = child_graph.nodeCount();
    for (uint16_t index = 0U; index < child_count; ++index) {
        // child：当前内部节点。
        PluginNode* child = child_graph.nodeAt(index);
        if (child == nullptr) {
            return PluginStatus::ConfigurationFault;
        }
        status = add(*child);
        if (status != PluginStatus::Ok) {
            return status;
        }
        status = composeNode(*child);
        if (status != PluginStatus::Ok) {
            return status;
        }
    }

    // child_edge_count：内部依赖边数量。
    const uint16_t child_edge_count = child_graph.edgeCount();
    for (uint16_t index = 0U; index < child_edge_count; ++index) {
        // edge：当前内部依赖边。
        const PluginEdge* edge = child_graph.edgeAt(index);
        if (edge == nullptr) {
            return PluginStatus::ConfigurationFault;
        }
        status = addDependency(*edge->before, *edge->after, edge->delayed);
        if (status != PluginStatus::Ok && status != PluginStatus::DuplicateEdge) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

// 只在启动阶段调用一次：让每个节点注册自己的内部节点和边。
PluginStatus PluginGraph::compose()
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    if (composed_) {
        return PluginStatus::Ok;
    }

    composed_ = true;
    // root_count：展开前的根节点数量。
    const uint16_t root_count = node_count_;
    for (uint16_t index = 0U; index < root_count; ++index) {
        // status：当前根节点的展开结果。
        PluginStatus status = composeNode(*nodes_[index]);
        if (status != PluginStatus::Ok) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

// 检查节点、边和 ID 的基本合法性。
PluginStatus PluginGraph::validate() const
{
    for (uint16_t index = 0U; index < node_count_; ++index) {
        if (nodes_[index] == nullptr || nodes_[index]->id() == kInvalidPluginId) {
            return PluginStatus::ConfigurationFault;
        }
        for (uint16_t other = index + 1U; other < node_count_; ++other) {
            if (nodes_[index]->id() == nodes_[other]->id()) {
                return PluginStatus::DuplicateNode;
            }
        }
    }

    for (uint16_t index = 0U; index < edge_count_; ++index) {
        const PluginEdge& edge = edges_[index];
        if (edge.before == nullptr || edge.after == nullptr ||
            findNodeIndex(edge.before) < 0 || findNodeIndex(edge.after) < 0) {
            return PluginStatus::MissingNode;
        }
    }
    return PluginStatus::Ok;
}

// 使用 Kahn 拓扑排序生成确定执行顺序；成功后冻结图结构。
PluginStatus PluginGraph::compile()
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }

    PluginStatus status = compose();
    if (status != PluginStatus::Ok) {
        return status;
    }
    status = validate();
    if (status != PluginStatus::Ok) {
        return status;
    }

    // indegree：每个节点尚未满足的前置依赖数量。
    uint16_t indegree[ROBOT_MAX_PLUGIN_NODES]{};
    // selected：节点是否已经加入执行顺序。
    bool selected[ROBOT_MAX_PLUGIN_NODES]{};
    for (uint16_t edge_index = 0U; edge_index < edge_count_; ++edge_index) {
        // edge：当前待分析的依赖边。
        const PluginEdge& edge = edges_[edge_index];
        if (!edge.delayed) {
            // after_index：边终点在节点数组中的位置。
            const int after_index = findNodeIndex(edge.after);
            if (after_index < 0) {
                return PluginStatus::MissingNode;
            }
            ++indegree[static_cast<uint16_t>(after_index)];
        }
    }

    execution_count_ = 0U;
    while (execution_count_ < node_count_) {
        // ready_index：当前可执行节点的位置。
        int ready_index = -1;
        for (uint16_t index = 0U; index < node_count_; ++index) {
            if (!selected[index] && indegree[index] == 0U) {
                ready_index = static_cast<int>(index);
                break;
            }
        }
        if (ready_index < 0) {
            execution_count_ = 0U;
            return PluginStatus::CycleDetected;
        }

        // ready_node：当前选中的节点。
        PluginNode* ready_node = nodes_[static_cast<uint16_t>(ready_index)];
        selected[static_cast<uint16_t>(ready_index)] = true;
        execution_order_[execution_count_] = ready_node;
        ++execution_count_;

        for (uint16_t edge_index = 0U; edge_index < edge_count_; ++edge_index) {
            const PluginEdge& edge = edges_[edge_index];
            if (!edge.delayed && edge.before == ready_node) {
                // after_index：后继节点的位置。
                const int after_index = findNodeIndex(edge.after);
                if (after_index >= 0 && indegree[static_cast<uint16_t>(after_index)] > 0U) {
                    --indegree[static_cast<uint16_t>(after_index)];
                }
            }
        }
    }

    compiled_ = true;
    frozen_ = true;
    return PluginStatus::Ok;
}

// 按已编译顺序初始化所有节点。
PluginStatus PluginGraph::configureAll()
{
    if (!compiled_) {
        return PluginStatus::InvalidState;
    }
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        // status：当前节点配置结果。
        PluginStatus status = execution_order_[index]->configure();
        if (status != PluginStatus::Ok) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

// 按已编译顺序切换所有节点到 Running。
PluginStatus PluginGraph::startAll()
{
    if (!compiled_) {
        return PluginStatus::InvalidState;
    }
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        // status：当前节点启动结果。
        PluginStatus status = execution_order_[index]->start();
        if (status != PluginStatus::Ok) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

// 运行期只遍历执行计划，不重新排序、不分配内存。
ProcessResult PluginGraph::processFrame(FrameContext& context, IProfiler* profiler)
{
    if (!compiled_) {
        return ProcessResult::Fault;
    }

    // aggregate：汇总本帧所有节点的最严重结果。
    ProcessResult aggregate = ProcessResult::Ok;
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        // node：当前执行的节点。
        PluginNode& node = *execution_order_[index];
        if (node.state() == PluginState::Disabled || node.state() == PluginState::Faulted) {
            continue;
        }
        // result：当前节点本帧的处理结果。
        const ProcessResult result = node.process(context);
        node.recordProcessResult(result);
        if (result == ProcessResult::Fault) {
            aggregate = ProcessResult::Fault;
        } else if (result == ProcessResult::Degraded && aggregate == ProcessResult::Ok) {
            aggregate = ProcessResult::Degraded;
        } else if (result == ProcessResult::OutputHeld && aggregate == ProcessResult::Ok) {
            aggregate = ProcessResult::OutputHeld;
        }
    }
    return aggregate;
}

// 停止时反向执行，保证后置节点先停止。
void PluginGraph::stopAll()
{
    if (!compiled_) {
        return;
    }
    for (uint16_t index = execution_count_; index > 0U; --index) {
        execution_order_[index - 1U]->stop();
    }
}

} // namespace robot::framework


