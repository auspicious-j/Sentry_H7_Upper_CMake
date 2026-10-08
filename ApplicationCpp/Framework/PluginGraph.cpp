#include "PluginGraph.hpp"

#include "PluginNode.hpp"

namespace robot::framework {

int PluginGraph::findNodeIndex(const PluginNode* node) const
{
    for (uint16_t index = 0U; index < node_count_; ++index) {
        if (nodes_[index] == node) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool PluginGraph::hasEdge(const PluginNode* before, const PluginNode* after, bool delayed) const
{
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        const PluginEdge& edge = edges_[index];
        if (edge.before == before && edge.after == after && edge.delayed == delayed) {
            return true;
        }
    }
    return false;
}

PluginStatus PluginGraph::add(PluginNode& node)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    if (findNodeIndex(&node) >= 0) {
        return PluginStatus::DuplicateNode;
    }
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

PluginStatus PluginGraph::composeNode(PluginNode& node)
{
    if (node.composed_) {
        return PluginStatus::Ok;
    }

    node.composed_ = true;
    PluginStatus status = node.compose();
    if (status != PluginStatus::Ok) {
        return status;
    }

    PluginGraph& child_graph = node.children();
    const uint16_t child_count = child_graph.nodeCount();
    for (uint16_t index = 0U; index < child_count; ++index) {
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

    const uint16_t child_edge_count = child_graph.edgeCount();
    for (uint16_t index = 0U; index < child_edge_count; ++index) {
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

PluginStatus PluginGraph::compose()
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    if (composed_) {
        return PluginStatus::Ok;
    }

    composed_ = true;
    const uint16_t root_count = node_count_;
    for (uint16_t index = 0U; index < root_count; ++index) {
        PluginStatus status = composeNode(*nodes_[index]);
        if (status != PluginStatus::Ok) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

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

    uint16_t indegree[ROBOT_MAX_PLUGIN_NODES]{};
    bool selected[ROBOT_MAX_PLUGIN_NODES]{};
    for (uint16_t edge_index = 0U; edge_index < edge_count_; ++edge_index) {
        const PluginEdge& edge = edges_[edge_index];
        if (!edge.delayed) {
            const int after_index = findNodeIndex(edge.after);
            if (after_index < 0) {
                return PluginStatus::MissingNode;
            }
            ++indegree[static_cast<uint16_t>(after_index)];
        }
    }

    execution_count_ = 0U;
    while (execution_count_ < node_count_) {
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

        PluginNode* ready_node = nodes_[static_cast<uint16_t>(ready_index)];
        selected[static_cast<uint16_t>(ready_index)] = true;
        execution_order_[execution_count_] = ready_node;
        ++execution_count_;

        for (uint16_t edge_index = 0U; edge_index < edge_count_; ++edge_index) {
            const PluginEdge& edge = edges_[edge_index];
            if (!edge.delayed && edge.before == ready_node) {
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

PluginStatus PluginGraph::configureAll()
{
    if (!compiled_) {
        return PluginStatus::InvalidState;
    }
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        PluginStatus status = execution_order_[index]->configure();
        if (status != PluginStatus::Ok) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

PluginStatus PluginGraph::startAll()
{
    if (!compiled_) {
        return PluginStatus::InvalidState;
    }
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        PluginStatus status = execution_order_[index]->start();
        if (status != PluginStatus::Ok) {
            return status;
        }
    }
    return PluginStatus::Ok;
}

ProcessResult PluginGraph::processFrame(FrameContext& context, IProfiler* profiler)
{
    if (!compiled_) {
        return ProcessResult::Fault;
    }

    ProcessResult aggregate = ProcessResult::Ok;
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        PluginNode& node = *execution_order_[index];
        if (node.state() == PluginState::Disabled || node.state() == PluginState::Faulted) {
            continue;
        }
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


