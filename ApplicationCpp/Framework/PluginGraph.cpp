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

// 注册节点
PluginStatus PluginGraph::add(PluginNode& node)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    if (findNodeIndex(&node) >= 0) {
        return PluginStatus::DuplicateNode;
    }
    // 检查已注册节点 ID有无重复
    for (uint16_t index = 0U; index < node_count_; ++index) {
        if (nodes_[index]->id() == node.id()) {
            return PluginStatus::DuplicateNode;
        }
    }
    // 检查是否大于允许的最大节点数量
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
    return addEdge(before, after, delayed, false);
}

// before/after：互斥分支末端与结果汇合点。
PluginStatus PluginGraph::addBranchDependency(PluginNode& before, PluginNode& after)
{
    return addEdge(before, after, false, true);
}

// node：直接节点；selector/route：其激活条件。
PluginStatus PluginGraph::setBranch(PluginNode& node, BranchSelector& selector, uint16_t route)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    const int slot = findNodeIndex(&node); // 本图注册下标。
    if (slot < 0) {
        return PluginStatus::MissingNode;
    }
    if (!selector.configured() || !selector.contains(route)) {
        return PluginStatus::ConfigurationFault;
    }
    conditions_[slot].selector = &selector;
    conditions_[slot].route = route;
    return PluginStatus::Ok;
}

// node：查询对象；统计地址在图生命周期内固定。
const NodeExecutionStats* PluginGraph::executionStats(const PluginNode& node) const
{
    const int slot = findNodeIndex(&node); // 节点注册下标。
    return slot < 0 ? nullptr : &stats_[slot];
}

// before/after：端点；标志决定本帧依赖或可跳过汇合。
PluginStatus PluginGraph::addEdge(PluginNode& before, PluginNode& after, bool delayed, bool allow_branch_skip)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    // 端点可能位于尚未展开的子图；compile 展开后统一检查归属。
    //! 怎么感觉有点问题
    if (&before == &after && !delayed) {
        return PluginStatus::CycleDetected;
    }
    if (hasEdge(&before, &after, delayed)) {
        return PluginStatus::DuplicateEdge;
    }
    if (edge_count_ >= ROBOT_MAX_PLUGIN_EDGES) {
        return PluginStatus::CapacityExceeded;
    }
    edges_[edge_count_] = PluginEdge{&before, &after, delayed, allow_branch_skip};
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
        // child_slot：刚加入展开图的子节点下标。
        const uint16_t child_slot = static_cast<uint16_t>(node_count_ - 1U);
        conditions_[child_slot] = child_graph.conditions_[index];
        conditions_[child_slot].parent = static_cast<int16_t>(findNodeIndex(&node));
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
        status = addEdge(*edge->before, *edge->after, edge->delayed, edge->allow_branch_skip);
        if (status != PluginStatus::Ok && status != PluginStatus::DuplicateEdge) {
            return status;
        }
    }
    // 子图关系已复制到根计划；立即冻结，禁止返回成功却只修改旧副本。
    child_graph.frozen_ = true;
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
    selector_count_ = 0U;
    // slot：展开图下标；收集选择器并再次验证分支条件。
    for (uint16_t slot = 0U; slot < node_count_; ++slot) {
        const BranchCondition& condition = conditions_[slot]; // 本节点显式条件。
        if (condition.selector == nullptr) {
            continue;
        }
        if (!condition.selector->configured() || !condition.selector->contains(condition.route)) {
            return PluginStatus::ConfigurationFault;
        }
        bool found = false; // 是否已收集此选择器。
        // index：去重表下标。
        for (uint16_t index = 0U; index < selector_count_; ++index) {
            found = found || selectors_[index] == condition.selector;
        }
        if (!found) {
            selectors_[selector_count_++] = condition.selector;
        }
    }
    // index：依赖边下标；缓存端点，避免每帧反复查找指针。
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        edges_[index].before_slot = static_cast<uint16_t>(findNodeIndex(edges_[index].before));
        edges_[index].after_slot = static_cast<uint16_t>(findNodeIndex(edges_[index].after));
    }
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
        execution_slots_[execution_count_] = static_cast<uint16_t>(ready_index);
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

    // index：展开图下标；一并冻结每个内部组装图。
    for (uint16_t index = 0U; index < node_count_; ++index) {
        nodes_[index]->children().frozen_ = true;
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

// slot：展开节点下标；子图条件与所有祖先条件相与。
bool PluginGraph::branchActive(uint16_t slot) const
{
    int current = static_cast<int>(slot); // 沿父关系向上检查的位置。
    while (current >= 0) {
        const BranchCondition& condition = conditions_[current]; // 当前层条件。
        if (condition.selector != nullptr && condition.selector->activeRoute() != condition.route) {
            return false;
        }
        current = condition.parent;
    }
    return true;
}

// slot：消费节点；分支汇合不忽略故障、未启动或无数据。
bool PluginGraph::dependenciesReady(uint16_t slot) const
{
    bool has_alternative = false; // 是否声明了分支汇合输入。
    bool completed_alternative = false; // 是否有至少一个分支实际完成。
    // index：本帧依赖边下标。
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        const PluginEdge& edge = edges_[index]; // 当前依赖。
        if (edge.delayed || edge.after_slot != slot) {
            continue;
        }
        const NodeFrameState state = stats_[edge.before_slot].frame_state; // 前驱本帧状态。
        if (edge.allow_branch_skip) {
            has_alternative = true;
            if (state == NodeFrameState::SkippedBranch) {
                continue;
            }
            if (state != NodeFrameState::Executed) {
                return false;
            }
            completed_alternative = true;
        } else if (state != NodeFrameState::Executed) {
            return false;
        }
    }
    return !has_alternative || completed_alternative;
}

// context：本帧上下文；profiler 沿用预留接口，节点只执行一次。
ProcessResult PluginGraph::processFrame(FrameContext& context, IProfiler* profiler)
{
    (void)profiler; // 本板块不改变现有探针实现。
    if (!compiled_) {
        return ProcessResult::Fault;
    }
    // index：选择器下标；所有请求在第一节点执行前统一锁存。
    for (uint16_t index = 0U; index < selector_count_; ++index) {
        selectors_[index]->latch();
    }
    // index：注册下标；清除上一帧调度状态。
    for (uint16_t index = 0U; index < node_count_; ++index) {
        stats_[index].frame_state = NodeFrameState::Pending;
    }
    ProcessResult aggregate = ProcessResult::Ok; // 本帧汇总结果。
    // index：已编译执行顺序下标。
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        const uint16_t slot = execution_slots_[index]; // 注册下标。
        PluginNode& node = *execution_order_[index]; // 当前节点。
        NodeExecutionStats& stats = stats_[slot]; // 当前节点统计。
        if (!branchActive(slot)) {
            stats.frame_state = NodeFrameState::SkippedBranch;
            ++stats.branch_skip_count;
            node.onSkipped(context);
            continue;
        }
        if (node.state() != PluginState::Running && node.state() != PluginState::Degraded) {
            stats.frame_state = NodeFrameState::SkippedState;
            ++stats.blocked_count;
            node.onSkipped(context);
            if (node.state() == PluginState::Faulted) {
                aggregate = ProcessResult::Fault;
            } else if (aggregate == ProcessResult::Ok) {
                aggregate = ProcessResult::OutputHeld;
            }
            continue;
        }
        if (!dependenciesReady(slot)) {
            stats.frame_state = NodeFrameState::Blocked;
            ++stats.blocked_count;
            node.onSkipped(context);
            if (aggregate == ProcessResult::Ok) {
                aggregate = ProcessResult::OutputHeld;
            }
            continue;
        }
        const ProcessResult result = node.process(context); // 本次业务结果。
        ++stats.execution_count;
        node.recordProcessResult(result);
        stats.frame_state = NodeFrameState::Executed;
        if (result == ProcessResult::Fault) {
            stats.frame_state = NodeFrameState::Faulted;
            node.onSkipped(context);
            aggregate = ProcessResult::Fault;
        } else if (result == ProcessResult::NoNewData) {
            stats.frame_state = NodeFrameState::NoData;
            node.onSkipped(context);
            if (aggregate == ProcessResult::Ok) {
                aggregate = ProcessResult::OutputHeld;
            }
        } else if (result == ProcessResult::Degraded && aggregate != ProcessResult::Fault) {
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


