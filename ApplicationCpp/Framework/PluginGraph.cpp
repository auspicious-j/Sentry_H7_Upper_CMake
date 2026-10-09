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

// before/after/delayed/plan：寻找在同一计划中重叠的依赖声明。
int PluginGraph::findOverlappingEdge(const PluginNode* before, const PluginNode* after,
                                    bool delayed, uint16_t plan) const
{
    // index：依赖边下标。
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        const PluginEdge& edge = edges_[index]; // 当前已有边。
        const bool overlap = edge.plan_id == kAllExecutionPlans || plan == kAllExecutionPlans
            || edge.plan_id == plan; // 作用计划是否相交。
        if (edge.before == before && edge.after == after && edge.delayed == delayed && overlap) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

// edge/plan：普通边适用于全部计划，专属边只属于对应计划。
bool PluginGraph::edgeApplies(const PluginEdge& edge, uint16_t plan)
{
    return edge.plan_id == kAllExecutionPlans || edge.plan_id == plan;
}

// count：根执行器全局计划数，不能运行时改变。
PluginStatus PluginGraph::setPlanCount(uint16_t count)
{
    if (frozen_) { return PluginStatus::Frozen; }
    if (compile_attempted_) { return PluginStatus::InvalidState; }
    if (count == 0U || count > ROBOT_MAX_EXECUTION_PLANS) {
        return PluginStatus::ConfigurationFault;
    }
    plan_count_ = count;
    return PluginStatus::Ok;
}

// before/after/plan：专属计划依赖；其存在性在子图展开后检查。
PluginStatus PluginGraph::addPlanDependency(PluginNode& before, PluginNode& after, uint16_t plan, bool delayed)
{
    if (plan >= ROBOT_MAX_EXECUTION_PLANS) { return PluginStatus::ConfigurationFault; }
    return addEdge(before, after, delayed, false, plan);
}

// plan：执行任务提交的请求；不在这里改变当前执行顺序。
bool PluginGraph::requestPlan(uint32_t plan)
{
    if (!compiled_ || plan >= plan_count_) { return false; }
    requested_plan_ = static_cast<uint16_t>(plan);
    return true;
}

// 本帧开始前锁存，重复请求同一计划不计为切换。
void PluginGraph::latchPlan()
{
    if (active_plan_ != requested_plan_) {
        active_plan_ = requested_plan_;
        ++plan_switch_count_;
        ++plan_generation_;
    }
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
PluginStatus PluginGraph::addEdge(PluginNode& before, PluginNode& after, bool delayed,
                                 bool allow_branch_skip, uint16_t plan)
{
    if (frozen_) {
        return PluginStatus::Frozen;
    }
    // 端点可能位于尚未展开的子图；compile 展开后统一检查归属。
    //! 怎么感觉有点问题
    if (&before == &after && !delayed) {
        return PluginStatus::CycleDetected;
    }
    const int duplicate = findOverlappingEdge(&before, &after, delayed, plan); // 重叠边位置。
    if (duplicate >= 0) {
        const PluginEdge& edge = edges_[duplicate]; // 已有声明。
        // 不允许把计划专属边静默当成全部计划边，或混用汇合语义。
        return edge.plan_id == plan && edge.allow_branch_skip == allow_branch_skip
            ? PluginStatus::DuplicateEdge : PluginStatus::ConfigurationFault;
    }
    if (edge_count_ >= ROBOT_MAX_PLUGIN_EDGES) {
        return PluginStatus::CapacityExceeded;
    }
    edges_[edge_count_] = PluginEdge{&before, &after, delayed, allow_branch_skip};
    edges_[edge_count_].plan_id = plan;
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
        status = addEdge(*edge->before, *edge->after, edge->delayed, edge->allow_branch_skip, edge->plan_id);
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
        if (edge.plan_id != kAllExecutionPlans && edge.plan_id >= plan_count_) {
            return PluginStatus::ConfigurationFault;
        }
    }
    return PluginStatus::Ok;
}

// 所有计划都成功才冻结运行；失败对象不能使用部分编译结果。
PluginStatus PluginGraph::compile()
{
    if (frozen_) { return PluginStatus::Frozen; }
    if (compile_attempted_) { return compile_status_; }
    compile_attempted_ = true;
    compile_status_ = compose();
    if (compile_status_ != PluginStatus::Ok) { return compile_status_; }
    compile_status_ = validate();
    if (compile_status_ != PluginStatus::Ok) { return compile_status_; }

    selector_count_ = 0U;
    // slot：展开图下标，收集去重选择器。
    for (uint16_t slot = 0U; slot < node_count_; ++slot) {
        const BranchCondition& condition = conditions_[slot]; // 显式分支条件。
        if (condition.selector == nullptr) { continue; }
        if (!condition.selector->configured() || !condition.selector->contains(condition.route)) {
            compile_status_ = PluginStatus::ConfigurationFault;
            return compile_status_;
        }
        bool found = false; // 是否已收集。
        // index：去重表位置。
        for (uint16_t index = 0U; index < selector_count_; ++index) {
            found = found || selectors_[index] == condition.selector;
        }
        if (!found) { selectors_[selector_count_++] = condition.selector; }
    }
    // index：依赖边下标，缓存端点。
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        edges_[index].before_slot = static_cast<uint16_t>(findNodeIndex(edges_[index].before));
        edges_[index].after_slot = static_cast<uint16_t>(findNodeIndex(edges_[index].after));
    }
    // plan：逐个验证的计划编号。
    for (uint16_t plan = 0U; plan < plan_count_; ++plan) {
        compile_status_ = compilePlan(plan);
        if (compile_status_ != PluginStatus::Ok) {
            failed_plan_ = plan;
            execution_count_ = 0U;
            return compile_status_;
        }
    }
    // index：展开图下标，冻结所有子图。
    for (uint16_t index = 0U; index < node_count_; ++index) {
        nodes_[index]->children().frozen_ = true;
    }
    execution_count_ = node_count_;
    compiled_ = true;
    frozen_ = true;
    return compile_status_;
}

// plan：本次排序的计划；只考虑其有效边，允许不同计划采用相反顺序。
PluginStatus PluginGraph::compilePlan(uint16_t plan)
{
    uint16_t indegree[ROBOT_MAX_PLUGIN_NODES]{}; // 剩余前驱数。
    bool selected[ROBOT_MAX_PLUGIN_NODES]{}; // 是否已加入本计划。
    uint16_t count = 0U; // 本计划已排序数量。
    // index：依赖边下标。
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        const PluginEdge& edge = edges_[index]; // 当前依赖。
        if (!edge.delayed && edgeApplies(edge, plan)) { ++indegree[edge.after_slot]; }
    }
    while (count < node_count_) {
        int ready = -1; // 本轮就绪下标。
        // slot：按注册顺序稳定选取节点。
        for (uint16_t slot = 0U; slot < node_count_; ++slot) {
            if (!selected[slot] && indegree[slot] == 0U) {
                ready = static_cast<int>(slot);
                break;
            }
        }
        if (ready < 0) { return PluginStatus::CycleDetected; }
        selected[ready] = true;
        execution_slots_[plan][count++] = static_cast<uint16_t>(ready);
        // index：释放本计划后继的依赖。
        for (uint16_t index = 0U; index < edge_count_; ++index) {
            const PluginEdge& edge = edges_[index]; // 当前依赖。
            if (!edge.delayed && edgeApplies(edge, plan) && edge.before_slot == ready) {
                --indegree[edge.after_slot];
            }
        }
    }
    return PluginStatus::Ok;
}

// 固定按计划 0 初始化一次，禁止在帧或生命周期回调中递归调用。
PluginStatus PluginGraph::configureAll()
{
    if (!compiled_ || frame_in_progress_ || lifecycle_in_progress_) {
        return PluginStatus::InvalidState;
    }
    lifecycle_in_progress_ = true;
    // index：计划0的配置顺序位置。
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        const PluginStatus status = nodes_[execution_slots_[0U][index]]->configure(); // 本节点结果。
        if (status != PluginStatus::Ok) {
            lifecycle_in_progress_ = false;
            return status;
        }
    }
    lifecycle_in_progress_ = false;
    return PluginStatus::Ok;
}

// 固定按计划 0 启动；真正的首次进入通知发生在下一帧边界。
PluginStatus PluginGraph::startAll()
{
    if (!compiled_ || frame_in_progress_ || lifecycle_in_progress_) {
        return PluginStatus::InvalidState;
    }
    lifecycle_in_progress_ = true;
    // index：计划0的启动顺序位置。
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        const PluginStatus status = nodes_[execution_slots_[0U][index]]->start(); // 本节点结果。
        if (status != PluginStatus::Ok) {
            lifecycle_in_progress_ = false;
            return status;
        }
    }
    lifecycle_in_progress_ = false;
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

// slot：节点下标；容器禁用或故障也阻止其内部节点执行。
bool PluginGraph::hierarchyRunnable(uint16_t slot) const
{
    int current = static_cast<int>(slot); // 沿对象组合关系向上检查。
    while (current >= 0) {
        const PluginState state = nodes_[current]->state(); // 当前层运行状态。
        if (state != PluginState::Running && state != PluginState::Degraded) {
            return false;
        }
        current = conditions_[current].parent;
    }
    return true;
}

// context/previous_plan：当前帧及上一计划；通知完成后才进入业务计算。
void PluginGraph::updateTransitions(const FrameContext& context, uint16_t previous_plan)
{
    bool next_active[ROBOT_MAX_PLUGIN_NODES]{}; // 本帧锁存激活集合，不随回调请求变化。
    last_transition_.frame_id = context.frame_id;
    last_transition_.timestamp_us = context.timestamp_us;
    last_transition_.previous_plan = previous_plan;
    last_transition_.current_plan = active_plan_;
    last_transition_.plan_generation = plan_generation_;
    last_transition_.stopping = false;
    // slot：注册下标；输入短暂缺失不导致反复进入/退出。
    for (uint16_t slot = 0U; slot < node_count_; ++slot) {
        next_active[slot] = branchActive(slot) && hierarchyRunnable(slot);
    }
    // index：上一执行计划逆序；先完成全部退出通知。
    for (uint16_t index = execution_count_; index > 0U; --index) {
        const uint16_t slot = execution_slots_[previous_plan][index - 1U]; // 退出检查下标。
        NodeExecutionStats& stats = stats_[slot]; // 节点通知统计。
        if (stats.active && !next_active[slot]) {
            stats.active = false;
            ++stats.exit_count;
            nodes_[slot]->onExit(last_transition_);
        }
    }
    // index：当前计划顺序；新激活只进入，继续激活者才收到计划变化。
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        const uint16_t slot = execution_slots_[active_plan_][index]; // 通知检查下标。
        NodeExecutionStats& stats = stats_[slot]; // 节点通知统计。
        if (!next_active[slot]) { continue; }
        if (!stats.active) {
            stats.active = true;
            ++stats.enter_count;
            nodes_[slot]->onEnter(last_transition_);
        } else if (previous_plan != active_plan_) {
            ++stats.plan_change_count;
            nodes_[slot]->onPlanChanged(last_transition_);
        }
    }
}

// slot：消费节点；分支汇合不忽略故障、未启动或无数据。
bool PluginGraph::dependenciesReady(uint16_t slot) const
{
    bool has_alternative = false; // 是否声明了分支汇合输入。
    bool completed_alternative = false; // 是否有至少一个分支实际完成。
    // index：本帧依赖边下标。
    for (uint16_t index = 0U; index < edge_count_; ++index) {
        const PluginEdge& edge = edges_[index]; // 当前依赖。
        if (edge.delayed || edge.after_slot != slot || !edgeApplies(edge, active_plan_)) {
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
    if (!compiled_ || frame_in_progress_ || lifecycle_in_progress_) {
        return ProcessResult::Fault;
    }
    frame_in_progress_ = true;
    const uint16_t previous_plan = active_plan_; // 本帧切换前计划。
    latchPlan();
    // index：选择器下标；所有请求在第一节点执行前统一锁存。
    for (uint16_t index = 0U; index < selector_count_; ++index) {
        selectors_[index]->latch();
    }
    // index：注册下标；清除上一帧调度状态。
    for (uint16_t index = 0U; index < node_count_; ++index) {
        stats_[index].frame_state = NodeFrameState::Pending;
        stats_[index].frame_order = 0xFFFFU;
    }
    updateTransitions(context, previous_plan);
    ProcessResult aggregate = ProcessResult::Ok; // 本帧汇总结果。
    uint16_t execution_ordinal = 0U; // 本帧实际 process 调用序号。
    // index：已编译执行顺序下标。
    for (uint16_t index = 0U; index < execution_count_; ++index) {
        const uint16_t slot = execution_slots_[active_plan_][index]; // 本计划注册下标。
        PluginNode& node = *nodes_[slot]; // 当前节点。
        NodeExecutionStats& stats = stats_[slot]; // 当前节点统计。
        if (!branchActive(slot)) {
            stats.frame_state = NodeFrameState::SkippedBranch;
            ++stats.branch_skip_count;
            node.onSkipped(context);
            continue;
        }
        if (!stats.active || !hierarchyRunnable(slot)) {
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
        stats.frame_order = execution_ordinal++;
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
    frame_in_progress_ = false;
    return aggregate;
}

// 帧外停止：先退出当前激活节点，再按初始化计划0逆序停止全部节点。
void PluginGraph::stopAll()
{
    if (!compiled_ || frame_in_progress_ || lifecycle_in_progress_) {
        return;
    }
    lifecycle_in_progress_ = true;
    NodeTransitionContext transition = last_transition_; // 停止沿用最近帧时间。
    transition.previous_plan = active_plan_;
    transition.current_plan = active_plan_;
    transition.stopping = true;
    // index：当前计划逆序，退出只通知一次。
    for (uint16_t index = execution_count_; index > 0U; --index) {
        const uint16_t slot = execution_slots_[active_plan_][index - 1U]; // 当前节点下标。
        NodeExecutionStats& stats = stats_[slot]; // 激活状态与统计。
        if (stats.active) {
            stats.active = false;
            ++stats.exit_count;
            nodes_[slot]->onExit(transition);
        }
    }
    // index：固定生命周期停止顺序。
    for (uint16_t index = execution_count_; index > 0U; --index) {
        PluginNode& node = *nodes_[execution_slots_[0U][index - 1U]]; // 待停止节点。
        if (node.state() != PluginState::Stopped) { node.stop(); }
    }
    lifecycle_in_progress_ = false;
}

} // namespace robot::framework


