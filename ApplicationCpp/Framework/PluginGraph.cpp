/**
 * @file plugin_graph.cpp
 * @brief 插件图的实现：负责节点/依赖的注册、组装(compose)、编译(compile)、
 *        生命周期管理(configure/start/stop)以及逐帧执行(process_frame)。
 *
 * 本文件是 RoboMaster 哨兵机器人 STM32H7 上位机"C++17 插件框架"的核心调度器。
 * 一句话概括：把若干插件节点和它们之间的依赖关系组织成一张有向图，
 * 编译成一条确定的执行顺序，然后每个控制周期按该顺序跑一遍。
 *
 * 关键术语（先有印象，细节见各函数）：
 *   - 拓扑排序：把有向图排成一个线性序列，保证"每条边的起点都排在终点之前"，
 *     这里用它决定每帧各节点谁先谁后执行。
 *   - 入度(indegree)：指向某节点的边的条数。拓扑排序中入度为 0 表示"前驱都没了/前驱都跑完"，
 *     可以安全执行。
 *   - 锁存(latch)：在某个时刻把"瞬时值"固定成"本周期使用的值"，周期内不再变化。例如分支
 *     选择结果和当前计划都会在执行前锁存，避免一帧中途被改。
 *   - 汇合(merge)：分支结构里多条互斥分支编号最终汇聚到同一个下游节点，该下游节点就是汇合点。
 *
 * 所有数据都存放在固定容量的线性数组里（见 plugin_graph.hpp 中的 ROBOT_MAX_* 常量），
 * 运行期不做任何动态内存分配，满足嵌入式实时性与确定性要求。
 */

#include "PluginGraph.hpp"

#include "PluginNode.hpp"

namespace robot::framework {

/**
 * 在已注册节点数组 nodes_ 中线性查找某个节点指针的位置。
 *
 * @param node 要查找的节点指针（可能不属于本图）。
 * @return 找到时返回其数组下标（>=0）；未找到时返回 -1。
 *
 * 关键不变量：下标即"注册槽位(slot)"，同一节点在本图生命周期内 slot 固定不变，
 *             因此 conditions_/stats_ 等并行数组都用它作索引。
 * 为何用线性查找：节点数量上限很小（ROBOT_MAX_PLUGIN_NODES），且该查找几乎只在
 *             组装/编译阶段调用，简单扫描足够快，无需哈希表。
 */
int plugin_graph::find_node_index(const plugin_node* node) const
{
    // index：被扫描的节点槽位下标，从 0 递增到 node_count_-1。
    for (uint16_t index = 0U; index < node_count_; ++index)
    {
        if (nodes_[index] == node)
        {
            return static_cast<int>(index);
        }
    }
    // 未命中：返回 -1 表示该节点不属于本图。
    return -1;
}

/**
 * 查找是否已存在一条"等价或重叠"的依赖边，用于判重/冲突检测。
 *
 * @param before 边的起点(前驱)
 * @param after  边的终点(后继)
 * @param is_delayed 该边是否跨帧(is_delayed)
 * @param plan_id 该边所属的执行计划编号(k_all_execution_plans 表示适用于全部计划)
 * @return 找到重叠边时返回其下标(>=0)，否则返回 -1。
 *
 * "重叠(is_overlap)"的含义：两条边的起点、终点、is_delayed 都相同，且它们作用的计划集合有交集。
 *   - 普通边 plan_id = k_all_execution_plans，会与任何计划重叠；
 *   - 专属边 plan_id = 某个具体计划，只在计划也相等(或对方是全部计划)时才算重叠。
 * 这样做既允许"同一对节点在不同计划里各声明一条边"，又能拦截真正重复的声明。
 */
int plugin_graph::find_overlapping_edge(const plugin_node* before, const plugin_node* after,
                                    bool is_delayed, uint16_t plan_id) const
{
    // index：已注册依赖边的扫描下标。
    for (uint16_t index = 0U; index < edge_count_; ++index)
    {
        const plugin_edge& edge = edges_[index]; // 当前已有边。
        // is_overlap：本次查询的 plan_id 与已有边的 plan_id 是否作用于同一个计划集合。
        //   k_all_execution_plans 是"通配"，只要一方是它就算重叠。
        const bool is_overlap = edge.plan_id == k_all_execution_plans || plan_id == k_all_execution_plans
            || edge.plan_id == plan_id; // 作用计划是否相交。
        if (edge.before == before && edge.after == after && edge.is_delayed == is_delayed && is_overlap)
        {
            return static_cast<int>(index);
        }
    }
    // 没有找到重叠边。
    return -1;
}

/**
 * 判断一条依赖边是否在指定计划下生效。
 *
 * @param edge 待判断的依赖边。
 * @param plan_id 当前正在处理/执行的计划编号。
 * @return true 表示该边参与该计划的拓扑排序与依赖检查。
 *
 * 规则：普通边(plan_id = k_all_execution_plans)在所有计划中都生效；
 *       专属边只有 plan_id 与当前 plan_id 相等时才生效。
 */
bool plugin_graph::edge_applies(const plugin_edge& edge, uint16_t plan_id)
{
    return edge.plan_id == k_all_execution_plans || edge.plan_id == plan_id;
}

/**
 * 设置根执行器的全局"执行计划"总数（必须在 compile() 之前调用，且只能设置一次）。
 *
 * @param count 计划数量，取值范围 [1, ROBOT_MAX_EXECUTION_PLANS]。
 * @return ok 成功；frozen 图已冻结；invalid_state 已经尝试过编译；configuration_fault 数量非法。
 *
 * 背景：一个"执行计划(plan)"就是一套完整的节点执行顺序。同一张图在不同业务模式下
 *       可以有不同的先后顺序（例如底盘优先还是云台优先），每种模式对应一个 plan。
 */
plugin_status plugin_graph::set_plan_count(uint16_t count)
{
    if (is_frozen_)
    {
        return plugin_status::frozen;
    } // 图已冻结，配置不可再改。
    if (is_compile_attempted_)
    {
        return plugin_status::invalid_state;
    } // 编译已开始，禁止再改计划数。
    if (count == 0U || count > ROBOT_MAX_EXECUTION_PLANS)
    { // 数量非法：至少 1 且不超上限。
        return plugin_status::configuration_fault;
    }
    plan_count_ = count;
    return plugin_status::ok;
}

/**
 * 注册一条"仅属于某个计划"的依赖边。edge中加上before和after和plan id
 *
 * @param before  起点(前驱)节点
 * @param after   终点(后继)节点
 * @param plan_id 该边所属的计划编号，必须 < ROBOT_MAX_EXECUTION_PLANS
 * @param is_delayed 是否为跨帧边(读取上一周期数据，不参与本帧排序)
 * @return 透传 add_edge 的结果；plan_id 越界时返回 configuration_fault。
 *
 * 说明：端点可能位于尚未展开的子图，因此这里不校验节点的存在性，
 *       真正的归属检查推迟到 compile() 展开子图之后统一进行。
 */
plugin_status plugin_graph::add_plan_dependency(plugin_node& before, plugin_node& after, uint16_t plan_id, bool is_delayed)
{
    if (plan_id >= ROBOT_MAX_EXECUTION_PLANS)
    {
        return plugin_status::configuration_fault;
    } // 计划号越界。
    return add_edge(before, after, is_delayed, false, plan_id);
}

/**
 * 由执行任务在"帧之间"提交下一次要执行的计划请求（只记录，不立即改当前顺序）。
 *
 * @param plan_id 请求的计划编号。
 * @return true 请求已接受；false 图未编译或计划号越界（此时保留上一次的请求不变）。
 *
 * 为什么只记录：真正生效要等到下一帧开始时由 latch_plan() 统一锁存，
 *               以保证一帧内部执行顺序自始至终一致。
 */
bool plugin_graph::request_plan(uint32_t plan_id)
{
    //检测合法性和是否生成计划
    if (!is_compiled_ || plan_id >= plan_count_)
    {
        return false;
    }

    requested_plan_id_ = static_cast<uint16_t>(plan_id);
    return true;
}

/**
 * 更新当前帧的计划id
 *
 * 关键点：
 *   - 锁存(latch)：本帧后续所有逻辑都只看 active_plan_id_，不受帧中途再次 request_plan 影响。
 *   - 重复请求同一个计划不算切换，因此 plan_switch_count_ 不会递增。
 *   - plan_switch_count_ 同时充当"计划版本号"，供节点感知切换事件。
 */
void plugin_graph::latch_plan()
{
    if (active_plan_id_ != requested_plan_id_)
    { // 只有请求与当前不同才是真正的切换。
        active_plan_id_ = requested_plan_id_;
        ++plan_switch_count_; // 累计切换次数
    }
}

/**
 * 注册一个插件节点到本图。
 *
 * @param node 要注册的节点引用。
 * @return ok 成功；frozen 图已冻结；duplicate_node 指针或 ID 重复；
 *         capacity_exceeded 已超过最大节点数。
 */
plugin_status plugin_graph::add(plugin_node& node)
{
    // region 合法性检测
    if (is_frozen_)
    {
        return plugin_status::frozen;
    }
    if (find_node_index(&node) >= 0)
    { // 同一个对象指针不能注册两次。
        return plugin_status::duplicate_node;
    }
    // 检查已注册节点 ID有无重复
    for (uint16_t index = 0U; index < node_count_; ++index)
    {
        if (nodes_[index]->id() == node.id())
        {
            return plugin_status::duplicate_node;
        }
    }
    // 检查是否大于允许的最大节点数量
    if (node_count_ >= ROBOT_MAX_PLUGIN_NODES)
    {
        return plugin_status::capacity_exceeded;
    }
    // endregion

    nodes_[node_count_] = &node; // 写入下一个空槽位。
    ++node_count_;
    return plugin_status::ok;
}

/**
 * 注册一条普通(适用于全部计划)的节点依赖边。
 *
 * @param before  前驱节点：先于 after 执行。
 * @param after   后继节点。
 * @param is_delayed true 表示 is_delayed(跨帧)边，只表达"读取上一周期的反馈"，不阻塞本帧拓扑排序。
 * @return 透传 add_edge 的结果。
 */
plugin_status plugin_graph::add_dependency(plugin_node& before, plugin_node& after, bool is_delayed)
{
    return add_edge(before, after, is_delayed, false);
}

/**
 * 注册一条"分支汇合边"：允许下游汇合点在互斥分支未选中时被跳过而不是被阻塞。
 *
 * @param before 分支末端节点。
 * @param after  结果汇合点节点。
 * @return 透传 add_edge 的结果(is_allow_branch_skip = true)。
 *
 * 场景：if/else 两条互斥分支编号各自结束时，都指向上游同一个汇合节点。
 *       运行时只有一条分支编号真正执行，另一条被跳过；汇合边告诉调度器
 *       "被跳过的前驱不构成阻塞"，从而汇合点仍能按时执行。
 */
plugin_status plugin_graph::add_branch_dependency(plugin_node& before, plugin_node& after)
{
    return add_edge(before, after, false, true);
}

/**
 * 给节点对应的condition配置选择器和分支id
 *
 * @param node     本图中的直接节点(必须是直接注册的，不是子图内的)。
 * @param selector 分支选择器，决定当前哪条分支编号激活。
 * @param branch_id    该节点所属的分支编号编号。
 * @return ok 成功；frozen 图已冻结；missing_node 节点不属于本图；
 *         configuration_fault selector 未配置或它不包含该 branch_id。
 *
 * 注意：只对"直接节点"设置；其子图内节点会通过 parent 关系继承该条件（见 compose_node）。
 */
plugin_status plugin_graph::set_branch(plugin_node& node, branch_selector& selector, uint16_t branch_id)
{
    if (is_frozen_)
    {
        return plugin_status::frozen;
    }
    const int slot = find_node_index(&node); // 本图注册下标。
    if (slot < 0)
    {
        return plugin_status::missing_node;
    }
    if (!selector.all_is_valid() || !selector.branch_id_is_valid(branch_id))
    { // 选择器必须已配置且含该分支编号。
        return plugin_status::configuration_fault;
    }

    conditions_[slot].selector = &selector; // 记录条件。
    conditions_[slot].branch_id = branch_id;
    return plugin_status::ok;
}

/**
 * 查询某个节点的执行统计信息。
 *
 * @param node 要查询的节点。
 * @return 指向其统计结构的指针；节点不属于本图时返回 nullptr。
 *
 * 关键不变量：统计结构在图中按 slot 存放，其地址在图生命周期内固定不变，
 *             因此上层可以长期持有该指针读取最新计数。
 */
const node_execution_stats* plugin_graph::execution_stats(const plugin_node& node) const
{
    const int slot = find_node_index(&node); // 节点注册下标。
    return slot < 0 ? nullptr : &stats_[slot];
}

/**
 * 内部通用：向图中加入一条依赖边（所有对外的 add_dependency/add_branch_dependency/
 * add_plan_dependency 最终都走这里）。
 *
 * @param before            前驱节点
 * @param after             后继节点
 * @param is_delayed           是否跨帧边
 * @param is_allow_branch_skip 是否为"可跳过"的分支汇合边
 * @param plan_id           所属计划(k_all_execution_plans 表示全部计划)
 * @return ok 成功；frozen 图已冻结；cycle_detected 自环；duplicate_edge 重复边；
 *         configuration_fault 与已存在的重叠边语义冲突；capacity_exceeded 边数超限。
 */
plugin_status plugin_graph::add_edge(plugin_node& before, plugin_node& after, bool is_delayed,
                                 bool is_allow_branch_skip, uint16_t plan_id)
{
    if (is_frozen_)
    {
        return plugin_status::frozen;
    }

    // 全程自环检测在编译期
    if (&before == &after && !is_delayed)
    { // 非跨帧的自环(自己依赖自己)会死循环，直接拒绝。
        return plugin_status::cycle_detected;
    }

    // 找重叠边位置
    const int duplicate = find_overlapping_edge(&before, &after, is_delayed, plan_id); 

    if (duplicate >= 0)
    {
        const plugin_edge& edge = edges_[duplicate]; // 已有声明。
        // 不允许把计划专属边静默当成全部计划边，或混用汇合语义。
        // 只有"计划号完全一致 且 汇合语义一致"才认定为可忽略的重复；否则视为配置冲突。
        return edge.plan_id == plan_id && edge.is_allow_branch_skip == is_allow_branch_skip
            ? plugin_status::duplicate_edge : plugin_status::configuration_fault;
    }
    
    // 检测边数量有没有超出预定大小
    if (edge_count_ >= ROBOT_MAX_PLUGIN_EDGES)
    {
        return plugin_status::capacity_exceeded;
    }

    // 写入下一个空槽位：先聚合初始化，再单独补上末尾的 plan_id 字段。
    edges_[edge_count_] = plugin_edge{&before, &after, is_delayed, is_allow_branch_skip};
    edges_[edge_count_].plan_id = plan_id;
    ++edge_count_;
    return plugin_status::ok;
}

/**
 * 递归展开一个节点的"内部子图"，把它内部的节点与边全部并入根图(本图)。
 *
 * @param node 待展开的节点。
 * @return ok 成功；其余为子图组装/注册过程中的错误码。
 *
 * 核心思想：复合节点(内部还有子图)在编译期被"拍平"到根图中，所有子节点最终
 *           都进入同一个执行计划，统一排序、统一执行。
 * 关键不变量：
 *   - node.is_composed_ 作为"只展开一次"的标记，避免重复展开。
 *   - 每个子节点用 parent 记录它在展开图中的父节点 slot，供分支条件继承使用。
 *   - 展开完成后立即把子图冻结(child_graph.is_frozen_ = true)，防止返回成功后
 *     还有人去修改那份旧副本，导致根图与子图不一致。
 */
plugin_status plugin_graph::compose_node(plugin_node& node)
{
    if (node.is_composed_)
    { // 已经展开过，幂等返回。
        return plugin_status::ok;
    }

    node.is_composed_ = true;
    // status：子图组装结果。
    plugin_status status = node.compose(); // 让节点自己先注册其内部节点与边。
    if (status != plugin_status::ok)
    {
        return status;
    }

    // child_graph：当前节点的内部子图。
    plugin_graph& child_graph = node.children();
    // child_count：内部节点数量。
    const uint16_t child_count = child_graph.node_count();
    for (uint16_t index = 0U; index < child_count; ++index) 
    {
        // child：当前内部节点。
        plugin_node* child = child_graph.node_at(index);
        if (child == nullptr) 
        {
            return plugin_status::configuration_fault;
        }
        status = add(*child); // 把子节点注册进根图(排在本节点之后)。
        if (status != plugin_status::ok) 
        {
            return status;
        }
        // child_slot：刚加入展开图的子节点下标。
        const uint16_t child_slot = static_cast<uint16_t>(node_count_ - 1U);
        conditions_[child_slot] = child_graph.conditions_[index]; // 继承子图内的分支条件。
        // parent 指向本节点在根图中的 slot → 分支条件沿父链向上继承(见 branch_active)。
        conditions_[child_slot].parent = static_cast<int16_t>(find_node_index(&node));
        status = compose_node(*child); // 递归：子节点自己也可能有子图。
        if (status != plugin_status::ok) 
        {
            return status;
        }
    }

    // child_edge_count：内部依赖边数量。
    const uint16_t child_edge_count = child_graph.edge_count();
    for (uint16_t index = 0U; index < child_edge_count; ++index)
    {
        // edge：当前内部依赖边。
        const plugin_edge* edge = child_graph.edge_at(index);
        if (edge == nullptr)
        {
            return plugin_status::configuration_fault;
        }
        // 把子图内的边也搬进根图；重复边(duplicate_edge)可忽略，其余错误上抛。
        status = add_edge(*edge->before, *edge->after, edge->is_delayed, edge->is_allow_branch_skip, edge->plan_id);
        if (status != plugin_status::ok && status != plugin_status::duplicate_edge)
        {
            return status;
        }
    }
    // 子图关系已复制到根计划；立即冻结，禁止返回成功却只修改旧副本。
    child_graph.is_frozen_ = true;
    return plugin_status::ok;
}

/**
 * 组装总入口：对所有"根节点"逐个做 compose_node，把整张图拍平。
 *
 * @return ok 成功；frozen 图已冻结；其余为展开过程中的错误码。
 *
 * 关键点：只遍历 compose 之前就已存在的根节点(root_count 在循环前一次性取好)，
 *         因为展开过程会不断往 nodes_ 尾部追加子节点，若动态读取 node_count_
 *         就会顺着新节点又展开一遍。本函数只调用一次。
 */
plugin_status plugin_graph::compose()
{
    if (is_frozen_)
    {
        return plugin_status::frozen;
    }
    if (is_composed_)
    { // 幂等：已经组装过。
        return plugin_status::ok;
    }

    is_composed_ = true;
    // root_count：展开前的根节点数量(循环过程中不再变化)。
    const uint16_t root_count = node_count_;
    for (uint16_t index = 0U; index < root_count; ++index)
    {
        // status：当前根节点的展开结果。
        plugin_status status = compose_node(*nodes_[index]);
        if (status != plugin_status::ok)
        {
            return status;
        }
    }
    return plugin_status::ok;
}

/**
 * 展开完成后对全图做一致性校验。
 *
 * @return ok 通过；configuration_fault 节点为空/ID 非法或边所属计划越界；
 *         duplicate_node 存在重复 ID；missing_node 边的端点未注册到本图。
 *
 * 校验内容：
 *   1. 每个节点非空、ID 合法(k_invalid_plugin_id 表示未设置)；
 *   2. 任意两个节点 ID 不重复（双重循环两两比较）；
 *   3. 每条边的两个端点都已注册进本图（此前端点可能来自未展开的子图）；
 *   4. 每条边的计划号要么是通配(k_all_execution_plans)，要么落在已声明的计划范围内。
 */
plugin_status plugin_graph::validate() const
{
    for (uint16_t index = 0U; index < node_count_; ++index)
    {
        if (nodes_[index] == nullptr || nodes_[index]->id() == k_invalid_plugin_id)
        { // 空节点或非法 ID。
            return plugin_status::configuration_fault;
        }
        // other：与 index 两两比较，检查 ID 是否重复。
        for (uint16_t other = index + 1U; other < node_count_; ++other)
        {
            if (nodes_[index]->id() == nodes_[other]->id())
            {
                return plugin_status::duplicate_node;
            }
        }
    }

    for (uint16_t index = 0U; index < edge_count_; ++index)
    {
        const plugin_edge& edge = edges_[index]; // 当前待校验的边。
        if (edge.before == nullptr || edge.after == nullptr || // 端点为空的边非法。
            find_node_index(edge.before) < 0 || find_node_index(edge.after) < 0) 
        { // 端点不在本图。
            return plugin_status::missing_node;
        }
        if (edge.plan_id != k_all_execution_plans && edge.plan_id >= plan_count_)
        { // 计划号越界。
            return plugin_status::configuration_fault;
        }
    }
    return plugin_status::ok;
}

/**
 * 编译总入口：把图展开、校验、缓存索引，并为每个执行计划生成拓扑顺序，最后冻结图。
 *
 * @return ok 全部计划均编译成功；否则返回首次失败的错误码。
 *
 * 设计要点：
 *   - 只允许成功一次：is_compile_attempted_ 保证重复调用返回同一结果，
 *     失败的对象视为"不可用"，不允许使用部分编译结果。
 *   - 顺序：compose(展开) → validate(校验) → 收集选择器 → 缓存边端点下标
 *     → 逐计划 compile_plan(拓扑排序) → 冻结全部子图 → 标记 is_compiled_/is_frozen_。
 */
plugin_status plugin_graph::compile()
{
    // 是否冻结、编译
    if (is_frozen_)
    {
        return plugin_status::frozen;
    } 
    if (is_compile_attempted_)
    {
        return compile_status_;
    }
    is_compile_attempted_ = true;

    // 第一步：展开子图
    compile_status_ = compose(); 
    if (compile_status_ != plugin_status::ok)
    {
        return compile_status_;
    }

    // 第二步：全图校验
    compile_status_ = validate(); 
    if (compile_status_ != plugin_status::ok)
    {
        return compile_status_;
    }

    selector_count_ = 0U;

    // slot：展开图下标，收集去重选择器。
    for (uint16_t slot = 0U; slot < node_count_; ++slot)
    {
        const branch_condition& condition = conditions_[slot]; // 显式分支条件。
        if (condition.selector == nullptr)
        {
            continue;
        } // 没有条件的节点跳过。
        if (!condition.selector->all_is_valid() || !condition.selector->branch_id_is_valid(condition.branch_id))
        {
            compile_status_ = plugin_status::configuration_fault; // 条件非法。
            return compile_status_;
        }
        // 去重
        bool is_found = false; // 是否已收集。
        for (uint16_t index = 0U; index < selector_count_; ++index)
        {
            is_found = is_found || selectors_[index] == condition.selector;
        }

        if (!is_found)
        {
            selectors_[selector_count_++] = condition.selector;
        } // 首次出现才登记。
    }

    // index：依赖边下标，缓存端点。把指针换算成稳定的 slot 下标，运行期避免反复查找。
    for (uint16_t index = 0U; index < edge_count_; ++index)
    {
        edges_[index].before_slot = static_cast<uint16_t>(find_node_index(edges_[index].before));
        edges_[index].after_slot = static_cast<uint16_t>(find_node_index(edges_[index].after));
    }

    // 为每个plan画一张execution_slots_节点执行表
    for (uint16_t plan_id = 0U; plan_id < plan_count_; ++plan_id)
    {
        compile_status_ = compile_plan(plan_id);
        if (compile_status_ != plugin_status::ok)
        {
            failed_plan_id_ = plan_id; // 记录失败的计划号，便于上层报错定位。
            execution_count_ = 0U; // 失败即作废，不能使用部分结果。
            return compile_status_;
        }
    }

    // index：展开图下标，冻结所有子图。此后子图结构只读。
    for (uint16_t index = 0U; index < node_count_; ++index)
    {
        nodes_[index]->children().is_frozen_ = true;
    }
    execution_count_ = node_count_; // 之后每帧都执行全部展开节点。
    is_compiled_ = true;
    is_frozen_ = true;
    return compile_status_;
}

/**
 * 为单个plan画出一个execution_slots_执行表（Kahn 算法）。
 *
 * @param plan_id 要排序的计划编号。
 * @return ok 排序成功；cycle_detected 图中存在环(再也找不到就绪节点 → 死锁)。
 *
 * 拓扑排序(Kahn)：反复"取出一个入度为 0 的节点 → 输出它 → 把它所有后继的入度减 1"，
 *   直到所有节点都被输出。入度(indegree)指指向该节点的边数，入度为 0 意味着
 *   它没有(未满足的)前驱，可以安全执行。若某轮找不到入度为 0 的节点，说明存在环。
 *   is_delayed(跨帧)边不参与本帧排序：它表达"读上一周期数据"，不构成同帧先后约束。
 * 稳定性：每轮从 slot 0 起扫描第一个入度为 0 的节点，保证顺序确定、可复现。
 * 不同计划各自调用本函数，允许对同一对节点给出相反的执行顺序。
 */
plugin_status plugin_graph::compile_plan(uint16_t plan_id)
{
    uint16_t indegree[ROBOT_MAX_PLUGIN_NODES]{}; // 每个节点的剩余前驱数(初值全 0)。
    bool is_selected[ROBOT_MAX_PLUGIN_NODES]{}; // 每个节点是否已加入本计划顺序。
    uint16_t count = 0U; // 本计划已排序数量，达到 node_count_ 即完成。

    //计算每一个节点的依赖节点数
    for (uint16_t index = 0U; index < edge_count_; ++index)
    {
        const plugin_edge& edge = edges_[index];
        // 不跨帧且在本计划中可用
        if (!edge.is_delayed && edge_applies(edge, plan_id))
        {
            ++indegree[edge.after_slot];
        }
    }

    while (count < node_count_)
    { // 每轮输出一个节点，直到全部输出。
        int ready = -1; // 本轮就绪下标(-1 表示没找到)。
        // slot：按注册顺序稳定选取节点。
        for (uint16_t slot = 0U; slot < node_count_; ++slot)
        {
            // 还没被编译进图且此时依赖节点数为0
            if (!is_selected[slot] && indegree[slot] == 0U)
            {
                ready = static_cast<int>(slot);
                break;
            }
        }
        if (ready < 0)
        {
            return plugin_status::cycle_detected;
        } // 有环：再也找不到就绪节点。
        is_selected[ready] = true;
        execution_slots_[plan_id][count++] = static_cast<uint16_t>(ready); // 记录到本计划顺序中。
        // index：释放本计划后继的依赖。删除"从 ready 出发的有效边"，让后继入度减 1。
        for (uint16_t index = 0U; index < edge_count_; ++index)
        {
            const plugin_edge& edge = edges_[index]; // 当前依赖。

            // 不跨帧且这个边在计划中且前一帧就是这个发现前驱帧数为0的帧
            if (!edge.is_delayed && edge_applies(edge, plan_id) && edge.before_slot == ready)
            {
                --indegree[edge.after_slot];// -1前驱数
            }
        }
    }
    return plugin_status::ok;
}

/**
 * 生命周期：按计划 0 的顺序，依次调用每个节点的 configure()（初始化一次）。
 *
 * @return ok 全部成功；invalid_state 未编译或正处于帧/生命周期回调中；否则为某节点的错误码。
 *
 * 关键约束：
 *   - 固定用计划 0 的顺序做初始化，保证所有节点无论属于哪个分支都能被初始化到。
 *   - is_lifecycle_in_progress_ / is_frame_in_progress_ 互斥，防止在回调里递归触发配置。
 *   - 任一节点失败即中止并复位标志，返回错误码。
 */
plugin_status plugin_graph::configure_all()
{
    if (!is_compiled_ || is_frame_in_progress_ || is_lifecycle_in_progress_)
    { // 前置状态检查。
        return plugin_status::invalid_state;
    }
    is_lifecycle_in_progress_ = true; // 进入生命周期回调，禁止重入。
    // index：计划0的配置顺序位置。
    for (uint16_t index = 0U; index < execution_count_; ++index)
    {
        const plugin_status status = nodes_[execution_slots_[0U][index]]->configure(); // 本节点结果。
        if (status != plugin_status::ok)
        {
            is_lifecycle_in_progress_ = false; // 失败也要复位标志，否则图将永久卡死。
            return status;
        }
    }
    is_lifecycle_in_progress_ = false;
    return plugin_status::ok;
}

/**
 * 生命周期：按计划 0 的顺序，依次调用每个节点的 start()。
 *
 * @return ok 全部成功；invalid_state 未编译或正处于帧/生命周期回调中；否则为某节点的错误码。
 *
 * 注意：真正的"首次进入(on_enter)"通知并不在这里发生，而是在下一帧边界由
 *       update_transitions 依据激活集合决定，因此 start 只负责节点自身的启动逻辑。
 */
plugin_status plugin_graph::start_all()
{
    if (!is_compiled_ || is_frame_in_progress_ || is_lifecycle_in_progress_)
    {
        return plugin_status::invalid_state;
    }
    is_lifecycle_in_progress_ = true;
    // index：计划0的启动顺序位置。
    for (uint16_t index = 0U; index < execution_count_; ++index)
    {
        const plugin_status status = nodes_[execution_slots_[0U][index]]->start(); // 本节点结果。
        if (status != plugin_status::ok)
        {
            is_lifecycle_in_progress_ = false;
            return status;
        }
    }
    is_lifecycle_in_progress_ = false;
    return plugin_status::ok;
}

/**
 * 判断某个节点在当前分支选择下是否"被激活"。
 *
 * @param slot 展开图节点下标。
 * @return true 表示该节点与其所有祖先的分支条件都满足。
 *
 * 规则：自身若带条件(selector != nullptr)且当前激活分支编号不等于它的 branch_id，则未激活；
 *       然后沿 parent 父链逐层向上检查，任一祖先不满足即整体不激活(逻辑与)。
 *       越靠近根的条件越"粗"，子节点因而继承了祖先的分支语义。
 */
bool plugin_graph::branch_active(uint16_t slot) const
{
    int current = static_cast<int>(slot); // 沿父关系向上检查的位置，-1 表示到达根之上。
    while (current >= 0)
    {
        const branch_condition& condition = conditions_[current]; // 当前层条件。
        if (condition.selector != nullptr && condition.selector->active_branch_id() != condition.branch_id)
        {
            return false; // 祖先或自身所在分支编号未被选中。
        }
        current = condition.parent; // 上移到父节点。
    }
    return true; // 全链条件都满足。
}

/**
 * 判断某节点在"组合层级"上是否可运行：其自身及所有祖先必须处于 running 或 degraded。
 *
 * @param slot 展开图节点下标。
 * @return true 表示整条父链都处于可运行状态。
 *
 * 与 branch_active 的区别：branch_active 看的是"分支条件"，这里看的是"运行状态"。
 * 若父容器被禁用/故障，其内部节点即便分支满足也不应执行。
 */
bool plugin_graph::hierarchy_runnable(uint16_t slot) const
{
    int current = static_cast<int>(slot); // 沿对象组合关系向上检查。
    while (current >= 0)
    {
        const plugin_state state = nodes_[current]->state(); // 当前层运行状态。
        if (state != plugin_state::running && state != plugin_state::degraded)
        { // 非运行/降级即不可用。
            return false;
        }
        current = conditions_[current].parent; // 上移一层。
    }
    return true;
}

/**
 * 帧开始时计算并下发"激活/退出"生命周期通知。
 *
 * @param context       本帧上下文(帧号、时间戳等)。
 * @param previous_plan_id 本帧切换前的计划编号(用于判断是否发生计划切换)。
 *
 * 关键不变量：先把本帧的激活集合 is_next_active[] 一次性算好并"锁存"，
 *             再统一做退出/进入通知；这样即使某个节点的回调里又改了请求，
 *             也不会造成本帧内反复进入/退出。
 * 通知顺序：
 *   1) 先按"上一计划"的顺序逆序，发出 on_exit(先退)；
 *   2) 再按"当前计划"的顺序，发出 on_enter / on_plan_changed(后进)。
 * 为何先退后进：保证同一帧内不会出现"新旧共存"的中间态。
 */
void plugin_graph::update_transitions(const frame_context& context, uint16_t previous_plan_id)
{
    bool is_next_active[ROBOT_MAX_PLUGIN_NODES]{}; // 本帧锁存激活集合，不随回调请求变化。
    // 填充本帧过渡上下文(共享给所有回调)。
    last_transition_.frame_id = context.frame_id;
    last_transition_.timestamp_us = context.timestamp_us;
    last_transition_.previous_plan_id = previous_plan_id;
    last_transition_.current_plan_id = active_plan_id_;
    last_transition_.plan_switch_count = plan_switch_count_;
    last_transition_.is_stopping = false;
    // slot：注册下标；输入短暂缺失不导致反复进入/退出。分支满足且层级可运行才算激活。
    for (uint16_t slot = 0U; slot < node_count_; ++slot)
    {
        is_next_active[slot] = branch_active(slot) && hierarchy_runnable(slot);
    }
    // index：上一执行计划逆序；先完成全部退出通知。
    for (uint16_t index = execution_count_; index > 0U; --index)
    {
        const uint16_t slot = execution_slots_[previous_plan_id][index - 1U]; // 退出检查下标。
        node_execution_stats& stats = stats_[slot]; // 节点通知统计。
        if (stats.is_active && !is_next_active[slot])
        { // 上帧激活、本帧不再激活 → 退出。
            stats.is_active = false;
            ++stats.exit_count;
            nodes_[slot]->on_exit(last_transition_);
        }
    }
    // index：当前计划顺序；新激活只进入，继续激活者才收到计划变化。
    for (uint16_t index = 0U; index < execution_count_; ++index)
    {
        const uint16_t slot = execution_slots_[active_plan_id_][index]; // 通知检查下标。
        node_execution_stats& stats = stats_[slot]; // 节点通知统计。
        if (!is_next_active[slot])
        {
            continue;
        } // 本帧不激活，跳过。
        if (!stats.is_active)
        { // 由"未激活"变为"激活" → 进入。
            stats.is_active = true;
            ++stats.enter_count;
            nodes_[slot]->on_enter(last_transition_);
        }
        else if (previous_plan_id != active_plan_id_)
        { // 一直激活但计划切了 → 计划变化。
            ++stats.plan_change_count;
            nodes_[slot]->on_plan_changed(last_transition_);
        }
    }
}

/**
 * 判断某节点的全部"本帧前驱依赖"是否已就绪，可以执行。
 *
 * @param slot 消费节点(即边终点 after)的下标。
 * @return true 表示依赖已满足，可以 process；false 表示本帧应阻塞。
 *
 * 规则：
 *   - 只看非 is_delayed、在当前计划生效、且终点为本节点的边；
 *   - 普通边：其前驱本帧状态必须是 executed(已执行)，否则阻塞；
 *   - 汇合边(is_allow_branch_skip)：把被跳过(skipped_branch)的前驱视为"条件未选中，可忽略"，
 *     但至少要有一个分支真正 executed；故障/未启动/无数据都不算完成。
 *   - 若没有任何汇合边，则普通边全通过即就绪。
 */
bool plugin_graph::dependencies_ready(uint16_t slot) const
{
    bool has_alternative = false; // 是否声明了分支汇合输入。
    bool is_completed_alternative = false; // 是否有至少一个分支实际完成。
    // index：本帧依赖边下标。
    for (uint16_t index = 0U; index < edge_count_; ++index)
    {
        const plugin_edge& edge = edges_[index]; // 当前依赖。
        // 延迟边/非本节点/非本计划的边都不参与本帧就绪判断。
        if (edge.is_delayed || edge.after_slot != slot || !edge_applies(edge, active_plan_id_))
        {
            continue;
        }
        const node_frame_state state = stats_[edge.before_slot].frame_state; // 前驱本帧状态。
        if (edge.is_allow_branch_skip)
        { // 汇合边：允许忽略被跳过的分支。
            has_alternative = true;
            if (state == node_frame_state::skipped_branch)
            { // 条件未选中 → 忽略该前驱。
                continue;
            }
            if (state != node_frame_state::executed)
            { // 其他非完成状态(故障/无数据…)一律阻塞。
                return false;
            }
            is_completed_alternative = true; // 至少一个分支已完成。
        }
        else if (state != node_frame_state::executed)
        { // 普通边必须有已完成的前驱。
            return false;
        }
    }
    // 没有汇合边 → 就绪；有汇合边 → 至少要有一个分支完成。
    return !has_alternative || is_completed_alternative;
}

/**
 * 执行当前一帧：锁存计划与选择器、下发生命周期通知、按顺序调度每个节点。
 *
 * @param context  本帧上下文(会被传给各节点的 process/on_skipped)。
 * @param profiler 性能采样器，可为 nullptr；只包裹真正执行的 process 调用。
 * @return 本帧汇总结果，严重度优先级 fault > degraded > output_held > ok。
 *
 * 单帧流程：
 *   1. 记录切换前计划，latch_plan() 锁存本帧计划；
 *   2. latch 所有分支选择器(本帧内不再变化)；
 *   3. 复位上一帧的调度状态(frame_state / frame_order)；
 *   4. update_transitions() 下发 on_exit/on_enter/on_plan_changed；
 *   5. 按本计划拓扑顺序遍历节点：不激活→跳过；不可运行→跳过；依赖未就绪→阻塞；
 *      否则 process，并按结果更新本帧状态与汇总结果。
 * 关键不变量：process_frame 与 configure/start/stop 通过 is_frame_in_progress_ /
 *             is_lifecycle_in_progress_ 互斥，禁止在节点回调中重入。
 */
process_result plugin_graph::process_frame(frame_context& context, i_profiler* profiler)
{
    // 未编译、上了锁、
    if (!is_compiled_ || is_frame_in_progress_ || is_lifecycle_in_progress_)
    { // 未编译或已在帧/生命周期中。
        return process_result::fault;
    }
    is_frame_in_progress_ = true; // 上锁

    // 更新plan id
    const uint16_t previous_plan_id = active_plan_id_; // 本帧计划
    latch_plan(); // 更新下一帧

    // 更新选择器的分支id
    for (uint16_t index = 0U; index < selector_count_; ++index)
    {
        selectors_[index]->latch();
    }
    // index：注册下标；清除上一帧调度状态。
    for (uint16_t index = 0U; index < node_count_; ++index)
    {
        stats_[index].frame_state = node_frame_state::pending; // 本帧初始为待定。
        stats_[index].frame_order = 0xFFFFU; // 0xFFFF 表示"本帧未执行"。
    }
    
    update_transitions(context, previous_plan_id); // 下发激活/退出通知。
    process_result aggregate = process_result::ok; // 本帧汇总结果，按严重度逐级升级。
    uint16_t execution_ordinal = 0U; // 本帧实际 process 调用序号(仅统计真正执行者)。
    // index：已编译执行顺序下标(按本计划拓扑序)。
    for (uint16_t index = 0U; index < execution_count_; ++index)
    {
        const uint16_t slot = execution_slots_[active_plan_id_][index]; // 本计划注册下标。
        plugin_node& node = *nodes_[slot]; // 当前节点。
        node_execution_stats& stats = stats_[slot]; // 当前节点统计。
        if (!branch_active(slot))
        { // 分支未选中：本帧不参与。
            stats.frame_state = node_frame_state::skipped_branch;
            ++stats.branch_skip_count;
            node.on_skipped(context); // 通知节点"因分支跳过"。
            continue;
        }
        if (!stats.is_active || !hierarchy_runnable(slot))
        { // 未激活或父层级不可运行。
            stats.frame_state = node_frame_state::skipped_state;
            ++stats.blocked_count;
            node.on_skipped(context);
            if (node.state() == plugin_state::faulted)
            { // 节点自身故障 → 整帧升为 fault。
                aggregate = process_result::fault;
            }
            else if (aggregate == process_result::ok)
            { // 否则降级为"输出保持"。
                aggregate = process_result::output_held;
            }
            continue;
        }
        if (!dependencies_ready(slot))
        { // 前驱数据未就绪：阻塞等待。
            stats.frame_state = node_frame_state::blocked;
            ++stats.blocked_count;
            node.on_skipped(context);
            if (aggregate == process_result::ok)
            {
                aggregate = process_result::output_held;
            }
            continue;
        }
        stats.frame_order = execution_ordinal++; // 记录本次执行序号。
        if (profiler != nullptr)
        {
            profiler->begin_node(node.id());
        } // 采样开始(仅包住 process)。
        const process_result result = node.process(context); // 本次业务结果。
        if (profiler != nullptr)
        {
            profiler->end_node();
        } // 采样结束。
        ++stats.execution_count;
        node.record_process_result(result); // 把结果反馈给节点自身。
        stats.frame_state = node_frame_state::executed; // 默认视为已执行。
        if (result == process_result::fault)
        { // 故障：最严重。
            stats.frame_state = node_frame_state::faulted;
            node.on_skipped(context);
            aggregate = process_result::fault;
        }
        else if (result == process_result::no_new_data)
        { // 无新数据：视为未产出新输出。
            stats.frame_state = node_frame_state::no_data;
            node.on_skipped(context);
            if (aggregate == process_result::ok)
            {
                aggregate = process_result::output_held;
            }
        }
        else if (result == process_result::degraded && aggregate != process_result::fault)
        { // 降级。
            aggregate = process_result::degraded;
        }
        else if (result == process_result::output_held && aggregate == process_result::ok)
        { // 保持输出。
            aggregate = process_result::output_held;
        }
    }
    is_frame_in_progress_ = false; // 退出帧处理。
    return aggregate;
}

/**
 * 生命周期：帧外停止全部节点。
 *
 * 图未编译或正处于帧/生命周期回调中则直接返回。
 *
 * 停止顺序：
 *   1. 按"当前计划"逆序，给仍激活的节点发 on_exit(每个只通知一次)；
 *   2. 再按固定的初始化顺序(计划 0)逆序，对尚未 stopped 的节点调用 stop()。
 * 为何逆序：与启动顺序相反，先停后启动者，符合资源依赖的释放次序。
 */
void plugin_graph::stop_all()
{
    if (!is_compiled_ || is_frame_in_progress_ || is_lifecycle_in_progress_)
    { // 前置状态检查。
        return;
    }
    is_lifecycle_in_progress_ = true; // 进入生命周期回调，禁止重入。
    node_transition_context transition = last_transition_; // 停止沿用最近帧时间。
    transition.previous_plan_id = active_plan_id_;
    transition.current_plan_id = active_plan_id_;
    transition.is_stopping = true; // 标记本次是停止通知。
    // index：当前计划逆序，退出只通知一次。
    for (uint16_t index = execution_count_; index > 0U; --index)
    {
        const uint16_t slot = execution_slots_[active_plan_id_][index - 1U]; // 当前节点下标。
        node_execution_stats& stats = stats_[slot]; // 激活状态与统计。
        if (stats.is_active)
        {
            stats.is_active = false;
            ++stats.exit_count;
            nodes_[slot]->on_exit(transition);
        }
    }
    // index：固定生命周期停止顺序(计划 0 逆序)。
    for (uint16_t index = execution_count_; index > 0U; --index)
    {
        plugin_node& node = *nodes_[execution_slots_[0U][index - 1U]]; // 待停止节点。
        if (node.state() != plugin_state::stopped)
        {
            node.stop();
        } // 幂等：已停止的跳过。
    }
    is_lifecycle_in_progress_ = false;
}

} // namespace robot::framework


