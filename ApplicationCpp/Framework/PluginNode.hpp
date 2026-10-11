/*
 * PluginNode.hpp —— 插件框架中「节点」的基类定义。
 *
 * 一、节点在框架中的角色
 *   整个上位机被拆成一张有向图（PluginGraph），图上的每个点就是一个节点
 *   （PluginNode 的派生类）。节点是「最小的可执行单元」：接收一帧输入，
 *   产出一帧结果。底盘解算、云台控制、电机驱动等都会各自派生一个节点。
 *
 * 二、「子图」的概念
 *   节点内部还可以再挂一张自己的图（成员 children_）。好处是：一个复杂子系统
 *   （例如底盘）可以在自己的 compose() 里声明内部的若干子节点和它们之间的依赖，
 *   而对外只表现为一个节点。带子图的节点叫「组合节点」；子图也可以为空
 *   （就是最普通的叶子节点）。compose 阶段框架会把所有层级的子图递归「摊平」
 *   进同一张根图，所以运行期并不区分层级。
 *
 * 三、「生命周期」与调用顺序
 *   节点从创建到停止依次经过下列回调，全部由框架（PluginGraph）统一驱动：
 *     1) 构造       —— 创建对象，此时状态为 Created。
 *     2) compose    —— 启动前调用一次，声明内部子节点与依赖边（只声明，不干活）。
 *     3) configure  —— 图冻结（compile）之后调用一次，加载静态参数。
 *     4) start      —— 进入运行态前调用一次。
 *     5) process    —— 每个执行周期调用，是真正的业务入口（纯虚，必须实现）。
 *     6) onEnter / onExit / onPlanChanged —— 帧边界的「进入/退出/计划切换」通知。
 *     7) onSkipped  —— 本帧该节点被跳过时调用。
 *     8) stop       —— 停机时调用，只改状态，不销毁对象。
 *   由于子图会被摊平进根图，上述回调对「组合节点」内部的小节点同样按根图顺序
 *   逐个调用，无需在组合节点里手动转发。
 *
 * 四、「虚拟函数（virtual）」
 *   带 virtual 的方法都是「可被派生类重写」的钩子：基类给出了默认实现
 *   （大多只是改状态或什么都不做），派生类按需重写。process() 是纯虚函数（= 0），
 *   没有默认实现，派生类必须实现它，否则无法实例化。
 *
 * 本文件只定义接口与少量状态管理，不含任何业务逻辑。
 */

#ifndef ROBOT_PLUGIN_NODE_HPP
#define ROBOT_PLUGIN_NODE_HPP

#include "PluginGraph.hpp"
#include "PluginTypes.hpp"
#include "NodeTransitionContext.hpp"

namespace robot::framework {

// 所有层级都使用同一个节点接口；节点可以拥有空的或非空的内部子图。
// 也就是说：无论是顶层底盘节点，还是某个组合节点内部的小节点，类型都是
// PluginNode 的派生类，框架对它们一视同仁。
class PluginNode {
public:
    // 构造函数：使用固定 ID 创建节点。
    // 参数 id：节点在图内的唯一编号（PluginId，16 位无符号整数）。
    // 说明：需由调用方保证 ID 在全图内唯一，validate() 会检查重复。
    explicit PluginNode(PluginId id) : id_(id) {}
    // 虚析构函数：保证通过 PluginNode* 删除派生类对象时能正确析构。
    // 之所以要 virtual，是因为框架用基类指针数组 nodes_[] 统一持有所有节点。
    virtual ~PluginNode() = default;

    // 返回本节点的唯一 ID。
    PluginId id() const { return id_; }
    // 返回当前生命周期状态（PluginState：Created/Configured/Running/...）。
    PluginState state() const { return snapshot_.state; }
    // 返回当前健康状态（PluginHealth：Unknown/Healthy/Degraded/Faulted）。
    PluginHealth health() const { return snapshot_.health; }
    // 返回只读的调试快照，供故障诊断和调试窗口读取
    // （含状态、健康度、最近一次处理结果、故障码）。
    const PluginHealthSnapshot& healthSnapshot() const { return snapshot_; }

    // 每个节点都自带一张内部子图，它可以是空的（叶子节点）。
    // 返回可编辑的内部子图。仅在 compose() 阶段使用：在这里 add() 子节点、
    // 用 addDependency() 声明依赖；图冻结后框架会把它摊平进根图。
    PluginGraph& children() { return children_; }
    // 返回只读的内部子图，供框架在 const 语境下遍历。
    const PluginGraph& children() const { return children_; }

    // 【生命周期第 2 步】compose：只负责声明内部节点和依赖，不执行业务。
    // 何时调用：启动阶段由 PluginGraph::compose() 递归调用，每个节点至多一次
    //           （框架用 composed_ 标志保证不重复声明）。
    // 返回：PluginStatus::Ok 表示声明成功；其它值表示失败，会中止启动流程。
    // 默认行为：什么都不做，直接返回 Ok（适合没有内部子图的叶子节点）。
    // 重写建议：组合节点在这里调用 children().add(...) / addDependency(...)。
    virtual PluginStatus compose() { return PluginStatus::Ok; }
    // 【生命周期第 3 步】configure：在图冻结（compile）后调用一次，适合加载静态参数。
    // 何时调用：由 PluginGraph::configureAll() 按计划 0 的顺序依次调用。
    // 返回：PluginStatus::Ok 表示配置成功；非 Ok 会中止配置流程。
    // 默认行为：状态置为 Configured、健康置为 Healthy，返回 Ok。
    virtual PluginStatus configure()
    {
        snapshot_.state = PluginState::Configured;
        snapshot_.health = PluginHealth::Healthy;
        return PluginStatus::Ok;
    }
    // 【生命周期第 4 步】start：在进入运行态前调用一次。
    // 何时调用：由 PluginGraph::startAll() 按计划 0 的顺序依次调用。
    // 返回：PluginStatus::Ok 表示启动成功；非 Ok 会中止启动流程。
    // 默认行为：状态置为 Running、健康置为 Healthy，返回 Ok。
    // 注意：这里只是把节点标成「运行态」，真正的首次 onEnter 通知发生在下一帧边界。
    virtual PluginStatus start()
    {
        snapshot_.state = PluginState::Running;
        snapshot_.health = PluginHealth::Healthy;
        return PluginStatus::Ok;
    }
    // 【生命周期·过渡通知】transition：首次/重新激活通知；默认不清空业务状态。
    // 何时调用：帧边界由 PluginGraph::updateTransitions() 调用，只有「本帧由未激活
    //           变为激活」的节点才会收到。
    // 参数 transition：帧边界上下文（帧号、时间戳、计划切换信息等）。
    // 默认行为：什么都不做。重写时通常用来重置本节点或子图的业务状态。
    virtual void onEnter(const NodeTransitionContext& transition) { (void)transition; }
    // 【生命周期·过渡通知】transition：失去激活或整图停止通知；默认不释放静态资源。
    // 何时调用：帧边界由 updateTransitions() 调用（由激活变为不激活时）；整图停止时
    //           也会由 PluginGraph::stopAll() 调用一次。
    // 默认行为：什么都不做。重写时通常用来暂停输出、释放运行期资源。
    virtual void onExit(const NodeTransitionContext& transition) { (void)transition; }
    // 【生命周期·过渡通知】transition：保持激活时计划改变；新激活节点只收到 onEnter。
    // 何时调用：帧边界由 updateTransitions() 调用，仅当节点「上一帧和这一帧都激活」
    //           且执行计划（active_plan）发生了切换。
    // 默认行为：什么都不做。重写时用它处理「换计划但不换节点」的平滑过渡。
    virtual void onPlanChanged(const NodeTransitionContext& transition) { (void)transition; }
    // 【生命周期第 5 步】process 是节点的业务入口，由执行器按节点周期调用。
    // 纯虚函数（= 0）：派生类必须实现，否则该类不能被实例化。
    // 何时调用：每个执行周期由 PluginGraph::processFrame() 调用（仅当节点激活、
    //           层级可运行且所有本帧输入依赖就绪时才真正执行）。
    // 参数 context：本帧的时间上下文（帧号、周期、时间戳），同一帧内所有节点共享。
    // 返回 ProcessResult：本次处理结果（Ok/NoNewData/OutputHeld/Degraded/Fault），
    //           框架据此更新节点状态与健康度。
    virtual ProcessResult process(FrameContext& context) = 0;
    // context：被跳过的帧；有输出的分支节点应在重写中使输出失效。
    // 何时调用：本帧该节点没有被 process（分支未选中、状态不可运行、依赖未就绪、
    //           上游故障或无数据等）时，由 processFrame() 调用。
    // 默认行为：什么都不做。若你的节点会把上一帧输出留在外部（例如电机指令），
    //           应重写此函数把输出标记为无效，避免下游误用过期数据。
    virtual void onSkipped(FrameContext& context) { (void)context; }
    // 【生命周期第 8 步】stop 只改变运行状态，不释放静态对象。
    // 何时调用：停机时由 PluginGraph::stopAll() 按计划 0 的逆序调用（已是 Stopped 的跳过）。
    // 默认行为：状态置为 Stopped。
    virtual void stop()
    {
        snapshot_.state = PluginState::Stopped;
    }

protected:
    // 将节点置为禁用状态（Disabled/Degraded）。供派生类在运行期主动下线自己。
    // 被禁用的节点不会被删除，只是不再执行 process，其子节点也会一并停摆。
    void disable()
    {
        snapshot_.state = PluginState::Disabled;
        snapshot_.health = PluginHealth::Degraded;
    }

    // 记录节点故障码并进入 Faulted（故障）状态。供派生类报告不可恢复的错误。
    // 参数 fault_code：自定义故障编号，之后可从 healthSnapshot().fault_code 读出。
    void setFault(uint32_t fault_code)
    {
        snapshot_.state = PluginState::Faulted;
        snapshot_.health = PluginHealth::Faulted;
        snapshot_.fault_code = fault_code;
    }

private:
    // 友元：允许图与节点互相访问对方的私有成员（节点注册、状态汇报都在图内完成）。
    friend class PluginGraph;

    // 保存最近一次 process 结果，并据此更新状态和健康度。
    // 调用者：仅由 PluginGraph::processFrame() 在每次 process 之后调用，业务代码无需关心。
    // 参数 result：本次 process 的返回值。
    void recordProcessResult(ProcessResult result)
    {
        snapshot_.last_result = result;
        if (result == ProcessResult::Fault) {
            snapshot_.state = PluginState::Faulted;
            snapshot_.health = PluginHealth::Faulted;
        } else if (result == ProcessResult::Degraded) {
            snapshot_.state = PluginState::Degraded;
            snapshot_.health = PluginHealth::Degraded;
        }
    }

    // 节点唯一 ID。默认 kInvalidPluginId（0xFFFF，表示尚未赋予合法 ID）。
    PluginId id_{kInvalidPluginId};
    // 节点内部子图。默认为空图；由本节点在 compose() 中声明。
    PluginGraph children_{};
    // 节点健康和处理结果快照（默认 state=Created、health=Unknown）。
    PluginHealthSnapshot snapshot_{};
    // 是否已经调用过 compose。由 PluginGraph::composeNode() 负责置位，防止重复声明。
    bool composed_{false};
};

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_NODE_HPP */


