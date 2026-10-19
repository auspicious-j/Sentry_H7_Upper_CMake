/*
 * plugin_node.hpp —— 插件框架中「节点」的基类定义。
 *
 * 一、节点在框架中的角色
 *   整个上位机被拆成一张有向图（plugin_graph），图上的每个点就是一个节点
 *   （plugin_node 的派生类）。节点是「最小的可执行单元」：接收一帧输入，
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
 *   节点从创建到停止依次经过下列回调，全部由框架（plugin_graph）统一驱动：
 *     1) 构造       —— 创建对象，此时状态为 created。
 *     2) compose    —— 启动前调用一次，声明内部子节点与依赖边（只声明，不干活）。
 *     3) configure  —— 图冻结（compile）之后调用一次，加载静态参数。
 *     4) start      —— 进入运行态前调用一次。
 *     5) process    —— 每个执行周期调用，是真正的业务入口（纯虚，必须实现）。
 *     6) on_enter / on_exit / on_plan_changed —— 帧边界的「进入/退出/计划切换」通知。
 *     7) on_skipped  —— 本帧该节点被跳过时调用。
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

#ifndef robot_plugin_node_hpp
#define robot_plugin_node_hpp

#include "PluginGraph.hpp"
#include "PluginTypes.hpp"
#include "NodeTransitionContext.hpp"

namespace robot::framework {

// 所有层级都使用同一个节点接口；节点可以拥有空的或非空的内部子图。
// 也就是说：无论是顶层底盘节点，还是某个组合节点内部的小节点，类型都是
// plugin_node 的派生类，框架对它们一视同仁。
class plugin_node 
{
public:
    // 构造函数：使用固定 ID 创建节点。
    // 参数 id：节点在图内的唯一编号（plugin_id，16 位无符号整数）。
    // 说明：需由调用方保证 ID 在全图内唯一，validate() 会检查重复。
    explicit plugin_node(plugin_id id) : id_(id) {}
    // 虚析构函数：保证通过 plugin_node* 删除派生类对象时能正确析构。
    // 之所以要 virtual，是因为框架用基类指针数组 nodes_[] 统一持有所有节点。
    virtual ~plugin_node() = default;

    plugin_id id() const { return id_; }
    plugin_state state() const { return snapshot_.state; }
    plugin_health health() const { return snapshot_.health; }
    const plugin_health_snapshot& health_snapshot() const { return snapshot_; }
    plugin_graph& children() { return children_; }
    const plugin_graph& children() const { return children_; }

    // 虚函数，重写时内容是添加子节点和其依赖
    virtual plugin_status compose() { return plugin_status::ok; }
    // 节点状态改成就绪和健康
    virtual plugin_status configure()
    {
        snapshot_.state = plugin_state::configured;
        snapshot_.health = plugin_health::healthy;
        return plugin_status::ok;
    }
    // 节点状态设置成运行中和健康
    virtual plugin_status start()
    {
        snapshot_.state = plugin_state::running;
        snapshot_.health = plugin_health::healthy;
        return plugin_status::ok;
    }
    // 【生命周期·过渡通知】transition：首次/重新激活通知；默认不清空业务状态。
    // 何时调用：帧边界由 plugin_graph::update_transitions() 调用，只有「本帧由未激活
    //           变为激活」的节点才会收到。
    // 参数 transition：帧边界上下文（帧号、时间戳、计划切换信息等）。
    // 默认行为：什么都不做。重写时通常用来重置本节点或子图的业务状态。
    virtual void on_enter(const node_transition_context& transition) { (void)transition; }
    // 【生命周期·过渡通知】transition：失去激活或整图停止通知；默认不释放静态资源。
    // 何时调用：帧边界由 update_transitions() 调用（由激活变为不激活时）；整图停止时
    //           也会由 plugin_graph::stop_all() 调用一次。
    // 默认行为：什么都不做。重写时通常用来暂停输出、释放运行期资源。
    virtual void on_exit(const node_transition_context& transition) { (void)transition; }
    // 【生命周期·过渡通知】transition：保持激活时计划改变；新激活节点只收到 on_enter。
    // 何时调用：帧边界由 update_transitions() 调用，仅当节点「上一帧和这一帧都激活」
    //           且执行计划（active_plan_id_）发生了切换。
    // 默认行为：什么都不做。重写时用它处理「换计划但不换节点」的平滑过渡。
    virtual void on_plan_changed(const node_transition_context& transition) { (void)transition; }
    // 【生命周期第 5 步】process 是节点的业务入口，由执行器按节点周期调用。
    // 纯虚函数（= 0）：派生类必须实现，否则该类不能被实例化。
    // 何时调用：每个执行周期由 plugin_graph::process_frame() 调用（仅当节点激活、
    //           层级可运行且所有本帧输入依赖就绪时才真正执行）。
    // 参数 context：本帧的时间上下文（帧号、周期、时间戳），同一帧内所有节点共享。
    // 返回 process_result：本次处理结果（ok/no_new_data/output_held/degraded/fault），
    //           框架据此更新节点状态与健康度。
    virtual process_result process(frame_context& context) = 0;
    // context：被跳过的帧；有输出的分支节点应在重写中使输出失效。
    // 何时调用：本帧该节点没有被 process（分支未选中、状态不可运行、依赖未就绪、
    //           上游故障或无数据等）时，由 process_frame() 调用。
    // 默认行为：什么都不做。若你的节点会把上一帧输出留在外部（例如电机指令），
    //           应重写此函数把输出标记为无效，避免下游误用过期数据。
    virtual void on_skipped(frame_context& context) { (void)context; }
    // 【生命周期第 8 步】stop 只改变运行状态，不释放静态对象。
    // 何时调用：停机时由 plugin_graph::stop_all() 按计划 0 的逆序调用（已是 stopped 的跳过）。
    // 默认行为：状态置为 stopped。
    virtual void stop()
    {
        snapshot_.state = plugin_state::stopped;
    }

protected:
    // 将节点置为禁用和降级状态
    void disable()
    {
        snapshot_.state = plugin_state::disabled;
        snapshot_.health = plugin_health::degraded;
    }

    // 节点设置为故障状态并记录
    void set_fault(uint32_t fault_code)
    {
        snapshot_.state = plugin_state::faulted;
        snapshot_.health = plugin_health::faulted;
        snapshot_.fault_code = fault_code;
    }

private:
    friend class plugin_graph;

    // 保存最近一次 process 结果，并据此更新状态和健康度
    void record_process_result(process_result result)
    {
        snapshot_.last_result = result;
        if (result == process_result::fault)
        {
            snapshot_.state = plugin_state::faulted;
            snapshot_.health = plugin_health::faulted;
        }
        else if (result == process_result::degraded)
        {
            snapshot_.state = plugin_state::degraded;
            snapshot_.health = plugin_health::degraded;
        }
    }

    // 节点唯一 ID。默认 k_invalid_plugin_id（0xFFFF，表示尚未赋予合法 ID）。
    plugin_id id_{k_invalid_plugin_id};
    // 节点内部子图。默认为空图；由本节点在 compose() 中声明。
    plugin_graph children_{};
    // 节点健康和处理结果快照（默认 state=created、health=unknown）。
    plugin_health_snapshot snapshot_{};
    // 是否已经调用过 compose。由 plugin_graph::compose_node() 负责置位，防止重复声明。
    bool is_composed_{false};
};

} // namespace robot::framework

#endif /* robot_plugin_node_hpp */


