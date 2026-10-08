#ifndef ROBOT_PLUGIN_NODE_HPP
#define ROBOT_PLUGIN_NODE_HPP

#include "PluginGraph.hpp"
#include "PluginTypes.hpp"

namespace robot::framework {

// 所有层级都使用同一个节点接口；节点可以拥有空的或非空的内部子图。
class PluginNode {
public:
    // 使用固定 ID 创建节点。
    explicit PluginNode(PluginId id) : id_(id) {}
    // 多态节点的虚析构。
    virtual ~PluginNode() = default;

    // 返回节点 ID。
    PluginId id() const { return id_; }
    // 返回生命周期状态。
    PluginState state() const { return snapshot_.state; }
    // 返回健康状态。
    PluginHealth health() const { return snapshot_.health; }
    // 返回调试快照。
    const PluginHealthSnapshot& healthSnapshot() const { return snapshot_; }

    /* Every node has a child graph. It may remain empty. */
    // 返回可编辑的内部子图。
    PluginGraph& children() { return children_; }
    // 返回只读内部子图。
    const PluginGraph& children() const { return children_; }

    // compose 只负责声明内部节点和依赖，不执行业务。
    virtual PluginStatus compose() { return PluginStatus::Ok; }
    // configure 在图冻结后调用一次，适合加载静态参数。
    virtual PluginStatus configure()
    {
        snapshot_.state = PluginState::Configured;
        snapshot_.health = PluginHealth::Healthy;
        return PluginStatus::Ok;
    }
    // start 在进入运行态前调用一次。
    virtual PluginStatus start()
    {
        snapshot_.state = PluginState::Running;
        snapshot_.health = PluginHealth::Healthy;
        return PluginStatus::Ok;
    }
    // process 是节点的业务入口，由执行器按节点周期调用。
    virtual ProcessResult process(FrameContext& context) = 0;
    // stop 只改变运行状态，不释放静态对象。
    virtual void stop()
    {
        snapshot_.state = PluginState::Stopped;
    }

protected:
    // 将节点置为禁用状态。
    void disable()
    {
        snapshot_.state = PluginState::Disabled;
        snapshot_.health = PluginHealth::Degraded;
    }

    // 记录节点故障码并进入 Faulted。
    void setFault(uint32_t fault_code)
    {
        snapshot_.state = PluginState::Faulted;
        snapshot_.health = PluginHealth::Faulted;
        snapshot_.fault_code = fault_code;
    }

private:
    friend class PluginGraph;

    // 保存最近一次 process 结果。
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

    // 节点唯一 ID。
    PluginId id_{kInvalidPluginId};
    // 节点内部子图。
    PluginGraph children_{};
    // 节点健康和处理结果快照。
    PluginHealthSnapshot snapshot_{};
    // 是否已经调用 compose。
    bool composed_{false};
};

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_NODE_HPP */


