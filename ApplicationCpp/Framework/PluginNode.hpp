#ifndef ROBOT_PLUGIN_NODE_HPP
#define ROBOT_PLUGIN_NODE_HPP

#include "PluginGraph.hpp"
#include "PluginTypes.hpp"

namespace robot::framework {

// 所有层级都使用同一个节点接口；节点可以拥有空的或非空的内部子图。
class PluginNode {
public:
    explicit PluginNode(PluginId id) : id_(id) {}
    virtual ~PluginNode() = default;

    PluginId id() const { return id_; }
    PluginState state() const { return snapshot_.state; }
    PluginHealth health() const { return snapshot_.health; }
    const PluginHealthSnapshot& healthSnapshot() const { return snapshot_; }

    /* Every node has a child graph. It may remain empty. */
    PluginGraph& children() { return children_; }
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
    void disable()
    {
        snapshot_.state = PluginState::Disabled;
        snapshot_.health = PluginHealth::Degraded;
    }

    void setFault(uint32_t fault_code)
    {
        snapshot_.state = PluginState::Faulted;
        snapshot_.health = PluginHealth::Faulted;
        snapshot_.fault_code = fault_code;
    }

private:
    friend class PluginGraph;

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

    PluginId id_{kInvalidPluginId};
    PluginGraph children_{};
    PluginHealthSnapshot snapshot_{};
    bool composed_{false};
};

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_NODE_HPP */


