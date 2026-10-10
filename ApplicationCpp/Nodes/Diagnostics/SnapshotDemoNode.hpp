#ifndef ROBOT_SNAPSHOT_DEMO_NODE_HPP
#define ROBOT_SNAPSHOT_DEMO_NODE_HPP

#include "Framework/PluginNode.hpp"
#include "Framework/SnapshotMailbox.hpp"
#include "Platform/IClock.hpp"
#include "../../../Platform/STM32/InterruptSnapshotLock.hpp"

namespace robot::diagnostics {

// 低频发布一个递增样本，用于观察异步快照年龄。
class SnapshotProducerNode final : public robot::framework::PluginNode {
public:
    // id/clock/mailbox：节点编号、单调时钟和最新值邮箱。
    SnapshotProducerNode(robot::framework::PluginId id, const robot::platform::IClock& clock,
                         robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox);
    // enabled：控制发布，不影响消费者读取旧样本。
    void setEnabled(bool enabled) { enabled_ = enabled; }
    // context：每 10 帧发布一次当前帧号。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
private:
    const robot::platform::IClock& clock_; // 单调采样时钟。
    robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox_; // 最新值邮箱。
    uint32_t publish_count_{0U}; // 实际发布次数。
    bool enabled_{true}; // 是否允许继续发布。
};

// 每帧读取邮箱并判断样本有效期。
class SnapshotConsumerNode final : public robot::framework::PluginNode {
public:
    // id/clock/mailbox/max_age：消费者配置。
    SnapshotConsumerNode(robot::framework::PluginId id, const robot::platform::IClock& clock,
                         const robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox,
                         uint64_t max_age_us);
    // context：读取当前快照并更新内部观察结果。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // 返回最近一次快照观察结果。
    uint32_t sequence() const { return snapshot_.sequence; }
    uint64_t ageUs() const { return age_us_; }
    robot::framework::SnapshotStatus status() const { return status_; }
    uint32_t readCount() const { return read_count_; }
private:
    const robot::platform::IClock& clock_; // 当前平台单调时钟。
    const robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox_; // 只读邮箱。
    uint64_t max_age_us_; // 允许的最大样本年龄。
    robot::framework::TimedSnapshot<uint32_t> snapshot_{}; // 最近复制的样本。
    uint64_t age_us_{0U}; // 最近样本年龄。
    robot::framework::SnapshotStatus status_{robot::framework::SnapshotStatus::Empty}; // 最近新鲜度。
    uint32_t read_count_{0U}; // 消费次数。
};

// 组合生产和消费节点，提供 Watch 控制入口。
class SnapshotDemoNode final : public robot::framework::PluginNode {
public:
    // clock：平台时钟；内部节点使用同一时钟域。
    explicit SnapshotDemoNode(const robot::platform::IClock& clock);
    // 注册生产与消费节点，并声明依赖顺序。
    robot::framework::PluginStatus compose() override;
    // context：容器不重复执行内部节点。
    robot::framework::ProcessResult process(robot::framework::FrameContext& context) override;
    // enable：修改下一次生产行为，建议在所属执行任务中调用。
    void setPublishEnabled(bool enable) { producer_.setEnabled(enable); }
    // 输出当前发布控制状态。
    bool publishEnabled() const { return producer_enabled_; }
    // 应用层刷新启停控制后调用。
    void syncPublishEnabled(bool enable) { producer_enabled_ = enable; producer_.setEnabled(enable); }
    // 返回最近快照序号。
    uint32_t sequence() const { return consumer_.sequence(); }
    // 返回最近样本年龄。
    uint64_t ageUs() const { return consumer_.ageUs(); }
    // 返回最近状态。
    robot::framework::SnapshotStatus status() const { return consumer_.status(); }
    // 返回读取次数。
    uint32_t readCount() const { return consumer_.readCount(); }
private:
    robot::platform::stm32::InterruptSnapshotLock lock_{}; // STM32 快照锁。
    robot::framework::LatestSnapshotMailbox<uint32_t> mailbox_; // 最新值邮箱。
    SnapshotProducerNode producer_; // 低频发布节点。
    SnapshotConsumerNode consumer_; // 高频读取节点。
    bool producer_enabled_{true}; // Watch 控制镜像。
};

} // namespace robot::diagnostics
#endif // ROBOT_SNAPSHOT_DEMO_NODE_HPP

