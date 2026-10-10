#include "SnapshotDemoNode.hpp"

namespace robot::diagnostics {

SnapshotProducerNode::SnapshotProducerNode(robot::framework::PluginId id,
                                           const robot::platform::IClock& clock,
                                           robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox)
    : PluginNode(id), clock_(clock), mailbox_(mailbox) {}

robot::framework::ProcessResult SnapshotProducerNode::process(robot::framework::FrameContext& context)
{
    if ((context.frame_id % 10U) != 0U || !enabled_) {
        return robot::framework::ProcessResult::OutputHeld;
    }
    mailbox_.publish(context.frame_id, clock_.nowUs());
    ++publish_count_;
    return robot::framework::ProcessResult::Ok;
}

SnapshotConsumerNode::SnapshotConsumerNode(
    robot::framework::PluginId id, const robot::platform::IClock& clock,
    const robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox, uint64_t max_age_us)
    : PluginNode(id), clock_(clock), mailbox_(mailbox), max_age_us_(max_age_us) {}

robot::framework::ProcessResult SnapshotConsumerNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    mailbox_.copyTo(snapshot_);
    ++read_count_;
    // 复制完成后再取时钟；序号、值和有效期始终属于同一次发布。
    const auto freshness = robot::framework::evaluateSnapshot(snapshot_, clock_.nowUs(), max_age_us_);
    status_ = freshness.status;
    age_us_ = freshness.age_us;
    return robot::framework::ProcessResult::Ok;
}

SnapshotDemoNode::SnapshotDemoNode(const robot::platform::IClock& clock)
    : PluginNode(90U), mailbox_(lock_), producer_(91U, clock, mailbox_),
      consumer_(92U, clock, mailbox_, 5000U) {}

robot::framework::PluginStatus SnapshotDemoNode::compose()
{
    robot::framework::PluginStatus status = children().add(producer_);
    if (status == robot::framework::PluginStatus::Ok) { status = children().add(consumer_); }
    if (status == robot::framework::PluginStatus::Ok) { status = children().addDependency(producer_, consumer_); }
    return status;
}

robot::framework::ProcessResult SnapshotDemoNode::process(robot::framework::FrameContext& context)
{
    (void)context;
    return robot::framework::ProcessResult::Ok;
}

} // namespace robot::diagnostics
