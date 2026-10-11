/* ============================================================================
 * 文件：SnapshotDemoNode.cpp
 * 作用：SnapshotDemoNode.hpp 中三个节点的实现（异步快照演示）。
 *
 * 内容概览：
 *   - SnapshotProducerNode：每 10 帧发布一次当前帧号到邮箱（低频生产）。
 *   - SnapshotConsumerNode：每帧复制邮箱并评估样本新鲜度（高频消费）。
 *   - SnapshotDemoNode    ：组合上述两者，组装锁与邮箱并转发控制/查询接口。
 *
 * 概念提示：
 *   - 快照（snapshot）：某一时刻的「一份完整数据」；这里用 值+采样时刻+序号
 *     共同描述，保证读取者拿到的是一致的一份，而不是写了一半的半成品。
 *   - 邮箱（mailbox）：只保留「最新一份」的单槽位，新的覆盖旧的，不做排队，
 *     因此不会积压，适合状态类数据。
 *   - 临界区（critical section）：发布/复制整份数据期间用锁短暂串行化访问，
 *     避免读写并发导致的数据撕裂（详见 InterruptSnapshotLock）。
 *   - 发布频率：生产者「抽稀」到每 10 帧一次，让消费者在两次发布之间重复读到
 *     同一份样本，从而能观察到样本年龄增长、超时变 Stale 的现象。
 * ========================================================================== */
#include "SnapshotDemoNode.hpp"

namespace robot::diagnostics {

// ---------------------------------------------------------------------------
// SnapshotProducerNode 实现
// ---------------------------------------------------------------------------

// 构造：初始化节点基类与三个引用/依赖成员。
// 参数见头文件说明；本处只做成员初始化，无额外逻辑。
// 注意：邮箱与时钟均为外部传入（引用），本节点不拥有它们的生命周期。
SnapshotProducerNode::SnapshotProducerNode(robot::framework::PluginId id,
                                           const robot::platform::IClock& clock,
                                           robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox)
    : PluginNode(id), clock_(clock), mailbox_(mailbox) {}

// 每帧调用：判断本帧是否需要发布，需要则把帧号作为样本写入邮箱。
// 参数 context：本帧上下文，提供 frame_id（帧号）——这里直接当作样本值使用。
// 返回：发布成功返回 Ok；本帧不发布返回 OutputHeld（表示无新输出）。
robot::framework::ProcessResult SnapshotProducerNode::process(robot::framework::FrameContext& context)
{
    // 抽稀 + 使能判断：只有「帧号为 10 的整数倍」且「发布已使能」时才继续发布。
    // 任一条件不满足就直接返回，不写邮箱（消费端会继续读到上一份旧样本）。
    if ((context.frame_id % 10U) != 0U || !enabled_) {
        return robot::framework::ProcessResult::OutputHeld;
    }
    // 发布一份新样本：值为当前帧号，采样时刻取自单调时钟 nowUs()（微秒）。
    // publish() 内部会在临界区内「整份覆盖」写入，保证消费者不会读到半份数据。
    mailbox_.publish(context.frame_id, clock_.nowUs());
    // 记录一次真实发布（诊断计数）。
    ++publish_count_;
    return robot::framework::ProcessResult::Ok;
}

// ---------------------------------------------------------------------------
// SnapshotConsumerNode 实现
// ---------------------------------------------------------------------------

// 构造：初始化节点基类、时钟/邮箱引用与 TTL。
// 参数见头文件；max_age_us_ 决定多旧算「过期(Stale)」。
SnapshotConsumerNode::SnapshotConsumerNode(
    robot::framework::PluginId id, const robot::platform::IClock& clock,
    const robot::framework::LatestSnapshotMailbox<uint32_t>& mailbox, uint64_t max_age_us)
    : PluginNode(id), clock_(clock), mailbox_(mailbox), max_age_us_(max_age_us) {}

// 每帧调用：复制邮箱最新副本并评估其新鲜度。
// 参数 context：本帧上下文（本节点不使用，显式忽略以避免编译告警）。
// 返回：始终 Ok（读取动作不产生业务输出，只是更新内部观察结果）。
robot::framework::ProcessResult SnapshotConsumerNode::process(robot::framework::FrameContext& context)
{
    (void)context; // 显式忽略本帧上下文：消费者不需要帧号/时间等信息。
    // 在锁保护下「整份复制」邮箱当前内容到本地副本 snapshot_。
    // 复制是原子的：要么拿到完整的新样本，要么拿到完整的旧样本，绝不会撕裂。
    mailbox_.copyTo(snapshot_);
    // 记录一次读取（诊断计数），确认消费者确实每帧都在工作。
    ++read_count_;
    // 复制完成后再取时钟；序号、值和有效期始终属于同一次发布。
    // 先复制、后取时间，是为了让「评估所用的时间」与「复制到的这份样本」严格对应，
    // 避免在复制与取时之间发生新发布而导致判断依据错位。
    // evaluateSnapshot 是纯函数：只根据传入副本与当前时间算出状态和年龄，不再访问共享邮箱。
    const auto freshness = robot::framework::evaluateSnapshot(snapshot_, clock_.nowUs(), max_age_us_);
    status_ = freshness.status; // 缓存新鲜度状态（Empty/Fresh/Stale/ClockMismatch）。
    age_us_ = freshness.age_us; // 缓存样本年龄（微秒）。
    return robot::framework::ProcessResult::Ok;
}

// ---------------------------------------------------------------------------
// SnapshotDemoNode 实现（容器/组合节点）
// ---------------------------------------------------------------------------

// 构造：创建「锁 + 邮箱 + 生产者 + 消费者」这一整套演示装置。
// 参数 clock：平台时钟，生产者与消费者共用，确保时间戳在同一时钟域内可比。
// 初始化顺序（须与头文件成员声明顺序一致）：
//   PluginNode(90U)              —— 容器节点自身 ID = 90。
//   mailbox_(lock_)              —— 邮箱绑定内部锁，发布/复制由该锁串行化。
//   producer_(91U, clock, mailbox_)          —— 生产者 ID = 91，输出到同一邮箱。
//   consumer_(92U, clock, mailbox_, 5000U)   —— 消费者 ID = 92，TTL = 5000us(5ms)。
// 说明：TTL 取 5ms，配合「每 10 帧才发布一次」，可制造出样本偶尔变 Stale 的
//       场景，便于观察年龄与新鲜度的变化。
SnapshotDemoNode::SnapshotDemoNode(const robot::platform::IClock& clock)
    : PluginNode(90U), mailbox_(lock_), producer_(91U, clock, mailbox_),
      consumer_(92U, clock, mailbox_, 5000U) {}

// 【启动阶段调用一次】把两个子节点与其依赖关系声明进容器的子图。
// 行为：先加生产者，再加消费者，最后声明「生产者 → 消费者」的依赖边
//       （addDependency(前置, 后置) 表示后置节点应在前置之后执行），
//       保证同一帧内消费者读到的是生产者刚发布的最新样本。
// 返回：任一步失败即返回该错误状态；全部成功返回 Ok。
robot::framework::PluginStatus SnapshotDemoNode::compose()
{
    // 第 1 步：把生产者注册为子节点。
    robot::framework::PluginStatus status = children().add(producer_);
    // 第 2 步：上一步成功则继续注册消费者。
    if (status == robot::framework::PluginStatus::Ok) { status = children().add(consumer_); }
    // 第 3 步：前两步都成功则声明依赖：消费者(后置)依赖生产者(前置)。
    if (status == robot::framework::PluginStatus::Ok) { status = children().addDependency(producer_, consumer_); }
    return status;
}

// 每帧调用：容器自身不做任何业务。
// 参数 context：本帧上下文（本节点不使用，显式忽略）。
// 返回：始终 Ok；实际的发布/读取由子图中的两个节点在摊平后各自执行。
robot::framework::ProcessResult SnapshotDemoNode::process(robot::framework::FrameContext& context)
{
    (void)context; // 显式忽略：容器只是壳，不参与每帧计算。
    return robot::framework::ProcessResult::Ok;
}

} // namespace robot::diagnostics
