/* ============================================================================
 * 文件：snapshot_demo_node.hpp
 * 作用：异步快照（Snapshot Mailbox）机制的「演示/自检」节点。
 *
 * 背景 —— 什么是异步快照：
 *   在实时系统中，数据往往由「高频的生产者」产生（例如每毫秒更新的姿态、
 *   功率、里程计），而「低速的读取者」（如调试串口、上位机协议、诊断界面）
 *   并不需要每一个中间值，只需要能拿到「当前最新的一份完整数据」。
 *   异步快照邮箱（latest_snapshot_mailbox）正是为此设计的：生产者把数据
 *   「发布(publish)」到邮箱里（单槽位、覆盖写、只保留最新），读取者再
 *   「复制(copy_to)」走一份稳定副本。这样两者速度不必匹配，也不会读到
 *   写了一半的撕裂数据。
 *
 * 本文件做什么：
 *   用一个最简单的例子（递增的 uint32_t 样本）把上述机制跑通并暴露可观测指标，
 *   以便验证「发布频率」「样本年龄(age)」「新鲜度(fresh/stale)」等行为：
 *     - snapshot_producer_node：低频生产者，每 10 帧发布一次当前帧号。
 *     - snapshot_consumer_node：高频消费者，每帧复制邮箱并判断样本是否过期。
 *     - snapshot_demo_node    ：把上面两个节点组合起来的容器，另持有邮箱与快照锁。
 *
 * 为什么需要锁（临界区）：
 *   生产者与消费者可能运行在不同的任务（甚至中断）上下文，读写同一份邮箱
 *   就会被并发访问。interrupt_snapshot_lock 提供的是一段「极短临界区」：
 *   在复制整份数据期间短暂地屏蔽可屏蔽中断，保证复制是原子的、不会撕裂。
 *   临界区内只做拷贝，绝不做计算或阻塞，以免拉长中断延迟。
 *
 * 发布频率：
 *   生产者并非每帧都发布，而是「每 10 帧发布一次」（抽稀）。这既降低了
 *   发布开销，也便于观察消费者在两次发布之间读到的是同一份旧样本，
 *   从而验证 age 随时间增长、超过 TTL 后变为 stale 的行为。
 *
 * 说明：本文件只做演示与验证，不含真正的控制算法；所有成员均为固定存储，
 *       不使用动态内存，适合裸机/RTOS 环境。
 * ========================================================================== */
#ifndef robot_snapshot_demo_node_hpp
#define robot_snapshot_demo_node_hpp

// 依赖：节点基类（提供 compose/process 等生命周期钩子）、异步快照邮箱、
//       平台无关单调时钟，以及 STM32 平台的快照临界区锁实现。
// 注意：下面 interrupt_snapshot_lock 的 #include 路径较为特殊，改动它可能触发
//       CMSIS 头（cmsis_gcc.h / cmsis_armclang.h）的兼容编译问题，切勿修改。
#include "Framework/PluginNode.hpp"
#include "Framework/SnapshotMailbox.hpp"
#include "Platform/IClock.hpp"
#include "../../../Platform/STM32/InterruptSnapshotLock.hpp"

namespace robot::diagnostics {

// ===========================================================================
// snapshot_producer_node —— 低频「生产者」节点
// ---------------------------------------------------------------------------
// 职责：每隔若干帧把当前帧号作为样本值发布到邮箱，并附带「发布时刻」。
//       只写邮箱、不读邮箱；发布与否可通过 set_enabled() 开关。
// 生命周期：作为子节点被 snapshot_demo_node 加入子图，由框架按帧驱动 process()。
// 关键点：它由「外部传入」邮箱引用，因此可与消费者共享同一个邮箱实例。
// ===========================================================================
class snapshot_producer_node final : public robot::framework::plugin_node {
public:
    // 构造：绑定节点编号、采样时钟与目标邮箱（三者都以引用/值保存）。
    // 参数：
    //   id      —— 本节点在图内的唯一编号（plugin_id）。
    //   clock   —— 平台单调时钟，用于给每次发布打「采样时间戳」。
    //   mailbox —— 目标「最新值邮箱」；发布时把样本写到这里（非拥有，仅引用）。
    snapshot_producer_node(robot::framework::plugin_id id, const robot::platform::i_clock& clock,
                         robot::framework::latest_snapshot_mailbox<uint32_t>& mailbox);
    // 设置发布使能开关。
    // 参数 is_enabled：true 允许继续发布；false 暂停发布。
    // 说明：关闭后生产者不再写入新样本，但邮箱里已有的旧样本仍然保留，
    //       消费者照旧能读到它（并随时间变旧、最终变为 stale）。
    void set_enabled(bool is_enabled) 
    { 
        is_enabled_ = is_enabled; 
    }
    // 【生命周期·每帧调用】发布逻辑入口。
    // 参数 context：本帧上下文，含 frame_id（帧号）等。
    // 行为：仅当「帧号是 10 的整数倍」且「已使能」时才发布一次当前帧号与采样时刻；
    //       其它帧不发布，返回 output_held（表示本帧无新输出）。
    // 返回：发布成功返回 process_result::ok；未发布返回 process_result::output_held。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
private:
    const robot::platform::i_clock& clock_; // 采样时钟（外部持有）：每次发布用它取 now_us() 作为采样时间。
    robot::framework::latest_snapshot_mailbox<uint32_t>& mailbox_; // 目标邮箱（外部持有）：publish() 写这里；引用非拥有。
    uint32_t publish_count_{0U}; // 实际成功发布的次数计数（诊断用），默认 0。
    bool is_enabled_{true}; // 发布使能开关，默认 true（一上来就允许发布）。
};

// ===========================================================================
// snapshot_consumer_node —— 高频「消费者」节点
// ---------------------------------------------------------------------------
// 职责：每帧从邮箱复制一份「稳定副本」，随后（在锁外）用当前时钟评估这份副本的
//       新鲜度（age 是否超过 TTL），并把结果缓存下来供外部查询。
//       只读邮箱、不写邮箱，因此与生产者天然解耦。
// 生命周期：作为子节点被 snapshot_demo_node 加入子图，由框架按帧驱动 process()。
// 关键点：它按「每帧」的频率读取，比生产者（每 10 帧）高，故大多数帧读到的是
//         同一份旧样本——这正是演示「年龄会增长」的场景。
// ===========================================================================
class snapshot_consumer_node final : public robot::framework::plugin_node {
public:
    // 构造：绑定节点编号、时钟、只读邮箱与 TTL。
    // 参数：
    //   id         —— 本节点在图内的唯一编号。
    //   clock      —— 平台单调时钟，用于评估样本年龄（now_us 与采样时间需同域）。
    //   mailbox    —— 被读取的邮箱（const 引用，消费者无权发布）。
    //   max_age_us —— 允许的最大样本年龄（TTL，微秒）；超过即判为 stale。
    snapshot_consumer_node(robot::framework::plugin_id id, const robot::platform::i_clock& clock,
                         const robot::framework::latest_snapshot_mailbox<uint32_t>& mailbox,
                         uint64_t max_age_us);
    // 【生命周期·每帧调用】读取并评估快照。
    // 参数 context：本帧上下文（本节点不使用，故显式忽略）。
    // 行为：复制邮箱最新副本 → 取当前时间 → 评估新鲜度 → 缓存状态与年龄。
    // 返回：始终 process_result::ok（读取动作本身不产生业务输出）。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // 返回最近一次复制到的快照序号（0 表示从未读到过有效样本）。
    uint32_t sequence() const { return snapshot_.sequence; }
    // 返回最近一份副本的年龄（微秒）：当前时间减去采样时刻。
    uint64_t age_us() const { return age_us_; }
    // 返回最近一份副本的新鲜度状态（empty/fresh/stale/clock_mismatch）。
    robot::framework::snapshot_status status() const { return status_; }
    // 返回累计读取（复制）次数，用于确认消费者确实在每帧工作。
    uint32_t read_count() const { return read_count_; }
private:
    const robot::platform::i_clock& clock_; // 当前平台单调时钟（外部持有）：evaluate_snapshot 的时间来源。
    const robot::framework::latest_snapshot_mailbox<uint32_t>& mailbox_; // 只读邮箱（外部持有）：copy_to() 的来源。
    uint64_t max_age_us_; // TTL：允许的最大样本年龄（微秒），超过则判 stale。
    robot::framework::timed_snapshot<uint32_t> snapshot_{}; // 最近一次复制出的稳定副本（值+时间戳+序号+有效标志）。
    uint64_t age_us_{0U}; // 最近一次评估得到的样本年龄（微秒），默认 0。
    robot::framework::snapshot_status status_{robot::framework::snapshot_status::empty}; // 最近新鲜度，默认 empty（尚未读取）。
    uint32_t read_count_{0U}; // 消费（复制）次数计数，默认 0。
};

// ===========================================================================
// snapshot_demo_node —— 组合节点（容器）：把「生产者 + 消费者 + 邮箱 + 锁」组装起来
// ---------------------------------------------------------------------------
// 职责：
//   1. 拥有并持有互斥后端（interrupt_snapshot_lock）和邮箱（latest_snapshot_mailbox），
//      把邮箱与同一个锁绑定，保证生产者发布/消费者复制被正确串行化。
//   2. 在 compose() 里把生产者、消费者两个子节点注册进自己的子图，并声明依赖顺序。
//   3. 对外提供发布开关与只读指标（sequence/age/status/read_count）。
// 说明：容器自身的 process() 不做任何业务，真正的执行由子图中的两个节点完成；
//       框架会把子图摊平进根图后统一驱动它们。
// ===========================================================================
class snapshot_demo_node final : public robot::framework::plugin_node {
public:
    // 构造：创建内部邮箱（绑定内部锁）与两个子节点，并为它们分配固定 ID。
    // 参数 clock：平台时钟；生产者与消费者共用它，保证时间戳处于同一时钟域。
    explicit snapshot_demo_node(const robot::platform::i_clock& clock);
    // 【生命周期第 2 步】compose：把子节点与依赖关系声明进自己的子图。
    // 行为：依次把生产者、消费者加入 children()，再声明「生产者 → 消费者」的
    //       依赖（即消费者应排在生产之后执行，保证读到的是当帧发布的最新样本）；
    //       任一步失败即原样返回该错误状态。
    // 返回：plugin_status::ok 表示声明成功（务必在启动前调用一次）。
    robot::framework::plugin_status compose() override;
    // 【生命周期·每帧调用】容器自身不执行任何业务。
    // 参数 context：本帧上下文（本节点不使用，故显式忽略）。
    // 返回：始终 process_result::ok（子节点由框架按摊平后的根图分别驱动）。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // 设置是否允许生产者继续发布（直接转发给生产者）。
    // 参数 is_enabled：true 允许发布，false 暂停发布。
    // 建议：在「生产者所属的执行任务上下文」中调用，避免跨上下文改状态。
    void set_publish_enabled(bool is_enabled) { producer_.set_enabled(is_enabled); }
    // 返回当前记录的发布使能状态（注意：这是本地镜像 is_producer_enabled_，
    // 由 sync_publish_enabled() 同步，不一定等于生产者的即时真实状态）。
    bool is_publish_enabled() const { return is_producer_enabled_; }
    // 应用层刷新「启停控制」后调用：同时更新本地镜像与生产者自身开关，使二者一致。
    // 参数 is_enabled：目标发布使能状态。
    void sync_publish_enabled(bool is_enabled) 
    { 
        is_producer_enabled_ = is_enabled; 
        producer_.set_enabled(is_enabled); 
    }
    // 返回最近快照序号（转发自消费者）。
    uint32_t sequence() const { return consumer_.sequence(); }
    // 返回最近样本年龄（微秒，转发自消费者）。
    uint64_t age_us() const { return consumer_.age_us(); }
    // 返回最近新鲜度状态（转发自消费者）。
    robot::framework::snapshot_status status() const { return consumer_.status(); }
    // 返回消费者累计读取次数（转发自消费者）。
    uint32_t read_count() const { return consumer_.read_count(); }
private:
    robot::platform::stm32::interrupt_snapshot_lock lock_{}; // 互斥后端：被邮箱引用，用于保护「整份快照」的读写临界区。
    robot::framework::latest_snapshot_mailbox<uint32_t> mailbox_; // 最新值邮箱（单槽覆盖写）：生产者发布、消费者复制，二者共享。
    snapshot_producer_node producer_; // 低频生产者子节点（每 10 帧发布一次）。
    snapshot_consumer_node consumer_; // 高频消费者子节点（每帧读取一次）。
    bool is_producer_enabled_{true}; // 发布使能的本地镜像，默认 true；经 sync_publish_enabled() 与生产者保持一致。
};

} // namespace robot::diagnostics
#endif // robot_snapshot_demo_node_hpp

