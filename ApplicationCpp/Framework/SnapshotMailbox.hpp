// =============================================================================
// SnapshotMailbox.hpp —— 异步快照邮箱（Header-only）
// -----------------------------------------------------------------------------
// 用途：在“实时生产任务（高优先级、周期短）”与“低速读取者（如调试输出、
//       上位机通信、诊断节点）”之间安全传递“最新一份数据快照”。
//
// 设计目标：
//   1. 生产者只需发布“最新值”，旧的未读样本可以丢弃（只保留最新），因此
//      邮箱内部是单槽位覆盖写，而非 FIFO 队列，不产生积压。
//   2. 不使用动态内存（无 new/malloc），所有存储为固定容量，适合裸机/RTOS。
//   3. 数据跨任务/跨中断域传递时，靠一段极短的“临界区锁”（i_snapshot_lock）
//      保证“整份值 + 元数据”的一致性读取，避免读到写了一半的撕裂数据。
//   4. 提供解锁 RAII 守卫（snapshot_lock_guard），异常/早退路径也不会漏解锁。
//
// 关键概念：
//   - 双缓冲（double buffering）：本质上每次读取都把共享槽位“整份复制”到
//     调用者自己的局部副本，之后在锁外基于副本做判断。这样锁持有时间极短，
//     且后续计算不再触碰共享区，天然避免二次取样导致的数据不一致。
//   - 临界区（critical section）：lock()/unlock() 之间禁止被打断的那一小段
//     代码，本文件用它保护“读或写整份快照”这个原子操作。
//   - RAII 锁保护：用对象的构造/析构自动完成加锁/解锁，杜绝忘记 unlock。
//
// 线程/中断安全：
//   - publish() 与 copy_to()/clear()/status() 对同一邮箱的并发访问由传入的
//     i_snapshot_lock 后端串行化；本类自身不额外加锁。
//   - 若 i_snapshot_lock 后端实现了真正的可嵌套临界区（如可重入的关中断），
//     则可在中断中使用；否则不得在中断上下文直接调用。
// =============================================================================
#ifndef robot_snapshot_mailbox_hpp
#define robot_snapshot_mailbox_hpp

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace robot::framework {

// 快照新鲜度：空、有效、过期或时钟域不一致。
// 取值用于把“这份副本还算不算可用的数据”这一判断结果显式表达出来。
enum class snapshot_status : uint8_t {
    empty = 0, // 从未发布过有效样本（is_present 为 false）。
    fresh, // 有效且 age 小于等于 TTL（允许的最大年龄）。
    stale, // 有效但已超过 TTL，数据太旧需要谨慎使用。
    clock_mismatch, // 当前时间早于采样时间，说明两个时间不在同一时钟域（或时间倒流）。
};

// 平台同步接口（抽象基类）；把“如何互斥”从邮箱实现中解耦，便于替换后端。
// STM32 后端的实现通常是一段极短的关中断/临界区；未来也可替换为互斥量、
// 自旋锁或原子操作实现，而无需改动本文件里的邮箱逻辑。
class i_snapshot_lock {
public:
    // 虚析构：允许通过基类指针安全销毁派生实现。
    virtual ~i_snapshot_lock() = default;
    // 进入共享快照临界区：此后对邮箱的读/写应被串行化保护。
    virtual void lock() = 0;
    // 离开共享快照临界区：与 lock() 一一配对，必须成对调用。
    virtual void unlock() = 0;
};

// RAII 锁守卫：构造即加锁、析构即解锁，避免任何早退/异常路径忘记 unlock。
// 注意：它持有的是“同步后端的引用”，后端对象的生命周期必须长于本守卫。
class snapshot_lock_guard {
public:
    // 参数 lock：同步后端；构造时立即调用其 lock() 进入临界区。
    explicit snapshot_lock_guard(i_snapshot_lock& lock) : lock_(lock) { lock_.lock(); }
    // 析构时自动 unlock()，即使函数中途 return 也能正确释放。
    ~snapshot_lock_guard() { lock_.unlock(); }
    // 禁止拷贝：锁的所有权唯一，复制会导致重复解锁/死锁。
    snapshot_lock_guard(const snapshot_lock_guard&) = delete;
    // 禁止赋值：同上，锁守卫不可被重新绑定。
    snapshot_lock_guard& operator=(const snapshot_lock_guard&) = delete;
private:
    i_snapshot_lock& lock_; // 当前同步后端（外部持有，本类不负责其生命周期）。
};

// 一次完整快照及其采样元数据（值 + 时间戳 + 序号 + 有效标志）。
// 模板参数 t：被传递的快照数据类型（须可平凡复制，见下方 static_assert）。
// 该结构既是邮箱内部槽位的类型，也是消费者复制出来的“稳定副本”的类型。
template <typename t>
struct timed_snapshot {
    t value{}; // 快照值（默认值初始化）。
    uint64_t sampled_at_us{0U}; // 生产者采样时刻（微秒），时间来源由调用者指定。
    uint32_t sequence{0U}; // 发布序号，单调递增；0 只表示从未发布过。
    bool is_present{false}; // 是否已有有效样本（true 表示 value/timestamp 可用）。
};

// 一份稳定副本的新鲜度判断结果。
// 它是纯粹的计算结果（值语义），不含任何共享引用，因此拿到后即可安全使用，
// 无需也不应再次访问共享邮箱。
struct snapshot_freshness {
    snapshot_status status{snapshot_status::empty}; // 当前副本的状态（默认 empty）。
    uint64_t age_us{0U}; // 同一副本的年龄（微秒）；副本为空或时钟异常时为零。
};

// 判断一份“已复制的副本”的新鲜度（纯函数，不访问共享邮箱、无副作用）。
// 约定：now_us 应在“复制结束之后”取得，从而只对传入的这一份副本作判断，
//       避免在复制与取时之间发生竞态导致结论不一致。
// 参数：
//   snapshot      —— 待判断的稳定副本（通常是 copy_to 得到的结果）。
//   now_us        —— 当前时间（微秒），必须与 snapshot.sampled_at_us 同一时钟域。
//   max_age_us    —— 允许的最大年龄（TTL，微秒）。
// 返回：新鲜度结果 {状态, 年龄}，空或时钟异常时年龄记为 0。
// 副作用：无。
template <typename t>
constexpr snapshot_freshness evaluate_snapshot(const timed_snapshot<t>& snapshot,
                                             uint64_t now_us, uint64_t max_age_us)
{
    // 从未发布过有效样本 → 直接判空。
    if (!snapshot.is_present)
    {
        return {snapshot_status::empty, 0U};
    }
    // 当前时间早于采样时间 → 两个时间不在同一时钟域（或时间倒流），无法比较年龄。
    if (now_us < snapshot.sampled_at_us)
    {
        return {snapshot_status::clock_mismatch, 0U};
    }
    const uint64_t age_us = now_us - snapshot.sampled_at_us; // 无下溢的样本年龄（已确保 now >= 采样时刻）。
    // 年龄不超过 TTL 视为 fresh，否则 stale；两种情况都带回真实年龄。
    return {age_us <= max_age_us ? snapshot_status::fresh : snapshot_status::stale, age_us};
}

// 异步边界邮箱：只保留“最新一份”样本，不使用动态内存。
// 语义：后发布的样本会覆盖先前的样本；读取者不一定能看到每一个中间值，
//       但一定能看到“当前最新值”，适合状态类数据（如姿态、里程计、功率等）。
// 模板参数 t：快照值类型，须为可平凡复制（trivially copyable）且尺寸受限（见 assert）。
template <typename t>
class latest_snapshot_mailbox {
public:
    // 编译期约束：t 必须可平凡复制，保证在临界区内可安全进行字节级整体赋值。
    static_assert(std::is_trivially_copyable<t>::value, "Snapshot value must be trivially copyable");
    // 编译期约束：t 尺寸不超过 128 字节，保证固定存储可控、赋值耗时确定。
    static_assert(sizeof(t) <= 128U, "Snapshot value exceeds fixed mailbox size");

    // 构造：绑定一个外部同步后端（引用，生命周期需长于本邮箱）。
    // 参数 lock：用于保护整份快照的读写；本邮箱不拥有它，只保存引用。
    explicit latest_snapshot_mailbox(i_snapshot_lock& lock) : lock_(lock) {}

    // 发布一份新样本并递增序号（生产者调用，通常位于实时高优先级任务）。
    // 参数：
    //   value          —— 要写入的新快照值（按引用传入，内部做整体复制）。
    //   sample_time_us —— 该值的采样时刻（微秒），由调用者按自身时钟填写。
    // 副作用：覆盖邮箱内的全部字段，并使 is_present 置 true。
    // 线程安全：通过 lock_ 加锁，与读取者串行化；锁粒度覆盖“整份值 + 元数据”。
    void publish(const t& value, uint64_t sample_time_us)
    {
        snapshot_lock_guard guard(lock_); // 保护整份值和元数据，期间禁止被读取者打断。
        snapshot_.value = value; // 写入新值。
        snapshot_.sampled_at_us = sample_time_us; // 写入采样时刻。
        ++snapshot_.sequence; // 发布序号单调递增。
        if (snapshot_.sequence == 0U)
        {
            ++snapshot_.sequence;
        } // 序号回绕时跳过 0（0 专门表示“从未发布”）。
        snapshot_.is_present = true; // 标记已有有效样本。
    }

    // 复制完整快照到调用者提供的目标结构中。
    // 参数 destination：输出参数，接收一份原子性的稳定副本。
    // 返回：无（成功即整份复制）。
    // 线程安全：加锁复制，故拷贝期间不会被并发 publish 撕裂；复制完成后，
    //           调用者应在“锁外”基于该副本做后续处理（双缓冲思想）。
    void copy_to(timed_snapshot<t>& destination) const
    {
        snapshot_lock_guard guard(lock_); // 一次性取走整份快照，锁持有时间极短。
        destination = snapshot_; // 整体赋值（t 可平凡复制，故为字节级拷贝）。
    }

    // 使当前样本失效（标记为空）。
    // 副作用：仅将 is_present 置 false，刻意保留 sequence 与最后数值以便诊断/观测。
    // 线程安全：加锁保护。
    void clear()
    {
        snapshot_lock_guard guard(lock_);
        snapshot_.is_present = false; // 只清有效标志，保留序号和最后值用于排查问题。
    }

    // 便捷接口：先复制一份稳定副本，再查询其新鲜度，仅返回状态。
    // 参数：
    //   now_us     —— 当前时间（微秒）。
    //   max_age_us —— 允许的最大年龄（TTL，微秒）。
    //   age_us     —— 输出参数，返回该副本的年龄（微秒）。
    // 返回：状态（empty/fresh/stale/clock_mismatch）。
    // 重要：
    //   - now_us 与 max_age_us 必须与采样时间同属一个时钟域，否则会误判。
    //   - 已持有副本的消费者应直接调用 evaluate_snapshot()，避免这里“再次取样”
    //     导致判断所依据的快照与手中副本不一致。
    //   - 并发发布可能晚于调用者取得 now_us 的时刻，故结果只代表取 now_us 时的状态。
    // 副作用：加锁读取一次共享邮箱（不修改数据）。
    snapshot_status status(uint64_t now_us, uint64_t max_age_us, uint64_t& age_us) const
    {
        timed_snapshot<t> copy{}; // 锁外判断使用的稳定副本（先复制后判断）。
        copy_to(copy); // 加锁取得原子性副本，此后不再接触共享区。
        const snapshot_freshness freshness = evaluate_snapshot(copy, now_us, max_age_us); // 纯计算，无共享访问。
        age_us = freshness.age_us; // 回传年龄。
        return freshness.status; // 回传状态。
    }

private:
    i_snapshot_lock& lock_; // 外部持有的同步后端（引用；本邮箱不拥有其生命周期）。
    timed_snapshot<t> snapshot_{}; // 固定容量样本存储，单槽位覆盖写（最新值语义）。
};

} // namespace robot::framework
#endif // robot_snapshot_mailbox_hpp
