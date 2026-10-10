#ifndef ROBOT_SNAPSHOT_MAILBOX_HPP
#define ROBOT_SNAPSHOT_MAILBOX_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace robot::framework {

// 快照新鲜度：空、有效、过期或时钟域不一致。
enum class SnapshotStatus : uint8_t {
    Empty = 0, // 从未发布过有效样本。
    Fresh, // age 小于等于 TTL。
    Stale, // 已超过 TTL。
    ClockMismatch, // 当前时间早于采样时间。
};

// 平台同步接口；STM32 后端用短临界区，未来可替换互斥/原子实现。
class ISnapshotLock {
public:
    virtual ~ISnapshotLock() = default;
    virtual void lock() = 0; // 进入共享快照临界区。
    virtual void unlock() = 0; // 离开共享快照临界区。
};

// 自动释放快照锁，避免早退路径忘记 unlock。
class SnapshotLockGuard {
public:
    explicit SnapshotLockGuard(ISnapshotLock& lock) : lock_(lock) { lock_.lock(); }
    ~SnapshotLockGuard() { lock_.unlock(); }
    SnapshotLockGuard(const SnapshotLockGuard&) = delete;
    SnapshotLockGuard& operator=(const SnapshotLockGuard&) = delete;
private:
    ISnapshotLock& lock_; // 当前同步后端。
};

// 一次完整快照和其采样元数据。
template <typename T>
struct TimedSnapshot {
    T value{}; // 快照值。
    uint64_t sampled_at_us{0U}; // 生产者采样时刻。
    uint32_t sequence{0U}; // 发布序号，0 只表示从未发布。
    bool present{false}; // 是否有有效样本。
};

// 一份稳定副本的新鲜度判断结果，不再次访问共享邮箱。
struct SnapshotFreshness {
    SnapshotStatus status{SnapshotStatus::Empty}; // 当前副本的状态。
    uint64_t age_us{0U}; // 同一副本的年龄，空或时钟异常时为零。
};

// snapshot/now/max_age：在复制结束后取得 now，只判断传入的这份副本。
template <typename T>
constexpr SnapshotFreshness evaluateSnapshot(const TimedSnapshot<T>& snapshot,
                                             uint64_t now_us, uint64_t max_age_us)
{
    if (!snapshot.present) {
        return {SnapshotStatus::Empty, 0U};
    }
    if (now_us < snapshot.sampled_at_us) {
        return {SnapshotStatus::ClockMismatch, 0U};
    }
    const uint64_t age_us = now_us - snapshot.sampled_at_us; // 无下溢的样本年龄。
    return {age_us <= max_age_us ? SnapshotStatus::Fresh : SnapshotStatus::Stale, age_us};
}

// 异步边界只保留最新样本，不使用动态内存。
template <typename T>
class LatestSnapshotMailbox {
public:
    static_assert(std::is_trivially_copyable<T>::value, "Snapshot value must be trivially copyable");
    static_assert(sizeof(T) <= 128U, "Snapshot value exceeds fixed mailbox size");

    explicit LatestSnapshotMailbox(ISnapshotLock& lock) : lock_(lock) {}

    // value/sample_time_us：发布一份新样本并递增序号。
    void publish(const T& value, uint64_t sample_time_us)
    {
        SnapshotLockGuard guard(lock_); // 保护整份值和元数据。
        snapshot_.value = value;
        snapshot_.sampled_at_us = sample_time_us;
        ++snapshot_.sequence;
        if (snapshot_.sequence == 0U) { ++snapshot_.sequence; }
        snapshot_.present = true;
    }

    // destination：复制完整快照；调用者在锁外处理。
    void copyTo(TimedSnapshot<T>& destination) const
    {
        SnapshotLockGuard guard(lock_);
        destination = snapshot_;
    }

    // 清除有效标志但保留序号和最后数值用于诊断。
    void clear()
    {
        SnapshotLockGuard guard(lock_);
        snapshot_.present = false;
    }

    // 仅查询当前新鲜度；已复制值的消费者应使用 evaluateSnapshot，避免二次取样。
    // now/max_age：必须属于同一时钟域；并发发布可能晚于调用者取得 now 的时刻。
    SnapshotStatus status(uint64_t now_us, uint64_t max_age_us, uint64_t& age_us) const
    {
        TimedSnapshot<T> copy{}; // 锁外判断使用的稳定副本。
        copyTo(copy);
        const SnapshotFreshness freshness = evaluateSnapshot(copy, now_us, max_age_us);
        age_us = freshness.age_us;
        return freshness.status;
    }

private:
    ISnapshotLock& lock_; // 外部持有的同步后端。
    TimedSnapshot<T> snapshot_{}; // 固定容量样本存储。
};

} // namespace robot::framework
#endif // ROBOT_SNAPSHOT_MAILBOX_HPP
