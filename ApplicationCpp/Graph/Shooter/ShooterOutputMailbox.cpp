/* ============================================================================
 * 文件：ShooterOutputMailbox.cpp
 * 作用：ShooterOutputMailbox.hpp 的实现。
 * 线程安全：发布者（Shooter 任务）与读取者（Motor 任务）跨任务访问，
 *           由 interrupt_snapshot_lock 的短临界区保证整份快照原子读写。
 * ========================================================================== */
#include "ShooterOutputMailbox.hpp"

#include "InterruptSnapshotLock.hpp"

namespace {

robot::platform::stm32::interrupt_snapshot_lock g_shooter_output_lock{};
robot::framework::latest_snapshot_mailbox<robot::Shooter::shooter_output>
    g_shooter_output_mailbox{g_shooter_output_lock};

} // namespace

namespace robot::Shooter {

robot::framework::latest_snapshot_mailbox<shooter_output>& shooter_output_mailbox()
{
    return g_shooter_output_mailbox;
}

} // namespace robot::Shooter
