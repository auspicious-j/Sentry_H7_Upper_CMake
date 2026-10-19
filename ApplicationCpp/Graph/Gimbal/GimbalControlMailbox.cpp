/* ============================================================================
 * 文件：GimbalControlMailbox.cpp
 * 作用：GimbalControlMailbox.hpp 的实现。
 * 线程安全：发布者（Gimbal 任务）与读取者（Motor / RS485 任务）跨任务访问，
 *           由 interrupt_snapshot_lock 的短临界区保证整份快照原子的读写。
 * ========================================================================== */
#include "GimbalControlMailbox.hpp"

#include "InterruptSnapshotLock.hpp"

namespace {

// 邮箱与同步后端：先声明锁，再声明邮箱（邮箱构造时保存锁的引用）。
robot::platform::stm32::interrupt_snapshot_lock g_gimbal_control_lock{};
robot::framework::latest_snapshot_mailbox<robot::Gimbal::gimbal_control>
    g_gimbal_control_mailbox{g_gimbal_control_lock};

} // namespace

namespace robot::Gimbal {

robot::framework::latest_snapshot_mailbox<gimbal_control>& gimbal_control_mailbox()
{
    return g_gimbal_control_mailbox;
}

} // namespace robot::Gimbal
