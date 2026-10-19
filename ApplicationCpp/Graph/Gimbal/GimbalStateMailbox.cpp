/* ============================================================================
 * 文件：GimbalStateMailbox.cpp
 * 作用：GimbalStateMailbox.hpp 的实现。
 * 线程安全：发布者（Gimbal 任务）与读取者（Vision 任务）跨任务访问，
 *           由 interrupt_snapshot_lock 的短临界区保证整份快照原子读写。
 * ========================================================================== */
#include "GimbalStateMailbox.hpp"

#include "InterruptSnapshotLock.hpp"

namespace {

robot::platform::stm32::interrupt_snapshot_lock g_gimbal_state_lock{};
robot::framework::latest_snapshot_mailbox<robot::Gimbal::gimbal_state>
    g_gimbal_state_mailbox{g_gimbal_state_lock};

} // namespace

namespace robot::Gimbal {

robot::framework::latest_snapshot_mailbox<gimbal_state>& gimbal_state_mailbox()
{
    return g_gimbal_state_mailbox;
}

} // namespace robot::Gimbal
