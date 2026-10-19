/* ============================================================================
 * 文件：ChassisModeMailbox.cpp
 * 作用：ChassisModeMailbox.hpp 的实现。
 * 线程安全：发布者（Chassis 任务）与读取者（Gimbal 任务）跨任务访问，
 *           由 interrupt_snapshot_lock 的短临界区保证整份快照原子读写。
 * ========================================================================== */
#include "ChassisModeMailbox.hpp"

#include "InterruptSnapshotLock.hpp"

namespace {

robot::platform::stm32::interrupt_snapshot_lock g_chassis_mode_lock{};
robot::framework::latest_snapshot_mailbox<robot::Chassis::chassis_mode>
    g_chassis_mode_mailbox{g_chassis_mode_lock};

} // namespace

namespace robot::Chassis {

robot::framework::latest_snapshot_mailbox<chassis_mode>& chassis_mode_mailbox()
{
    return g_chassis_mode_mailbox;
}

} // namespace robot::Chassis
