/* ============================================================================
 * 文件：ChassisOutputMailbox.cpp
 * 作用：ChassisOutputMailbox.hpp 的实现。
 * 线程安全：发布者（Chassis 任务）与读取者（RS485 任务）跨任务访问，
 *           由 interrupt_snapshot_lock 的短临界区保证整份快照原子读写。
 * ========================================================================== */
#include "ChassisOutputMailbox.hpp"

#include "InterruptSnapshotLock.hpp"

namespace {

robot::platform::stm32::interrupt_snapshot_lock g_chassis_output_lock{};
robot::framework::latest_snapshot_mailbox<robot::Chassis::chassis_output>
    g_chassis_output_mailbox{g_chassis_output_lock};

} // namespace

namespace robot::Chassis {

robot::framework::latest_snapshot_mailbox<chassis_output>& chassis_output_mailbox()
{
    return g_chassis_output_mailbox;
}

} // namespace robot::Chassis
