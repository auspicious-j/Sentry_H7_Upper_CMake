/* ============================================================================
 * 文件：ShooterOutputMailbox.hpp
 * 作用：发射机构目标量（摩擦轮转速、拨弹目标角）的跨任务快照邮箱。
 *
 * 数据流：
 *   Shooter 任务：Shooter_Logic_Node（状态机 + 弹速自适应）
 *     → shooter_output_mailbox()
 *       → Motor 任务：MotoApp（CAN1 0x200 电流下发）
 *
 * 说明：以前这些目标写在 C 侧 shooter 全局里，由 Motor 任务直接读取。
 * ========================================================================== */
#ifndef robot_shooter_output_mailbox_hpp
#define robot_shooter_output_mailbox_hpp

#include "Shooter_Types.hpp"
#include "SnapshotMailbox.hpp"

namespace robot::Shooter {

// 全程序唯一的发射目标量邮箱：Shooter 任务发布，Motor 任务读取。
robot::framework::latest_snapshot_mailbox<shooter_output>& shooter_output_mailbox();

} // namespace robot::Shooter

#endif /* robot_shooter_output_mailbox_hpp */
