/* ============================================================================
 * 文件：ChassisModeMailbox.hpp
 * 作用：底盘模式（pattern / 旋转模式）的跨任务快照邮箱。
 *
 * 数据流：
 *   Chassis 任务：Chassis_Mode_Node（模式仲裁）
 *     → chassis_mode_mailbox()
 *       → Gimbal 任务：Gimbal_Input_Node（Target/State 节点据此选择策略）
 *
 * 说明：以前 Gimbal_Input_Node 用一把短临界区锁直接读 chassis.pattern /
 *       chassis.rotate.mode；那把锁就是「这本来是跨任务数据」的证据。
 * ========================================================================== */
#ifndef robot_chassis_mode_mailbox_hpp
#define robot_chassis_mode_mailbox_hpp

#include "Chassis_Types.hpp"
#include "SnapshotMailbox.hpp"

namespace robot::Chassis {

// 全程序唯一的底盘模式邮箱：Chassis 任务发布，Gimbal 任务读取。
robot::framework::latest_snapshot_mailbox<chassis_mode>& chassis_mode_mailbox();

} // namespace robot::Chassis

#endif /* robot_chassis_mode_mailbox_hpp */
