/* ============================================================================
 * 文件：ChassisOutputMailbox.hpp
 * 作用：底盘运动学输出（四舵轮目标角/转速、当前舵角）的跨任务快照邮箱。
 *
 * 数据流：
 *   Chassis 任务：Chassis_Kinematics_Node
 *     → chassis_output_mailbox()
 *       → RS485 任务：B2B_Transmit（帧 1–24 字节）
 *
 * 说明：以前这些字段写在 C 侧 chassis.motors[] 的 target 与 now_angle 里。
 * ========================================================================== */
#ifndef robot_chassis_output_mailbox_hpp
#define robot_chassis_output_mailbox_hpp

#include "Chassis_Types.hpp"
#include "SnapshotMailbox.hpp"

namespace robot::Chassis {

// 全程序唯一的底盘点输出邮箱：Chassis 任务发布，RS485 任务读取。
robot::framework::latest_snapshot_mailbox<chassis_output>& chassis_output_mailbox();

} // namespace robot::Chassis

#endif /* robot_chassis_output_mailbox_hpp */
