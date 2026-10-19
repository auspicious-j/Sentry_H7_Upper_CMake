/* ============================================================================
 * 文件：GimbalStateMailbox.hpp
 * 作用：云台测量状态（角度 / 累计角 / 陀螺）的跨任务快照邮箱。
 *
 * 数据流：
 *   Gimbal 任务：Gimbal_State_Node（角度解算）
 *     → gimbal_state_mailbox()（本帧测量量）
 *       → Vision 任务：Vision_DataTransmit（回传 top_yaw）
 *
 * 说明：以前这些字段写在 C 侧 gimbal.top_yaw.* 等全局量里，由 Vision 任务
 *       直接读取；没有同步、没有帧号。邮箱语义是「最新值覆盖」。
 * ========================================================================== */
#ifndef robot_gimbal_state_mailbox_hpp
#define robot_gimbal_state_mailbox_hpp

#include "Gimbal_Types.hpp"
#include "SnapshotMailbox.hpp"

namespace robot::Gimbal {

// 全程序唯一的云台状态邮箱：Gimbal 任务发布，Vision 任务读取。
robot::framework::latest_snapshot_mailbox<gimbal_state>& gimbal_state_mailbox();

} // namespace robot::Gimbal

#endif /* robot_gimbal_state_mailbox_hpp */
