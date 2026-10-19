/* ============================================================================
 * 文件：GimbalControlMailbox.hpp
 * 作用：云台控制量（PID 输出）的跨任务快照邮箱。
 *
 * 数据流：
 *   Gimbal 任务：Gimbal_Control_Node（PID 计算）
 *     → gimbal_control_mailbox()（本帧控制量）
 *       → Motor 任务：CAN 下发（小 yaw 电流 / pitch MIT）
 *       → RS485 任务：B2B 帧 25–28 字节（大 yaw 速度指令）
 *
 * 说明：
 *   - 语义是「最新值覆盖」，消费者可能跳过中间帧，但一定拿到当前最新值；
 *   - 消费者必须 copy_to() 整份复制后使用，不得跨任务直接读共享槽位；
 *   - 之前这几个量是通过 C 全局 gimbal.*.imuPID.output 传递的，
 *     没有帧号也没有同步，正确性依赖任务周期的相对快慢（见 README §11）。
 * ========================================================================== */
#ifndef robot_gimbal_control_mailbox_hpp
#define robot_gimbal_control_mailbox_hpp

#include "Gimbal_Types.hpp"
#include "SnapshotMailbox.hpp"

namespace robot::Gimbal {

// 全程序唯一的云台控制量邮箱：Gimbal 任务发布，Motor / RS485 任务读取。
robot::framework::latest_snapshot_mailbox<gimbal_control>& gimbal_control_mailbox();

} // namespace robot::Gimbal

#endif /* robot_gimbal_control_mailbox_hpp */
