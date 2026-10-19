/* ============================================================================
 * 文件：VisionTargetMailbox.hpp
 * 作用：视觉“瞄准结果”的跨任务快照邮箱。
 *
 * 数据流：
 *   Vision 任务：vision_frame_mailbox()（原始帧）
 *     → Gimbal 任务中的 Vision_Target_Node（解包/判定）
 *       → vision_target_mailbox()（瞄准结果）
 *         → Vision 任务（遥测 see_enemy 等其它消费者）
 *
 * 说明：
 *   - 目标判定依赖 IMU 与发射状态，属于业务决策，因此邮箱只负责“运输”，
 *     判定逻辑在 Vision_Target_Node 内；
 *   - 消费者必须 copy_to() 整份复制后使用，不得跨任务直接读取共享槽位。
 * ========================================================================== */
#ifndef robot_io_vision_target_mailbox_hpp
#define robot_io_vision_target_mailbox_hpp

#include <cstdint>

#include "SnapshotMailbox.hpp"

namespace robot::io::vision {

// 一帧视觉瞄准结果（对应旧全局 VisionTarget，字段语义保持一致）。
struct vision_target_snapshot {
    bool is_valid;                 // 对应旧字段 vaild：本帧是否存在可跟踪目标。
    float base_yaw;                // 大 yaw 目标角，度。
    float yaw;                     // 小 yaw（目标）角度，度。
    float pitch;                   // pitch 目标角，度。
    std::int8_t id_to_trigger;     // 待触发符 id，-1 表示无。
    std::uint8_t shooter_command;  // 发射请求（IDLE/TRIGGER/...，见 Shooter.h）。
    std::uint32_t triggering_hit_tick; // 命中延时（ms），仅在识别到正在触发的符时有意义。
};

// 全程序唯一的视觉瞄准结果邮箱。
robot::framework::latest_snapshot_mailbox<vision_target_snapshot>& vision_target_mailbox();

} // namespace robot::io::vision

#endif // robot_io_vision_target_mailbox_hpp
