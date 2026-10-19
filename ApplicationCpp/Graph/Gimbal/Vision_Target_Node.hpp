#ifndef robot_vision_target_node_hpp
#define robot_vision_target_node_hpp

#include <cstdint>

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "VisionTargetMailbox.hpp"

namespace robot::Gimbal {

// Vision_Target_Node：把 Io/Vision 的原始视觉帧翻译成“瞄准目标 + 发射请求”。
//
// 输入：vision_frame_mailbox()（跨任务，Vision 任务发布原始帧）。
// 输出：
//   1) frame_signal<vision_target_snapshot>：同帧供 Gimbal_Input_Node 读取；
//   2) vision_target_mailbox()：跨任务供 Vision 任务（遥测）等消费者读取。
//
// 迁移说明：本节点是旧 VisionApp.cpp 中 Vision_UseArmor / Vision_UseRune 的
//           逐行等价迁移；只依赖传入快照与 IMU/发射状态，不写业务全局。
class Vision_Target_Node final : public robot::framework::plugin_node
{
public:
    explicit Vision_Target_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    const robot::framework::frame_signal<robot::io::vision::vision_target_snapshot>& output() const
    {
        return output_;
    }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    robot::framework::frame_signal<robot::io::vision::vision_target_snapshot> output_{};
    std::uint32_t last_sequence_{0U};                       // 已处理的视觉帧发布序号。
    robot::io::vision::vision_target_snapshot last_result_{}; // 最近一次判定结果（无新帧时保持）。
    bool has_last_result_{false};
};

} // namespace robot::Gimbal

#endif // robot_vision_target_node_hpp
