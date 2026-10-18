#ifndef robot_gimbal_output_node_hpp
#define robot_gimbal_output_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

namespace robot::Gimbal {

// Output_Node：把本帧最终结果写回 C 侧接口。
// 迁移期 C 侧的消费方（Motor 任务、B2B 发送、Shooter 任务）仍读 gimbal/shooter
// 全局量，所以由本节点作为唯一写者完成“C 接口提交”：
//   - gimbal.*.targetAngle；
//   - shooter.fricOpen / command / idToTrigger。
// PID 输出由 Control_Node 直接写入 gimbal.*.imuPID，Motor 任务照旧读取。
class Gimbal_Output_Node final : public robot::framework::plugin_node {
public:
    explicit Gimbal_Output_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_state(const robot::framework::frame_signal<gimbal_state>& state) { state_ = &state; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override { (void)context; }

private:
    const robot::framework::frame_signal<gimbal_state>* state_{nullptr};
};

} // namespace robot::Gimbal

#endif // robot_gimbal_output_node_hpp
