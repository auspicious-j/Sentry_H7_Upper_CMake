#ifndef robot_gimbal_control_node_hpp
#define robot_gimbal_control_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

namespace robot::Gimbal {

// Control_Node：云台 PID 与控制输出。
// 对应旧 Gimbal.c 中 Task_Gimbal_Callback 的下半段：
//   模式相关增益 + top/base/pitch 的 PID 计算 + pitch 重力前馈。
// PID 状态仍保存在 C 侧 gimbal.*.imuPID 里（跨帧积分），本节点是唯一写者。
class Gimbal_Control_Node final : public robot::framework::plugin_node {
public:
    explicit Gimbal_Control_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_inputs(const robot::framework::frame_signal<gimbal_inputs>& inputs) { inputs_ = &inputs; }
    void bind_state(const robot::framework::frame_signal<gimbal_state>& state) { state_ = &state; }

    const robot::framework::frame_signal<gimbal_control>& output() const { return output_; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    const robot::framework::frame_signal<gimbal_inputs>* inputs_{nullptr};
    const robot::framework::frame_signal<gimbal_state>* state_{nullptr};
    robot::framework::frame_signal<gimbal_control> output_{};
};

} // namespace robot::Gimbal

#endif // robot_gimbal_control_node_hpp
