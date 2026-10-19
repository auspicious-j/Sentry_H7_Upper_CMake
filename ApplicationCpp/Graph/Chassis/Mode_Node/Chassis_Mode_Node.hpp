#ifndef robot_chassis_mode_node_hpp
#define robot_chassis_mode_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Chassis_Types.hpp"

namespace robot::Chassis {

// Mode_Node：底盘模式仲裁（对应旧 Chassis_ModeControl）。
// 输出最终 pattern/rotate_mode（节点私有状态）；跨任务通过
// chassis_mode_mailbox() 交给 Gimbal 任务，不再写 C 侧 chassis。
class Chassis_Mode_Node final : public robot::framework::plugin_node {
public:
    explicit Chassis_Mode_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_inputs(const robot::framework::frame_signal<chassis_inputs>& inputs) { inputs_ = &inputs; }

    const robot::framework::frame_signal<chassis_mode>& output() const { return output_; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    const robot::framework::frame_signal<chassis_inputs>* inputs_{nullptr};
    robot::framework::frame_signal<chassis_mode> output_{};

    // 模式归本节点所有（跨帧状态），不再写 C 侧 chassis.pattern / rotate.mode。
    std::uint8_t pattern_{0U};     // 0 == Chassis_Control
    std::uint8_t rotate_mode_{0U}; // 0 == ChassisMode_Follow
};

} // namespace robot::Chassis

#endif // robot_chassis_mode_node_hpp
