#ifndef robot_chassis_mode_node_hpp
#define robot_chassis_mode_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Chassis_Types.hpp"

namespace robot::Chassis {

// Mode_Node：底盘模式仲裁（对应旧 Chassis_ModeControl）。
// 输出最终 pattern/rotate_mode，并同步写回 C 侧 chassis（Gimbal 侧仍在读）。
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
};

} // namespace robot::Chassis

#endif // robot_chassis_mode_node_hpp
