#ifndef robot_chassis_input_node_hpp
#define robot_chassis_input_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Chassis_Types.hpp"

namespace robot::Chassis {

// Input_Node：采集底盘一帧所需的全部外部输入，不做决策。
class Chassis_Input_Node final : public robot::framework::plugin_node {
public:
    explicit Chassis_Input_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    const robot::framework::frame_signal<chassis_inputs>& output() const { return output_; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    robot::framework::frame_signal<chassis_inputs> output_{};
};

} // namespace robot::Chassis

#endif // robot_chassis_input_node_hpp
