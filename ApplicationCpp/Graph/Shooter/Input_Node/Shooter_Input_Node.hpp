#ifndef robot_shooter_input_node_hpp
#define robot_shooter_input_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Shooter_Types.hpp"

namespace robot::Shooter 
{

    // Input_Node：采集裁判数据、电机反馈与发射请求。
    class Shooter_Input_Node final : public robot::framework::plugin_node 
    {
    public:
        explicit Shooter_Input_Node(robot::framework::plugin_id id) : plugin_node(id) {}

        const robot::framework::frame_signal<shooter_inputs>& output() const 
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
        robot::framework::frame_signal<shooter_inputs> output_{};
    };

} // namespace robot::Shooter

#endif // robot_shooter_input_node_hpp
