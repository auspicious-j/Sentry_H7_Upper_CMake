#ifndef robot_gimbal_input_node_hpp
#define robot_gimbal_input_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

namespace robot::Gimbal {

// Input_Node：每帧把 C 侧输入整理成 gimbal_inputs 并发布。
// 只做搬运，不做任何决策；所有读取都在 Io/Inputs 的短临界区内完成。
class Gimbal_Input_Node final : public robot::framework::plugin_node {
public:
    explicit Gimbal_Input_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    // 下游读取本节点输出。
    const robot::framework::frame_signal<gimbal_inputs>& output() const { return output_; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    robot::framework::frame_signal<gimbal_inputs> output_{};
};

} // namespace robot::Gimbal

#endif // robot_gimbal_input_node_hpp
