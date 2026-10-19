#ifndef robot_shooter_logic_node_hpp
#define robot_shooter_logic_node_hpp

#include <cstdint>

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Shooter_Types.hpp"

namespace robot::Shooter {

// Logic_Node：发射状态机（对应旧 Task_Shooter_Callback）。
// 旧实现用 osDelay 阻塞等待（热量间隔、反转 500ms），这里改为时间戳状态机，
// 保证节点 process() 永不阻塞整张图。
class Shooter_Logic_Node final : public robot::framework::plugin_node {
public:
    explicit Shooter_Logic_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_inputs(const robot::framework::frame_signal<shooter_inputs>& inputs) { inputs_ = &inputs; }

    const robot::framework::frame_signal<shooter_output>& output() const { return output_; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    // 热量限制：更新内部发射间隔 t_ms_，返回值与旧 Heat_Limit() 一致（恒为 true）。
    bool update_heat_limit(const robot::io::inputs::judge_snapshot& judge);

    const robot::framework::frame_signal<shooter_inputs>* inputs_{nullptr};
    robot::framework::frame_signal<shooter_output> output_{};

    std::uint32_t t_ms_{0U};                     // 两次拨弹之间的热量间隔。
    std::uint32_t trigger_block_until_ms_{0U};   // 下一次允许拨弹的时刻。
    bool reverse_waiting_{false};                // 堵转反转的第二段是否在等待。
    std::uint32_t reverse_until_ms_{0U};         // 反转第二段的时刻。
    std::uint8_t over_speed_{0U};
    std::uint8_t down_speed_{0U};
};

} // namespace robot::Shooter

#endif // robot_shooter_logic_node_hpp
