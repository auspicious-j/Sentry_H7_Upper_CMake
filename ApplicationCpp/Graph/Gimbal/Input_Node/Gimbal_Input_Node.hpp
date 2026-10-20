#ifndef robot_gimbal_input_node_hpp
#define robot_gimbal_input_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"
#include "VisionTargetMailbox.hpp"

namespace robot::Gimbal 
{

    // Input_Node：每帧把 C 侧输入整理成 gimbal_inputs 并发布。
    class Gimbal_Input_Node final : public robot::framework::plugin_node 
    {
    public:
        explicit Gimbal_Input_Node(robot::framework::plugin_id id) : plugin_node(id) {}

        // 组装
        void bind_vision_target(const robot::framework::frame_signal<robot::io::vision::vision_target_snapshot>& target)
        {
            vision_target_ = &target;
        }

        // 输出
        const robot::framework::frame_signal<gimbal_inputs>& output() const 
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
        const robot::framework::frame_signal<robot::io::vision::vision_target_snapshot>* vision_target_{nullptr};
        robot::framework::frame_signal<gimbal_inputs> output_{};

        // pitch 限幅
        float pitch_min_deg_{-20.0f};
        float pitch_max_deg_{34.0f};
    };

} // namespace robot::Gimbal

#endif // robot_gimbal_input_node_hpp
