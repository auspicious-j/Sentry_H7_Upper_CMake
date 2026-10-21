#ifndef robot_gimbal_output_node_hpp
#define robot_gimbal_output_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

namespace robot::Gimbal 
{

    // Output_Node：把本帧最终结果提交到 C 侧接口。
    // 迁移期 C 侧的消费方（Shooter 任务）仍读 shooter 全局量，本节点是它们唯一的写者：
    //   - shooter.fric_open / shooter_state / id_to_trigger。
    // gimbal.*.targetAngle 不再写：全工程没有读者，控制节点直接用 gimbal_state 端口。
    // PID 输出由 Control_Node 自己持有并发布到 gimbal_control_mailbox()，
    // Motor / B2B 任务从邮箱读取（见 README §11 的归属表）。
    class Gimbal_Output_Node final : public robot::framework::plugin_node 
    {
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
