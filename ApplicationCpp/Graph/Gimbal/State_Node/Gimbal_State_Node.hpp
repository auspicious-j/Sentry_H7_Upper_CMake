#ifndef robot_gimbal_state_node_hpp
#define robot_gimbal_state_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

namespace robot::Gimbal {

// State_Node：角度累计与目标解缠。
// 对应旧 Gimbal.c 的 Gimbal_UpdataAngle()：
//   - 用 IMU + 小 yaw 电机反馈推导大 yaw 角度；
//   - 累计 top_yaw.totalAngle，解缠 top_yaw.targetAngle；
//   - 计算 base_yaw.totalAngle；
//   - AI 模式下按遥控器左拨杆加 45°（加弹动作）。
// 测量量通过 gimbal_state_mailbox() 交给 Vision 任务，不再写 C 侧 gimbal。
class Gimbal_State_Node final : public robot::framework::plugin_node {
public:
    explicit Gimbal_State_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_inputs(const robot::framework::frame_signal<gimbal_inputs>& inputs) 
    { 
        inputs_ = &inputs; 
    }

    void bind_targets(const robot::framework::frame_signal<gimbal_targets>& targets) 
    { 
        targets_ = &targets; 
    }

    const robot::framework::frame_signal<gimbal_state>& output() const 
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
    const robot::framework::frame_signal<gimbal_inputs>* inputs_{nullptr};
    const robot::framework::frame_signal<gimbal_targets>* targets_{nullptr};
    robot::framework::frame_signal<gimbal_state> output_{};

    // 跨帧持久状态（旧实现存放在 gimbal.* 里）。
    float top_total_deg_{0.0f};
    float last_top_angle_deg_{0.0f};
    float base_total_deg_{0.0f};
};

} // namespace robot::Gimbal

#endif // robot_gimbal_state_node_hpp
