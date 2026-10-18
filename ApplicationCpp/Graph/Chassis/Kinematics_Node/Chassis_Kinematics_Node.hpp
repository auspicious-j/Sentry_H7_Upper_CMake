#ifndef robot_chassis_kinematics_node_hpp
#define robot_chassis_kinematics_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Chassis_Types.hpp"

namespace robot::Chassis {

// Kinematics_Node：底盘速度解算与四轮目标计算。
// 对应旧 Chassis.c 的 Task_Chassis_Callback() 主体（去掉模式仲裁部分）。
// 持续状态（斜坡、电机偏置）保存在 chassis 结构体中；本节点是
// chassis.motors[].targetTurnAngle/targetDriveSpeed 的唯一写者。
class Chassis_Kinematics_Node final : public robot::framework::plugin_node {
public:
    explicit Chassis_Kinematics_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_inputs(const robot::framework::frame_signal<chassis_inputs>& inputs) { inputs_ = &inputs; }
    void bind_mode(const robot::framework::frame_signal<chassis_mode>& mode) { mode_ = &mode; }

    const robot::framework::frame_signal<chassis_output>& output() const { return output_; }

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    const robot::framework::frame_signal<chassis_inputs>* inputs_{nullptr};
    const robot::framework::frame_signal<chassis_mode>* mode_{nullptr};
    robot::framework::frame_signal<chassis_output> output_{};
};

} // namespace robot::Chassis

#endif // robot_chassis_kinematics_node_hpp
