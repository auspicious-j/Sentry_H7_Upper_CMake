#ifndef robot_chassis_kinematics_node_hpp
#define robot_chassis_kinematics_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Chassis_Types.hpp"
#include "ChassisMode.h"

/* PID / Slope 是 C 侧类型，C++ 里包在 extern "C" 中。 */
extern "C"
{
#include "PID.h"
#include "Slope.h"
}

namespace robot::Chassis {

// Kinematics_Node：底盘速度解算与四轮目标计算。
// 对应旧 Chassis.c 的 Task_Chassis_Callback() 主体（去掉模式仲裁部分）。
//
// 底盘运动学状态（几何、舵角零点、斜坡、旋转 PID、三轴速度）是本节点的私有成员：
// configure() 初始化、process() 更新，不再写 C 全局 chassis。
// C 侧 chassis.motors[] 只剩 B2B 中断写入的硬件反馈。
// 四轮目标通过 chassis_output_mailbox() 交给 RS485 任务。
class Chassis_Kinematics_Node final : public robot::framework::plugin_node {
public:
    explicit Chassis_Kinematics_Node(robot::framework::plugin_id id) : plugin_node(id) {}

    void bind_inputs(const robot::framework::frame_signal<chassis_inputs>& inputs) { inputs_ = &inputs; }
    void bind_mode(const robot::framework::frame_signal<chassis_mode>& mode) { mode_ = &mode; }

    const robot::framework::frame_signal<chassis_output>& output() const { return output_; }

    // 图编译完成后调用一次：几何参数、舵角零点、斜坡与旋转 PID 的初始化。
    robot::framework::plugin_status configure() override;

    robot::framework::process_result process(robot::framework::frame_context& context) override;

    void on_skipped(robot::framework::frame_context& context) override
    {
        (void)context;
        output_.invalidate();
    }

private:
    // 三轴速度上限与斜坡推进（只有本节点使用）。
    void spin_speed_update(ChassisMode mode);
    void update_slope(ChassisMode mode);

    const robot::framework::frame_signal<chassis_inputs>* inputs_{nullptr};
    const robot::framework::frame_signal<chassis_mode>* mode_{nullptr};
    robot::framework::frame_signal<chassis_output> output_{};

    // 底盘尺寸（原 chassis.info）
    struct
    {
        float wheelbase;
        float wheeltrack;
        float wheelRadius;
        float offsetX;
        float offsetY;
    } info_{};

    // 斜坡与旋转 PID（原 chassis.move.* / chassis.rotate.pid）
    Slope x_slope_{};
    Slope y_slope_{};
    Slope spin_slope_{};
    PID rotate_pid_{};

    // 三轴速度与上限（原 chassis.move.vx/vy/vw/max*）
    float vx_{0.0f};
    float vy_{0.0f};
    float vw_{0.0f};
    float max_vx_{0.0f};
    float max_vy_{0.0f};
    float max_vw_{0.0f};

    // 云台相对角（原 chassis.rotate.InitAngle/nowAngle/relativeAngle）
    float init_angle_{0.0f};
    float now_angle_{0.0f};
    float relative_angle_{0.0f};

    // 四轮舵角零点（原 chassis.motors[].TurnOffset）
    float turn_offset_[4]{};
};

} // namespace robot::Chassis

#endif // robot_chassis_kinematics_node_hpp
