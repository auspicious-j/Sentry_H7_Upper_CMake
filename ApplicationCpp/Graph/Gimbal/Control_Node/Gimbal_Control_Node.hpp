#ifndef robot_gimbal_control_node_hpp
#define robot_gimbal_control_node_hpp

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

/* CascadePID 是 C 侧类型（PID.h），C++ 里必须包在 extern "C" 中，
   否则与 PID.c 编译出的符号对不上。 */
extern "C"
{
#include "PID.h"
}

namespace robot::Gimbal 
{

    // Control_Node：云台 PID 与控制输出。
    // 对应旧 Gimbal.c 中 Task_Gimbal_Callback 的下半段：
    //   模式相关增益 + top/base/pitch 的 PID 计算 + pitch 重力前馈。
    //
    // 状态归属：三个 CascadePID 是本节点的私有成员，跨帧积分保存在这里，
    // 不再写 C 侧的 gimbal.*.imuPID。算出的控制量通过 gimbal_control_mailbox()
    // 交给 Motor / RS485 任务（跨任务通道，见 README §11 的归属表）。
    class Gimbal_Control_Node final : public robot::framework::plugin_node 
    {
    public:
        explicit Gimbal_Control_Node(robot::framework::plugin_id id) : plugin_node(id) {}

        // 组装
        void bind_inputs(const robot::framework::frame_signal<gimbal_inputs>& inputs) 
        { 
            inputs_ = &inputs; 
        }
        void bind_state(const robot::framework::frame_signal<gimbal_state>& state) 
        { 
            state_ = &state; 
        }

        // 配置参数
        robot::framework::plugin_status configure() override;

        robot::framework::process_result process(robot::framework::frame_context& context) override;

        void on_skipped(robot::framework::frame_context& context) override 
        { 
            (void)context; 
        }

    private:
        const robot::framework::frame_signal<gimbal_inputs>* inputs_{nullptr};
        const robot::framework::frame_signal<gimbal_state>* state_{nullptr};

        // PID 状态归本节点所有：只有这里读写，跨帧积分保存在成员里。
        CascadePID top_yaw_pid_{};
        CascadePID base_yaw_pid_{};
        CascadePID pitch_pid_{};
    };

} // namespace robot::Gimbal

#endif // robot_gimbal_control_node_hpp
