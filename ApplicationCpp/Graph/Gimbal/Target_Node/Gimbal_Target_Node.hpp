#ifndef robot_gimbal_target_node_hpp
#define robot_gimbal_target_node_hpp

#include <cstdint>

#include "PluginNode.hpp"
#include "Ports.hpp"
#include "Gimbal_Types.hpp"

namespace robot::Gimbal 
{

    // Target_Node：模式仲裁 + 目标角决策。
    // 对应旧 Gimbal.c 中 Task_Gimbal_Callback 的上半段：
    //   Gimbal_Shooter_VisionControl / Gimbal_Scan / Gimbal_RockerControl /
    //   Shooter_RockerControl / Fric_RockerControl。
    // 不直接写任何 C 全局量，结果通过 gimbal_targets 信号交给下游。
    class Gimbal_Target_Node final : public robot::framework::plugin_node 
    {
    public:
        explicit Gimbal_Target_Node(robot::framework::plugin_id id) : plugin_node(id) {}

        void bind_inputs(const robot::framework::frame_signal<gimbal_inputs>& inputs) { inputs_ = &inputs; }
        void bind_state(const robot::framework::frame_signal<gimbal_state>& state) { state_ = &state; }

        const robot::framework::frame_signal<gimbal_targets>& output() const { return output_; }

        robot::framework::process_result process(robot::framework::frame_context& context) override;

        void on_skipped(robot::framework::frame_context& context) override
        {
            (void)context;
            output_.invalidate();
        }

    private:
        // 视觉跟踪：返回 false 表示当前没有有效目标（调用方回退到扫描/手动）。
        bool apply_vision_control(const gimbal_inputs& inputs, const gimbal_state& previous_state);
        // 自动扫描（对应 Gimbal_Scan）。
        void apply_scan(const gimbal_inputs& inputs, float dt_s);
        // 遥控器摇杆控制（对应 Gimbal_RockerControl）。
        void apply_rocker(const gimbal_inputs& inputs);
        // 摩擦轮开关拨轮（对应 Fric_RockerControl）。
        void apply_fric_toggle(const gimbal_inputs& inputs);

        const robot::framework::frame_signal<gimbal_inputs>* inputs_{nullptr};
        const robot::framework::frame_signal<gimbal_state>* state_{nullptr};
        robot::framework::frame_signal<gimbal_targets> output_{};

        float base_target_deg_{0.0f};       // 大 yaw 解缠目标角（度）
        float top_target_deg_{0.0f};        // 小 yaw 解缠目标角（度）
        float pitch_target_deg_{0.0f};      // pitch 解缠目标角（度）
        float pitch_phase_{0.0f};           // pitch 扫描正弦波相位（rad），每帧累加 2πfΔt
        std::int16_t last_wheel_{0};        // 上一帧拨轮值（用于摩擦轮开关的边沿检测）
        bool fric_open_{false};             // 摩擦轮是否开启
        std::uint8_t shooter_command_{0U};  // 0 == IDLE
        std::int8_t id_to_trigger_{-1};     // 待触发符 id，-1 表示无
    };

} // namespace robot::Gimbal

#endif // robot_gimbal_target_node_hpp
