#ifndef ROBOT_STM32_PROFILER_HPP
#define ROBOT_STM32_PROFILER_HPP

#include "../../ApplicationCpp/Platform/IClock.hpp"
#include "../../ApplicationCpp/Framework/IProfiler.hpp"

namespace robot::platform::stm32 {

// 用 STM32 单调时钟记录帧和节点耗时，并写入 Keil 可观察快照。
class Stm32Profiler final : public robot::framework::IProfiler {
public:
    // 使用外部时钟构造性能统计器。
    explicit Stm32Profiler(const robot::platform::IClock& clock) : clock_(clock) {}

    // 开始记录一帧。
    void beginFrame(uint32_t frame_id, uint64_t timestamp_us) override;
    // 开始记录一个节点。
    void beginNode(robot::framework::PluginId id) override;
    // 结束记录一个节点。
    void endNode() override;
    // 结束记录一帧。
    void endFrame(uint64_t timestamp_us) override;

private:
    // 依赖抽象时钟，便于未来替换计时后端。
    const robot::platform::IClock& clock_;
    uint64_t frame_start_us_{0U};
    uint64_t node_start_us_{0U};
    robot::framework::PluginId current_node_id_{robot::framework::kInvalidPluginId};
};

} // namespace robot::platform::stm32

#endif /* ROBOT_STM32_PROFILER_HPP */


