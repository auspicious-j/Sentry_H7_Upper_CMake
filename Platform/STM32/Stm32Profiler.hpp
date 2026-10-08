#ifndef ROBOT_STM32_PROFILER_HPP
#define ROBOT_STM32_PROFILER_HPP

#include "../../ApplicationCpp/Platform/IClock.hpp"
#include "../../ApplicationCpp/Framework/IProfiler.hpp"

namespace robot::platform::stm32 {

class Stm32Profiler final : public robot::framework::IProfiler {
public:
    explicit Stm32Profiler(const robot::platform::IClock& clock) : clock_(clock) {}

    void beginFrame(uint32_t frame_id, uint64_t timestamp_us) override;
    void beginNode(robot::framework::PluginId id) override;
    void endNode() override;
    void endFrame(uint64_t timestamp_us) override;

private:
    const robot::platform::IClock& clock_;
    uint64_t frame_start_us_{0U};
    uint64_t node_start_us_{0U};
    robot::framework::PluginId current_node_id_{robot::framework::kInvalidPluginId};
};

} // namespace robot::platform::stm32

#endif /* ROBOT_STM32_PROFILER_HPP */


