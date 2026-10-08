#ifndef ROBOT_PLUGIN_TYPES_HPP
#define ROBOT_PLUGIN_TYPES_HPP

#include <cstdint>

namespace robot::framework {

enum class PluginStatus : uint8_t {
    Ok = 0,
    InvalidState,
    DuplicateNode,
    DuplicateEdge,
    CapacityExceeded,
    MissingNode,
    CycleDetected,
    Frozen,
    ConfigurationFault,
};

enum class ProcessResult : uint8_t {
    Ok = 0,
    NoNewData,
    OutputHeld,
    Degraded,
    Fault,
};

enum class PluginState : uint8_t {
    Created = 0,
    Configured,
    Running,
    Disabled,
    Degraded,
    Faulted,
    Stopped,
};

enum class PluginHealth : uint8_t {
    Unknown = 0,
    Healthy,
    Degraded,
    Faulted,
};

using PluginId = uint16_t;
constexpr PluginId kInvalidPluginId = 0xFFFFU;

struct FrameContext {
    uint32_t frame_id{0U};
    uint32_t period_us{1000U};
    uint64_t timestamp_us{0U};
};

struct PluginHealthSnapshot {
    PluginState state{PluginState::Created};
    PluginHealth health{PluginHealth::Unknown};
    ProcessResult last_result{ProcessResult::NoNewData};
    uint32_t fault_code{0U};
};

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_TYPES_HPP */
