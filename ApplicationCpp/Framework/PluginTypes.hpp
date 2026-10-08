#ifndef ROBOT_PLUGIN_TYPES_HPP
#define ROBOT_PLUGIN_TYPES_HPP

#include <cstdint>

namespace robot::framework {

// 组装、验证、编译和生命周期操作的返回状态。
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

// 节点一次 process() 调用的结果。
enum class ProcessResult : uint8_t {
    Ok = 0,
    NoNewData,
    OutputHeld,
    Degraded,
    Fault,
};

// 节点生命周期状态；运行期禁用节点不会被删除。
enum class PluginState : uint8_t {
    Created = 0,
    Configured,
    Running,
    Disabled,
    Degraded,
    Faulted,
    Stopped,
};

// 对外暴露的健康状态，便于统一诊断。
enum class PluginHealth : uint8_t {
    Unknown = 0,
    Healthy,
    Degraded,
    Faulted,
};

using PluginId = uint16_t;
constexpr PluginId kInvalidPluginId = 0xFFFFU;

// 一帧内所有节点共享的时间上下文。
struct FrameContext {
    uint32_t frame_id{0U};
    uint32_t period_us{1000U};
    uint64_t timestamp_us{0U};
};

// 保留最近一次处理结果，供调试窗口和故障管理读取。
struct PluginHealthSnapshot {
    PluginState state{PluginState::Created};
    PluginHealth health{PluginHealth::Unknown};
    ProcessResult last_result{ProcessResult::NoNewData};
    uint32_t fault_code{0U};
};

} // namespace robot::framework

#endif /* ROBOT_PLUGIN_TYPES_HPP */
