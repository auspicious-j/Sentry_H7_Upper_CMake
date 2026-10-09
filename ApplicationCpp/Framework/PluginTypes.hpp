#ifndef ROBOT_PLUGIN_TYPES_HPP
#define ROBOT_PLUGIN_TYPES_HPP

#include <cstdint>

namespace robot::framework {

// 组装、验证、编译和生命周期操作的返回状态。
enum class PluginStatus : uint8_t {
    Ok = 0,              // 操作成功。
    InvalidState,        // 当前状态不允许该操作（如已初始化后再初始化）。
    DuplicateNode,       // 重复注册：同一节点对象或相同节点 ID 已存在。
    DuplicateEdge,       // 重复添加：该依赖边（含 delayed 标志）已存在。
    CapacityExceeded,    // 超出容量上限：节点数或依赖边数已达最大值。
    MissingNode,         // 依赖边引用了尚未注册的节点。
    CycleDetected,       // 检测到依赖回环（本周期边自环或拓扑排序成环）。
    Frozen,              // 图已冻结：compile() 之后不允许再改结构。
    ConfigurationFault,  // 配置失败：节点配置错误或子图非法（如空指针）。
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
