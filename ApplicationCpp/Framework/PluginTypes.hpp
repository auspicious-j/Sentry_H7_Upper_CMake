#ifndef robot_plugin_types_hpp
#define robot_plugin_types_hpp

#include <cstdint>
// =============================================================================
// PluginTypes.hpp —— 插件框架的公共类型定义（最底层、无外部依赖的基础头文件）
// =============================================================================
// 【本文件在插件框架中的位置】
//   它是整个插件框架的“地基”：只定义数据类型，不含任何业务逻辑，也不依赖其它
//   框架文件。上层文件（plugin_node、plugin_graph、ControlExecutorTask、各业务
//   节点等）都会 #include 它，反过来它谁都不依赖，因此可被任意模块安全引用。
//
// 【本文件定义了 4 类东西】
//   1. 若干枚举：分别描述“操作结果”“节点一次处理的结果”“节点状态”“健康度”。
//   2. plugin_id：节点在图中的唯一编号（类似身份证号）。
//   3. frame_context：一帧的时间上下文（帧号、周期、时间戳）。
//   4. plugin_health_snapshot：某个节点最近一次运行结果的快照。
//
// 【新手术语速查（后文会反复出现）】
//   * 节点(Node)：框架里可执行的最小单元，负责一小段业务（如底盘解算、电机
//     控制）。每个节点都有唯一的 plugin_id。
//   * 图/拓扑(Graph/Topology)：节点不是随意执行的，而是按“谁先谁后”的依赖关系
//     连成一张有向图；执行顺序（拓扑序）由 compile() 计算得出。
//   * 帧(Frame)：控制周期的一拍。执行器每隔固定时间（例如 1ms）把所有节点跑
//     一遍，这一拍就叫一帧。
//   * 快照(Snapshot)：把对象“当前状态”复制下来的一份只读副本，供调试窗口、
//     故障管理等外部模块查看，不会反过来影响正在运行的业务数据。
//   * 枚举(enum class)：一组具名的整型常量。带 class 的枚举是“强类型”的，只能
//     用它自己的名字书写（如 plugin_status::ok），更安全、不会与其它枚举重名。
//     底层类型写成 uint8_t，表示每个值只占 1 字节。
// =============================================================================

namespace robot::framework {

// plugin_status：几乎所有“组装/验证/编译/生命周期”类接口的统一返回码。
// 嵌入式环境里不方便用异常，这些操作就用返回本枚举表示成功或具体失败原因，
// 调用方 switch 一下即可分辨。底层类型 uint8_t（1 字节），省内存。
enum class plugin_status : uint8_t {
    ok = 0,              // 操作成功，一切正常。
    invalid_state,        // 当前状态不允许该操作（例：节点已初始化后再初始化一次）。
    duplicate_node,       // 重复注册：同一节点对象，或相同节点 ID 已被注册过。
    duplicate_edge,       // 重复添加：这条依赖边（含 is_delayed 跨帧标志都相同）已存在。
    capacity_exceeded,    // 超出容量上限：节点数或依赖边数已达编译期配置的最大值。
    missing_node,         // 缺失节点：依赖边引用了尚未注册（不在图中）的节点。
    cycle_detected,       // 检测到依赖回环（本周期边自环或拓扑排序时发现成环）。
    frozen,              // 图已冻结：compile() 之后图结构被锁定，不允许再改结构。
    configuration_fault,  // 配置失败：节点配置错误或子图非法（如空指针）。
};

// process_result：一个节点完成一次 process() 调用后的结果，告诉执行器
// “这次算出来的数据能不能用”，比 plugin_status 更贴近运行期。
enum class process_result : uint8_t {
    ok = 0,        // 正常：本帧产生了新的、有效的输出。
    no_new_data,     // 没有新数据：本帧无有效输出（例如传感器还没更新），不算错误。
    output_held,    // 输出保持：沿用上一帧的输出，本帧不刷新（如降频运行）。
    degraded,      // 降级：能出结果但质量下降（如部分传感器失效），需关注。
    fault,         // 故障：节点出错，结果不可用，会被标记为故障态。
};

// plugin_state：节点的生命周期状态机，表示它当前处在哪个阶段。
// 注意：运行期被“禁用(disabled)”的节点只是不再执行，并不会被删除。
enum class plugin_state : uint8_t {
    created = 0,   // 已创建：刚构造出来，还没配置。
    configured,    // 已配置：configure() 完成，参数就绪。
    running,       // 运行中：start() 完成，正在正常处理数据。
    disabled,      // 已禁用：被主动停用，不参与执行（对象仍存在）。
    degraded,      // 降级：仍在运行但能力受限（健康度为 degraded）。
    faulted,       // 故障：出错进入故障态，通常带 fault_code。
    stopped,       // 已停止：stop() 完成，退出运行但静态资源未释放。
};

// plugin_health：对外暴露的“健康度”，便于调试窗口/故障管理统一诊断。
// 它是比 plugin_state 更粗粒度的汇总：外部只关心“好不好”，不必了解细节状态。
enum class plugin_health : uint8_t {
    unknown = 0,   // 未知：还没运行过，健康度无法判断。
    healthy,       // 健康：运行正常。
    degraded,      // 降级：能工作但性能/精度下降。
    faulted,       // 故障：已经出错，需要处理。
};

using plugin_id = uint16_t;                     // 节点 ID 类型：16 位无符号整数。
constexpr plugin_id k_invalid_plugin_id = 0xFFFFU; // 无效 ID 的哨兵值（16 位全 1），表示“未分配/不存在”。

// frame_context：一帧内所有节点共享的时间上下文（这一拍的“计时卡”）。
struct frame_context {
    uint32_t frame_id{0U};          // 帧序号：从 0 起递增，每执行一帧加 1，用于区分不同帧。
    uint32_t period_us{1000U};      // 本帧周期，单位微秒(us)，默认 1000us = 1ms。
    uint64_t timestamp_us{0U};      // 本帧时间戳，单位微秒(us)，通常来自 DWT 等高精度计时器。
};

// plugin_health_snapshot：保存节点最近一次处理结果的快照，供调试窗口和故障管理读取。
// 说明：这是“只读副本”，外部读取它不会影响节点内部正在运行的业务数据。
struct plugin_health_snapshot {
    plugin_state state{plugin_state::created};             // 当前生命周期状态，默认 created。
    plugin_health health{plugin_health::unknown};          // 当前健康度，默认 unknown（尚未运行）。
    process_result last_result{process_result::no_new_data}; // 最近一次 process() 结果，默认 no_new_data。
    uint32_t fault_code{0U};                             // 故障码：仅故障时有意义，0 表示无故障。
};

} // namespace robot::framework

#endif /* robot_plugin_types_hpp */
