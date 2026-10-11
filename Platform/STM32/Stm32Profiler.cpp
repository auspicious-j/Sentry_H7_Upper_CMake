#include "Stm32Profiler.hpp"

#include "../../ApplicationCpp/Platform/DebugSnapshot.hpp"

// ============================================================================
// stm32_profiler.cpp —— STM32 性能探针的实现
// ----------------------------------------------------------------------------
// 本文件实现 i_profiler 接口的四个打点函数，把测量结果写入全局调试快照
// g_robot_debug（定义在 DebugSnapshot.hpp），供 Keil Watch 直接观察。
//
// 计时流程：
//   1) begin_frame(frame_id, 帧起点)：沿用执行器已取好的帧起点时间，并刷新帧号。
//   2) 逐个节点：begin_node(id) 实时读时钟记为节点起点 → end_node() 实时读时钟求差，
//      得到该节点耗时，更新“最近节点耗时 / 历史最大节点耗时”。
//   3) end_frame(帧终点)：与帧起点求差，更新“最近帧耗时 / 历史最大帧耗时”。
//
// 时间单位：全部为微秒（us）。取时统一走注入的时钟 clock_.now_us()（STM32 侧为 DWT）。
//
// 容错：若时间戳出现回绕 / 反序（终点小于起点），或节点配对状态无效，则直接返回，
//       不写入错误数据，避免把异常值污染成“最大值”。
// ============================================================================
namespace robot::platform::stm32 {

// 记录一帧的开始。
// 帧级计时不在这里读时钟，而是直接用执行器传入的帧起点时间，保证帧耗时与实际调度
// 起点一致；同一调用顺便把帧号写入调试快照。
// 参数 frame_id     ：本帧帧号。
// 参数 timestamp_us ：本帧起点时间戳，单位微秒（us）。
void stm32_profiler::begin_frame(uint32_t frame_id, uint64_t timestamp_us)
{
    frame_start_us_ = timestamp_us;
    g_robot_debug.frame_id = frame_id;
}

// 记录一个节点的开始。
// 与 begin_frame 不同：节点计时必须“实时”读取当前时钟作为起点，否则会把整帧起点
// 误当作节点起点，导致该节点耗时被算成大半个帧的时间。
// 参数 id：当前节点的插件 ID；同时置为“正在计时”，供 end_node 配对校验。
void stm32_profiler::begin_node(robot::framework::plugin_id id)
{
    current_node_id_ = id;
    node_start_us_ = clock_.now_us();
}

// 记录当前节点的结束，并更新节点耗时统计。
// 无参数。步骤：实时读取结束时间 → 校验配对与时间顺序 → 计算耗时（微秒）→
// 写入“最近节点耗时”并刷新“历史最大节点耗时”。
void stm32_profiler::end_node()
{
    // timestamp_us：节点结束时刻的当前时间，单位微秒（us）。
    const uint64_t timestamp_us = clock_.now_us();
    // 配对失败（没有正在计时的节点）或时钟回绕（结束早于开始）时放弃本次统计。
    if (current_node_id_ == robot::framework::k_invalid_plugin_id || timestamp_us < node_start_us_) {
        return;
    }
    // duration：本次节点的实际耗时，单位微秒（us）；起点终点均已确认有序，可安全截断为 32 位。
    const uint32_t duration = static_cast<uint32_t>(timestamp_us - node_start_us_);
    // 刷新“最近一次节点耗时”。
    g_robot_debug.last_node_duration_us = duration;
    // 仅当本次更慢时才更新历史最大值（最大值单调不减，不会因某帧变快而回退）。
    if (duration > g_robot_debug.max_node_duration_us) {
        g_robot_debug.max_node_duration_us = duration;
    }
    // 清空计时状态，表示当前没有节点在计时。
    current_node_id_ = robot::framework::k_invalid_plugin_id;
}

// 记录一帧的结束，并更新整帧耗时统计。
// 参数 timestamp_us：本帧结束时刻的时间戳，单位微秒（us）。
// 步骤：校验时间顺序（终点不早于起点）→ 与帧起点相减得整帧耗时 → 更新“最近帧耗时”
// 和“历史最大帧耗时”。
void stm32_profiler::end_frame(uint64_t timestamp_us)
{
    // 时钟回绕（终点早于起点）时放弃本次统计，避免写入异常的大值。
    if (timestamp_us < frame_start_us_) {
        return;
    }
    // duration：本帧整帧耗时，单位微秒（us）。
    const uint32_t duration = static_cast<uint32_t>(timestamp_us - frame_start_us_);
    // 刷新“最近一帧耗时”。
    g_robot_debug.last_frame_duration_us = duration;
    // 仅当本帧更慢时才更新历史最大值（最大值单调不减）。
    if (duration > g_robot_debug.max_frame_duration_us) {
        g_robot_debug.max_frame_duration_us = duration;
    }
}

} // namespace robot::platform::stm32
