#ifndef robot_stm32_profiler_hpp
#define robot_stm32_profiler_hpp

#include "../../ApplicationCpp/Platform/IClock.hpp"
#include "../../ApplicationCpp/Framework/IProfiler.hpp"

// ============================================================================
// stm32_profiler.hpp —— STM32 平台侧的性能探针（profiler）具体实现
// ----------------------------------------------------------------------------
// 作用：实现框架的 robot::framework::i_profiler 抽象接口，用 STM32 上的单调时钟
//       （实际由 DWT 计数器提供，封装在 i_clock 中）为“每帧”和“每个节点”打点计时。
//
// 记录方式（打点法）：
//   - 帧计时：begin_frame 记下本帧起点，end_frame 再传入终点时刻，两者相减得到整帧耗时。
//   - 节点计时：begin_node 在节点真正开始处实时读时钟，end_node 再实时读一次时钟，
//               两者相减得到单个节点的耗时。
//   逐帧、逐节点统计出的结果写入全局调试快照 g_robot_debug，可在 Keil Watch 窗口里
//   直接观察（如 last_frame_duration_us 最近帧耗时、max_frame_duration_us 历史最大帧
//   耗时），用于定位到底是哪一层拖慢了控制周期。
//
// 统计口径：
//   - “最近值”（last_*）每次都会刷新，反映刚刚结束的那一帧 / 那个节点的耗时。
//   - “最大值”（max_*）只增不减，记录自启动以来出现过的最坏耗时（掉帧、超时排查用）。
//
// 时间单位：本文件所有时间戳与耗时参数均为微秒（us）。
//
// 设计要点：本类只依赖抽象时钟接口 i_clock，不直接访问硬件寄存器，方便以后替换计时
//           后端或在 PC 上做离线回放测试。
// ============================================================================
namespace robot::platform::stm32 {

// 基于 STM32 单调时钟的性能探针。
// 负责记录每一帧以及帧内每个节点的耗时，并把“最近一次”和“历史最大值”写入 Keil
// Watch 可观察的全局调试快照 g_robot_debug。
// final 表示本类不再被继承；类内部不访问任何硬件寄存器，取时全部依赖注入的 i_clock。
class stm32_profiler final : public robot::framework::i_profiler {
public:
    // 构造函数：注入一个单调时钟源。
    // 参数 clock：平台无关的单调时钟接口（STM32 侧由 DWT 实现）；仅保存其引用，不拷贝。
    explicit stm32_profiler(const robot::platform::i_clock& clock) : clock_(clock) {}

    // 开始记录一帧（在整帧的所有节点执行之前调用）。
    // 参数 frame_id     ：本帧帧号，仅用于写入调试快照，便于把统计结果对应到具体帧。
    // 参数 timestamp_us ：本帧起点时间戳，单位微秒（us）；此处直接采用执行器传入的时间。
    void begin_frame(uint32_t frame_id, uint64_t timestamp_us) override;
    // 开始记录一个节点（在某个节点即将执行时调用）。
    // 参数 id：当前节点的插件 ID，用于标识是哪个节点在计时。
    // 注意：实现会在此刻实时读取时钟作为节点起点，而不是复用帧起点。
    void begin_node(robot::framework::plugin_id id) override;
    // 结束记录当前节点（与最近一次 begin_node 配对，实时读时钟以算出该节点耗时）。
    void end_node() override;
    // 结束记录一帧（在整帧所有节点执行完毕之后调用）。
    // 参数 timestamp_us：本帧终点时间戳，单位微秒（us）；与 begin_frame 的起点相减得整帧耗时。
    void end_frame(uint64_t timestamp_us) override;

private:
    // 抽象的单调时钟引用：依赖接口而非具体实现，便于将来替换计时后端。
    const robot::platform::i_clock& clock_;
    // 本帧起点时间戳，单位微秒（us）；由 begin_frame 写入，end_frame 使用，默认 0。
    uint64_t frame_start_us_{0U};
    // 当前节点起点时间戳，单位微秒（us）；由 begin_node 写入，end_node 使用，默认 0。
    uint64_t node_start_us_{0U};
    // 当前正在计时的节点 ID；无节点在计时时为 k_invalid_plugin_id（默认值），用于配对校验。
    robot::framework::plugin_id current_node_id_{robot::framework::k_invalid_plugin_id};
};

} // namespace robot::platform::stm32

#endif /* robot_stm32_profiler_hpp */


