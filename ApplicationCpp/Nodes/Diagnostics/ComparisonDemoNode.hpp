#ifndef robot_comparison_demo_node_hpp
#define robot_comparison_demo_node_hpp

#include "ComparisonDemoState.hpp"
#include "Framework/PluginNode.hpp"
#include "Framework/Ports.hpp"

namespace robot::diagnostics {

/* ============================================================================
 * comparison_demo_node.hpp —— 比较演示节点（演示/验证用，非功能代码）
 * ----------------------------------------------------------------------------
 * 组装一条最小验证链路：
 *   comparison_input_node（唯一输入生产者）
 *        ├──> comparison_accumulator_node（reference 参考实例）
 *        └──> comparison_accumulator_node（candidate 候选实例）
 *                   两路 ──> comparison_result_node（比较器）
 * 两个累加器各自持有独立的 independent_accumulator，对同一输入做相同的确定性
 * 整数累计；比较器逐帧核对两路结果，从而验证框架的调度/快照/输入传递是否保持
 * 确定性（同一输入 ⇒ 同一输出）。任何差异都会被计入 observation()。
 * 关键概念：
 *   - 确定性：相同输入序列 + 相同 epoch ⇒ 逐帧输出完全一致。
 *   - 可复现：重置（换 epoch）后重放同一输入序列，应得到与上次相同的结果。
 *   - 未对齐(unaligned)：两路不在同一条时间线（例如候选被暂停/跳帧导致读到旧样本）。
 *   - 比较键：由样本自带的身份键构成，不能用"当前帧号"代替，否则会掩盖暂停。
 * ========================================================================== */

using demo_comparison_sample = robot::framework::comparison_sample<2U>; // 2 维比较样本的类型别名，供整条演示链路统一使用。

// 应用每帧一次性采样的 Watch 请求，只用于算术演示。
struct comparison_demo_controls {
    int32_t input_value{7}; // 公共整数输入，范围 -1000..1000；越界则本帧被拒绝、保留旧值。默认 7。
    int32_t candidate_bias{0}; // 候选输出偏差，范围 -1000..1000；只叠加到候选结果、不进入状态机。默认 0。
    uint32_t reset_all_request{0U}; // 数值变化时请求"共同重置"（参考 + 候选）。默认 0。
    uint32_t reset_candidate_request{0U}; // 数值变化时请求"单侧重置"（仅候选）。默认 0。
    bool is_pause_candidate{false}; // 暂停候选状态推进（会读到旧样本、造成时间线落后）。默认 false。
};

// 一个比较帧以及累计诊断；缺失/未对齐不视为数值不同。
struct comparison_demo_observation {
    demo_comparison_sample reference{}; // 参考实例本帧的完整结果（键 + 数值 + 有效标志）。
    demo_comparison_sample candidate{}; // 候选实例本帧的完整结果。
    robot::framework::comparison_result<2U> result{}; // 当前帧的比较结果（状态 + 最大绝对差）。
    uint32_t frame_id{0U}; // 最近一次比较所在帧的帧号。默认 0。
    uint32_t equal_count{0U}; // 累计：比较键对齐且数值相等的帧数。默认 0。
    uint32_t different_count{0U}; // 累计：比较键对齐但数值不同的帧数。默认 0。
    uint32_t unaligned_count{0U}; // 累计：两路时间线不一致（比较键不同）的帧数。默认 0。
    uint32_t invalid_count{0U}; // 累计：缺失或数值非有限（NaN/Inf）的帧数。默认 0。
    double max_abs_difference{0.0}; // 历史"已对齐"结果中的最大绝对差（仅 equal/different 时更新）。默认 0.0。
};

// 每帧只生产一次输入，供两个状态独立的节点只读。
// 唯一写者：本节点是 signal_ 的唯一修改者，因此下游无需加锁即可只读该信号。
class comparison_input_node final : public robot::framework::plugin_node {
public:
    comparison_input_node() : plugin_node(101U) {} // 构造：使用固定演示 ID 101。
    void set_value(int32_t value) { value_ = value; } // 设置下一帧要发布的输入值（由 apply_controls 在帧边界调用）。
    int32_t value() const { return value_; } // 返回最近一次设置（合法）的输入值。
    // process —— 每帧把当前 value 发布为一条可见信号。
    // 参数 context：帧上下文，提供 frame_id 与 timestamp_us。
    // 返回：process_result::ok。发布内容为 {value_, 本帧 frame_id, 本帧时间戳}。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    void on_skipped(robot::framework::frame_context&) override { signal_.invalidate(); } // 本帧被调度跳过：作废信号，避免下游读到陈旧输入。
    const robot::framework::frame_signal<comparison_demo_input>& signal() const { return signal_; } // 只读访问本节点输出信号，供下游 bind。
private:
    int32_t value_{7}; // 公共测试输入。默认 7。
    robot::framework::frame_signal<comparison_demo_input> signal_{}; // 唯一写者的固定存储（本节点独占写入）。
};

// 相同类的两个实例持有不同的 independent_accumulator 对象。
// 即：reference 与 candidate 是两个独立对象，各自 state_ 互不相干，保证"独立状态机"。
class comparison_accumulator_node final : public robot::framework::plugin_node {
public:
    explicit comparison_accumulator_node(robot::framework::plugin_id id) : plugin_node(id) {} // 构造：由调用方指定节点 ID（参考=102、候选=103）。
    void bind(const robot::framework::frame_signal<comparison_demo_input>& source) { input_.bind(source); } // 绑定公共输入信号（只读）。
    void set_paused(bool is_paused) { is_paused_ = is_paused; } // 设置是否暂停；暂停时保持最后一次完整结果，既不推进也不清空。
    void set_bias(int32_t bias) { bias_ = bias; } // 设置输出偏差（只影响导出值，不写入 state_）。
    int32_t bias() const { return bias_; } // 返回最近一次设置的合法偏差。
    void reset(uint32_t epoch) { state_.reset(epoch); signal_.invalidate(); } // 以新 epoch 重置本实例状态并作废本节点输出；不触碰另一实例。
    // configure —— 启动前检查必填输入端口是否已连接。
    // 返回：已连接则走基类默认配置；否则 configuration_fault（配置错误）。
    robot::framework::plugin_status configure() override;
    // process —— 每帧按暂停/输入有效性推进状态并发布结果。
    // 参数 context：帧上下文（frame_id / timestamp_us / 调度信息）。
    // 返回：暂停→output_held；无新输入→no_new_data；重复输入→output_held；正常推进→ok。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    void on_skipped(robot::framework::frame_context&) override { signal_.invalidate(); } // 本帧被跳过：作废输出，防止下游读到陈旧样本。
    const robot::framework::frame_signal<demo_comparison_sample>& signal() const { return signal_; } // 只读访问本节点输出信号，供比较器 bind。
private:
    independent_accumulator state_{}; // 本节点独占的状态，完全独立于另一节点（独立状态机的核心）。
    robot::framework::input_port<comparison_demo_input> input_{}; // 公共只读输入端口。
    robot::framework::frame_signal<demo_comparison_sample> signal_{}; // 本节点独占的结果信号（唯一写者）。
    int32_t bias_{0}; // 输出偏差；不进入 state_，因此不污染状态。默认 0。
    bool is_paused_{false}; // 是否暂停并保持旧结果。默认 false。
};

// 只读两个计算结果，容许读到暂停一路的旧样本并明确标为未对齐。
// 比较器对状态机"零反馈"：只观察，不影响任何累加器。
class comparison_result_node final : public robot::framework::plugin_node {
public:
    comparison_result_node() : plugin_node(104U) {} // 构造：使用固定演示 ID 104。
    // bind —— 绑定参考与候选两路的只读结果信号。
    // 参数 reference：参考实例结果信号；candidate：候选实例结果信号。
    void bind(const robot::framework::frame_signal<demo_comparison_sample>& reference,
              const robot::framework::frame_signal<demo_comparison_sample>& candidate);
    // configure —— 启动前要求两个结果端口都已连接。
    // 返回：都连接则走基类默认配置；否则 configuration_fault。
    robot::framework::plugin_status configure() override;
    // process —— 每帧读取两路样本、比较并更新累计诊断。
    // 参数 context：帧上下文（用于记录 frame_id）。
    // 返回：恒为 ok（诊断差异不视为图故障，也不上报为硬件故障）。
    robot::framework::process_result process(robot::framework::frame_context& context) override;
    // on_skipped —— 本帧被调度跳过：清空当帧结果（防陈旧），但保留累计计数。
    void on_skipped(robot::framework::frame_context& context) override;
    const comparison_demo_observation& observation() const { return observation_; } // 只读访问累计观测结果。
private:
    robot::framework::input_port<demo_comparison_sample> reference_{}; // 参考只读口。
    robot::framework::input_port<demo_comparison_sample> candidate_{}; // 候选只读口。
    comparison_demo_observation observation_{}; // 由比较节点独占更新的观测/累计诊断。
};

// 容器只组装与接收请求，内部节点统一由根执行计划执行。
// 作用：把四个子节点组成一张内部图（compose），并提供控制入口（apply_controls）。
class comparison_demo_node final : public robot::framework::plugin_node {
public:
    comparison_demo_node() : plugin_node(100U) {} // 构造：容器固定演示 ID 100。
    // compose —— 组装内部子图：添加节点、连线、声明依赖，返回组装状态。
    robot::framework::plugin_status compose() override;
    // process —— 容器自身不参与计算，恒返回 ok（实际计算由子节点完成）。
    robot::framework::process_result process(robot::framework::frame_context&) override
    { return robot::framework::process_result::ok; }
    void apply_controls(const comparison_demo_controls& controls); // 应用一帧 Watch 请求；仅执行器任务在帧边界调用。
    const comparison_demo_observation& observation() const { return comparison_.observation(); } // 转发比较器的观测结果。
    uint32_t rejected_frames() const { return rejected_frames_; } // 因参数非法被拒绝的帧数。
    int32_t applied_input_value() const { return source_.value(); } // 当前生效的输入值。
    int32_t applied_candidate_bias() const { return candidate_.bias(); } // 当前生效的候选偏差。
private:
    comparison_input_node source_{}; // 唯一公共输入生产者（ID 101）。
    comparison_accumulator_node reference_{102U}; // 参考实例（ID 102）。
    comparison_accumulator_node candidate_{103U}; // 候选实例（ID 103）。
    comparison_result_node comparison_{}; // 只观察、无反馈的比较节点（ID 104）。
    uint32_t last_reset_all_{0U}; // 上一次已消费的"共同重置"请求值（用于边沿检测）。默认 0。
    uint32_t last_reset_candidate_{0U}; // 上一次已消费的"单侧重置"请求值（用于边沿检测）。默认 0。
    uint32_t epoch_{1U}; // 演示状态版本分配器；每次重置递增，回绕到 0 时跳过 0。默认 1。
    uint32_t rejected_frames_{0U}; // 非法参数帧计数（仅诊断，不改变计算状态）。默认 0。
};

} // namespace robot::diagnostics
#endif // robot_comparison_demo_node_hpp
