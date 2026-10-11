#ifndef robot_comparison_demo_state_hpp
#define robot_comparison_demo_state_hpp

#include "Framework/ComparisonTypes.hpp"

namespace robot::diagnostics {

/* ============================================================================
 * ComparisonDemoState.hpp —— 比较演示的状态层（演示/验证用，非功能代码）
 * ----------------------------------------------------------------------------
 * 目的：用两路彼此完全独立的状态机，对同一算术输入做相同的累计运算，再比较
 *       两路结果是否逐帧一致。由于两路状态隔离、运算又是整数确定性运算，若出现
 *       分歧，就说明框架的调度/快照/输入传递破坏了"确定性"或"可复现性"。
 * 关键概念：
 *   - 序列号 sequence：输入发布者每帧递增的编号，用来判定"是否是新输入"。
 *   - epoch：状态版本号，由上层演示控制层分配；改变 epoch 表示要求该实例重置。
 *            用版本号而非布尔标志，可避免"重置后又被误判为同一状态"。
 *   - 比较键 comparison_key：由（输入序号 + 采样时间 + 状态版本 + 步数）构成的
 *            身份标识；比较器据此判断两路是否处在"同一条时间线"，否则报未对齐。
 *   - 确定性：给定相同输入序列与相同 epoch，状态机逐帧产生完全相同的输出。
 * 本文件只定义数据结构与纯状态推进/重置/导出，不做调度、不访问硬件。
 * ========================================================================== */

// 合成的完整输入；只用于通用框架验证，不映射任何传感器或控制量。
struct comparison_demo_input {
    int32_t value{0}; // 本次整数输入（算术演示的唯一数据来源）。默认 0。
    uint32_t sequence{0U}; // 输入发布序号，由生产者每帧 +1；允许自然回绕（0xFFFFFFFF→0）。默认 0。
    uint64_t sampled_at_us{0U}; // 原始输入采样时间（微秒），用于判断两路是否同一时间线。默认 0。
};

// 每个实例直接拥有自己的状态；不借用全局对象或另一实例的数据。
// 独立累加器：把输入整数做有界累计并记录消费进度，作为可比较的"有状态计算"。
// 实例之间不共享任何数据，因此两路只要输入相同，结果必然相同。
class independent_accumulator {
public:
    // advance —— 消费一帧输入并按需推进内部状态。
    // 参数 input：本帧的合成输入（值 + 序列号 + 采样时间）。
    // 返回：true 表示状态确有推进；false 表示该输入此前已被消费（重复帧，不重复累计）。
    // 生命周期时机：每个节点在自己 process() 内调用一次，操作的是节点独占状态。
    // 幂等性：同一 (sequence, sampled_at_us) 只会推进一次，重复调用直接返回 false。
    constexpr bool advance(const comparison_demo_input& input)
    {
        if (is_present_ && key_.input_sequence == input.sequence && key_.sampled_at_us == input.sampled_at_us)
        {
            return false; // 同一身份（序号+时间）已处理过：不再推进，保证幂等。
        }
        const int64_t next = static_cast<int64_t>(sum_) + input.value; // 先用 int64 求和，防止任意 int32 输入相加时溢出。
        sum_ = static_cast<int32_t>(next % 1000000); // 取模限幅到 ±1000000，保证累计值有界（避免溢出与数值发散）。
        key_.input_sequence = input.sequence; // 记录本次输入序列号，作为身份键的一部分。
        key_.sampled_at_us = input.sampled_at_us; // 记录本次输入采样时间。
        ++key_.step; // 消费步数 +1，代表状态机走过的步数（也参与比较键）。
        is_present_ = true; // 标记已具备有效状态，此后 output() 才会报告 is_present=true。
        return true; // 状态已推进。
    }

    // reset —— 把实例重置到干净初值，并打上新的状态版本 epoch。
    // 参数 epoch：由演示控制层分配的状态版本号（见 comparison_demo_node::apply_controls）。
    // 返回：无。只影响当前实例，绝不动另一路状态。
    constexpr void reset(uint32_t epoch)
    {
        sum_ = 0; // 累计值清零。
        key_ = {}; // 比较键整体清零（序号/采样时间/步数归零，epoch 暂为默认 0）。
        key_.state_epoch = epoch; // 写入新的状态版本，使重置后的结果与重置前在身份上区分开。
        is_present_ = false; // 重置后尚无有效状态。
    }

    // output —— 导出本实例当前状态为可比较样本。
    // 参数 bias：仅叠加到导出值的偏差量，不写入任何内部状态（默认 0）。
    //            因此注入偏置只改变"显示值"、不污染状态机，便于演示"结果不同但状态未变"。
    // 返回：comparison_sample<2U>，含比较键、两维数值（累计值+bias、步数）以及有效标志 is_present。
    constexpr robot::framework::comparison_sample<2U> output(int32_t bias = 0) const
    {
        return {key_, {static_cast<double>(sum_) + bias, static_cast<double>(key_.step)}, is_present_};
    }

private:
    int32_t sum_{0}; // 本实例的有界累计值（取模 1000000 后的结果）。默认 0。
    robot::framework::comparison_key key_{0U, 0U, 1U, 0U}; // 独立输入/状态身份：{序号, 采样时间, epoch=1, 步数}。默认 epoch=1、其余 0。
    bool is_present_{false}; // 是否已经消费过输入（即是否具备有效状态）。默认 false。
};

} // namespace robot::diagnostics
#endif // robot_comparison_demo_state_hpp
