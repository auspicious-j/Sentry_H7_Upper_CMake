#ifndef ROBOT_COMPARISON_DEMO_STATE_HPP
#define ROBOT_COMPARISON_DEMO_STATE_HPP

#include "Framework/ComparisonTypes.hpp"

namespace robot::diagnostics {

// 合成的完整输入；只用于通用框架验证，不映射任何传感器或控制量。
struct ComparisonDemoInput {
    int32_t value{0}; // 本次整数输入。
    uint32_t sequence{0U}; // 输入发布序号，允许自然回绕。
    uint64_t sampled_at_us{0U}; // 原始输入时间。
};

// 每个实例直接拥有自己的状态；不借用全局对象或另一实例的数据。
class IndependentAccumulator {
public:
    // input：同一身份只推进一次；计数与取模仅演示有状态计算。
    constexpr bool advance(const ComparisonDemoInput& input)
    {
        if (present_ && key_.input_sequence == input.sequence && key_.sampled_at_us == input.sampled_at_us) {
            return false;
        }
        const int64_t next = static_cast<int64_t>(sum_) + input.value; // 防止任意 int32 输入溢出。
        sum_ = static_cast<int32_t>(next % 1000000);
        key_.input_sequence = input.sequence;
        key_.sampled_at_us = input.sampled_at_us;
        ++key_.step;
        present_ = true;
        return true;
    }

    // epoch：由演示控制层分配的状态版本；只重置当前实例。
    constexpr void reset(uint32_t epoch)
    {
        sum_ = 0;
        key_ = {};
        key_.state_epoch = epoch;
        present_ = false;
    }

    // bias：仅影响导出值，不写入累计状态。
    constexpr robot::framework::ComparisonSample<2U> output(int32_t bias = 0) const
    {
        return {key_, {static_cast<double>(sum_) + bias, static_cast<double>(key_.step)}, present_};
    }

private:
    int32_t sum_{0}; // 本实例的有界累计值。
    robot::framework::ComparisonKey key_{0U, 0U, 1U, 0U}; // 独立输入/状态身份。
    bool present_{false}; // 是否已经消费过输入。
};

} // namespace robot::diagnostics
#endif // ROBOT_COMPARISON_DEMO_STATE_HPP
