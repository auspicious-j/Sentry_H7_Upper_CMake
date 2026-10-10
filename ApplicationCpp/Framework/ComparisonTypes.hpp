#ifndef ROBOT_COMPARISON_TYPES_HPP
#define ROBOT_COMPARISON_TYPES_HPP

#include <cstddef>
#include <cstdint>

namespace robot::framework {

// 比较身份必须由输入交接层与两套独立计算共同遵守，不能用读取时刻冒充采样时间。
struct ComparisonKey {
    uint64_t sampled_at_us{0U}; // 原始输入采样时刻，双方使用同一时间域。
    uint32_t input_sequence{0U}; // 原始输入序号，不因消费而递增。
    uint32_t state_epoch{0U}; // 独立状态共同的初始化/重置版本。
    uint32_t step{0U}; // 自该状态版本开始的计算步数。
};

// left/right：判断输入和状态时间线是否一致，不比较输出值。
constexpr bool sameComparisonKey(const ComparisonKey& left, const ComparisonKey& right)
{
    return left.sampled_at_us == right.sampled_at_us
        && left.input_sequence == right.input_sequence
        && left.state_epoch == right.state_epoch && left.step == right.step;
}

// 固定数量的通用数值结果；物理单位、坐标系与通道排列必须在调用前统一。
template <std::size_t ChannelCount>
struct ComparisonSample {
    static_assert(ChannelCount > 0U, "Comparison needs at least one channel");
    ComparisonKey key{}; // 产生这份结果的输入和状态身份。
    double value[ChannelCount]{}; // 固定顺序的数值通道，不包含硬件发送行为。
    bool present{false}; // 是否已有完整有效结果。
};

// 按通道配置绝对容差；零表示要求数值严格相等。
template <std::size_t ChannelCount>
struct ComparisonTolerance {
    double absolute[ChannelCount]{}; // 每个值都必须有限且非负。
};

// 无法比较与比较后不同分开，防止把未对齐样本计为算法错误。
enum class ComparisonStatus : uint8_t {
    Missing = 0, // 至少一方没有完整结果。
    Unaligned, // 输入、状态版本或计算步数不一致。
    InvalidTolerance, // 容差为负、NaN 或无穷。
    NonFinite, // 输入值或差值超出有限数值范围。
    Equal, // 所有通道都在各自容差内。
    Different // 至少一个通道超出容差。
};

// 只在 Equal/Different 状态下使用差值字段；其他状态不发布部分差值。
template <std::size_t ChannelCount>
struct ComparisonResult {
    ComparisonStatus status{ComparisonStatus::Missing}; // 本次比较结论。
    double difference[ChannelCount]{}; // candidate - reference。
    double max_abs_difference{0.0}; // 本次各通道绝对差的最大值。
};

} // namespace robot::framework
#endif // ROBOT_COMPARISON_TYPES_HPP

