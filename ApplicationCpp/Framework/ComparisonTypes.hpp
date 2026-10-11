#ifndef ROBOT_COMPARISON_TYPES_HPP
#define ROBOT_COMPARISON_TYPES_HPP

// ============================================================================
// 文件：ComparisonTypes.hpp
// 作用：定义“双路计算对比”框架使用的通用数据类型与判定工具。
//      哨兵上位机为验证算法正确性，会用两套独立实现分别算出同一物理量
//      （reference 参考实现 与 candidate 候选实现），再把两者结果逐通道比较。
//      本文件只描述“比较所需的数据结构与身份标识”，不做实际比较计算
//      （真正的比较逻辑在同目录的 OutputComparison.hpp）。
//
// 设计要点：
//   1) 比较的前提是“两个样本来自同一次输入、同一份状态”，因此用 ComparisonKey
//      记录采样时刻/输入序号/状态版本/计算步数，禁止用“读取结果的时刻”冒充采样时刻。
//   2) 浮点数存在 NaN（非数）与 ±Inf（无穷大）两种特殊值，它们不满足普通大小
//      关系，直接比较会得到反直觉结果，所以必须单独判定（见 ComparisonStatus）。
//   3) 通道数量在编译期以模板参数 ChannelCount 固定，避免运行期动态分配。
//
// 约定：本文件中所有类型均以“值语义”使用，可安全拷贝；无任何硬件副作用。
// ============================================================================

#include <cstddef>
#include <cstdint>

namespace robot::framework {

// ComparisonKey —— “比较身份”：唯一标识一份计算结果所对应的输入与状态来源。
// 为什么需要：两套实现只有在“看同一份输入、处于同一份状态”时算出的结果才可比；
//            否则差异来自输入/状态不同，而非算法不同。
// 该身份必须由输入交接层与两套独立计算共同遵守，不能用“读取结果的那一刻”冒充采样时刻。
struct ComparisonKey {
    uint64_t sampled_at_us{0U}; // 原始输入采样时刻（微秒，默认 0）；双方必须处于同一时间域（同一时钟源）。
    uint32_t input_sequence{0U}; // 原始输入序号（默认 0，递增计数）；标识“第几帧输入”，不因被消费而递增。
    uint32_t state_epoch{0U}; // 状态版本号（默认 0）；每次初始化/复位后加一，用于区分复位前后的状态。
    uint32_t step{0U}; // 自该状态版本（state_epoch）开始后的累计计算步数（默认 0）。
};

// sameComparisonKey —— 判断两个比较身份是否一致。
// 参数：left、right 为待比较的两个身份键。
// 返回值：四个字段全部相等返回 true，否则 false。
// 注意：仅比较“输入时间线与状态版本”是否一致，不比较任何输出数值。
constexpr bool sameComparisonKey(const ComparisonKey& left, const ComparisonKey& right)
{
    return left.sampled_at_us == right.sampled_at_us
        && left.input_sequence == right.input_sequence
        && left.state_epoch == right.state_epoch && left.step == right.step;
}

// ComparisonSample —— 一份固定通道数的通用数值结果（模板参数 ChannelCount 为通道数）。
// 用途：承载某次计算产出的各通道数值，作为比较的输入。
// 约定：物理单位、坐标系与通道排列顺序必须在调用前由上层统一；
//      本结构体不携带单位信息，也不包含“向硬件发送”之类的动作（纯数据）。
template <std::size_t ChannelCount>
struct ComparisonSample {
    static_assert(ChannelCount > 0U, "Comparison needs at least one channel"); // 编译期约束：至少 1 个通道，避免无意义的空比较。
    ComparisonKey key{}; // 产生这份结果的“输入与状态身份”（默认全 0），用于判定能否与另一份结果比较。
    double value[ChannelCount]{}; // 固定顺序的数值通道（默认全 0.0），不含硬件发送行为。
    bool present{false}; // 是否已写入完整有效结果；默认 false（表示“尚无结果”），防止未初始化数据被误比较。
};

// ComparisonTolerance —— 逐通道的绝对容差配置（模板参数 ChannelCount 为通道数）。
// 用途：为每个通道分别指定“允许的最大绝对误差”，用于容忍浮点舍入与实现顺序差异。
// 约定：零表示要求数值严格相等；每个容差都必须有限且非负。
template <std::size_t ChannelCount>
struct ComparisonTolerance {
    double absolute[ChannelCount]{}; // 每个通道的绝对容差（默认全 0.0）；必须有限且 >= 0，负值或 NaN/Inf 视为非法配置。
};

// ComparisonStatus —— 比较结论枚举，用于区分“没得比/比不了”与“比过之后确实不同”。
// 为什么单独区分：若把“样本未对齐”也计为“算法结果不同”，会污染诊断、误报为算法错误；
//                因此把 Missing/Unaligned/InvalidTolerance/NonFinite 与 Equal/Different 分开表达。
// 说明：底层类型固定为 uint8_t，省空间且便于跨模块传递。
enum class ComparisonStatus : uint8_t {
    Missing = 0, // 至少一方没有完整结果（present 为 false），无法比较。
    Unaligned, // 输入、状态版本(state_epoch)或计算步数(step)不一致，两份结果不可比。
    InvalidTolerance, // 容差配置非法：出现负值、NaN（非数）或无穷大。
    NonFinite, // 输入值或差值出现 NaN/无穷大，超出有限数值范围，结果不可信。
    Equal, // 所有通道的绝对差都在各自容差之内，判定为一致。
    Different // 至少一个通道的绝对差超出其容差，判定为不一致。
};

// ComparisonResult —— 一次比较的完整输出（模板参数 ChannelCount 为通道数）。
// 用途：返回结论 status；在“可比且已比较”时附带逐通道差值，便于定位偏差。
// 约定：只有 status 为 Equal 或 Different（即真正完成了比较）时才发布差值字段；
//      其他状态一律不发布部分差值，difference 与 max_abs_difference 保持默认值。
template <std::size_t ChannelCount>
struct ComparisonResult {
    ComparisonStatus status{ComparisonStatus::Missing}; // 本次比较的结论，默认 Missing（尚无结果）。
    double difference[ChannelCount]{}; // 逐通道有符号差值 candidate - reference（默认全 0.0）。
    double max_abs_difference{0.0}; // 本次各通道绝对差 |difference| 的最大值（默认 0.0），用于快速判断最坏偏差。
};

} // namespace robot::framework
#endif // ROBOT_COMPARISON_TYPES_HPP

