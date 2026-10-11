#ifndef robot_output_comparison_hpp
#define robot_output_comparison_hpp

// ============================================================================
// 文件：OutputComparison.hpp
// 作用：实现“两套计算结果逐通道对比”的核心纯函数 compare_outputs。
//      给定参考结果(reference)、候选结果(candidate)与逐通道绝对容差(tolerance)，
//      返回结构化结论，供上层诊断/断言使用。
//
// 关键概念（为什么浮点比较需要特殊处理）：
//   1) NaN（Not a Number，非数）：由 0/0、Inf-Inf 等非法运算产生。它与任何数
//      比较都返回 false——包括它自己，即 (NaN == NaN) 为假、(NaN > x) 为假、
//      (NaN < x) 也为假。若不单独判定，NaN 会被误判为“相等”或绕过阈值检查。
//   2) Inf（无穷大）：由非零数除以 0 等溢出产生。Inf - Inf 会得到 NaN，且
//      Inf 与有限数相减可能仍为 Inf，直接作差会产生“看似很大却无法判定”的差值。
//   3) 有限性检查：本文件用 finite() 显式排除 NaN/Inf，避开上述陷阱。
//   4) 减法溢出：两个接近 double 上限的数相减，或阈值与差值相加，都可能发生
//      舍入甚至溢出为 Inf，因此先做“半值缩放作差”再判断范围。
//
// 线程/性能：compare_outputs 为 constexpr 纯函数——不驱动业务、不修改任一方状态、
//           不发任何硬件输出，既可在运行期调用，也可在编译期对常量求值。
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <limits>
#include "ComparisonTypes.hpp"

// ARMClang 默认可能假设程序中不存在 NaN/Inf（-ffast-math / __FINITE_MATH_ONLY__），
// 而本比较诊断依赖精确识别 NaN 与无穷大，在该假设下会失效。因此：
//   - 只为包含本接口的新翻译单元设置 -fno-fast-math；
//   - 不修改旧业务代码的编译选项，避免影响既有行为。
// 若编译单元仍处于“仅有限数学”模式，直接以 #error 中止编译，提示需要 -fno-fast-math。
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ != 0)
#error "OutputComparison requires -fno-fast-math to validate NaN and infinity"
#endif

namespace robot::framework {


namespace comparison_detail {
// finite —— 判断一个 double 是否为有限值（既不是 NaN，也不是 ±Inf）。
// 实现原理：NaN 与任何数的有序比较（>=、<=）都返回 false，故 (NaN >= -max) 为假；
//          若 value 为 +Inf，则 value <= max 为假；若为 -Inf，则 value >= -max 为假。
// 参数：value 待判定的浮点数。
// 返回值：有限返回 true；NaN 或 ±Inf 返回 false。
// 注：仅在 -fno-fast-math 下该判断才能正确识别 NaN/Inf。
constexpr bool finite(double value)
{
    return value >= -std::numeric_limits<double>::max()
        && value <= std::numeric_limits<double>::max();
}

// subtraction_fits —— 预判 (candidate - reference) 是否落在有限范围内，避免溢出。
// 为什么这样做：直接算 a - b 时，两个接近 double 上限的大数相减，中间结果可能
//              溢出为 ±Inf；先各自乘 0.5 再作差（等价于差的一半），其余量仍然有限，
//              从而能安全地判断“差值本身是否会溢出”。
// 参数：candidate 被减数（候选值）；reference 减数（参考值）。
// 返回值：差值可安全表示（|candidate - reference| 不超过 double 上限）返回 true，否则 false。
constexpr bool subtraction_fits(double candidate, double reference)
{
    constexpr double half_limit = std::numeric_limits<double>::max() * 0.5; // double 上限的一半（编译期常量）。
    // 先缩放再作差；半差最大仍在有限范围，避免阈值加法的舍入漏洞。
    const double half_difference = candidate * 0.5 - reference * 0.5; // (candidate - reference) 的一半。
    return half_difference >= -half_limit && half_difference <= half_limit; // 半差在 ±half_limit 内视为安全。
}
} // namespace comparison_detail

// compare_outputs —— 逐通道比较两份计算结果，返回结构化结论（本文件的核心纯函数）。
// 参数：
//   reference：参考实现产出的只读完整结果；
//   candidate：候选实现产出的只读完整结果；
//   tolerance：统一物理单位下的逐通道绝对容差。
// 返回值：comparison_result，含结论 status 与（在 equal/different 时的）逐通道差值。
// 纯函数语义：不驱动业务、不修改任一方状态、不发出任何硬件输出；
//            既可在运行期调用，也可在编译期对常量求值。
template <std::size_t channel_count>
constexpr comparison_result<channel_count> compare_outputs(
    const comparison_sample<channel_count>& reference,
    const comparison_sample<channel_count>& candidate,
    const comparison_tolerance<channel_count>& tolerance)
{
    // 前置检查 1：任一方没有完整结果（is_present 为 false）则无法比较，返回默认的 missing 结论。
    if (!reference.is_present || !candidate.is_present)
    {
        return {};
    }
    // 前置检查 2：两方身份（采样时刻/输入序号/状态版本/计算步数）必须一致，
    // 否则差异可能源于输入或状态不同，判为 unaligned 而非算法错误。
    if (!same_comparison_key(reference.key, candidate.key))
    {
        return {comparison_status::unaligned};
    }
    // 前置检查 3：先逐通道验证容差合法，禁止负值以及 NaN/无穷掩盖真实差异。
    // index：遍历通道下标。
    for (std::size_t index = 0U; index < channel_count; ++index)
    {
        if (!comparison_detail::finite(tolerance.absolute[index]) || tolerance.absolute[index] < 0.0)
        {
            return {comparison_status::invalid_tolerance};
        }
    }
    // result：局部累积结果，只有全部通道校验通过后才对外发布，绝不外露未完成的计算。
    comparison_result<channel_count> result{};
    result.status = comparison_status::equal; // 先假定一致，发现超差再改成 different。
    // index：逐通道计算差值并累计；一旦发现 NaN/Inf 或减法溢出，立即返回空差值而非部分有效数据。
    for (std::size_t index = 0U; index < channel_count; ++index)
    {
        // 任一侧数值非有限，或两者之差可能溢出，都判为 non_finite。
        if (!comparison_detail::finite(reference.value[index])
            || !comparison_detail::finite(candidate.value[index])
            || !comparison_detail::subtraction_fits(candidate.value[index], reference.value[index])) {
            return {comparison_status::non_finite};
        }
        const double delta = candidate.value[index] - reference.value[index]; // 有符号差（candidate - reference）。
        if (!comparison_detail::finite(delta))
        {
            return {comparison_status::non_finite};
        } // 差值仍须复核有限性。
        const double absolute = delta < 0.0 ? -delta : delta; // 绝对差 |delta|。
        result.difference[index] = delta; // 记录本通道差值。
        if (absolute > result.max_abs_difference)
        {
            result.max_abs_difference = absolute;
        } // 更新各通道最大绝对差。
        if (absolute > tolerance.absolute[index])
        {
            result.status = comparison_status::different;
        } // 超出本通道容差即判为不一致。
    }
    return result;
}

} // namespace robot::framework
#endif // robot_output_comparison_hpp
