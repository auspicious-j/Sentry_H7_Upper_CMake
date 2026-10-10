#ifndef ROBOT_OUTPUT_COMPARISON_HPP
#define ROBOT_OUTPUT_COMPARISON_HPP

#include <cstddef>
#include <cstdint>
#include <limits>
#include "ComparisonTypes.hpp"

// ARMClang 默认可能假设不存在 NaN/Inf；比较诊断不能在该假设下编译。
// 只为包含本接口的新翻译单元设置 -fno-fast-math，不修改旧业务编译选项。
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ != 0)
#error "OutputComparison requires -fno-fast-math to validate NaN and infinity"
#endif

namespace robot::framework {


namespace comparison_detail {
// value：constexpr 有限数判断；NaN 的有序比较均为假。
constexpr bool finite(double value)
{
    return value >= -std::numeric_limits<double>::max()
        && value <= std::numeric_limits<double>::max();
}

// candidate/reference：先判断减法范围，避免在编译期或运行期产生溢出。
constexpr bool subtractionFits(double candidate, double reference)
{
    constexpr double half_limit = std::numeric_limits<double>::max() * 0.5;
    // 先缩放再作差；半差最大仍在有限范围，避免阈值加法的舍入漏洞。
    const double half_difference = candidate * 0.5 - reference * 0.5;
    return half_difference >= -half_limit && half_difference <= half_limit;
}
} // namespace comparison_detail

// reference/candidate：两个只读完整结果；tolerance：统一单位下的逐通道容差。
// 纯函数：不驱动业务、不修改任一方状态、不发出任何硬件输出。
template <std::size_t ChannelCount>
constexpr ComparisonResult<ChannelCount> compareOutputs(
    const ComparisonSample<ChannelCount>& reference,
    const ComparisonSample<ChannelCount>& candidate,
    const ComparisonTolerance<ChannelCount>& tolerance)
{
    if (!reference.present || !candidate.present) {
        return {};
    }
    if (!sameComparisonKey(reference.key, candidate.key)) {
        return {ComparisonStatus::Unaligned};
    }
    // index：先验证全部容差，禁止负值和无穷掩盖差异。
    for (std::size_t index = 0U; index < ChannelCount; ++index) {
        if (!comparison_detail::finite(tolerance.absolute[index]) || tolerance.absolute[index] < 0.0) {
            return {ComparisonStatus::InvalidTolerance};
        }
    }
    ComparisonResult<ChannelCount> result{}; // 局部结果，不外露未完成的计算。
    result.status = ComparisonStatus::Equal;
    // index：计算通道下标；异常返回空差值而非部分有效数据。
    for (std::size_t index = 0U; index < ChannelCount; ++index) {
        if (!comparison_detail::finite(reference.value[index])
            || !comparison_detail::finite(candidate.value[index])
            || !comparison_detail::subtractionFits(candidate.value[index], reference.value[index])) {
            return {ComparisonStatus::NonFinite};
        }
        const double delta = candidate.value[index] - reference.value[index]; // 有符号差。
        if (!comparison_detail::finite(delta)) { return {ComparisonStatus::NonFinite}; }
        const double absolute = delta < 0.0 ? -delta : delta; // 绝对差。
        result.difference[index] = delta;
        if (absolute > result.max_abs_difference) { result.max_abs_difference = absolute; }
        if (absolute > tolerance.absolute[index]) { result.status = ComparisonStatus::Different; }
    }
    return result;
}

} // namespace robot::framework
#endif // ROBOT_OUTPUT_COMPARISON_HPP
