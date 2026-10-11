// 使用目标 C++17 编译器检查契约；不进入固件、不建立主机测试运行器。
#include <limits>
#include "Framework/SnapshotMailbox.hpp"
#include "Framework/OutputComparison.hpp"

namespace {
using namespace robot::framework;

constexpr timed_snapshot<unsigned> k_empty{}; // 从未发布的快照。
constexpr timed_snapshot<unsigned> k_at_zero{7U, 0U, 1U, true}; // 时间零也是合法样本。
constexpr timed_snapshot<unsigned> k_at_ten{7U, 10U, 2U, true}; // 用于 TTL 边界检查。
static_assert(evaluate_snapshot(k_empty, 100U, 5U).status == snapshot_status::empty);
static_assert(evaluate_snapshot(k_at_zero, 0U, 0U).status == snapshot_status::fresh);
static_assert(evaluate_snapshot(k_at_ten, 15U, 5U).status == snapshot_status::fresh);
static_assert(evaluate_snapshot(k_at_ten, 16U, 5U).status == snapshot_status::stale);
static_assert(evaluate_snapshot(k_at_ten, 9U, 5U).status == snapshot_status::clock_mismatch);
static_assert(evaluate_snapshot(k_at_ten, 16U, 5U).age_us == 6U);
static_assert(k_at_ten.sequence == 2U && k_at_ten.sampled_at_us == 10U);

// 相同采样、状态版本及步数的结果才能比较。
constexpr comparison_key k_key{100U, 7U, 1U, 3U};
constexpr comparison_sample<2U> k_reference{k_key, {10.0, -2.0}, true};
constexpr comparison_sample<2U> k_same{k_key, {10.0, -2.0}, true};
constexpr comparison_sample<2U> k_different{k_key, {12.0, -2.0}, true};
constexpr comparison_tolerance<2U> k_exact{};
constexpr comparison_tolerance<2U> k_within_two{{2.0, 0.0}};
static_assert(compare_outputs(k_reference, k_same, k_exact).status == comparison_status::equal);
static_assert(compare_outputs(k_reference, k_different, k_exact).status == comparison_status::different);
static_assert(compare_outputs(k_reference, k_different, k_within_two).status == comparison_status::equal);
static_assert(compare_outputs(k_reference, k_different, k_exact).difference[0] == 2.0);
static_assert(compare_outputs(k_reference, k_different, k_exact).max_abs_difference == 2.0);

constexpr comparison_sample<2U> k_other_step{{100U, 7U, 1U, 4U}, {10.0, -2.0}, true};
constexpr comparison_sample<2U> k_other_epoch{{100U, 7U, 2U, 3U}, {10.0, -2.0}, true};
constexpr comparison_sample<2U> k_other_input{{100U, 8U, 1U, 3U}, {10.0, -2.0}, true};
constexpr comparison_sample<2U> k_other_time{{101U, 7U, 1U, 3U}, {10.0, -2.0}, true};
constexpr comparison_sample<2U> k_missing{};
constexpr comparison_tolerance<2U> k_negative{{-1.0, 0.0}};
static_assert(compare_outputs(k_reference, k_other_step, k_exact).status == comparison_status::unaligned);
static_assert(compare_outputs(k_reference, k_other_epoch, k_exact).status == comparison_status::unaligned);
static_assert(compare_outputs(k_reference, k_other_input, k_exact).status == comparison_status::unaligned);
static_assert(compare_outputs(k_reference, k_other_time, k_exact).status == comparison_status::unaligned);
static_assert(compare_outputs(k_reference, k_missing, k_exact).status == comparison_status::missing);
static_assert(compare_outputs(k_reference, k_same, k_negative).status == comparison_status::invalid_tolerance);

constexpr double k_nan = std::numeric_limits<double>::quiet_NaN(); // 禁止把 NaN 当作相等。
constexpr double k_infinity = std::numeric_limits<double>::infinity(); // 禁止比较无限值。
constexpr comparison_sample<2U> k_non_finite{k_key, {k_nan, -2.0}, true};
constexpr comparison_tolerance<2U> k_infinite_tolerance{{k_infinity, 0.0}};
static_assert(compare_outputs(k_reference, k_non_finite, k_exact).status == comparison_status::non_finite);
static_assert(compare_outputs(k_reference, k_same, k_infinite_tolerance).status == comparison_status::invalid_tolerance);

// 浮点舍入不能让有限输入的减法溢出绕过验证。
constexpr double k_largest = std::numeric_limits<double>::max();
constexpr comparison_sample<1U> k_overflow_reference{k_key, {-0x1.8p971}, true};
constexpr comparison_sample<1U> k_overflow_candidate{k_key, {k_largest - 0x1p971}, true};
constexpr comparison_sample<1U> k_negative_reference{k_key, {0x1.8p971}, true};
constexpr comparison_sample<1U> k_negative_candidate{k_key, {-k_largest + 0x1p971}, true};
constexpr comparison_tolerance<1U> k_single_exact{};
static_assert(compare_outputs(k_overflow_reference, k_overflow_candidate, k_single_exact).status
    == comparison_status::non_finite);
static_assert(compare_outputs(k_negative_reference, k_negative_candidate, k_single_exact).status
    == comparison_status::non_finite);
} // namespace
