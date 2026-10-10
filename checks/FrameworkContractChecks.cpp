// 使用目标 C++17 编译器检查契约；不进入固件、不建立主机测试运行器。
#include <limits>
#include "Framework/SnapshotMailbox.hpp"
#include "Framework/OutputComparison.hpp"

namespace {
using namespace robot::framework;

constexpr TimedSnapshot<unsigned> kEmpty{}; // 从未发布的快照。
constexpr TimedSnapshot<unsigned> kAtZero{7U, 0U, 1U, true}; // 时间零也是合法样本。
constexpr TimedSnapshot<unsigned> kAtTen{7U, 10U, 2U, true}; // 用于 TTL 边界检查。
static_assert(evaluateSnapshot(kEmpty, 100U, 5U).status == SnapshotStatus::Empty);
static_assert(evaluateSnapshot(kAtZero, 0U, 0U).status == SnapshotStatus::Fresh);
static_assert(evaluateSnapshot(kAtTen, 15U, 5U).status == SnapshotStatus::Fresh);
static_assert(evaluateSnapshot(kAtTen, 16U, 5U).status == SnapshotStatus::Stale);
static_assert(evaluateSnapshot(kAtTen, 9U, 5U).status == SnapshotStatus::ClockMismatch);
static_assert(evaluateSnapshot(kAtTen, 16U, 5U).age_us == 6U);
static_assert(kAtTen.sequence == 2U && kAtTen.sampled_at_us == 10U);

// 相同采样、状态版本及步数的结果才能比较。
constexpr ComparisonKey kKey{100U, 7U, 1U, 3U};
constexpr ComparisonSample<2U> kReference{kKey, {10.0, -2.0}, true};
constexpr ComparisonSample<2U> kSame{kKey, {10.0, -2.0}, true};
constexpr ComparisonSample<2U> kDifferent{kKey, {12.0, -2.0}, true};
constexpr ComparisonTolerance<2U> kExact{};
constexpr ComparisonTolerance<2U> kWithinTwo{{2.0, 0.0}};
static_assert(compareOutputs(kReference, kSame, kExact).status == ComparisonStatus::Equal);
static_assert(compareOutputs(kReference, kDifferent, kExact).status == ComparisonStatus::Different);
static_assert(compareOutputs(kReference, kDifferent, kWithinTwo).status == ComparisonStatus::Equal);
static_assert(compareOutputs(kReference, kDifferent, kExact).difference[0] == 2.0);
static_assert(compareOutputs(kReference, kDifferent, kExact).max_abs_difference == 2.0);

constexpr ComparisonSample<2U> kOtherStep{{100U, 7U, 1U, 4U}, {10.0, -2.0}, true};
constexpr ComparisonSample<2U> kOtherEpoch{{100U, 7U, 2U, 3U}, {10.0, -2.0}, true};
constexpr ComparisonSample<2U> kOtherInput{{100U, 8U, 1U, 3U}, {10.0, -2.0}, true};
constexpr ComparisonSample<2U> kOtherTime{{101U, 7U, 1U, 3U}, {10.0, -2.0}, true};
constexpr ComparisonSample<2U> kMissing{};
constexpr ComparisonTolerance<2U> kNegative{{-1.0, 0.0}};
static_assert(compareOutputs(kReference, kOtherStep, kExact).status == ComparisonStatus::Unaligned);
static_assert(compareOutputs(kReference, kOtherEpoch, kExact).status == ComparisonStatus::Unaligned);
static_assert(compareOutputs(kReference, kOtherInput, kExact).status == ComparisonStatus::Unaligned);
static_assert(compareOutputs(kReference, kOtherTime, kExact).status == ComparisonStatus::Unaligned);
static_assert(compareOutputs(kReference, kMissing, kExact).status == ComparisonStatus::Missing);
static_assert(compareOutputs(kReference, kSame, kNegative).status == ComparisonStatus::InvalidTolerance);

constexpr double kNan = std::numeric_limits<double>::quiet_NaN(); // 禁止把 NaN 当作相等。
constexpr double kInfinity = std::numeric_limits<double>::infinity(); // 禁止比较无限值。
constexpr ComparisonSample<2U> kNonFinite{kKey, {kNan, -2.0}, true};
constexpr ComparisonTolerance<2U> kInfiniteTolerance{{kInfinity, 0.0}};
static_assert(compareOutputs(kReference, kNonFinite, kExact).status == ComparisonStatus::NonFinite);
static_assert(compareOutputs(kReference, kSame, kInfiniteTolerance).status == ComparisonStatus::InvalidTolerance);

// 浮点舍入不能让有限输入的减法溢出绕过验证。
constexpr double kLargest = std::numeric_limits<double>::max();
constexpr ComparisonSample<1U> kOverflowReference{kKey, {-0x1.8p971}, true};
constexpr ComparisonSample<1U> kOverflowCandidate{kKey, {kLargest - 0x1p971}, true};
constexpr ComparisonSample<1U> kNegativeReference{kKey, {0x1.8p971}, true};
constexpr ComparisonSample<1U> kNegativeCandidate{kKey, {-kLargest + 0x1p971}, true};
constexpr ComparisonTolerance<1U> kSingleExact{};
static_assert(compareOutputs(kOverflowReference, kOverflowCandidate, kSingleExact).status
    == ComparisonStatus::NonFinite);
static_assert(compareOutputs(kNegativeReference, kNegativeCandidate, kSingleExact).status
    == ComparisonStatus::NonFinite);
} // namespace
