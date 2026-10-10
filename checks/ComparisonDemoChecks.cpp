// 编译期验证合成输入下的独立状态；不进入固件。
#include "Nodes/Diagnostics/ComparisonDemoState.hpp"
#include "Framework/OutputComparison.hpp"

namespace {
using robot::diagnostics::ComparisonDemoInput;
using robot::diagnostics::IndependentAccumulator;
using robot::framework::ComparisonStatus;
using robot::framework::ComparisonTolerance;
using robot::framework::compareOutputs;

constexpr bool independentStates()
{
    IndependentAccumulator reference;
    IndependentAccumulator candidate;
    const ComparisonDemoInput first{7, 1U, 100U};
    reference.advance(first);
    candidate.advance(first);
    candidate.reset(2U);
    return reference.output().value[0] == 7.0 && reference.output().key.step == 1U
        && reference.output().key.state_epoch == 1U && !candidate.output().present;
}

constexpr bool alignedAndBiased()
{
    IndependentAccumulator reference;
    IndependentAccumulator candidate;
    const ComparisonDemoInput first{7, 1U, 100U};
    reference.advance(first);
    candidate.advance(first);
    const ComparisonTolerance<2U> exact{};
    return compareOutputs(reference.output(), candidate.output(), exact).status == ComparisonStatus::Equal
        && compareOutputs(reference.output(), candidate.output(3), exact).status == ComparisonStatus::Different
        && candidate.output().value[0] == 7.0;
}

constexpr bool skipDoesNotCatchUp()
{
    IndependentAccumulator reference;
    IndependentAccumulator candidate;
    reference.advance({1, 1U, 100U});
    candidate.advance({1, 1U, 100U});
    reference.advance({1, 2U, 200U});
    reference.advance({1, 3U, 300U});
    candidate.advance({1, 3U, 300U});
    return compareOutputs(reference.output(), candidate.output(), ComparisonTolerance<2U>{}).status
        == ComparisonStatus::Unaligned;
}

constexpr bool commonResetRestoresAlignment()
{
    IndependentAccumulator reference;
    IndependentAccumulator candidate;
    reference.advance({5, 1U, 100U});
    reference.reset(3U);
    candidate.reset(3U);
    reference.advance({-7, 2U, 200U});
    candidate.advance({-7, 2U, 200U});
    return compareOutputs(reference.output(), candidate.output(), ComparisonTolerance<2U>{}).status
        == ComparisonStatus::Equal;
}

constexpr bool duplicateInputIsHeld()
{
    IndependentAccumulator state;
    state.advance({7, 1U, 100U});
    const bool consumed_again = state.advance({7, 1U, 100U});
    return !consumed_again && state.output().key.step == 1U && state.output().value[0] == 7.0;
}

constexpr bool sequenceWrapUsesTime()
{
    IndependentAccumulator state;
    state.advance({7, 0xFFFFFFFFU, 100U});
    state.advance({7, 0U, 200U});
    return state.output().present && state.output().key.step == 2U
        && state.output().key.input_sequence == 0U;
}

constexpr bool boundedArithmetic()
{
    IndependentAccumulator state;
    state.advance({2147483647, 1U, 100U});
    state.advance({-2147483647 - 1, 2U, 200U});
    return state.output().value[0] > -1000000.0 && state.output().value[0] < 1000000.0;
}

static_assert(independentStates());
static_assert(alignedAndBiased());
static_assert(skipDoesNotCatchUp());
static_assert(commonResetRestoresAlignment());
static_assert(duplicateInputIsHeld());
static_assert(sequenceWrapUsesTime());
static_assert(boundedArithmetic());
} // namespace
