// 编译期验证合成输入下的独立状态；不进入固件。
#include "Nodes/Diagnostics/ComparisonDemoState.hpp"
#include "Framework/OutputComparison.hpp"

namespace {
using robot::diagnostics::comparison_demo_input;
using robot::diagnostics::independent_accumulator;
using robot::framework::comparison_status;
using robot::framework::comparison_tolerance;
using robot::framework::compare_outputs;

constexpr bool independent_states()
{
    independent_accumulator reference;
    independent_accumulator candidate;
    const comparison_demo_input first{7, 1U, 100U};
    reference.advance(first);
    candidate.advance(first);
    candidate.reset(2U);
    return reference.output().value[0] == 7.0 && reference.output().key.step == 1U
        && reference.output().key.state_epoch == 1U && !candidate.output().is_present;
}

constexpr bool aligned_and_biased()
{
    independent_accumulator reference;
    independent_accumulator candidate;
    const comparison_demo_input first{7, 1U, 100U};
    reference.advance(first);
    candidate.advance(first);
    const comparison_tolerance<2U> exact{};
    return compare_outputs(reference.output(), candidate.output(), exact).status == comparison_status::equal
        && compare_outputs(reference.output(), candidate.output(3), exact).status == comparison_status::different
        && candidate.output().value[0] == 7.0;
}

constexpr bool skip_does_not_catch_up()
{
    independent_accumulator reference;
    independent_accumulator candidate;
    reference.advance({1, 1U, 100U});
    candidate.advance({1, 1U, 100U});
    reference.advance({1, 2U, 200U});
    reference.advance({1, 3U, 300U});
    candidate.advance({1, 3U, 300U});
    return compare_outputs(reference.output(), candidate.output(), comparison_tolerance<2U>{}).status
        == comparison_status::unaligned;
}

constexpr bool common_reset_restores_alignment()
{
    independent_accumulator reference;
    independent_accumulator candidate;
    reference.advance({5, 1U, 100U});
    reference.reset(3U);
    candidate.reset(3U);
    reference.advance({-7, 2U, 200U});
    candidate.advance({-7, 2U, 200U});
    return compare_outputs(reference.output(), candidate.output(), comparison_tolerance<2U>{}).status
        == comparison_status::equal;
}

constexpr bool duplicate_input_is_held()
{
    independent_accumulator state;
    state.advance({7, 1U, 100U});
    const bool is_consumed_again = state.advance({7, 1U, 100U});
    return !is_consumed_again && state.output().key.step == 1U && state.output().value[0] == 7.0;
}

constexpr bool sequence_wrap_uses_time()
{
    independent_accumulator state;
    state.advance({7, 0xFFFFFFFFU, 100U});
    state.advance({7, 0U, 200U});
    return state.output().is_present && state.output().key.step == 2U
        && state.output().key.input_sequence == 0U;
}

constexpr bool bounded_arithmetic()
{
    independent_accumulator state;
    state.advance({2147483647, 1U, 100U});
    state.advance({-2147483647 - 1, 2U, 200U});
    return state.output().value[0] > -1000000.0 && state.output().value[0] < 1000000.0;
}

static_assert(independent_states());
static_assert(aligned_and_biased());
static_assert(skip_does_not_catch_up());
static_assert(common_reset_restores_alignment());
static_assert(duplicate_input_is_held());
static_assert(sequence_wrap_uses_time());
static_assert(bounded_arithmetic());
} // namespace
