#include <type_traits>

#include "orvd/integrators/system_continuous_state_advancer.h"

using orvd::integrators::SystemContinuousStateAdvancer;

static_assert(!std::is_copy_constructible_v<SystemContinuousStateAdvancer>);
static_assert(!std::is_copy_assignable_v<SystemContinuousStateAdvancer>);
static_assert(!std::is_move_constructible_v<SystemContinuousStateAdvancer>);
static_assert(!std::is_move_assignable_v<SystemContinuousStateAdvancer>);
