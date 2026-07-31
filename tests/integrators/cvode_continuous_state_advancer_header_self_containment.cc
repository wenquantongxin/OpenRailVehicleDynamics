#include "orvd/integrators/cvode_continuous_state_advancer.h"

#include <type_traits>

using orvd::integrators::ContinuousStateAdvancer;
using orvd::integrators::CvodeContinuousStateAdvancer;

static_assert(std::is_base_of_v<ContinuousStateAdvancer,
                                CvodeContinuousStateAdvancer>);
static_assert(!std::is_abstract_v<CvodeContinuousStateAdvancer>);
static_assert(!std::is_copy_constructible_v<CvodeContinuousStateAdvancer>);
static_assert(!std::is_copy_assignable_v<CvodeContinuousStateAdvancer>);
static_assert(!std::is_move_constructible_v<CvodeContinuousStateAdvancer>);
static_assert(!std::is_move_assignable_v<CvodeContinuousStateAdvancer>);
