#include "orvd/integrators/continuous_state_advancer.h"

#include <type_traits>

static_assert(
    std::is_abstract_v<orvd::integrators::ContinuousStateAdvancer>);
