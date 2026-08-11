#include "orvd/control/sampled_filtered_pi.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::control::SampledFilteredPi&>()
                           .Step(
                               std::declval<double>(),
                               std::declval<double>(),
                               std::declval<const orvd::control::
                                                SampledFilteredPiState&>())),
              orvd::control::SampledFilteredPiResult>);
