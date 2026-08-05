#include "orvd/wheel_rail_contact/monotone_cubic_interpolant.h"

#include <type_traits>
#include <utility>

static_assert(
    std::is_same_v<decltype(orvd::wheel_rail_contact::MonotoneCubicInterpolant::
                                FromValues(std::declval<std::span<const double>>(),
                                           std::declval<std::span<const double>>(),
                                           orvd::wheel_rail_contact::
                                               OutsideDerivativeRule::
                                                   kZeroOutsideKnots)),
                   orvd::wheel_rail_contact::MonotoneCubicInterpolant>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::wheel_rail_contact::
                                        MonotoneCubicInterpolant&&>()
                           .knots()),
              std::vector<double>>);
