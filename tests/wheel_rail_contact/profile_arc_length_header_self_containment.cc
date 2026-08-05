#include "orvd/wheel_rail_contact/profile_arc_length.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::IntegrateProfileArcLength(
                  std::declval<const orvd::wheel_rail_contact::NaturalCubicSpline&>(),
                  0.0, 1.0)),
              double>);
