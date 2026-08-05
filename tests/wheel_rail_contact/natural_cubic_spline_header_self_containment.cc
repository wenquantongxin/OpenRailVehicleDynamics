#include "orvd/wheel_rail_contact/natural_cubic_spline.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::NaturalCubicSpline&>()
                           .Evaluate(0.0)),
              double>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::wheel_rail_contact::NaturalCubicSpline&&>()
                           .nodal_slopes()),
              std::vector<double>>);
