#include "orvd/wheel_rail_contact/normal_contact_force.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::NormalContactLaw&>()
                           .Solve(std::declval<const orvd::wheel_rail_contact::NormalContactGeometry&>(),
                                  std::declval<double>())),
              orvd::wheel_rail_contact::NormalContactResult>);
