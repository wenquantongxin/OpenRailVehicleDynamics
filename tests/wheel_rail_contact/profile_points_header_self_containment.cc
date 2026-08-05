#include "orvd/wheel_rail_contact/profile_points.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::ProfilePoints&>()
                           .ResolveForSide(
                               orvd::wheel_rail_contact::WheelSide::kLeft)),
              orvd::wheel_rail_contact::SideResolvedProfile>);
