#include "orvd/wheel_rail_contact/profile_points.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::ProfilePoints&>()
                           .ResolveForSide(
                               orvd::wheel_rail_contact::WheelSide::kLeft)),
              orvd::wheel_rail_contact::SideResolvedProfile>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::wheel_rail_contact::ProfilePoints&&>()
                           .authored_lateral_meters()),
              std::vector<double>>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::wheel_rail_contact::SideResolvedProfile&&>()
                           .vertical_meters()),
              std::vector<double>>);
