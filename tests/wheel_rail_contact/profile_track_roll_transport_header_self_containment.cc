#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::ResolveRollYawPitch(
                  std::declval<const Eigen::Matrix3d&>())),
              orvd::wheel_rail_contact::RollYawPitchAngles>);
