#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::BuildContactPoseScalars(
                  std::declval<const orvd::wheel_rail_contact::WheelRailPoseConstants&>(),
                  std::declval<const orvd::wheel_rail_contact::WheelRailPoseInput&>())),
              orvd::wheel_rail_contact::ContactPoseScalars>);
