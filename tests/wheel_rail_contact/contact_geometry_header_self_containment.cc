#include "orvd/wheel_rail_contact/contact_geometry.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::ContactGeometrySolver&>()
                           .Solve(std::declval<const orvd::wheel_rail_contact::ContactPoseScalars&>(),
                                  std::declval<orvd::wheel_rail_contact::ContactGeometryWorkspace&>())),
              orvd::wheel_rail_contact::ContactPatchSet>);
