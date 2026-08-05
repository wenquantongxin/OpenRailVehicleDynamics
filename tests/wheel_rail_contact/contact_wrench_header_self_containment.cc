#include "orvd/wheel_rail_contact/contact_wrench.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::TransportWrench(
                  std::declval<const orvd::wheel_rail_contact::SpatialWrench&>(),
                  std::declval<const Eigen::Vector3d&>(),
                  std::declval<const Eigen::Vector3d&>())),
              orvd::wheel_rail_contact::SpatialWrench>);
