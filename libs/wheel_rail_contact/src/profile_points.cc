#include "orvd/wheel_rail_contact/profile_points.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace orvd::wheel_rail_contact {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("profile points: " + detail);
}

}  // namespace

ProfilePoints::ProfilePoints(ProfileRole role, std::string identifier,
                             std::vector<double> lateral_meters,
                             std::vector<double> vertical_meters)
    : role_(role),
      identifier_(std::move(identifier)),
      lateral_meters_(std::move(lateral_meters)),
      vertical_meters_(std::move(vertical_meters)) {}

ProfilePoints ProfilePoints::FromAuthoredOrder(ProfileRole role,
                                               std::string identifier,
                                               std::vector<double> lateral_meters,
                                               std::vector<double> vertical_meters) {
    if (identifier.empty()) {
        Reject("the identifier is empty");
    }
    if (lateral_meters.size() != vertical_meters.size()) {
        Reject("'" + identifier +
               "' states " + std::to_string(lateral_meters.size()) +
               " lateral coordinates and " +
               std::to_string(vertical_meters.size()) +
               " vertical ones; a point needs both");
    }
    if (lateral_meters.size() < 2) {
        Reject("'" + identifier + "' states " +
               std::to_string(lateral_meters.size()) +
               " points; a profile needs at least two");
    }
    for (std::size_t index = 0; index < lateral_meters.size(); ++index) {
        if (!std::isfinite(lateral_meters[index]) ||
            !std::isfinite(vertical_meters[index])) {
            Reject("'" + identifier + "' has a coordinate at index " +
                   std::to_string(index) + " that is not finite");
        }
    }

    // Preserve either authored direction, but reject a list that doubles back.
    // Sorting such a list later would quietly turn it into a different surface.
    if (lateral_meters[1] == lateral_meters[0]) {
        Reject("'" + identifier + "' repeats the lateral coordinate " +
               std::to_string(lateral_meters[0]) +
               " m; every point needs its own");
    }
    const bool increases = lateral_meters[1] > lateral_meters[0];
    for (std::size_t index = 1; index < lateral_meters.size(); ++index) {
        const bool keeps_direction =
            increases ? lateral_meters[index] > lateral_meters[index - 1]
                      : lateral_meters[index] < lateral_meters[index - 1];
        if (!keeps_direction) {
            Reject("'" + identifier +
                   "' does not run strictly in one lateral direction at "
                   "point index " +
                   std::to_string(index));
        }
    }

    return ProfilePoints(role, std::move(identifier), std::move(lateral_meters),
                         std::move(vertical_meters));
}

SideResolvedProfile ProfilePoints::ResolveForSide(WheelSide side) const {
    const std::size_t count = lateral_meters_.size();
    std::vector<double> lateral(count, 0.0);
    std::vector<double> vertical(count, 0.0);
    const double sign = (side == WheelSide::kLeft) ? -1.0 : 1.0;

    std::vector<std::size_t> order(count);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(),
              [this, sign](std::size_t left, std::size_t right) {
                  return sign * lateral_meters_[left] <
                         sign * lateral_meters_[right];
              });
    for (std::size_t index = 0; index < count; ++index) {
        lateral[index] = sign * lateral_meters_[order[index]];
        vertical[index] = vertical_meters_[order[index]];
    }
    return SideResolvedProfile(role_, side, std::move(lateral),
                               std::move(vertical));
}

SideResolvedProfile::SideResolvedProfile(ProfileRole role, WheelSide side,
                                         std::vector<double> lateral_meters,
                                         std::vector<double> vertical_meters)
    : role_(role),
      side_(side),
      lateral_meters_(std::move(lateral_meters)),
      vertical_meters_(std::move(vertical_meters)) {}

}  // namespace orvd::wheel_rail_contact
