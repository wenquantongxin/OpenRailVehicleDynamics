#include "orvd/wheel_rail_contact/rail_gauge_datum.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace orvd::wheel_rail_contact {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("rail gauge datum: " + detail);
}

}  // namespace

RailGaugeDatum ComputeRailGaugeDatum(const ProfilePoints& rail_profile,
                                     double track_gauge_meters,
                                     double gauge_measuring_depth_meters,
                                     double rail_cant_radians, WheelSide side) {
    if (rail_profile.role() != ProfileRole::kRail) {
        Reject("'" + rail_profile.identifier() +
               "' is a wheel profile; a gauge datum is a property of a rail");
    }
    if (!std::isfinite(track_gauge_meters) || !(track_gauge_meters > 0.0)) {
        Reject("the track gauge is " + std::to_string(track_gauge_meters) +
               " m; it must be finite and positive");
    }
    if (!std::isfinite(gauge_measuring_depth_meters) ||
        !(gauge_measuring_depth_meters > 0.0)) {
        Reject("the gauge measuring depth is " +
               std::to_string(gauge_measuring_depth_meters) +
               " m; it must be finite and positive, since the gauge face lies "
               "below the crown");
    }
    if (!std::isfinite(rail_cant_radians)) {
        Reject("the rail cant is not finite");
    }

    // The right rail leans toward the track centre: with the lateral axis
    // pointing right and the vertical axis pointing down, that is a negative
    // roll about the forward axis. The left rail is its mirror.
    const double roll = (side == WheelSide::kRight) ? -rail_cant_radians
                                                    : rail_cant_radians;

    const std::span<const double> lateral = rail_profile.authored_lateral_meters();
    const std::span<const double> vertical = rail_profile.authored_vertical_meters();
    const std::size_t count = lateral.size();
    const double cosine = std::cos(roll);
    const double sine = std::sin(roll);

    std::vector<double> rolled_lateral(count, 0.0);
    std::vector<double> rolled_vertical(count, 0.0);
    for (std::size_t index = 0; index < count; ++index) {
        rolled_lateral[index] = cosine * lateral[index] - sine * vertical[index];
        rolled_vertical[index] = sine * lateral[index] + cosine * vertical[index];
    }

    std::vector<std::size_t> order(count);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(),
              [&rolled_lateral](std::size_t left, std::size_t right) {
                  return rolled_lateral[left] < rolled_lateral[right];
              });

    // The crown is the least vertical coordinate, because the vertical axis
    // points down.
    const double crown =
        *std::min_element(rolled_vertical.begin(), rolled_vertical.end());
    const double measuring_level = crown + gauge_measuring_depth_meters;

    std::vector<double> crossings;
    for (std::size_t index = 0; index + 1 < count; ++index) {
        const std::size_t here = order[index];
        const std::size_t next = order[index + 1];
        const double below = rolled_vertical[here] - measuring_level;
        const double above = rolled_vertical[next] - measuring_level;
        if (below * above > 0.0) {
            continue;
        }
        const double left = rolled_lateral[here];
        const double right = rolled_lateral[next];
        // A segment lying exactly on the measuring level contributes its
        // midpoint; a segment crossing it contributes the crossing.
        crossings.push_back(above == below
                                ? 0.5 * (left + right)
                                : left + (0.0 - below) * (right - left) /
                                              (above - below));
    }
    if (crossings.empty()) {
        Reject("'" + rail_profile.identifier() +
               "' never reaches " + std::to_string(gauge_measuring_depth_meters) +
               " m below its crown, so it has no gauge face");
    }

    // The gauge face is the one facing the track centre.
    const double gauge_face =
        (side == WheelSide::kRight)
            ? *std::min_element(crossings.begin(), crossings.end())
            : *std::max_element(crossings.begin(), crossings.end());
    const double offset =
        (side == WheelSide::kRight) ? -gauge_face : gauge_face;

    RailGaugeDatum datum;
    datum.gauge_face_offset_meters = offset;
    datum.lateral_datum_meters = (side == WheelSide::kRight)
                                     ? (0.5 * track_gauge_meters + offset)
                                     : -(0.5 * track_gauge_meters + offset);
    datum.roll_radians = roll;
    return datum;
}

}  // namespace orvd::wheel_rail_contact
