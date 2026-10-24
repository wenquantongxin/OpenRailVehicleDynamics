#include "orvd/scene_observation/track_heading.h"

#include <cmath>
#include <stdexcept>

namespace orvd::scene_observation {
namespace {
Eigen::Vector3d HorizontalDirection(const Eigen::Vector3d& direction) {
    const double length = direction.stableNorm();
    if (!direction.allFinite() || !std::isfinite(length) || length == 0.0) {
        throw std::invalid_argument("track heading requires a finite nonzero direction");
    }
    const Eigen::Vector3d unit = direction / length;
    if (std::hypot(unit.x(), unit.y()) < 1e-6) {
        throw std::invalid_argument("track heading has no resolved horizontal projection");
    }
    return unit;
}
}  // namespace

double ForwardHeadingRadians(const Eigen::Vector3d& forward_in_track) {
    const auto direction = HorizontalDirection(forward_in_track);
    return std::atan2(direction.y(), direction.x());
}

double AxleHeadingRadians(const Eigen::Vector3d& axle_in_track) {
    auto direction = HorizontalDirection(axle_in_track);
    if (direction.y() < 0.0 ||
        (direction.y() == 0.0 && direction.x() > 0.0)) {
        direction = -direction;
    }
    return std::atan2(-direction.x(), direction.y());
}

}  // namespace orvd::scene_observation
