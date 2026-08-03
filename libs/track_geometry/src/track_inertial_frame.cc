#include "orvd/track_geometry/track_inertial_frame.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace orvd::track_geometry {

Eigen::Vector3d GravitationalAccelerationInInertial(
    double magnitude_meters_per_second_squared) {
    if (!std::isfinite(magnitude_meters_per_second_squared) ||
        magnitude_meters_per_second_squared <= 0.0) {
        throw std::invalid_argument(
            "GravitationalAccelerationInInertial: the magnitude must be finite "
            "and positive, got " +
            std::to_string(magnitude_meters_per_second_squared) +
            " m/s^2; the direction is fixed by the inertial frame and is not "
            "the caller's to reverse through a negative magnitude");
    }
    return magnitude_meters_per_second_squared * DownwardUnitVectorInInertial();
}

}  // namespace orvd::track_geometry
