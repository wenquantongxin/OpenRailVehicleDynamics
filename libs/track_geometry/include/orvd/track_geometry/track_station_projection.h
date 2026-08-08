#pragma once

#include <stdexcept>
#include <utility>

#include <Eigen/Core>

// The result of projecting a point in space onto the line centerline.
//
// The projection primitive keeps no history. A seeded query searches a stated
// interval around a seed the caller supplies; there is deliberately no
// whole-line cold-start query. Nothing an integrator rejects can therefore leak
// into the next evaluation through this object. A consumer that wants a
// predicted seed computes the prediction itself and passes the predicted
// station in; the line layer does not take a station rate or a step length,
// because it has no way to tell an accepted step from a trial one.

namespace orvd::track_geometry {

class TrackGeometry;

/// A valid local projection window does not contain an admissible minimum.
///
/// This is distinct from an ambiguous window, invalid geometry input or a
/// window outside the line domain. An ODE trial state may temporarily produce
/// this condition even though a smaller trial step remains on the same local
/// branch.
class TrackStationProjectionWindowMiss final : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

class TrackStationProjection {
   public:
    [[nodiscard]] double track_station_meters() const {
        return track_station_meters_;
    }
    [[nodiscard]] const Eigen::Vector3d&
    closest_centerline_point_in_inertial_meters() const & {
        return closest_centerline_point_in_inertial_meters_;
    }
    [[nodiscard]] Eigen::Vector3d
    closest_centerline_point_in_inertial_meters() && {
        return std::move(closest_centerline_point_in_inertial_meters_);
    }
    [[nodiscard]] Eigen::Vector3d
    closest_centerline_point_in_inertial_meters() const && {
        return closest_centerline_point_in_inertial_meters_;
    }

   private:
    friend class TrackGeometry;

    TrackStationProjection(
        double track_station_meters,
        const Eigen::Vector3d& closest_centerline_point_in_inertial_meters)
        : track_station_meters_(track_station_meters),
          closest_centerline_point_in_inertial_meters_(
              closest_centerline_point_in_inertial_meters) {}

    double track_station_meters_;
    Eigen::Vector3d closest_centerline_point_in_inertial_meters_;
};

}  // namespace orvd::track_geometry
