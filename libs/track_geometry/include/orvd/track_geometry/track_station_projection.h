#pragma once

#include <stdexcept>
#include <utility>

#include <Eigen/Core>

// The result of projecting a point in space onto the line centerline.
//
// The projection primitive keeps no history. A seeded query makes at most two
// Newton corrections from the branch-identifying station supplied by its
// caller; there is deliberately no whole-line cold-start query. Nothing an
// integrator rejects can therefore leak into the next evaluation through this
// object.

namespace orvd::track_geometry {

class TrackGeometry;

/// The supplied station does not identify a regular local projection within
/// the admitted two-Newton-correction contract.
class TrackStationLocalProjectionFailure final : public std::runtime_error {
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
