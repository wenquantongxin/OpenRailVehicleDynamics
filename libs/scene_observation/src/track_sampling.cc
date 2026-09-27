#include "orvd/scene_observation/track_sampling.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace orvd::scene_observation {
namespace {

std::array<double, 3> Triple(const Eigen::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}

}  // namespace

void SampleTrackGeometry(const track_geometry::TrackGeometry& line,
                         std::span<const double> stations_meters,
                         RailDatumPlacement left_rail,
                         RailDatumPlacement right_rail,
                         TrackSampleTable& output) {
    if (stations_meters.empty()) {
        throw std::invalid_argument(
            "track sampling: at least one station is required");
    }
    if (!std::isfinite(left_rail.lateral_meters) ||
        !std::isfinite(left_rail.vertical_meters) ||
        !std::isfinite(right_rail.lateral_meters) ||
        !std::isfinite(right_rail.vertical_meters)) {
        throw std::invalid_argument(
            "track sampling: rail datum placement must be finite");
    }
    const std::size_t count = stations_meters.size();
    output.stations_meters.assign(stations_meters.begin(),
                                  stations_meters.end());
    output.centerline_in_inertial_meters.resize(count);
    output.rotation_inertial_from_track_wxyz.resize(count);
    output.curvature_radians_per_meter.resize(count);
    output.superelevation_meters.resize(count);
    output.left_rail_datum_in_inertial_meters.resize(count);
    output.right_rail_datum_in_inertial_meters.resize(count);
    const Eigen::Vector3d left_in_track(0.0, left_rail.lateral_meters,
                                        left_rail.vertical_meters);
    const Eigen::Vector3d right_in_track(0.0, right_rail.lateral_meters,
                                         right_rail.vertical_meters);
    for (std::size_t index = 0; index < count; ++index) {
        const double station = stations_meters[index];
        const auto kinematics = line.EvaluateTrackFrame(station);
        const Eigen::Vector3d& origin =
            kinematics.pose().origin_in_inertial_meters();
        const Eigen::Matrix3d& rotation =
            kinematics.pose().rotation_inertial_from_track();
        const Eigen::Quaterniond orientation(rotation);
        output.centerline_in_inertial_meters[index] = Triple(origin);
        output.rotation_inertial_from_track_wxyz[index] = {
            orientation.w(), orientation.x(), orientation.y(),
            orientation.z()};
        output.curvature_radians_per_meter[index] =
            line.CurvatureRadiansPerMeter(station);
        output.superelevation_meters[index] =
            line.SuperelevationMeters(station);
        output.left_rail_datum_in_inertial_meters[index] =
            Triple(origin + rotation * left_in_track);
        output.right_rail_datum_in_inertial_meters[index] =
            Triple(origin + rotation * right_in_track);
    }
}

}  // namespace orvd::scene_observation
