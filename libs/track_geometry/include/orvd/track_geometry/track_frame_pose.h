#pragma once

#include <utility>

#include <Eigen/Core>

// The pose of the track frame in the track inertial frame, and its first
// derivative with respect to track station.
//
// These fixed-size values are produced by TrackGeometry and returned by value.

namespace orvd::track_geometry {

class TrackGeometry;

class TrackFramePose {
   public:
    [[nodiscard]] const Eigen::Matrix3d& rotation_inertial_from_track()
        const & {
        return rotation_inertial_from_track_;
    }
    [[nodiscard]] Eigen::Matrix3d rotation_inertial_from_track() && {
        return std::move(rotation_inertial_from_track_);
    }
    [[nodiscard]] Eigen::Matrix3d rotation_inertial_from_track() const && {
        return rotation_inertial_from_track_;
    }

    [[nodiscard]] const Eigen::Vector3d& origin_in_inertial_meters() const & {
        return origin_in_inertial_meters_;
    }
    [[nodiscard]] Eigen::Vector3d origin_in_inertial_meters() && {
        return std::move(origin_in_inertial_meters_);
    }
    [[nodiscard]] Eigen::Vector3d origin_in_inertial_meters() const && {
        return origin_in_inertial_meters_;
    }

   private:
    friend class TrackGeometry;

    TrackFramePose(const Eigen::Matrix3d& rotation_inertial_from_track,
                   const Eigen::Vector3d& origin_in_inertial_meters)
        : rotation_inertial_from_track_(rotation_inertial_from_track),
          origin_in_inertial_meters_(origin_in_inertial_meters) {}

    Eigen::Matrix3d rotation_inertial_from_track_;
    Eigen::Vector3d origin_in_inertial_meters_;
};

class TrackFrameKinematics {
   public:
    [[nodiscard]] const TrackFramePose& pose() const & { return pose_; }
    [[nodiscard]] TrackFramePose pose() && { return std::move(pose_); }
    [[nodiscard]] TrackFramePose pose() const && { return pose_; }

    // d(centerline position)/d(track station), expressed in the inertial
    // frame. Its norm is sqrt(1 + grade^2), not one: the track station is
    // planar projected mileage, so only the horizontal projection of this
    // vector is a unit vector.
    [[nodiscard]] const Eigen::Vector3d&
    centerline_derivative_in_inertial_meters_per_meter() const & {
        return centerline_derivative_in_inertial_meters_per_meter_;
    }
    [[nodiscard]] Eigen::Vector3d
    centerline_derivative_in_inertial_meters_per_meter() && {
        return std::move(
            centerline_derivative_in_inertial_meters_per_meter_);
    }
    [[nodiscard]] Eigen::Vector3d
    centerline_derivative_in_inertial_meters_per_meter() const && {
        return centerline_derivative_in_inertial_meters_per_meter_;
    }

    // The rotation rate of the track frame per unit track station, expressed in
    // the inertial frame. It satisfies
    //
    //   d(R_inertial_from_track)/d(track station)
    //       = skew(rotation rate) * R_inertial_from_track,
    //
    // which is the identity the gates check rather than a comment that hopes to
    // be true.
    [[nodiscard]] const Eigen::Vector3d&
    track_frame_rotation_rate_in_inertial_radians_per_meter() const & {
        return track_frame_rotation_rate_in_inertial_radians_per_meter_;
    }
    [[nodiscard]] Eigen::Vector3d
    track_frame_rotation_rate_in_inertial_radians_per_meter() && {
        return std::move(
            track_frame_rotation_rate_in_inertial_radians_per_meter_);
    }
    [[nodiscard]] Eigen::Vector3d
    track_frame_rotation_rate_in_inertial_radians_per_meter() const && {
        return track_frame_rotation_rate_in_inertial_radians_per_meter_;
    }

   private:
    friend class TrackGeometry;

    TrackFrameKinematics(
        TrackFramePose pose,
        const Eigen::Vector3d& centerline_derivative_in_inertial_meters_per_meter,
        const Eigen::Vector3d&
            track_frame_rotation_rate_in_inertial_radians_per_meter)
        : pose_(pose),
          centerline_derivative_in_inertial_meters_per_meter_(
              centerline_derivative_in_inertial_meters_per_meter),
          track_frame_rotation_rate_in_inertial_radians_per_meter_(
              track_frame_rotation_rate_in_inertial_radians_per_meter) {}

    TrackFramePose pose_;
    Eigen::Vector3d centerline_derivative_in_inertial_meters_per_meter_;
    Eigen::Vector3d track_frame_rotation_rate_in_inertial_radians_per_meter_;
};

}  // namespace orvd::track_geometry
