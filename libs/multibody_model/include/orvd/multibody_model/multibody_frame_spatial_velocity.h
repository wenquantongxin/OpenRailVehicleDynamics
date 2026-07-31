#pragma once

/// @file
/// The angular and translational velocity of a frame.
///
/// A frame spatial velocity has two three-vectors. Which frame is moving,
/// which frame it is measured relative to, which frame its components are
/// expressed in, and which point the translational part belongs to are stated
/// by the query that returns it. Keeping those roles in the query avoids a
/// value type that is truthful for one query and misleading for another.
///
/// The component accessors state their units and the translational point. No
/// unlabelled six-vector is exposed: an ordering convention is not a substitute
/// for saying which three numbers are angular and which are translational.
///
/// Like `RigidPose`, this is produced rather than supplied. The accessors are
/// reference-qualified so reading a component from a returned temporary cannot
/// leave a reference to a member that has already died. Only a value the caller
/// is holding provides zero-copy component access.

#include <utility>

#include <Eigen/Dense>

namespace orvd::multibody_model {

class MultibodyModel;

class FrameSpatialVelocity {
   public:
    /// Angular velocity, in radians per second.
    [[nodiscard]] const Eigen::Vector3d&
    angular_velocity_radians_per_second() const& {
        return angular_velocity_radians_per_second_;
    }
    [[nodiscard]] Eigen::Vector3d angular_velocity_radians_per_second() && {
        return std::move(angular_velocity_radians_per_second_);
    }
    [[nodiscard]] Eigen::Vector3d angular_velocity_radians_per_second() const&& {
        return angular_velocity_radians_per_second_;
    }

    /// Translational velocity of the moving frame's origin, in metres per
    /// second.
    [[nodiscard]] const Eigen::Vector3d&
    translational_velocity_at_frame_origin_meters_per_second() const& {
        return translational_velocity_at_frame_origin_meters_per_second_;
    }
    [[nodiscard]] Eigen::Vector3d
    translational_velocity_at_frame_origin_meters_per_second() && {
        return std::move(
            translational_velocity_at_frame_origin_meters_per_second_);
    }
    [[nodiscard]] Eigen::Vector3d
    translational_velocity_at_frame_origin_meters_per_second() const&& {
        return translational_velocity_at_frame_origin_meters_per_second_;
    }

   private:
    friend class MultibodyModel;

    FrameSpatialVelocity(
        const Eigen::Vector3d& angular_velocity_radians_per_second,
        const Eigen::Vector3d&
            translational_velocity_at_frame_origin_meters_per_second)
        : angular_velocity_radians_per_second_(
              angular_velocity_radians_per_second),
          translational_velocity_at_frame_origin_meters_per_second_(
              translational_velocity_at_frame_origin_meters_per_second) {}

    Eigen::Vector3d angular_velocity_radians_per_second_;
    Eigen::Vector3d
        translational_velocity_at_frame_origin_meters_per_second_;
};

}  // namespace orvd::multibody_model
