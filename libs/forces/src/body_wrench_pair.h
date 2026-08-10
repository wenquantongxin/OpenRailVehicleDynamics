#pragma once

#include <span>

#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model.h"

namespace orvd::forces::internal {

/// Writes equal and opposite pure moments at two body-fixed points.
inline void EmitCoupleWrenchPair(
    const multibody_model::MultibodyModel& model,
    const multibody_model::BodyFixedPoint& positive,
    const multibody_model::BodyFixedPoint& negative,
    const Eigen::Vector3d& moment_on_positive_in_world,
    std::span<multibody_model::AppliedBodyWrench> destination) {
    const multibody_model::FrameHandle world = model.world_frame();
    destination[0] = multibody_model::AppliedBodyWrench{
        positive.body, positive.position_in_body_frame_meters, world,
        moment_on_positive_in_world, Eigen::Vector3d::Zero()};
    destination[1] = multibody_model::AppliedBodyWrench{
        negative.body, negative.position_in_body_frame_meters, world,
        -moment_on_positive_in_world, Eigen::Vector3d::Zero()};
}

}  // namespace orvd::forces::internal
