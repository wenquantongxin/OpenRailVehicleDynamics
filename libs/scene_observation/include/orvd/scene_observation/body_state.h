#pragma once

/// @file
/// The world-frame state of every rigid body, sampled from a finalized model.

#include <array>
#include <string>
#include <vector>

#include "orvd/multibody_model/multibody_model.h"

namespace orvd::scene_observation {

/// One rigid body's state at one sample.
///
/// Every quantity refers to the body's declared frame origin and is expressed
/// in the model's world frame. The quaternion order is w,x,y,z and the
/// rotation maps body coordinates to world coordinates. A wheel body's
/// orientation therefore already contains its spin; a consumer must not add a
/// second rotation from a joint angle or from speed over radius.
struct BodyState {
    std::array<double, 3> position_meters{};
    std::array<double, 4> orientation_wxyz{};
    std::array<double, 3> linear_velocity_meters_per_second{};
    std::array<double, 3> angular_velocity_radians_per_second{};
};

/// Resolves every rigid body's name and handle once, in model order.
///
/// The model must outlive the sampler. Sampling reads kinematics through the
/// public model queries only. The caller owns the accepted-state boundary: the
/// pose queries fill the context's own caches, so a context that another
/// thread is evaluating must not be handed to `Sample()`.
class BodyStateSampler {
   public:
    explicit BodyStateSampler(const multibody_model::MultibodyModel& model);

    /// Body names in slot order; `Sample()` returns states in the same order.
    [[nodiscard]] const std::vector<std::string>& names() const {
        return names_;
    }

    [[nodiscard]] std::vector<BodyState> Sample(
        const multibody_model::MultibodyEvaluationContext& context) const;

   private:
    const multibody_model::MultibodyModel* model_;
    std::vector<multibody_model::RigidBodyHandle> bodies_;
    std::vector<std::string> names_;
};

}  // namespace orvd::scene_observation
