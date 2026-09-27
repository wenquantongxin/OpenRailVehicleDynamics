#pragma once

/// @file
/// The unwrapped spin angle of every wheel that turns on a revolute joint.
///
/// A sampled wheel orientation already contains the spin, but two samples
/// more than half a turn apart cannot say which way the wheel went. The joint
/// position is continuous in time, so its difference between two samples is
/// the true rotation including whole turns. It is never added to the sampled
/// orientation; a display uses it only to choose the rotation branch between
/// two samples.

#include <cstddef>
#include <span>
#include <vector>

#include "orvd/multibody_model/multibody_coordinate_ranges.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/scene_observation/scene_topology.h"

namespace orvd::scene_observation {

class WheelSpinAngleSampler {
   public:
    /// Resolves each wheel's spin joint once and probes, through the public
    /// kinematics, which way a positive joint position turns the wheel about
    /// its placement's spin axis. A wheel without a spin joint makes the
    /// sampler unavailable; a joint whose axis is not the wheel's spin axis is
    /// refused.
    WheelSpinAngleSampler(const multibody_model::MultibodyModel& model,
                          const SceneTopology& topology);

    /// True when every wheel placement names a spin joint.
    [[nodiscard]] bool available() const noexcept { return available_; }
    [[nodiscard]] std::size_t wheel_count() const noexcept {
        return wheels_.size();
    }

    /// Writes one angle per wheel placement, positive about
    /// `spin_axis_in_wheel_body_frame`. Requires `available()`.
    void Sample(const multibody_model::MultibodyEvaluationContext& context,
                std::span<double> angles_radians) const;

   private:
    struct WheelBinding {
        int position_index{};
        double sign{1.0};
    };

    const multibody_model::MultibodyModel* model_;
    bool available_{false};
    std::vector<WheelBinding> wheels_;
};

}  // namespace orvd::scene_observation
