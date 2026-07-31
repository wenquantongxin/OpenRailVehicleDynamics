#pragma once

/// @file
/// The first-party finalize-time assignment of typed parameter slots.
///
/// Upstream asked a `System` for a numeric parameter of a given width and got
/// back an index into one undifferentiated pool. A rigid body's mass properties
/// were ten doubles at some index; a bushing's stiffnesses were four separate
/// three-vectors at four separate indices; whether two indices referred to
/// comparable things was not a question the type could answer.
///
/// Here each category has its own named allocation path and counter, and an
/// element receives an ordinal within its category — the third rigid body's
/// inertia, the first bushing's parameters. The stored ordinal remains an
/// integer because it indexes compact vectors; category safety comes from the
/// allocator and element APIs, not from pretending those integers are distinct
/// C++ types.
///
/// Two of upstream's splits are closed while doing this. An actuator's rotor
/// inertia and gear ratio were two one-wide parameters; they are one
/// `JointActuatorParameters` slot, because reflected inertia needs both and
/// nothing needs either alone. A bushing's four three-vectors are one
/// `LinearBushingRollPitchYawParameters` slot for the same reason.
///
/// The allocator runs once, during a deterministic traversal of the finalized
/// model, and produces the layout description. It does not discover elements:
/// the traversal visits them and each one asks for what it needs, so the
/// allocator never has to know what a rigid body is.

#include <stdexcept>
#include <string>
#include <vector>

#include "orvd/multibody_runtime/multibody_state_layout.h"

namespace orvd::rigid_multibody_tree::internal {

class MultibodyParameterSlotAllocator {
   public:
    MultibodyParameterSlotAllocator(int generalized_position_count,
                                    int generalized_velocity_count)
        : generalized_position_count_(generalized_position_count),
          generalized_velocity_count_(generalized_velocity_count) {
        if (generalized_position_count < 0 || generalized_velocity_count < 0) {
            throw std::invalid_argument(
                "multibody parameter slots: the generalized coordinate counts "
                "cannot be negative");
        }
    }

    /// Ordinals within their own category, in visit order.
    int AllocateRigidBodyInertiaSlot() { return rigid_body_count_++; }
    int AllocateFixedFramePoseSlot() { return fixed_frame_count_++; }
    int AllocateJointActuatorSlot() { return joint_actuator_count_++; }
    int AllocateRevoluteSpringSlot() { return revolute_spring_count_++; }
    int AllocateLinearBushingSlot() { return linear_bushing_count_++; }

    /// A joint's damping slot, which also fixes that joint's width in the flat
    /// damping store. The width is recorded here rather than looked up later so
    /// that the store's segmentation comes from the same traversal that assigned
    /// the ordinals, and cannot drift from it.
    int AllocateJointDampingSlot(int velocity_count) {
        if (velocity_count < 0) {
            throw std::invalid_argument(
                "multibody parameter slots: a joint cannot have a negative "
                "velocity count");
        }
        joint_velocity_counts_.push_back(velocity_count);
        return static_cast<int>(joint_velocity_counts_.size()) - 1;
    }

    /// What was allocated, as the runtime layer's description.
    multibody_runtime::MultibodyStateLayoutDescription description() const {
        multibody_runtime::MultibodyStateLayoutDescription description;
        description.generalized_position_count = generalized_position_count_;
        description.generalized_velocity_count = generalized_velocity_count_;
        description.rigid_body_count = rigid_body_count_;
        description.fixed_frame_count = fixed_frame_count_;
        description.joint_velocity_counts = joint_velocity_counts_;
        description.joint_actuator_count = joint_actuator_count_;
        description.revolute_spring_count = revolute_spring_count_;
        description.linear_bushing_count = linear_bushing_count_;
        return description;
    }

   private:
    int generalized_position_count_{0};
    int generalized_velocity_count_{0};
    int rigid_body_count_{0};
    int fixed_frame_count_{0};
    int joint_actuator_count_{0};
    int revolute_spring_count_{0};
    int linear_bushing_count_{0};
    std::vector<int> joint_velocity_counts_;
};

/// An unassigned slot.
///
/// A named constant rather than a bare -1 so that "this element has not been
/// finalized" reads as itself at the point where it is checked, and so that the
/// check cannot be confused with an ordinary bounds test.
inline constexpr int kUnassignedParameterSlot = -1;

}  // namespace orvd::rigid_multibody_tree::internal
