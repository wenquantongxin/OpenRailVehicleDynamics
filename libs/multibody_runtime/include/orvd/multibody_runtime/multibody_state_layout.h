#pragma once

/// @file
/// How much state and how many parameters a finalized multibody model has.
///
/// The layout is settled once, when the model is finalized, and never changes
/// after: indices handed out here stay valid for the life of the model, and any
/// number of evaluation states can be built against the same layout.
///
/// This layer does not read the finalized model itself. The model is the
/// vendored rigid tree, and a state store that included it would invert the
/// dependency this runtime exists to establish. Instead the adapter that owns
/// the tree fills in a description, and the layout is what survives validating
/// it. Tests can fill in a description directly, which is also why they do not
/// need a model.

#include <vector>

namespace orvd::multibody_runtime {

/// The counts a caller supplies to describe a finalized model's state and
/// parameters. Construction input only; `MultibodyStateLayout` is what results.
///
/// Deliberately not a list of `(category, size)` pairs or a bag of named values:
/// each family is a distinct field because each one means something different,
/// and a generic container would push that meaning into a string or an enum that
/// the compiler cannot check.
struct MultibodyStateLayoutDescription {
    /// Number of generalized positions in the model.
    int generalized_position_count{0};

    /// Number of generalized velocities in the model. Not generally equal to
    /// the position count: a quaternion floating joint carries seven positions
    /// and six velocities.
    int generalized_velocity_count{0};

    /// Number of rigid bodies whose mass properties are context parameters.
    int rigid_body_count{0};

    /// Number of frames whose pose in their parent is a context parameter.
    int fixed_frame_count{0};

    /// One entry per joint that carries damping, giving that joint's number of
    /// velocities. Lengths vary between joints, so the damping coefficients live
    /// in one flat store and this is what divides it.
    std::vector<int> joint_velocity_counts{};

    /// Number of joint actuators.
    int joint_actuator_count{0};

    /// Number of revolute springs.
    int revolute_spring_count{0};

    /// Number of roll-pitch-yaw bushings.
    int linear_bushing_count{0};
};

/// A validated, immutable model layout.
///
/// A `MultibodyStateInstance` binds to one of these by identity, not by value:
/// two layouts that happen to hold the same numbers still describe different
/// models, and letting state built for one be used with the other would turn a
/// modelling error into a silent index mix-up. There is no generation counter or
/// identifier for this — the object's own address is the identity, because the
/// finalized model owns exactly one.
class MultibodyStateLayout {
   public:
    /// Validates the description and forms the layout.
    ///
    /// @throws std::invalid_argument if any count is negative. Counts are the
    /// only thing this layer can check: whether they match a real model is the
    /// adapter's business, and whether the model is sound is the finalization's.
    explicit MultibodyStateLayout(MultibodyStateLayoutDescription description);

    MultibodyStateLayout(const MultibodyStateLayout&) = delete;
    MultibodyStateLayout& operator=(const MultibodyStateLayout&) = delete;
    MultibodyStateLayout(MultibodyStateLayout&&) = delete;
    MultibodyStateLayout& operator=(MultibodyStateLayout&&) = delete;

    int generalized_position_count() const {
        return description_.generalized_position_count;
    }
    int generalized_velocity_count() const {
        return description_.generalized_velocity_count;
    }
    int rigid_body_count() const { return description_.rigid_body_count; }
    int fixed_frame_count() const { return description_.fixed_frame_count; }
    int damped_joint_count() const {
        return static_cast<int>(description_.joint_velocity_counts.size());
    }
    int joint_actuator_count() const {
        return description_.joint_actuator_count;
    }
    int revolute_spring_count() const {
        return description_.revolute_spring_count;
    }
    int linear_bushing_count() const {
        return description_.linear_bushing_count;
    }

    /// Number of velocities of the damped joint at `joint_index`.
    int joint_velocity_count(int joint_index) const;

    /// Where that joint's damping coefficients start in the flat damping store.
    int joint_damping_start(int joint_index) const;

    /// Total length of the flat damping store.
    int total_joint_damping_count() const { return total_joint_damping_count_; }

   private:
    MultibodyStateLayoutDescription description_;
    /// Start offset per damped joint, precomputed so that a hot-path accessor
    /// does not add up a prefix on every call.
    std::vector<int> joint_damping_starts_;
    int total_joint_damping_count_{0};
};

}  // namespace orvd::multibody_runtime
