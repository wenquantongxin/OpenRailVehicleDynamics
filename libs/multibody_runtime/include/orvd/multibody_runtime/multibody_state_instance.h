#pragma once

/// @file
/// The single owner of one evaluation's mutable multibody state.
///
/// Generalized positions, generalized velocities and the context-mutable
/// physical parameters live here and nowhere else. Everything that computes with
/// them reads them from here; nothing keeps a second copy. That is the whole
/// point: two stores of the same quantity drift, and when they do, the
/// difference shows up as a dynamics discrepancy rather than as the
/// synchronisation bug it is.
///
/// Reads hand out const views. Writes take a whole value, validate all of it,
/// and only then store it — so a rejected write leaves the live state exactly as
/// it was, and nobody holds a mutable view that outlives the validation that
/// justified it. There is no editor that lets a caller modify live data and
/// commit later: between the modification and the commit the state would be
/// neither the old value nor the new one, and any read in that window is wrong
/// without any way to tell.
///
/// No time, no discrete state, no event state. The tree does not read them —
/// measured, not assumed — and a field reserved for an absent consumer is a
/// field somebody eventually fills in for the wrong reason.

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_physical_parameters.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"

namespace orvd::multibody_runtime {

class MultibodyStateInstance {
   public:
    /// Builds a zero-initialised state against `layout`.
    ///
    /// The layout must outlive this state, and the state is bound to that exact
    /// layout object: see `MultibodyStateLayout` for why identity rather than
    /// equal contents.
    explicit MultibodyStateInstance(const MultibodyStateLayout& layout);

    /// The layout this state is bound to.
    const MultibodyStateLayout& layout() const { return *layout_; }

    // --- Generalized coordinates -------------------------------------------

    const Eigen::VectorXd& generalized_positions() const {
        return generalized_positions_;
    }
    const Eigen::VectorXd& generalized_velocities() const {
        return generalized_velocities_;
    }

    /// @throws std::invalid_argument if the size does not match the layout, or
    /// if any value is not finite. Nothing is written unless everything passes.
    ///
    /// Which entries are a quaternion, and what should happen when one is not a
    /// unit quaternion, are questions about the model's joints. This layer does
    /// not know the joints, so it does not guess: it neither rejects nor
    /// silently normalises.
    void set_generalized_positions(const Eigen::Ref<const Eigen::VectorXd>& positions);

    /// @throws std::invalid_argument on a size mismatch or a non-finite value.
    void set_generalized_velocities(const Eigen::Ref<const Eigen::VectorXd>& velocities);

    // --- Context-mutable physical parameters --------------------------------

    const RigidBodyInertiaParameters& rigid_body_inertia_parameters(
        int rigid_body_index) const;
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, the mass or a unit-inertia moment is negative, or the
    /// moments violate the triangle inequality. Zero mass and zero inertia pass:
    /// a massless frame carrier is a normal modelling device.
    void set_rigid_body_inertia_parameters(
        int rigid_body_index, const RigidBodyInertiaParameters& parameters);

    const FixedFramePoseParameters& fixed_frame_pose_parameters(
        int fixed_frame_index) const;
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or the rotation is not a rotation. The matrix is validated,
    /// never orthonormalised: repairing it here would turn a wrong model into a
    /// plausible answer.
    void set_fixed_frame_pose_parameters(
        int fixed_frame_index, const FixedFramePoseParameters& parameters);

    /// The damping coefficients of one joint, as a view into the flat store.
    Eigen::Ref<const Eigen::VectorXd> joint_damping(int joint_index) const;
    /// @throws std::invalid_argument if the index is out of range, the length
    /// does not match that joint's velocity count, or a coefficient is not
    /// finite or is negative.
    void set_joint_damping(int joint_index,
                           const Eigen::Ref<const Eigen::VectorXd>& damping);

    const JointActuatorParameters& joint_actuator_parameters(
        int joint_actuator_index) const;
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or the rotor inertia is negative. The gear ratio's sign is
    /// not constrained.
    void set_joint_actuator_parameters(int joint_actuator_index,
                                       const JointActuatorParameters& parameters);

    const RevoluteSpringParameters& revolute_spring_parameters(
        int revolute_spring_index) const;
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or the stiffness is negative.
    void set_revolute_spring_parameters(
        int revolute_spring_index, const RevoluteSpringParameters& parameters);

    const LinearBushingRollPitchYawParameters& linear_bushing_parameters(
        int linear_bushing_index) const;
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or any stiffness or damping is negative.
    void set_linear_bushing_parameters(
        int linear_bushing_index,
        const LinearBushingRollPitchYawParameters& parameters);

   private:
    const MultibodyStateLayout* layout_;

    Eigen::VectorXd generalized_positions_;
    Eigen::VectorXd generalized_velocities_;

    std::vector<RigidBodyInertiaParameters> rigid_body_inertia_parameters_;
    std::vector<FixedFramePoseParameters> fixed_frame_pose_parameters_;
    /// All joints' damping in one vector, divided by the layout's offsets.
    Eigen::VectorXd joint_damping_;
    std::vector<JointActuatorParameters> joint_actuator_parameters_;
    std::vector<RevoluteSpringParameters> revolute_spring_parameters_;
    std::vector<LinearBushingRollPitchYawParameters> linear_bushing_parameters_;
};

}  // namespace orvd::multibody_runtime
