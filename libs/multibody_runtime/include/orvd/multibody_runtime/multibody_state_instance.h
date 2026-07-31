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
///
/// Five of the stored quantities carry a version: the generalized positions, the
/// generalized velocities, and the three parameter categories that the runtime
/// contract shows a long-lived cache actually reads. The other parameters —
/// joint damping, revolute springs, linear bushings — carry none, because no
/// cache consumes them; the force computations read them afresh every call. A
/// version with no consumer is not free symmetry: it is a number that later gets
/// compared against, and the comparison means nothing.
///
/// The categories version as wholes rather than per element, because the caches
/// they feed are whole-model caches. Nothing can invalidate the pose of one
/// frame without invalidating the pool, so a per-element version would draw a
/// distinction no consumer could act on.

#include <Eigen/Dense>

#include <vector>

#include "orvd/multibody_runtime/multibody_physical_parameters.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"
#include "orvd/multibody_runtime/multibody_state_version.h"

namespace orvd::multibody_runtime {

class MultibodyStateInstance {
   public:
    /// Builds a zero-initialised state against `layout`.
    ///
    /// `layout` must be a stable lvalue and must outlive this state. The state
    /// is bound to that exact layout object: see `MultibodyStateLayout` for why
    /// identity rather than equal contents.
    ///
    /// Zero initialization establishes storage, not a model-valid default
    /// configuration. This layer does not know mobilizer position segments; the
    /// model-aware adapter must install the finalized model's default positions
    /// before the state can be evaluated.
    explicit MultibodyStateInstance(const MultibodyStateLayout& layout);
    MultibodyStateInstance(MultibodyStateLayout&&) = delete;
    MultibodyStateInstance(const MultibodyStateLayout&&) = delete;

    // Assignment could silently rebind an instance to a different model and
    // bypass versioned setters. A default move would leave its source bound to
    // the old layout but without layout-sized storage, while a copy has no
    // explicit snapshot-version contract. Introduce those operations by name
    // if a real consumer needs them; do not inherit member-wise semantics.
    MultibodyStateInstance(const MultibodyStateInstance&) = delete;
    MultibodyStateInstance& operator=(const MultibodyStateInstance&) = delete;
    MultibodyStateInstance(MultibodyStateInstance&&) = delete;
    MultibodyStateInstance& operator=(MultibodyStateInstance&&) = delete;

    /// Reports whether this state is bound to this exact layout object.
    ///
    /// The boolean form preserves the identity check needed by the rigid-tree
    /// adapter without handing callers the model-owned layout. Exposing that
    /// layout would let ordinary code build a zero-filled state that passes the
    /// tree's identity gate while bypassing installation of the model defaults.
    bool is_bound_to(const MultibodyStateLayout& layout) const {
        return layout_ == &layout;
    }

    // --- Generalized coordinates -------------------------------------------

    const Eigen::VectorXd& generalized_positions() const {
        return generalized_positions_;
    }
    const Eigen::VectorXd& generalized_velocities() const {
        return generalized_velocities_;
    }

    MultibodyStateVersion generalized_positions_version() const {
        return generalized_positions_version_;
    }
    MultibodyStateVersion generalized_velocities_version() const {
        return generalized_velocities_version_;
    }

    /// @throws std::invalid_argument if the size does not match the layout, or
    /// if any value is not finite. Nothing is written unless everything passes.
    /// @throws std::overflow_error if the version counter is exhausted, before
    /// anything is written.
    ///
    /// Which entries are quaternion segments is a property of the finalized
    /// model. This raw store does not guess those segments and does not silently
    /// normalize them; the model-aware adapter validates their non-zero
    /// precondition before evaluation.
    void set_generalized_positions(const Eigen::Ref<const Eigen::VectorXd>& positions);

    /// @throws std::invalid_argument on a size mismatch or a non-finite value.
    /// @throws std::overflow_error if the version counter is exhausted, before
    /// anything is written.
    void set_generalized_velocities(const Eigen::Ref<const Eigen::VectorXd>& velocities);

    // --- Context-mutable physical parameters --------------------------------

    const RigidBodyInertiaParameters& rigid_body_inertia_parameters(
        int rigid_body_index) const;
    /// One version for the whole category: the caches that read mass properties
    /// are whole-model caches.
    MultibodyStateVersion rigid_body_inertias_version() const {
        return rigid_body_inertias_version_;
    }
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, the mass is negative, or the central principal moments are
    /// negative or violate the triangle inequality. Zero mass and zero inertia
    /// pass: a massless frame carrier is a normal modelling device.
    /// @throws std::overflow_error if the version counter is exhausted, before
    /// anything is written.
    void set_rigid_body_inertia_parameters(
        int rigid_body_index, const RigidBodyInertiaParameters& parameters);

    const FixedFramePoseParameters& fixed_frame_pose_parameters(
        int fixed_frame_index) const;
    /// One version for the whole pool, for the same reason.
    MultibodyStateVersion fixed_frame_poses_version() const {
        return fixed_frame_poses_version_;
    }
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or the rotation is not a rotation. The matrix is validated,
    /// never orthonormalised: repairing it here would turn a wrong model into a
    /// plausible answer.
    /// @throws std::overflow_error if the version counter is exhausted, before
    /// anything is written.
    void set_fixed_frame_pose_parameters(
        int fixed_frame_index, const FixedFramePoseParameters& parameters);

    /// The damping coefficients of one joint, as a view into the flat store.
    ///
    /// Unversioned: no cache holds a damping-derived quantity. The damping force
    /// is computed from these coefficients on the call that needs it, so there is
    /// nothing for a version to invalidate.
    Eigen::Ref<const Eigen::VectorXd> joint_damping(int joint_index) const;
    /// @throws std::invalid_argument if the index is out of range, the length
    /// does not match that joint's velocity count, or a coefficient is not
    /// finite or is negative.
    void set_joint_damping(int joint_index,
                           const Eigen::Ref<const Eigen::VectorXd>& damping);

    const JointActuatorParameters& joint_actuator_parameters(
        int joint_actuator_index) const;
    /// Versioned: reflected inertia reaches the articulated-body inertia cache,
    /// which is built once per configuration and reused.
    MultibodyStateVersion joint_actuator_parameters_version() const {
        return joint_actuator_parameters_version_;
    }
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or the rotor inertia is negative. The gear ratio's sign is
    /// not constrained.
    /// @throws std::overflow_error if the version counter is exhausted, before
    /// anything is written.
    void set_joint_actuator_parameters(int joint_actuator_index,
                                       const JointActuatorParameters& parameters);

    /// Unversioned, as for damping: the force element reads these when it builds
    /// its contribution, and no cache retains the result.
    const RevoluteSpringParameters& revolute_spring_parameters(
        int revolute_spring_index) const;
    /// @throws std::invalid_argument if the index is out of range, a value is
    /// not finite, or the stiffness is negative.
    void set_revolute_spring_parameters(
        int revolute_spring_index, const RevoluteSpringParameters& parameters);

    /// Unversioned, for the same reason.
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

    // Exactly five, one per source that a long-lived cache reads. Adding a sixth
    // is a claim that some cache depends on that category; make the claim in the
    // runtime contract first, or the number will be compared against by nobody.
    MultibodyStateVersion generalized_positions_version_;
    MultibodyStateVersion generalized_velocities_version_;
    MultibodyStateVersion fixed_frame_poses_version_;
    MultibodyStateVersion rigid_body_inertias_version_;
    MultibodyStateVersion joint_actuator_parameters_version_;
};

}  // namespace orvd::multibody_runtime
