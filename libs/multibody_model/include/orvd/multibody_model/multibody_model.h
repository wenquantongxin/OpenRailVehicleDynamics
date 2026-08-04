#pragma once

/// @file
/// Building a multibody model, programmatically.
///
/// This is the layer a caller writes against. Nothing below it appears here: no
/// vendored type, no upstream index, no mobilizer. Those describe how the rigid
/// tree arranges what it is given, and a caller who had to know them would be
/// writing against the arrangement rather than against the model.
///
/// What the model accepts is the set of things something actually builds today:
/// rigid bodies, fixed frames on them, revolute, prismatic and weld joints, and
/// a declaration that a body is free in the world. The rigid tree can express
/// more — planar, screw, universal, ball, curvilinear, roll-pitch-yaw floating —
/// and none of those has a consumer. An entry point for one would be an untested
/// path with an interface nobody had yet had a reason to shape.
///
/// Failures happen where they can first be seen. A duplicate name, a handle from
/// another model, a joint from a body to itself, a second relation between two
/// bodies already related, a relation that would close a loop — each of these is
/// decidable when it is offered, and each is refused then, by name. Exactly one
/// thing cannot be decided until the whole model is known — whether every body
/// reaches the world — and that one is decided by Finalize().
///
/// Nothing here silently repairs anything. A body nobody connected stays
/// unconnected: the rigid tree underneath would eventually give it a floating
/// relation to the world and carry on, and the model that came out would be one
/// the caller never described.
///
/// A model has two lives. While it is being described it accepts elements;
/// once finalized it accepts nothing and answers the questions about what it
/// came out as. The split is not ceremony: a count or a coordinate range asked
/// for mid description would be an answer to "what has been said so far", which
/// is a different question, and one that the next call could change.
///
/// Precisely: every query about the final topology — counts, enumeration,
/// lookup by name, the name behind a handle, whether a body is free, the
/// coordinate ranges, and creating a context — requires finalization. Three
/// things do not, because they do not depend on it: the two building helpers
/// (`world_frame()` and `body_frame()`), the model's own state
/// (`is_finalized()`), and the gravity vector, which has a value from the
/// moment the model exists and only stops being settable.

#include <memory>
#include <span>
#include <string_view>

#include <Eigen/Dense>

#include "orvd/multibody_model/forward_dynamics_workspace.h"
#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_coordinate_ranges.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_frame_spatial_velocity.h"
#include "orvd/multibody_model/multibody_model_handles.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace orvd::multibody_model {

class MultibodyModel {
   public:
    MultibodyModel();
    ~MultibodyModel();

    // A model owns its elements and hands out handles that name it. Copying
    // would produce a second model whose elements answer to the first one's
    // handles; moving would leave every handle naming a model that no longer
    // holds anything. Neither is a thing the design has a use for.
    MultibodyModel(const MultibodyModel&) = delete;
    MultibodyModel& operator=(const MultibodyModel&) = delete;
    MultibodyModel(MultibodyModel&&) = delete;
    MultibodyModel& operator=(MultibodyModel&&) = delete;

    // --- Bodies -------------------------------------------------------------

    /// Adds a rigid body with the given mass properties.
    ///
    /// @throws std::invalid_argument if the name is empty or already names a
    /// body or frame, or if the mass properties are not ones a body can have.
    RigidBodyHandle AddRigidBody(
        std::string_view name,
        const multibody_runtime::RigidBodyInertiaParameters& inertia);

    // --- Frames -------------------------------------------------------------

    /// The world frame. Always present; nothing adds it.
    [[nodiscard]] FrameHandle world_frame() const;

    /// The frame the body itself defines, at its origin.
    ///
    /// @throws std::invalid_argument if the handle is invalid or came from
    /// another model.
    [[nodiscard]] FrameHandle body_frame(RigidBodyHandle body) const;

    /// Adds a frame fixed to `body` at the given pose in the body's own frame.
    ///
    /// Fixed to a body, not to another frame. A frame on a frame on a frame is
    /// expressible and nothing needs it; the chain would have to be walked at
    /// every use, and the first consumer that wanted one could say so.
    ///
    /// @throws std::invalid_argument if the name is empty or already names a
    /// frame, if the handle is invalid or foreign, or if the pose contains a
    /// non-finite translation or a matrix that is not a rotation.
    FrameHandle AddFixedFrame(
        std::string_view name, RigidBodyHandle body,
        const multibody_runtime::FixedFramePoseParameters& pose_in_body);

    // --- Joints -------------------------------------------------------------

    /// Adds a revolute joint about `axis_in_parent`, between two frames.
    ///
    /// Which frame is the parent fixes the joint's coordinate and the sign of
    /// its angle. It does not decide which body the rigid tree will treat as
    /// inboard — the tree grows its own arrangement from the world outward, and
    /// may traverse this joint in either direction. A caller who needs a
    /// particular sign says so here; a caller who thinks this determines the
    /// tree's shape is thinking of a different thing.
    ///
    /// @throws std::invalid_argument if the name is empty or already names a
    /// joint, if either handle is invalid or foreign, if both frames belong to
    /// the same body, if those two bodies are already related, if the relation
    /// would close a loop, or if `axis_in_parent` is not a usable direction.
    JointHandle AddRevoluteJoint(std::string_view name, FrameHandle parent,
                                 FrameHandle child,
                                 const Eigen::Vector3d& axis_in_parent,
                                 double damping_newton_metre_seconds_per_radian);

    /// Adds a prismatic joint along `axis_in_parent`. Same refusals.
    JointHandle AddPrismaticJoint(std::string_view name, FrameHandle parent,
                                  FrameHandle child,
                                  const Eigen::Vector3d& axis_in_parent,
                                  double damping_newton_seconds_per_metre);

    /// Adds a weld: the two frames hold still with respect to each other.
    JointHandle AddWeldJoint(std::string_view name, FrameHandle parent,
                             FrameHandle child);

    /// Declares that `body` moves freely in the world.
    ///
    /// This is a relation like any other, and it is stated rather than inferred.
    /// A body left unrelated would be given one by the rigid tree at
    /// finalization — six degrees of freedom nobody asked for, in a model that
    /// otherwise came out looking exactly as described.
    ///
    /// A free body may still carry joints outboard of it: declaring it free says
    /// how it relates to the world, not that nothing hangs off it.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign, if the
    /// body is already declared free, or if it is already related to the world
    /// through joints — in which case one of the two statements is not what the
    /// caller meant, and the model cannot tell which.
    void DeclareFreeBody(RigidBodyHandle body);

    // --- Gravity ------------------------------------------------------------

    /// The uniform gravitational acceleration, expressed in the world frame.
    ///
    /// A model constant, not something that varies between two evaluations of
    /// one model. Making it a context parameter would give every cache that
    /// reads a gravity force a dependency to declare, in exchange for a freedom
    /// nothing in this product uses.
    [[nodiscard]] const Eigen::Vector3d& gravity_vector() const;

    /// States it, while the model is still being described.
    ///
    /// @throws std::invalid_argument if `g_W` is not finite.
    /// @throws std::logic_error if the model is finalized or its finalization
    /// failed.
    void SetGravityVector(const Eigen::Vector3d& g_W);

    // --- Finalization -------------------------------------------------------

    /// Fixes the model: no more elements, and the coordinates get their places.
    ///
    /// First the one thing that needed the whole model: every rigid body must
    /// reach the world. Bodies that do not are refused, all of them named in
    /// one message, and the model still describes exactly what it described —
    /// the caller can state the missing relations and finalize again. (The
    /// reachability test compacts the paths it walks, so the bookkeeping is not
    /// byte-for-byte what it was; what it means is.)
    ///
    /// Then the model is committed, and from that point a failure is the end of
    /// it. The underlying tree may be half finalized, and there is no state to
    /// go back to, so the model enters a failed state in which every modelling
    /// write and every final-topology or evaluation query is refused and says
    /// why. The four observations that never depend on final topology remain
    /// what the class-level contract says they are. This is deliberately not a
    /// retry: a retry would be building on a tree whose condition nobody knows.
    ///
    /// After it succeeds the structure, the gravity vector and the model's own
    /// force-element constants are frozen. Every Add, the gravity setter and a
    /// second Finalize() are refused at entry.
    ///
    /// @throws std::invalid_argument if some rigid body has no path to the
    /// world, naming each one.
    /// @throws std::logic_error if the model is already finalized or its
    /// finalization already failed.
    void Finalize();

    /// Whether Finalize() has succeeded.
    [[nodiscard]] bool is_finalized() const;

    // --- The finalized model ------------------------------------------------
    //
    // Everything below requires finalization and refuses without it.

    /// How many rigid bodies, not counting the world.
    ///
    /// The world is not one of them. Nothing accelerates it, it has no mass
    /// properties to read and no force to receive, so a caller walking the
    /// bodies would be skipping it every time.
    [[nodiscard]] int num_rigid_bodies() const;

    /// How many frames, counting the world frame.
    ///
    /// Only the frames this model has: the world frame, one per rigid body, and
    /// the fixed frames somebody added. The rigid tree makes frames of its own
    /// for its own purposes; those are not the model's and are not counted.
    [[nodiscard]] int num_frames() const;

    /// How many joints, counting only the ones a caller added.
    ///
    /// A declared free relation is not one of them. It is implemented with a
    /// joint, but the caller stated a property of a body, not a joint, and a
    /// joint they never named appearing in the count is the arrangement showing
    /// through.
    [[nodiscard]] int num_joints() const;

    /// The size of the model's q and v.
    [[nodiscard]] int num_generalized_positions() const;
    [[nodiscard]] int num_generalized_velocities() const;

    /// The `index`-th rigid body, frame or joint, in the order they were added.
    ///
    /// A position in an enumeration, not an identity: it is how a caller walks
    /// what a model holds without having kept every handle. `index` is not what
    /// a handle carries and the two are not interchangeable.
    ///
    /// @throws std::invalid_argument if `index` is out of range.
    [[nodiscard]] RigidBodyHandle GetRigidBody(int index) const;
    [[nodiscard]] FrameHandle GetFrame(int index) const;
    [[nodiscard]] JointHandle GetJoint(int index) const;

    /// The element with this name.
    ///
    /// @throws std::invalid_argument if nothing of that kind has that name.
    [[nodiscard]] RigidBodyHandle GetRigidBodyByName(std::string_view name) const;
    [[nodiscard]] FrameHandle GetFrameByName(std::string_view name) const;
    [[nodiscard]] JointHandle GetJointByName(std::string_view name) const;

    /// The name this element was added under.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign.
    [[nodiscard]] std::string_view GetRigidBodyName(RigidBodyHandle body) const;
    [[nodiscard]] std::string_view GetFrameName(FrameHandle frame) const;
    [[nodiscard]] std::string_view GetJointName(JointHandle joint) const;

    /// Whether this body was declared to move freely in the world.
    [[nodiscard]] bool IsFreeBody(RigidBodyHandle body) const;

    // --- Coordinates --------------------------------------------------------
    //
    // Every generalized coordinate belongs to exactly one public joint or one
    // declared free body. The two sets do not overlap and together they cover q
    // and v exactly; a coordinate belonging to neither would be a degree of
    // freedom the model was given without being asked.

    /// Where this joint's coordinates sit. A weld's are empty.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign.
    [[nodiscard]] GeneralizedPositionRange GetJointPositionRange(
        JointHandle joint) const;
    [[nodiscard]] GeneralizedVelocityRange GetJointVelocityRange(
        JointHandle joint) const;

    /// Where this free body's coordinates sit: seven positions, six velocities.
    ///
    /// Seven and six because a quaternion carries three degrees of freedom in
    /// four numbers. Within the position range: the first four are that
    /// quaternion, in w, x, y, z order, and the last three are the body
    /// origin's position in the world. Within the velocity range: the first
    /// three are the angular velocity, the last three the translational one.
    /// A caller indexing the range needs this; leaving it unsaid would make the
    /// range something you could take a block from but not read.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign, or if
    /// the body was not declared free — an unfree body's coordinates belong to
    /// the joints that relate it, and asking here would be asking the wrong
    /// question rather than getting an empty answer.
    [[nodiscard]] GeneralizedPositionRange GetFreeBodyPositionRange(
        RigidBodyHandle body) const;
    [[nodiscard]] GeneralizedVelocityRange GetFreeBodyVelocityRange(
        RigidBodyHandle body) const;

    // --- Evaluation ---------------------------------------------------------

    /// A context holding this model's defaults.
    ///
    /// The only way to get one. The model must outlive it.
    ///
    /// @throws std::logic_error if the model is not finalized.
    [[nodiscard]] std::unique_ptr<MultibodyEvaluationContext>
    CreateDefaultContext() const;

    /// States the model's generalized positions.
    ///
    /// The whole vector at once. Which numbers are a quaternion, and what makes
    /// one usable, is knowledge this model has and the context does not — which
    /// is why the write is here and not on the context, and why there is no way
    /// to reach past it to the numbers.
    ///
    /// A quaternion is stored exactly as written. Normalising it would answer a
    /// different question from the one asked and the caller would never learn
    /// that what they read back is not what they wrote.
    ///
    /// @throws std::invalid_argument if `context` is null or came from another
    /// model, if `positions` has the wrong size or a non-finite value, or if some
    /// free body's quaternion cannot be turned into a rotation.
    /// @throws std::logic_error if the model is not finalized.
    void SetGeneralizedPositions(MultibodyEvaluationContext* context,
                                 const Eigen::VectorXd& positions) const;

    /// States the model's generalized velocities. Same size and ownership
    /// rules; velocities have no analogue of the quaternion condition.
    ///
    /// @throws std::invalid_argument if `context` is null or came from another
    /// model, or if `velocities` has the wrong size or a non-finite value.
    /// @throws std::logic_error if the model is not finalized.
    void SetGeneralizedVelocities(MultibodyEvaluationContext* context,
                                  const Eigen::VectorXd& velocities) const;

    /// States q and v as one transaction for an ODE trial or accepted commit.
    /// Both vectors are validated before either one is changed.
    ///
    /// @throws std::invalid_argument under the union of the two individual
    /// setter contracts.  A refusal leaves both vectors unchanged.
    /// @throws std::logic_error if the model is not finalized.
    void SetGeneralizedState(
        MultibodyEvaluationContext* context,
        const Eigen::Ref<const Eigen::VectorXd>& positions,
        const Eigen::Ref<const Eigen::VectorXd>& velocities) const;

    /// Sets this context's revolute-joint viscous damping coefficient.
    ///
    /// The value is in N m s/rad and is read directly by every damping-force
    /// calculation; it has no cache version because no long-lived cache stores
    /// a damping-derived result.
    ///
    /// @throws std::invalid_argument if the context or joint is foreign, the
    /// joint is not revolute, or the value is non-finite or negative.
    /// @throws std::logic_error if the model is not finalized.
    void SetRevoluteJointDampingCoefficient(
        MultibodyEvaluationContext* context, JointHandle joint,
        double damping_newton_metre_seconds_per_radian) const;

    /// Sets this context's prismatic-joint viscous damping coefficient in
    /// N s/m.  The same ownership, value and no-cache rules apply.
    void SetPrismaticJointDampingCoefficient(
        MultibodyEvaluationContext* context, JointHandle joint,
        double damping_newton_seconds_per_metre) const;

    /// Copies every currently mutable joint-damping parameter between two
    /// contexts of this model.
    ///
    /// This is the narrow synchronization operation used by the continuous
    /// system advancer.  It does not copy q, v, caches or any parameter category
    /// without a public context-local write contract.
    void CopyJointDampingParameters(
        const MultibodyEvaluationContext& source,
        MultibodyEvaluationContext* destination) const;

    // --- Position kinematics ------------------------------------------------

    /// Where this body's own frame is, in the world.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign, or if
    /// the context came from another model.
    /// @throws std::logic_error if the model is not finalized.
    [[nodiscard]] RigidPose CalcPoseInWorld(
        const MultibodyEvaluationContext& context, RigidBodyHandle body) const;

    /// Where this frame is, in the world. The world frame's own pose is the
    /// identity, which is what it means for it to be the world frame.
    [[nodiscard]] RigidPose CalcPoseInWorld(
        const MultibodyEvaluationContext& context, FrameHandle frame) const;

    /// Resolves a frame origin to the rigid body and body-fixed point where a
    /// wrench at that origin must be applied.
    ///
    /// The answer comes from the finalized model and the frame parameters in
    /// `context`; it is not reconstructed from an external model record.  A
    /// frame fixed to a welded child still resolves to that named child and to
    /// coordinates in the child's own body frame; welding does not replace the
    /// endpoint's named-body identity.
    ///
    /// @throws std::invalid_argument if the frame handle is invalid, the frame
    /// or context is foreign, or `frame` is the world frame, which has no rigid
    /// body that can receive a wrench.
    /// @throws std::logic_error if the model is not finalized.
    [[nodiscard]] BodyFixedPoint CalcFrameOriginAsBodyFixedPoint(
        const MultibodyEvaluationContext& context, FrameHandle frame) const;

    // --- Velocity and spatial kinematics -----------------------------------

    /// The spatial velocity of rigid body B's frame, measured in the world
    /// frame W and expressed in W.
    ///
    /// The angular component is ω_WB_W. The translational component is
    /// v_WBo_W: the velocity of Bo, not of the body's centre of mass or another
    /// point fixed to B.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign, or if
    /// the context came from another model.
    /// @throws std::logic_error if the model is not finalized.
    [[nodiscard]] FrameSpatialVelocity
    CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
        const MultibodyEvaluationContext& context, RigidBodyHandle body) const;

    /// The spatial velocity of frame F, measured in the world frame W and
    /// expressed in W.
    ///
    /// A frame fixed away from its body's origin generally has a different
    /// translational velocity because its point Fo moves with ω×p.
    ///
    /// @throws std::invalid_argument if the handle is invalid or foreign, or if
    /// the context came from another model.
    /// @throws std::logic_error if the model is not finalized.
    [[nodiscard]] FrameSpatialVelocity
    CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
        const MultibodyEvaluationContext& context, FrameHandle frame) const;

    /// The spatial velocity of moving frame F relative to reference frame M,
    /// expressed in frame E.
    ///
    /// `moving_frame` names F, `reference_frame` names M, and
    /// `expressed_in_frame` names E. The angular component is ω_MF_E. The
    /// translational component is v_MFo_E: the velocity of F's origin Fo
    /// measured in M and expressed in E. Because that point is Fo, this is not
    /// in general the simple difference between the world translational
    /// velocities of Fo and Mo; M's velocity must first be shifted to Fo.
    ///
    /// @throws std::invalid_argument if any handle is invalid or foreign, or if
    /// the context came from another model.
    /// @throws std::logic_error if the model is not finalized.
    [[nodiscard]] FrameSpatialVelocity
    CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
        const MultibodyEvaluationContext& context, FrameHandle moving_frame,
        FrameHandle reference_frame, FrameHandle expressed_in_frame) const;

    // --- Differential kinematics ------------------------------------------

    /// Maps generalized velocities v to generalized-position derivatives qdot.
    ///
    /// For a quaternion free body the first four entries of that body's
    /// seven-entry position range are wxyz.
    /// The stored quaternion is used as written: this call neither normalizes
    /// nor changes it. Consequently scaling that quaternion scales its four
    /// derivative entries, while the three translational derivatives are
    /// unchanged.
    ///
    /// @throws std::invalid_argument if the context is foreign, either vector
    /// has the wrong size, an input is non-finite, the output is null, or input
    /// and output are the same object.
    /// @throws std::logic_error if the model is not finalized.
    void MapGeneralizedVelocitiesToPositionDerivatives(
        const MultibodyEvaluationContext& context,
        const Eigen::VectorXd& generalized_velocities,
        Eigen::VectorXd* generalized_position_derivatives) const;

    /// Maps generalized-position derivatives qdot to generalized velocities v.
    ///
    /// For quaternion coordinates this is a left pseudoinverse: the component
    /// of qdot parallel to the stored quaternion is discarded. Thus
    /// v -> qdot -> v is an identity, whereas an arbitrary qdot -> v -> qdot is
    /// its projection onto the quaternion tangent space.
    ///
    /// @throws std::invalid_argument if the context is foreign, either vector
    /// has the wrong size, an input is non-finite, the output is null, or input
    /// and output are the same object.
    /// @throws std::logic_error if the model is not finalized.
    void MapGeneralizedPositionDerivativesToVelocities(
        const MultibodyEvaluationContext& context,
        const Eigen::VectorXd& generalized_position_derivatives,
        Eigen::VectorXd* generalized_velocities) const;

    /// Computes every public rigid body's frame acceleration A_WB_W for an
    /// explicitly supplied generalized-velocity derivative vdot.
    ///
    /// Columns follow GetRigidBody() order and exclude the world. Outputs are
    /// 3 x num_rigid_bodies(): angular acceleration in rad/s^2 and the
    /// translational acceleration of body-frame origin Bo in m/s^2. The result
    /// is computed for this call; vdot is not hidden in a persistent cache.
    ///
    /// @throws std::invalid_argument if the context is foreign, vdot is the
    /// wrong size or non-finite, either output is null or the wrong size, or
    /// both physical outputs name the same object.
    /// @throws std::logic_error if the model is not finalized.
    void CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
        const MultibodyEvaluationContext& context,
        const Eigen::VectorXd& generalized_velocity_derivatives,
        Eigen::MatrixXd* angular_accelerations_radians_per_second_squared,
        Eigen::MatrixXd*
            translational_accelerations_at_body_origins_meters_per_second_squared)
        const;

    /// Computes the spatial-velocity Jacobian of a body-fixed point Q.
    ///
    /// `point_position_in_body_frame` is p_BoQ_B. The two 3 x nv outputs map
    /// generalized velocities to angular velocity w_WB_W and point velocity
    /// v_WQ_W, respectively. This deliberately exposes only the kV,
    /// World-measured, World-expressed operator current consumers need.
    ///
    /// @throws std::invalid_argument if the context or body is foreign, the
    /// body-fixed point is non-finite, either output is null or the wrong size,
    /// or both physical outputs name the same object.
    /// @throws std::logic_error if the model is not finalized.
    void CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
        const MultibodyEvaluationContext& context, RigidBodyHandle body,
        const Eigen::Vector3d& point_position_in_body_frame,
        Eigen::MatrixXd* angular_velocity_jacobian,
        Eigen::MatrixXd* point_translational_velocity_jacobian) const;

    // --- Dynamics ----------------------------------------------------------

    /// Computes the generalized mass matrix M(q).
    ///
    /// Rows and columns follow the model's generalized-velocity ranges. Thus
    /// `M * vdot` has the generalized-force component associated with each
    /// velocity: N for a translational coordinate and N m for a rotational
    /// coordinate. The caller supplies an already sized `nv x nv` matrix; this
    /// call neither resizes nor replaces its storage.
    ///
    /// @throws std::invalid_argument if the context is foreign or the output
    /// has the wrong dimensions. A refusal occurs before the output is changed.
    /// @throws std::logic_error if the model is not finalized.
    void CalcGeneralizedMassMatrix(
        const MultibodyEvaluationContext& context,
        Eigen::MatrixXd& generalized_mass_matrix) const;

    /// Computes the generalized force required to realize `vdot`, including
    /// the current velocity bias and subtracting gravity and joint damping.
    /// The caller supplies an `nv`-sized output; its storage is reused.
    void CalcRequiredGeneralizedForces(
        const MultibodyEvaluationContext& context,
        const Eigen::VectorXd& generalized_velocity_derivatives,
        Eigen::VectorXd& required_generalized_forces) const;

    /// Computes C(q, v)v in generalized-force coordinates.
    void CalcVelocityBiasGeneralizedForces(
        const MultibodyEvaluationContext& context,
        Eigen::VectorXd& velocity_bias_generalized_forces) const;

    /// Computes the applied generalized force due to uniform gravity.
    void CalcGravityAppliedGeneralizedForces(
        const MultibodyEvaluationContext& context,
        Eigen::VectorXd& gravity_applied_generalized_forces) const;

    /// Computes the applied generalized force due to joint viscous damping.
    void CalcJointDampingAppliedGeneralizedForces(
        const MultibodyEvaluationContext& context,
        Eigen::VectorXd& damping_applied_generalized_forces) const;

    /// Creates the model-bound storage reused by forward dynamics calls.
    [[nodiscard]] std::unique_ptr<ForwardDynamicsWorkspace>
    CreateForwardDynamicsWorkspace() const;

    /// Computes vdot with model gravity and damping plus the stated forces.
    void CalcGeneralizedVelocityDerivatives(
        const MultibodyEvaluationContext& context,
        std::span<const AppliedBodyWrench> body_wrenches,
        std::span<const AppliedRevoluteJointTorque> revolute_joint_torques,
        std::span<const AppliedPrismaticJointForce> prismatic_joint_forces,
        ForwardDynamicsWorkspace& workspace,
        Eigen::VectorXd& generalized_velocity_derivatives) const;

    /// Computes [N(q)v; vdot] without storing a call result in the context.
    void CalcStateTimeDerivatives(
        const MultibodyEvaluationContext& context,
        std::span<const AppliedBodyWrench> body_wrenches,
        std::span<const AppliedRevoluteJointTorque> revolute_joint_torques,
        std::span<const AppliedPrismaticJointForce> prismatic_joint_forces,
        ForwardDynamicsWorkspace& workspace,
        Eigen::VectorXd& state_time_derivatives) const;

   private:
    const Eigen::VectorXd& EvaluateForwardDynamics(
        const MultibodyEvaluationContext& context,
        std::span<const AppliedBodyWrench> body_wrenches,
        std::span<const AppliedRevoluteJointTorque> revolute_joint_torques,
        std::span<const AppliedPrismaticJointForce> prismatic_joint_forces,
        ForwardDynamicsWorkspace& workspace) const;

    template <typename Handle>
    static internal::ModelIdentity HandleModelIdentity(const Handle& handle) {
        return handle.model_;
    }

    template <typename Handle>
    static int HandleOrdinal(const Handle& handle) {
        return handle.ordinal_;
    }

    template <typename Handle>
    static Handle MakeHandle(internal::ModelIdentity model_identity,
                             int ordinal) {
        return Handle(model_identity, ordinal);
    }

    template <typename Range>
    static Range MakeRange(int start, int size) {
        return Range(start, size);
    }

    static RigidPose MakePose(const Eigen::Matrix3d& rotation,
                              const Eigen::Vector3d& translation) {
        return RigidPose(rotation, translation);
    }

    static FrameSpatialVelocity MakeFrameSpatialVelocity(
        const Eigen::Vector3d& angular_velocity_radians_per_second,
        const Eigen::Vector3d&
            translational_velocity_at_frame_origin_meters_per_second) {
        return FrameSpatialVelocity(
            angular_velocity_radians_per_second,
            translational_velocity_at_frame_origin_meters_per_second);
    }

    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::multibody_model
