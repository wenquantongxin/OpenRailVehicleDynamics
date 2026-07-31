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
#include <string_view>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_coordinate_ranges.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model_handles.h"
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
                                 const Eigen::Vector3d& axis_in_parent);

    /// Adds a prismatic joint along `axis_in_parent`. Same refusals.
    JointHandle AddPrismaticJoint(std::string_view name, FrameHandle parent,
                                  FrameHandle child,
                                  const Eigen::Vector3d& axis_in_parent);

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
    /// go back to, so the model enters a failed state in which every call is
    /// refused and says why. This is deliberately not a retry: a retry would be
    /// building on a tree whose condition nobody knows.
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

   private:
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

    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::multibody_model
