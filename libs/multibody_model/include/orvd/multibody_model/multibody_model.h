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
/// decidable when it is offered, and each is refused then, by name. What cannot
/// be decided until the whole model is known — whether every body reaches the
/// world — belongs to finalization, which is G30's.
///
/// Nothing here silently repairs anything. A body nobody connected stays
/// unconnected: the rigid tree underneath would eventually give it a floating
/// relation to the world and carry on, and the model that came out would be one
/// the caller never described.

#include <memory>
#include <string>
#include <string_view>

#include <Eigen/Dense>

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
    /// body, or if the mass properties are not ones a body can have.
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
    /// frame, if the handle is invalid or foreign, or if the rotation is not a
    /// rotation.
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

    // --- What has been described so far -------------------------------------

    [[nodiscard]] int num_rigid_bodies() const;
    [[nodiscard]] int num_frames() const;
    [[nodiscard]] int num_joints() const;

    /// Whether `body` has been given a relation: a joint, a weld, or a free
    /// declaration. Bodies without one are what finalization refuses.
    [[nodiscard]] bool is_related(RigidBodyHandle body) const;

    /// The name the element was added under, for diagnostics.
    [[nodiscard]] const std::string& name_of(RigidBodyHandle body) const;
    [[nodiscard]] const std::string& name_of(FrameHandle frame) const;
    [[nodiscard]] const std::string& name_of(JointHandle joint) const;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::multibody_model
