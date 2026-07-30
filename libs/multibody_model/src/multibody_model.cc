#include "orvd/multibody_model/multibody_model.h"

#include <atomic>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/weld_joint.h"
#include "orvd/multibody_runtime/multibody_physical_parameter_validation.h"
#include "orvd/rigid_multibody_tree/spatial_inertia_parameter_conversion.h"

namespace orvd::multibody_model {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("multibody model: " + detail);
}

/// Hands out an identity nobody else has had.
///
/// Monotonic rather than an address: two models can occupy the same address one
/// after the other, and a handle held across that would pass an address check
/// while naming an element of a model that is gone.
internal::ModelIdentity NextModelIdentity() {
    static std::atomic<internal::ModelIdentity> counter{0};
    return ++counter;
}

/// The pose as the rigid tree wants it, from the pose as the model states it.
drake::math::RigidTransform<double> ToRigidTransform(
    const multibody_runtime::FixedFramePoseParameters& pose) {
    return drake::math::RigidTransform<double>(
        drake::math::RotationMatrix<double>(pose.R_PF), pose.p_PoFo_P);
}

}  // namespace

class MultibodyModel::Implementation {
   public:
    Implementation() : identity_(NextModelIdentity()) {
        // The world is body zero and frame zero. It is the root every other body
        // has to reach, it is not something a caller adds, and it has no mass
        // properties to state: nothing accelerates it.
        body_names_.emplace_back("world");
        body_by_name_.emplace("world", 0);
        frame_names_.emplace_back("world");
        frame_by_name_.emplace("world", 0);
        frame_body_.push_back(0);
        frame_pose_.emplace_back();
        body_frame_.push_back(0);
        related_root_.push_back(0);
        is_free_.push_back(false);
        tree_body_.push_back(tree_.world_link().index());
    }

    template <typename Handle>
    Handle MakeHandle(int ordinal) const {
        return internal::HandleAccess::Make<Handle>(identity_, ordinal);
    }

    /// The ordinal behind a handle, once it is this model's and names something.
    template <typename Handle>
    int Resolve(const Handle& handle, int count, const char* what) const {
        if (!handle.is_valid()) {
            Reject(std::string("a ") + what +
                   " handle that names nothing was given");
        }
        if (internal::HandleAccess::model_of(handle) != identity_) {
            Reject(std::string("a ") + what +
                   " handle from a different model was given; a handle names "
                   "one model's elements and means nothing in another");
        }
        const int ordinal = internal::HandleAccess::ordinal_of(handle);
        if (ordinal < 0 || ordinal >= count) {
            Reject(std::string("a ") + what + " handle names element " +
                   std::to_string(ordinal) + " of a model that has " +
                   std::to_string(count));
        }
        return ordinal;
    }

    // Union-find over bodies, rooted at the world.
    //
    // Every explicit relation is one undirected edge. Which side the caller
    // called the parent fixes the joint's coordinate and the sign of its angle;
    // it does not decide which body the rigid tree treats as inboard, because
    // the tree grows from the world outward and may traverse a joint either way.
    //
    // So the question a new relation has to answer is not "does this body
    // already have a parent" — a body may appear in any number of relations —
    // but "are these two already connected". If they are, the new edge closes a
    // loop, and a loop is not a tree.
    int Root(int body) {
        while (related_root_[body] != body) {
            related_root_[body] = related_root_[related_root_[body]];
            body = related_root_[body];
        }
        return body;
    }

    bool AlreadyConnected(int first, int second) {
        return Root(first) == Root(second);
    }

    void Connect(int first, int second) {
        related_root_[Root(first)] = Root(second);
    }

    /// Checks the two frames, refuses the relations no model can mean, and
    /// returns the two bodies.
    std::pair<int, int> PrepareRelation(FrameHandle parent, FrameHandle child,
                                        const std::string& joint_name) {
        if (joint_name.empty()) {
            Reject("a joint needs a name");
        }
        if (joint_by_name_.contains(joint_name)) {
            Reject("there is already a joint named '" + joint_name + "'");
        }
        const int parent_frame =
            Resolve(parent, static_cast<int>(frame_names_.size()), "frame");
        const int child_frame =
            Resolve(child, static_cast<int>(frame_names_.size()), "frame");
        const int parent_body = frame_body_[parent_frame];
        const int child_body = frame_body_[child_frame];
        if (parent_body == child_body) {
            Reject("joint '" + joint_name + "' would relate body '" +
                   body_names_[parent_body] +
                   "' to itself; a body does not move with respect to itself");
        }
        if (AlreadyConnected(parent_body, child_body)) {
            Reject("joint '" + joint_name + "' would relate '" +
                   body_names_[parent_body] + "' and '" +
                   body_names_[child_body] +
                   "', which already reach each other through the relations "
                   "already stated; another one closes a loop, and a loop is "
                   "not a tree");
        }
        return {parent_body, child_body};
    }

    void RecordJoint(const std::string& name, int parent_body, int child_body) {
        joint_by_name_.emplace(name, static_cast<int>(joint_names_.size()));
        joint_names_.push_back(name);
        Connect(parent_body, child_body);
    }

    internal::ModelIdentity identity_;

    std::vector<std::string> body_names_;
    std::vector<std::string> frame_names_;
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> body_by_name_;
    std::unordered_map<std::string, int> frame_by_name_;
    std::unordered_map<std::string, int> joint_by_name_;

    /// For each frame: the body it is fixed to, and where on that body.
    std::vector<int> frame_body_;
    std::vector<multibody_runtime::FixedFramePoseParameters> frame_pose_;
    /// For each body: its own frame.
    std::vector<int> body_frame_;

    std::vector<int> related_root_;
    std::vector<bool> is_free_;

    /// The rigid tree, built as the model is described.
    ///
    /// Private to this file. The public header names no vendored type, so how
    /// the tree arranges what it is given never reaches a caller — and a caller
    /// who cannot see the arrangement cannot come to depend on it.
    drake::multibody::internal::MultibodyTree<double> tree_;
    std::vector<drake::multibody::BodyIndex> tree_body_;
};

MultibodyModel::MultibodyModel()
    : implementation_(std::make_unique<Implementation>()) {}

MultibodyModel::~MultibodyModel() = default;

// --- Bodies ------------------------------------------------------------------

RigidBodyHandle MultibodyModel::AddRigidBody(
    std::string_view name,
    const multibody_runtime::RigidBodyInertiaParameters& inertia) {
    Implementation& model = *implementation_;
    const std::string body_name(name);
    if (body_name.empty()) {
        Reject("a rigid body needs a name; an unnamed one cannot be named in "
               "any later diagnostic");
    }
    if (model.body_by_name_.contains(body_name)) {
        Reject("there is already a rigid body named '" + body_name + "'");
    }

    // Converting validates: mass properties no real distribution of mass can
    // have are refused here rather than at the first evaluation, where the
    // wrongness would surface as a dynamics discrepancy a long way from the
    // description that caused it.
    //
    // The layer below reports that as its own exception type and describes the
    // matrix it rejected, not the body. Both are converted here: what this
    // header promises is one exception type, and what a caller needs first is
    // which body they mis-described.
    // Checked by the first-party validator, not by the layer below. That layer
    // checks the same condition with an assertion, and assertions are compiled
    // out in a release build — a caller who described an impossible body would
    // be told in Debug and quietly obliged in Release.
    multibody_runtime::ThrowIfNotRealisableInertia(
        inertia, "rigid body '" + body_name + "'");
    const auto spatial_inertia =
        rigid_multibody_tree::internal::ToSpatialInertia(inertia);
    const auto& link = model.tree_.AddLink(body_name, spatial_inertia);

    const int ordinal = static_cast<int>(model.body_names_.size());
    model.body_names_.push_back(body_name);
    model.body_by_name_.emplace(body_name, ordinal);
    model.tree_body_.push_back(link.index());
    model.related_root_.push_back(ordinal);
    model.is_free_.push_back(false);

    // Every body brings its own frame, at its origin.
    const int frame_ordinal = static_cast<int>(model.frame_names_.size());
    model.frame_names_.push_back(body_name);
    model.frame_by_name_.emplace(body_name, frame_ordinal);
    model.frame_body_.push_back(ordinal);
    model.frame_pose_.emplace_back();
    model.body_frame_.push_back(frame_ordinal);

    return model.MakeHandle<RigidBodyHandle>(ordinal);
}

// --- Frames ------------------------------------------------------------------

FrameHandle MultibodyModel::world_frame() const {
    return implementation_->MakeHandle<FrameHandle>(0);
}

FrameHandle MultibodyModel::body_frame(RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    return model.MakeHandle<FrameHandle>(model.body_frame_[ordinal]);
}

FrameHandle MultibodyModel::AddFixedFrame(
    std::string_view name, RigidBodyHandle body,
    const multibody_runtime::FixedFramePoseParameters& pose_in_body) {
    Implementation& model = *implementation_;
    const std::string frame_name(name);
    if (frame_name.empty()) {
        Reject("a frame needs a name");
    }
    if (model.frame_by_name_.contains(frame_name)) {
        Reject("there is already a frame named '" + frame_name + "'");
    }
    const int body_ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");

    // The rotation is validated and never repaired: orthonormalising a
    // caller's matrix turns a modelling mistake into a plausible answer they
    // will never be told about. By the first-party validator, for the same
    // reason as the inertia above.
    multibody_runtime::ThrowIfNotARotation(
        pose_in_body.R_PF, "frame '" + frame_name + "''s rotation");

    const int ordinal = static_cast<int>(model.frame_names_.size());
    model.frame_names_.push_back(frame_name);
    model.frame_by_name_.emplace(frame_name, ordinal);
    model.frame_body_.push_back(body_ordinal);
    model.frame_pose_.push_back(pose_in_body);
    return model.MakeHandle<FrameHandle>(ordinal);
}

// --- Joints ------------------------------------------------------------------

namespace {

/// The direction a joint moves along or about, checked before it is used.
Eigen::Vector3d RequireUsableAxis(const Eigen::Vector3d& axis,
                                  const std::string& joint_name) {
    if (!axis.allFinite()) {
        Reject("joint '" + joint_name + "' was given an axis that is not finite");
    }
    const double norm = axis.norm();
    if (norm == 0.0) {
        Reject("joint '" + joint_name +
               "' was given a zero axis, which is not a direction and cannot be "
               "made into one");
    }
    return axis / norm;
}

}  // namespace

JointHandle MultibodyModel::AddRevoluteJoint(
    std::string_view name, FrameHandle parent, FrameHandle child,
    const Eigen::Vector3d& axis_in_parent) {
    Implementation& model = *implementation_;
    const std::string joint_name(name);
    const auto [parent_body, child_body] =
        model.PrepareRelation(parent, child, joint_name);
    const Eigen::Vector3d axis = RequireUsableAxis(axis_in_parent, joint_name);

    const int parent_frame = internal::HandleAccess::ordinal_of(parent);
    const int child_frame = internal::HandleAccess::ordinal_of(child);
    model.tree_.AddJoint<drake::multibody::RevoluteJoint>(
        joint_name,
        model.tree_.get_link(model.tree_body_[parent_body]),
        ToRigidTransform(model.frame_pose_[parent_frame]),
        model.tree_.get_link(model.tree_body_[child_body]),
        ToRigidTransform(model.frame_pose_[child_frame]), axis);

    model.RecordJoint(joint_name, parent_body, child_body);
    return model.MakeHandle<JointHandle>(
        static_cast<int>(model.joint_names_.size()) - 1);
}

JointHandle MultibodyModel::AddPrismaticJoint(
    std::string_view name, FrameHandle parent, FrameHandle child,
    const Eigen::Vector3d& axis_in_parent) {
    Implementation& model = *implementation_;
    const std::string joint_name(name);
    const auto [parent_body, child_body] =
        model.PrepareRelation(parent, child, joint_name);
    const Eigen::Vector3d axis = RequireUsableAxis(axis_in_parent, joint_name);

    const int parent_frame = internal::HandleAccess::ordinal_of(parent);
    const int child_frame = internal::HandleAccess::ordinal_of(child);
    model.tree_.AddJoint<drake::multibody::PrismaticJoint>(
        joint_name,
        model.tree_.get_link(model.tree_body_[parent_body]),
        ToRigidTransform(model.frame_pose_[parent_frame]),
        model.tree_.get_link(model.tree_body_[child_body]),
        ToRigidTransform(model.frame_pose_[child_frame]), axis);

    model.RecordJoint(joint_name, parent_body, child_body);
    return model.MakeHandle<JointHandle>(
        static_cast<int>(model.joint_names_.size()) - 1);
}

JointHandle MultibodyModel::AddWeldJoint(std::string_view name,
                                         FrameHandle parent,
                                         FrameHandle child) {
    Implementation& model = *implementation_;
    const std::string joint_name(name);
    const auto [parent_body, child_body] =
        model.PrepareRelation(parent, child, joint_name);

    const int parent_frame = internal::HandleAccess::ordinal_of(parent);
    const int child_frame = internal::HandleAccess::ordinal_of(child);
    // The weld's own transform is identity: where the two frames sit on their
    // bodies already says how the bodies are placed relative to each other.
    model.tree_.AddJoint<drake::multibody::WeldJoint>(
        joint_name,
        model.tree_.get_link(model.tree_body_[parent_body]),
        ToRigidTransform(model.frame_pose_[parent_frame]),
        model.tree_.get_link(model.tree_body_[child_body]),
        ToRigidTransform(model.frame_pose_[child_frame]),
        drake::math::RigidTransform<double>::Identity());

    model.RecordJoint(joint_name, parent_body, child_body);
    return model.MakeHandle<JointHandle>(
        static_cast<int>(model.joint_names_.size()) - 1);
}

void MultibodyModel::DeclareFreeBody(RigidBodyHandle body) {
    Implementation& model = *implementation_;
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    if (ordinal == 0) {
        Reject("the world cannot be declared free; it is what everything else "
               "is free with respect to");
    }
    if (model.is_free_[ordinal]) {
        Reject("rigid body '" + model.body_names_[ordinal] +
               "' is already declared free");
    }
    if (model.AlreadyConnected(ordinal, 0)) {
        Reject("rigid body '" + model.body_names_[ordinal] +
               "' already reaches the world through the joints stated; "
               "declaring it free as well says two different things about how "
               "it moves, and the model cannot tell which was meant");
    }

    // Stated, not inferred. A body left unrelated would be given a floating
    // relation to the world by the rigid tree at finalization: six degrees of
    // freedom nobody asked for, in a model that otherwise came out looking
    // exactly as described.
    const std::string joint_name =
        "free_" + model.body_names_[static_cast<std::size_t>(ordinal)];
    model.tree_.AddJoint<drake::multibody::QuaternionFloatingJoint>(
        joint_name, model.tree_.world_link(), std::nullopt,
        model.tree_.get_link(model.tree_body_[ordinal]), std::nullopt);

    model.is_free_[ordinal] = true;
    model.RecordJoint(joint_name, 0, ordinal);
}

// --- What has been described so far -----------------------------------------

int MultibodyModel::num_rigid_bodies() const {
    return static_cast<int>(implementation_->body_names_.size());
}

int MultibodyModel::num_frames() const {
    return static_cast<int>(implementation_->frame_names_.size());
}

int MultibodyModel::num_joints() const {
    return static_cast<int>(implementation_->joint_names_.size());
}

bool MultibodyModel::is_related(RigidBodyHandle body) const {
    Implementation& model = *implementation_;
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    return model.AlreadyConnected(ordinal, 0);
}

const std::string& MultibodyModel::name_of(RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    return model.body_names_[static_cast<std::size_t>(model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body"))];
}

const std::string& MultibodyModel::name_of(FrameHandle frame) const {
    const Implementation& model = *implementation_;
    return model.frame_names_[static_cast<std::size_t>(model.Resolve(
        frame, static_cast<int>(model.frame_names_.size()), "frame"))];
}

const std::string& MultibodyModel::name_of(JointHandle joint) const {
    const Implementation& model = *implementation_;
    return model.joint_names_[static_cast<std::size_t>(model.Resolve(
        joint, static_cast<int>(model.joint_names_.size()), "joint"))];
}

}  // namespace orvd::multibody_model
