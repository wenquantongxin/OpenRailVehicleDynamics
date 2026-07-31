#include "orvd/multibody_model/multibody_model.h"

#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "forward_dynamics_workspace_implementation.h"
#include "multibody_evaluation_context_implementation.h"
#include "orvd/multibody_runtime/multibody_physical_parameter_validation.h"
#include "orvd/rigid_multibody_tree/spatial_inertia_parameter_conversion.h"

namespace orvd::multibody_model {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("multibody model: " + detail);
}

/// Refuses something the model cannot be asked at this point in its life.
///
/// A different exception type from Reject() because it is a different kind of
/// mistake: the arguments were fine, the model was not in a state to receive
/// them. `std::invalid_argument` derives from `std::logic_error`, so a caller
/// who does not care about the distinction still catches both with one clause.
[[noreturn]] void RejectState(const std::string& detail) {
    throw std::logic_error("multibody model: " + detail);
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
        tree_frame_.push_back(tree_.world_frame().index());
        body_frame_.push_back(0);
        related_root_.push_back(0);
        is_free_.push_back(false);
        free_body_tree_joint_.emplace_back();
        tree_body_.push_back(tree_.world_link().index());
    }

    /// Refuses anything that would add to or restate a model that is past it.
    void ThrowIfNotBuildable(std::string_view action) const {
        if (finalization_failed_) {
            RejectState(
                "cannot " + std::string(action) +
                ": this model's finalization failed partway through, so the "
                "rigid tree underneath is in a condition nothing here can "
                "describe; build a new model rather than continuing with this "
                "one");
        }
        if (finalized_) {
            RejectState("cannot " + std::string(action) +
                        ": this model is finalized, and what a finalized model "
                        "contains is what its coordinates were assigned from");
        }
    }

    /// Refuses a context this model did not issue.
    ///
    /// Checked here rather than left to the rigid tree. The tree would refuse
    /// it too — a state is bound to its layout by object identity — but it
    /// would refuse in terms of a layout the caller has never seen, about a
    /// model they did not name.
    template <typename Context>
    static void RequireOwnContext(const Implementation& model, Context* context,
                                  const char* action) {
        if (context == nullptr) {
            Reject(std::string("cannot ") + action + " a context that is null");
        }
        if (context->implementation_->issuer() != model.identity_) {
            Reject(std::string("cannot ") + action +
                   " a context issued by a different model; a context holds one "
                   "model's coordinates and means nothing in another");
        }
    }

    /// Refuses anything that only a finalized model can answer.
    void ThrowIfNotFinalized(std::string_view query) const {
        if (finalization_failed_) {
            RejectState("cannot " + std::string(query) +
                        ": this model's finalization failed partway through, "
                        "so it never came to hold an answer");
        }
        if (!finalized_) {
            RejectState(
                "cannot " + std::string(query) +
                " before Finalize(): until then the answer is about what has "
                "been described so far, which the next call can still change");
        }
    }

    template <typename Handle>
    Handle MakeHandle(int ordinal) const {
        return MultibodyModel::MakeHandle<Handle>(identity_, ordinal);
    }

    /// The ordinal behind a handle, once it is this model's and names something.
    template <typename Handle>
    int Resolve(const Handle& handle, int count, const char* what) const {
        if (!handle.is_valid()) {
            Reject(std::string("a ") + what +
                   " handle that names nothing was given");
        }
        if (MultibodyModel::HandleModelIdentity(handle) != identity_) {
            Reject(std::string("a ") + what +
                   " handle from a different model was given; a handle names "
                   "one model's elements and means nothing in another");
        }
        const int ordinal = MultibodyModel::HandleOrdinal(handle);
        if (ordinal < 0 || ordinal >= count) {
            Reject(std::string("a ") + what + " handle names element " +
                   std::to_string(ordinal) + " of a model that has " +
                   std::to_string(count));
        }
        return ordinal;
    }

    // Union-find over bodies. Its representative is arbitrary; membership in
    // the world's component is tested by comparing with Root(0), never by
    // assuming that the representative itself remains zero.
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

    enum class JointKind { kRevolute, kPrismatic, kWeld };

    void RecordJoint(const std::string& name, int parent_body, int child_body,
                     drake::multibody::JointIndex tree_joint, JointKind kind) {
        joint_by_name_.emplace(name, static_cast<int>(joint_names_.size()));
        joint_names_.push_back(name);
        tree_joint_.push_back(tree_joint);
        joint_kind_.push_back(kind);
        Connect(parent_body, child_body);
    }

    internal::ModelIdentity identity_;

    std::vector<std::string> body_names_;
    std::vector<std::string> frame_names_;
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> body_by_name_;
    std::unordered_map<std::string, int> frame_by_name_;
    std::unordered_map<std::string, int> joint_by_name_;

    /// For each frame: the body it is fixed to.
    std::vector<int> frame_body_;
    /// For each body: its own frame.
    std::vector<int> body_frame_;

    std::vector<int> related_root_;
    std::vector<bool> is_free_;

    /// For each declared free body: the private joint that implements it.
    ///
    /// Assigned during finalization, not when the body is declared free.
    /// Deferring it is what lets the private name be picked against the
    /// complete set of public joint names: a private joint already sitting in
    /// the tree would make a caller's later, perfectly legitimate joint name
    /// collide with an implementation detail.
    std::vector<drake::multibody::JointIndex> free_body_tree_joint_;

    bool finalized_{false};
    bool finalization_failed_{false};

    /// The rigid tree, built as the model is described.
    ///
    /// Private to this file. The public header names no vendored type, so how
    /// the tree arranges what it is given never reaches a caller — and a caller
    /// who cannot see the arrangement cannot come to depend on it.
    drake::multibody::internal::MultibodyTree<double> tree_;
    std::vector<drake::multibody::BodyIndex> tree_body_;
    std::vector<drake::multibody::FrameIndex> tree_frame_;
    std::vector<drake::multibody::JointIndex> tree_joint_;
    std::vector<JointKind> joint_kind_;
};

MultibodyModel::MultibodyModel()
    : implementation_(std::make_unique<Implementation>()) {}

MultibodyModel::~MultibodyModel() = default;

// --- Bodies ------------------------------------------------------------------

RigidBodyHandle MultibodyModel::AddRigidBody(
    std::string_view name,
    const multibody_runtime::RigidBodyInertiaParameters& inertia) {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("add a rigid body");
    const std::string body_name(name);
    if (body_name.empty()) {
        Reject("a rigid body needs a name; an unnamed one cannot be named in "
               "any later diagnostic");
    }
    if (model.body_by_name_.contains(body_name)) {
        Reject("there is already a rigid body named '" + body_name + "'");
    }
    if (model.frame_by_name_.contains(body_name)) {
        Reject("there is already a frame named '" + body_name +
               "'; a rigid body brings a body frame with its own name");
    }

    // Validate once at the first-party boundary, then convert without a second
    // check. The adapter conversion deliberately skips repeated validation;
    // vendored Debug assertions are not an acceptance contract for this API.
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
    model.free_body_tree_joint_.emplace_back();

    // Every body brings its own frame, at its origin.
    const int frame_ordinal = static_cast<int>(model.frame_names_.size());
    model.frame_names_.push_back(body_name);
    model.frame_by_name_.emplace(body_name, frame_ordinal);
    model.frame_body_.push_back(ordinal);
    model.tree_frame_.push_back(link.link_frame().index());
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
    model.ThrowIfNotBuildable("add a fixed frame");
    const std::string frame_name(name);
    if (frame_name.empty()) {
        Reject("a frame needs a name");
    }
    if (model.frame_by_name_.contains(frame_name)) {
        Reject("there is already a frame named '" + frame_name + "'");
    }
    const int body_ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");

    // The whole pose is validated and never repaired. The same first-party
    // validator guards context writes, so Debug and Release accept exactly the
    // same finite translations and rotations.
    multibody_runtime::ThrowIfNotAValidFixedFramePose(
        pose_in_body, "frame '" + frame_name + "'");
    const auto& tree_frame =
        model.tree_.AddFrame<drake::multibody::FixedOffsetFrame>(
            frame_name, model.tree_.get_link(model.tree_body_[body_ordinal]),
            ToRigidTransform(pose_in_body));

    const int ordinal = static_cast<int>(model.frame_names_.size());
    model.frame_names_.push_back(frame_name);
    model.frame_by_name_.emplace(frame_name, ordinal);
    model.frame_body_.push_back(body_ordinal);
    model.tree_frame_.push_back(tree_frame.index());
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
    const double scale = axis.cwiseAbs().maxCoeff();
    if (scale == 0.0) {
        Reject("joint '" + joint_name +
               "' was given a zero axis, which is not a direction and cannot be "
               "made into one");
    }
    // Dividing by the largest component first keeps the norm in [1, sqrt(3)].
    // A direct norm can overflow for a perfectly usable finite direction such
    // as (max_double, max_double, 0), after which axis / inf becomes zero.
    const Eigen::Vector3d scaled = axis / scale;
    return scaled / scaled.norm();
}

}  // namespace

JointHandle MultibodyModel::AddRevoluteJoint(
    std::string_view name, FrameHandle parent, FrameHandle child,
    const Eigen::Vector3d& axis_in_parent,
    double damping_newton_metre_seconds_per_radian) {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("add a revolute joint");
    const std::string joint_name(name);
    const auto [parent_body, child_body] =
        model.PrepareRelation(parent, child, joint_name);
    const Eigen::Vector3d axis = RequireUsableAxis(axis_in_parent, joint_name);
    if (!std::isfinite(damping_newton_metre_seconds_per_radian) ||
        damping_newton_metre_seconds_per_radian < 0.0) {
        Reject("revolute joint '" + joint_name +
               "' damping must be finite and non-negative");
    }

    const int parent_frame = HandleOrdinal(parent);
    const int child_frame = HandleOrdinal(child);
    const auto& joint = model.tree_.AddJoint<drake::multibody::RevoluteJoint>(
        std::make_unique<drake::multibody::RevoluteJoint<double>>(
            joint_name, model.tree_.get_frame(model.tree_frame_[parent_frame]),
            model.tree_.get_frame(model.tree_frame_[child_frame]), axis,
            damping_newton_metre_seconds_per_radian));

    model.RecordJoint(joint_name, parent_body, child_body, joint.index(),
                      Implementation::JointKind::kRevolute);
    return model.MakeHandle<JointHandle>(
        static_cast<int>(model.joint_names_.size()) - 1);
}

JointHandle MultibodyModel::AddPrismaticJoint(
    std::string_view name, FrameHandle parent, FrameHandle child,
    const Eigen::Vector3d& axis_in_parent,
    double damping_newton_seconds_per_metre) {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("add a prismatic joint");
    const std::string joint_name(name);
    const auto [parent_body, child_body] =
        model.PrepareRelation(parent, child, joint_name);
    const Eigen::Vector3d axis = RequireUsableAxis(axis_in_parent, joint_name);
    if (!std::isfinite(damping_newton_seconds_per_metre) ||
        damping_newton_seconds_per_metre < 0.0) {
        Reject("prismatic joint '" + joint_name +
               "' damping must be finite and non-negative");
    }

    const int parent_frame = HandleOrdinal(parent);
    const int child_frame = HandleOrdinal(child);
    const auto& joint = model.tree_.AddJoint<drake::multibody::PrismaticJoint>(
        std::make_unique<drake::multibody::PrismaticJoint<double>>(
            joint_name, model.tree_.get_frame(model.tree_frame_[parent_frame]),
            model.tree_.get_frame(model.tree_frame_[child_frame]), axis,
            -std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            damping_newton_seconds_per_metre));

    model.RecordJoint(joint_name, parent_body, child_body, joint.index(),
                      Implementation::JointKind::kPrismatic);
    return model.MakeHandle<JointHandle>(
        static_cast<int>(model.joint_names_.size()) - 1);
}

JointHandle MultibodyModel::AddWeldJoint(std::string_view name,
                                         FrameHandle parent,
                                         FrameHandle child) {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("add a weld joint");
    const std::string joint_name(name);
    const auto [parent_body, child_body] =
        model.PrepareRelation(parent, child, joint_name);

    const int parent_frame = HandleOrdinal(parent);
    const int child_frame = HandleOrdinal(child);
    // The weld's own transform is identity: where the two frames sit on their
    // bodies already says how the bodies are placed relative to each other.
    const auto& joint = model.tree_.AddJoint<drake::multibody::WeldJoint>(
        std::make_unique<drake::multibody::WeldJoint<double>>(
            joint_name, model.tree_.get_frame(model.tree_frame_[parent_frame]),
            model.tree_.get_frame(model.tree_frame_[child_frame]),
            drake::math::RigidTransform<double>::Identity()));

    model.RecordJoint(joint_name, parent_body, child_body, joint.index(),
                      Implementation::JointKind::kWeld);
    return model.MakeHandle<JointHandle>(
        static_cast<int>(model.joint_names_.size()) - 1);
}

void MultibodyModel::DeclareFreeBody(RigidBodyHandle body) {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("declare a rigid body free");
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
    //
    // The relation is recorded here, in the model's own forest, so that every
    // topological conflict is still decided at the moment it is offered. The
    // joint that implements it is not created until finalization: a private
    // joint sitting in the tree from now on would occupy a name, and a caller
    // who later asked for a joint by that same perfectly ordinary name would be
    // refused because of something the model never told them about.
    model.is_free_[ordinal] = true;
    model.Connect(0, ordinal);
}

// --- Gravity -----------------------------------------------------------------

const Eigen::Vector3d& MultibodyModel::gravity_vector() const {
    return implementation_->tree_.gravity_field().gravity_vector();
}

void MultibodyModel::SetGravityVector(const Eigen::Vector3d& g_W) {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("set the gravity vector");
    if (!g_W.allFinite()) {
        Reject("the gravity vector is not finite");
    }
    model.tree_.mutable_gravity_field().set_gravity_vector(g_W);
}

// --- Finalization ------------------------------------------------------------

void MultibodyModel::Finalize() {
    Implementation& model = *implementation_;
    model.ThrowIfNotBuildable("finalize the model");

    // The precheck. It does not change the rigid tree or the model's meaning;
    // Root() only shortens paths in the model's union-find bookkeeping. A
    // caller whose model fails this check can state what is missing and
    // finalize again.
    //
    // Reachability is all there is to check. Every relation the model accepted
    // joined two components that were not yet joined, so the forest cannot
    // contain a cycle and no body can reach the world by two different paths.
    // What remains is whether it reaches the world at all.
    std::vector<std::string> unreachable;
    for (int ordinal = 1; ordinal < static_cast<int>(model.body_names_.size());
         ++ordinal) {
        if (!model.AlreadyConnected(ordinal, 0)) {
            unreachable.push_back(model.body_names_[ordinal]);
        }
    }
    if (!unreachable.empty()) {
        std::string names;
        for (std::size_t i = 0; i < unreachable.size(); ++i) {
            names += (i == 0 ? "'" : ", '") + unreachable[i] + "'";
        }
        Reject("cannot finalize: " + std::to_string(unreachable.size()) +
               (unreachable.size() == 1 ? " rigid body has" : " rigid bodies have") +
               " no path to the world: " + names +
               ". Relate each one, or declare it free if that is what was "
               "meant; nothing was changed, so finalizing again after saying so "
               "will work");
    }

    // The commit phase. Past this line the tree is being changed, and a failure
    // leaves it in a condition nothing here can describe — so the model records
    // that it is finished before it starts, and only getting all the way
    // through clears that. No exception is caught: the reason a finalization
    // failed is the reason the caller needs, not one this layer would restate.
    model.finalization_failed_ = true;

    for (int ordinal = 1; ordinal < static_cast<int>(model.body_names_.size());
         ++ordinal) {
        if (!model.is_free_[ordinal]) continue;
        // Private, and named so it cannot be mistaken for one of the caller's.
        // Every public joint is in the tree by now, so a name that is free here
        // is free for good.
        std::string joint_name = "__orvd_free_body_" + std::to_string(ordinal);
        while (model.tree_.HasJointNamed(joint_name)) {
            joint_name.push_back('_');
        }
        const auto& joint =
            model.tree_.AddJoint<drake::multibody::QuaternionFloatingJoint>(
                std::make_unique<
                    drake::multibody::QuaternionFloatingJoint<double>>(
                    joint_name, model.tree_.world_frame(),
                    model.tree_.get_link(model.tree_body_[ordinal])
                        .link_frame()));
        model.free_body_tree_joint_[ordinal] = joint.index();
    }

    model.tree_.Finalize();

    model.finalization_failed_ = false;
    model.finalized_ = true;
}

bool MultibodyModel::is_finalized() const {
    return implementation_->finalized_;
}

// --- What the finalized model contains ---------------------------------------

namespace {

/// A position in an enumeration, checked before it is used as one.
int RequireIndex(int index, int count, const std::string& what) {
    if (index < 0 || index >= count) {
        Reject("there is no " + what + " at index " + std::to_string(index) +
               "; this model has " + std::to_string(count));
    }
    return index;
}

}  // namespace

int MultibodyModel::num_rigid_bodies() const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("count the rigid bodies");
    // Less the world, which occupies ordinal zero and is not one of them.
    return static_cast<int>(model.body_names_.size()) - 1;
}

int MultibodyModel::num_frames() const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("count the frames");
    return static_cast<int>(model.frame_names_.size());
}

int MultibodyModel::num_joints() const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("count the joints");
    return static_cast<int>(model.joint_names_.size());
}

int MultibodyModel::num_generalized_positions() const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("count the generalized positions");
    return model.tree_.num_positions();
}

int MultibodyModel::num_generalized_velocities() const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("count the generalized velocities");
    return model.tree_.num_velocities();
}

RigidBodyHandle MultibodyModel::GetRigidBody(int index) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("enumerate the rigid bodies");
    RequireIndex(index, static_cast<int>(model.body_names_.size()) - 1,
                 "rigid body");
    // Ordinal zero is the world, which the enumeration does not include.
    return model.MakeHandle<RigidBodyHandle>(index + 1);
}

FrameHandle MultibodyModel::GetFrame(int index) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("enumerate the frames");
    RequireIndex(index, static_cast<int>(model.frame_names_.size()), "frame");
    return model.MakeHandle<FrameHandle>(index);
}

JointHandle MultibodyModel::GetJoint(int index) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("enumerate the joints");
    RequireIndex(index, static_cast<int>(model.joint_names_.size()), "joint");
    return model.MakeHandle<JointHandle>(index);
}

RigidBodyHandle MultibodyModel::GetRigidBodyByName(
    std::string_view name) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("look a rigid body up by name");
    const auto found = model.body_by_name_.find(std::string(name));
    if (found == model.body_by_name_.end()) {
        Reject("this model has no rigid body named '" + std::string(name) +
               "'");
    }
    if (found->second == 0) {
        Reject("'" + std::string(name) +
               "' names the world, which is what the model's rigid bodies are "
               "placed with respect to rather than one of them");
    }
    return model.MakeHandle<RigidBodyHandle>(found->second);
}

FrameHandle MultibodyModel::GetFrameByName(std::string_view name) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("look a frame up by name");
    const auto found = model.frame_by_name_.find(std::string(name));
    if (found == model.frame_by_name_.end()) {
        Reject("this model has no frame named '" + std::string(name) + "'");
    }
    return model.MakeHandle<FrameHandle>(found->second);
}

JointHandle MultibodyModel::GetJointByName(std::string_view name) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("look a joint up by name");
    const auto found = model.joint_by_name_.find(std::string(name));
    if (found == model.joint_by_name_.end()) {
        Reject("this model has no joint named '" + std::string(name) + "'");
    }
    return model.MakeHandle<JointHandle>(found->second);
}

std::string_view MultibodyModel::GetRigidBodyName(RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("name a rigid body");
    return model.body_names_[model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body")];
}

std::string_view MultibodyModel::GetFrameName(FrameHandle frame) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("name a frame");
    return model.frame_names_[model.Resolve(
        frame, static_cast<int>(model.frame_names_.size()), "frame")];
}

std::string_view MultibodyModel::GetJointName(JointHandle joint) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("name a joint");
    return model.joint_names_[model.Resolve(
        joint, static_cast<int>(model.joint_names_.size()), "joint")];
}

bool MultibodyModel::IsFreeBody(RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask whether a rigid body is free");
    return model.is_free_[model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body")];
}

// --- Coordinates -------------------------------------------------------------
//
// A weld reports a start with a zero length. That start is where its
// coordinates would have begun, which is a legitimate place to take an empty
// block from and is not a coordinate belonging to anything.

GeneralizedPositionRange MultibodyModel::GetJointPositionRange(
    JointHandle joint) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask where a joint's positions are");
    const int ordinal = model.Resolve(
        joint, static_cast<int>(model.joint_names_.size()), "joint");
    const auto& tree_joint = model.tree_.get_joint(model.tree_joint_[ordinal]);
    return MakeRange<GeneralizedPositionRange>(tree_joint.position_start(),
                                               tree_joint.num_positions());
}

GeneralizedVelocityRange MultibodyModel::GetJointVelocityRange(
    JointHandle joint) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask where a joint's velocities are");
    const int ordinal = model.Resolve(
        joint, static_cast<int>(model.joint_names_.size()), "joint");
    const auto& tree_joint = model.tree_.get_joint(model.tree_joint_[ordinal]);
    return MakeRange<GeneralizedVelocityRange>(tree_joint.velocity_start(),
                                               tree_joint.num_velocities());
}

GeneralizedPositionRange MultibodyModel::GetFreeBodyPositionRange(
    RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask where a free body's positions are");
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    if (!model.is_free_[ordinal]) {
        Reject("rigid body '" + model.body_names_[ordinal] +
               "' was not declared free, so its coordinates belong to the "
               "joints that relate it rather than to it");
    }
    const auto& tree_joint =
        model.tree_.get_joint(model.free_body_tree_joint_[ordinal]);
    return MakeRange<GeneralizedPositionRange>(tree_joint.position_start(),
                                               tree_joint.num_positions());
}

GeneralizedVelocityRange MultibodyModel::GetFreeBodyVelocityRange(
    RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask where a free body's velocities are");
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    if (!model.is_free_[ordinal]) {
        Reject("rigid body '" + model.body_names_[ordinal] +
               "' was not declared free, so its coordinates belong to the "
               "joints that relate it rather than to it");
    }
    const auto& tree_joint =
        model.tree_.get_joint(model.free_body_tree_joint_[ordinal]);
    return MakeRange<GeneralizedVelocityRange>(tree_joint.velocity_start(),
                                               tree_joint.num_velocities());
}

// --- Evaluation --------------------------------------------------------------

std::unique_ptr<MultibodyEvaluationContext> MultibodyModel::CreateDefaultContext()
    const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("create an evaluation context");
    auto implementation =
        std::make_unique<MultibodyEvaluationContext::Implementation>(
            model.identity_, model.tree_.CreateDefaultEvaluationContext(),
            model.tree_.num_links(), model.tree_.num_mobods(),
            model.tree_.num_velocities());
    return std::unique_ptr<MultibodyEvaluationContext>(
        new MultibodyEvaluationContext(std::move(implementation)));
}

// --- State ------------------------------------------------------------------

namespace {

/// Refuses a coordinate vector that is not the size the model's is.
void RequireCoordinateCount(int given, int expected, const char* what) {
    if (given != expected) {
        Reject(std::string("this model has ") + std::to_string(expected) + " " +
               what + ", but " + std::to_string(given) +
               " were given; nothing was written");
    }
}

void RequireFiniteVector(const Eigen::VectorXd& values, const char* what) {
    if (!values.allFinite()) {
        Reject(std::string(what) +
               " contains a non-finite value; nothing was written");
    }
}

void RequireVectorOutput(Eigen::VectorXd* output, int expected_size,
                         const char* what) {
    if (output == nullptr) {
        Reject(std::string(what) + " output is null; nothing was written");
    }
    RequireCoordinateCount(static_cast<int>(output->size()), expected_size,
                           what);
}

void RequireMatrixOutput(Eigen::MatrixXd* output, int expected_rows,
                         int expected_columns, const char* what) {
    if (output == nullptr) {
        Reject(std::string(what) + " output is null; nothing was written");
    }
    if (output->rows() != expected_rows ||
        output->cols() != expected_columns) {
        Reject(std::string(what) + " output must be " +
               std::to_string(expected_rows) + " x " +
               std::to_string(expected_columns) + ", but it is " +
               std::to_string(output->rows()) + " x " +
               std::to_string(output->cols()) + "; nothing was written");
    }
}

}  // namespace

void MultibodyModel::SetGeneralizedPositions(
    MultibodyEvaluationContext* context,
    const Eigen::VectorXd& positions) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("state the generalized positions");
    Implementation::RequireOwnContext(model, context, "write positions into");
    RequireCoordinateCount(static_cast<int>(positions.size()),
                           model.tree_.num_positions(),
                           "generalized positions");
    model.tree_.SetPositions(&context->implementation_->mutable_tree_context(),
                             positions);
}

void MultibodyModel::SetGeneralizedVelocities(
    MultibodyEvaluationContext* context,
    const Eigen::VectorXd& velocities) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("state the generalized velocities");
    Implementation::RequireOwnContext(model, context, "write velocities into");
    RequireCoordinateCount(static_cast<int>(velocities.size()),
                           model.tree_.num_velocities(),
                           "generalized velocities");
    model.tree_.SetVelocities(&context->implementation_->mutable_tree_context(),
                              velocities);
}

void MultibodyModel::SetGeneralizedState(
    MultibodyEvaluationContext* context,
    const Eigen::Ref<const Eigen::VectorXd>& positions,
    const Eigen::Ref<const Eigen::VectorXd>& velocities) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("state the generalized positions and velocities");
    Implementation::RequireOwnContext(model, context,
                                      "write continuous state into");
    model.tree_.SetPositionsAndVelocities(
        &context->implementation_->mutable_tree_context(), positions,
        velocities);
}

namespace {

void RequireDampingCoefficient(double damping, const char* joint_kind) {
    if (!std::isfinite(damping) || damping < 0.0) {
        Reject(std::string(joint_kind) +
               " joint damping must be finite and non-negative; nothing was "
               "written");
    }
}

}  // namespace

void MultibodyModel::SetRevoluteJointDampingCoefficient(
    MultibodyEvaluationContext* context, JointHandle joint,
    double damping_newton_metre_seconds_per_radian) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("state a revolute joint damping coefficient");
    Implementation::RequireOwnContext(model, context,
                                      "write joint damping into");
    const int ordinal = model.Resolve(
        joint, static_cast<int>(model.joint_names_.size()), "joint");
    if (model.joint_kind_[ordinal] != Implementation::JointKind::kRevolute) {
        Reject("joint '" + model.joint_names_[ordinal] +
               "' is not revolute and has no damping coefficient in N m s/rad");
    }
    RequireDampingCoefficient(damping_newton_metre_seconds_per_radian,
                              "revolute");
    Eigen::VectorXd damping(1);
    damping[0] = damping_newton_metre_seconds_per_radian;
    model.tree_.SetJointDamping(
        &context->implementation_->mutable_tree_context(),
        model.tree_.get_joint(model.tree_joint_[ordinal]), damping);
}

void MultibodyModel::SetPrismaticJointDampingCoefficient(
    MultibodyEvaluationContext* context, JointHandle joint,
    double damping_newton_seconds_per_metre) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("state a prismatic joint damping coefficient");
    Implementation::RequireOwnContext(model, context,
                                      "write joint damping into");
    const int ordinal = model.Resolve(
        joint, static_cast<int>(model.joint_names_.size()), "joint");
    if (model.joint_kind_[ordinal] != Implementation::JointKind::kPrismatic) {
        Reject("joint '" + model.joint_names_[ordinal] +
               "' is not prismatic and has no damping coefficient in N s/m");
    }
    RequireDampingCoefficient(damping_newton_seconds_per_metre, "prismatic");
    Eigen::VectorXd damping(1);
    damping[0] = damping_newton_seconds_per_metre;
    model.tree_.SetJointDamping(
        &context->implementation_->mutable_tree_context(),
        model.tree_.get_joint(model.tree_joint_[ordinal]), damping);
}

// --- Position kinematics -----------------------------------------------------

RigidPose MultibodyModel::CalcPoseInWorld(
    const MultibodyEvaluationContext& context, RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask where a rigid body is");
    Implementation::RequireOwnContext(model, &context, "read poses from");
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    const auto X_WB =
        model.tree_.get_link(model.tree_body_[ordinal])
            .body_frame()
            .CalcPoseInWorld(context.implementation_->tree_context());
    return MakePose(X_WB.rotation().matrix(), X_WB.translation());
}

RigidPose MultibodyModel::CalcPoseInWorld(
    const MultibodyEvaluationContext& context, FrameHandle frame) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask where a frame is");
    Implementation::RequireOwnContext(model, &context, "read poses from");
    const int ordinal = model.Resolve(
        frame, static_cast<int>(model.frame_names_.size()), "frame");
    const auto X_WF =
        model.tree_.get_frame(model.tree_frame_[ordinal])
            .CalcPoseInWorld(context.implementation_->tree_context());
    return MakePose(X_WF.rotation().matrix(), X_WF.translation());
}

// --- Velocity and spatial kinematics -----------------------------------------

FrameSpatialVelocity
MultibodyModel::CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
    const MultibodyEvaluationContext& context, RigidBodyHandle body) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask how a rigid body is moving");
    Implementation::RequireOwnContext(model, &context,
                                      "read spatial velocities from");
    const int ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    const auto& V_WB_W =
        model.tree_.get_link(model.tree_body_[ordinal])
            .EvalSpatialVelocityInWorld(
                context.implementation_->tree_context());
    return MakeFrameSpatialVelocity(V_WB_W.rotational(),
                                    V_WB_W.translational());
}

FrameSpatialVelocity
MultibodyModel::CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
    const MultibodyEvaluationContext& context, FrameHandle frame) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask how a frame is moving");
    Implementation::RequireOwnContext(model, &context,
                                      "read spatial velocities from");
    const int ordinal = model.Resolve(
        frame, static_cast<int>(model.frame_names_.size()), "frame");
    const auto V_WF_W =
        model.tree_.get_frame(model.tree_frame_[ordinal])
            .CalcSpatialVelocityInWorld(
                context.implementation_->tree_context());
    return MakeFrameSpatialVelocity(V_WF_W.rotational(),
                                    V_WF_W.translational());
}

FrameSpatialVelocity
MultibodyModel::
    CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
        const MultibodyEvaluationContext& context, FrameHandle moving_frame,
        FrameHandle reference_frame, FrameHandle expressed_in_frame) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("ask how one frame is moving relative to another");
    Implementation::RequireOwnContext(model, &context,
                                      "read spatial velocities from");
    const int moving_ordinal =
        model.Resolve(moving_frame, static_cast<int>(model.frame_names_.size()),
                      "moving frame");
    const int reference_ordinal = model.Resolve(
        reference_frame, static_cast<int>(model.frame_names_.size()),
        "reference frame");
    const int expressed_ordinal = model.Resolve(
        expressed_in_frame, static_cast<int>(model.frame_names_.size()),
        "expression frame");

    const auto V_MF_E =
        model.tree_.get_frame(model.tree_frame_[moving_ordinal])
            .CalcSpatialVelocity(
                context.implementation_->tree_context(),
                model.tree_.get_frame(model.tree_frame_[reference_ordinal]),
                model.tree_.get_frame(model.tree_frame_[expressed_ordinal]));
    return MakeFrameSpatialVelocity(V_MF_E.rotational(),
                                    V_MF_E.translational());
}

// --- Differential kinematics -----------------------------------------------

void MultibodyModel::MapGeneralizedVelocitiesToPositionDerivatives(
    const MultibodyEvaluationContext& context,
    const Eigen::VectorXd& generalized_velocities,
    Eigen::VectorXd* generalized_position_derivatives) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized(
        "map generalized velocities to position derivatives");
    Implementation::RequireOwnContext(
        model, &context, "map generalized velocities in");
    RequireCoordinateCount(static_cast<int>(generalized_velocities.size()),
                           model.tree_.num_velocities(),
                           "generalized velocities");
    RequireFiniteVector(generalized_velocities, "generalized velocities");
    RequireVectorOutput(generalized_position_derivatives,
                        model.tree_.num_positions(),
                        "generalized position derivatives");
    if (&generalized_velocities == generalized_position_derivatives) {
        Reject("generalized velocities and position derivatives are the same "
               "object; in-place mapping is not supported and nothing was "
               "written");
    }

    model.tree_.MapVelocityToQDot(
        context.implementation_->tree_context().state(),
        generalized_velocities, generalized_position_derivatives);
}

void MultibodyModel::MapGeneralizedPositionDerivativesToVelocities(
    const MultibodyEvaluationContext& context,
    const Eigen::VectorXd& generalized_position_derivatives,
    Eigen::VectorXd* generalized_velocities) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized(
        "map generalized position derivatives to velocities");
    Implementation::RequireOwnContext(
        model, &context, "map generalized position derivatives in");
    RequireCoordinateCount(
        static_cast<int>(generalized_position_derivatives.size()),
        model.tree_.num_positions(), "generalized position derivatives");
    RequireFiniteVector(generalized_position_derivatives,
                        "generalized position derivatives");
    RequireVectorOutput(generalized_velocities,
                        model.tree_.num_velocities(),
                        "generalized velocities");
    if (&generalized_position_derivatives == generalized_velocities) {
        Reject("generalized position derivatives and velocities are the same "
               "object; in-place mapping is not supported and nothing was "
               "written");
    }

    model.tree_.MapQDotToVelocity(
        context.implementation_->tree_context().state(),
        generalized_position_derivatives, generalized_velocities);
}

void MultibodyModel::
    CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
        const MultibodyEvaluationContext& context,
        const Eigen::VectorXd& generalized_velocity_derivatives,
        Eigen::MatrixXd* angular_accelerations_radians_per_second_squared,
        Eigen::MatrixXd*
            translational_accelerations_at_body_origins_meters_per_second_squared)
        const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate rigid-body spatial accelerations");
    Implementation::RequireOwnContext(
        model, &context, "calculate spatial accelerations from");
    RequireCoordinateCount(
        static_cast<int>(generalized_velocity_derivatives.size()),
        model.tree_.num_velocities(), "generalized velocity derivatives");
    RequireFiniteVector(generalized_velocity_derivatives,
                        "generalized velocity derivatives");
    const int body_count = static_cast<int>(model.body_names_.size()) - 1;
    RequireMatrixOutput(angular_accelerations_radians_per_second_squared, 3,
                        body_count, "angular acceleration");
    RequireMatrixOutput(
        translational_accelerations_at_body_origins_meters_per_second_squared,
        3, body_count, "body-origin translational acceleration");
    if (angular_accelerations_radians_per_second_squared ==
        translational_accelerations_at_body_origins_meters_per_second_squared) {
        Reject("angular and translational acceleration outputs are the same "
               "object; distinct physical quantities require distinct outputs "
               "and nothing was written");
    }

    std::vector<drake::multibody::SpatialAcceleration<double>>
        accelerations_by_mobod(model.tree_.num_mobods());
    model.tree_.CalcSpatialAccelerationsFromVdot(
        context.implementation_->tree_context(),
        generalized_velocity_derivatives, &accelerations_by_mobod);

    for (int body_index = 0; body_index < body_count; ++body_index) {
        const int body_ordinal = body_index + 1;
        const auto mobod_index =
            model.tree_.get_link(model.tree_body_[body_ordinal]).mobod_index();
        const auto& acceleration = accelerations_by_mobod[mobod_index];
        angular_accelerations_radians_per_second_squared->col(body_index) =
            acceleration.rotational();
        translational_accelerations_at_body_origins_meters_per_second_squared
            ->col(body_index) = acceleration.translational();
    }
}

void MultibodyModel::
    CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
        const MultibodyEvaluationContext& context, RigidBodyHandle body,
        const Eigen::Vector3d& point_position_in_body_frame,
        Eigen::MatrixXd* angular_velocity_jacobian,
        Eigen::MatrixXd* point_translational_velocity_jacobian) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate a rigid-body point Jacobian");
    Implementation::RequireOwnContext(model, &context,
                                      "calculate a point Jacobian from");
    const int body_ordinal = model.Resolve(
        body, static_cast<int>(model.body_names_.size()), "rigid body");
    if (!point_position_in_body_frame.allFinite()) {
        Reject("body-fixed point position contains a non-finite value; nothing "
               "was written");
    }
    const int velocity_count = model.tree_.num_velocities();
    RequireMatrixOutput(angular_velocity_jacobian, 3, velocity_count,
                        "angular-velocity Jacobian");
    RequireMatrixOutput(point_translational_velocity_jacobian, 3,
                        velocity_count,
                        "point translational-velocity Jacobian");
    if (angular_velocity_jacobian == point_translational_velocity_jacobian) {
        Reject("angular and point translational Jacobian outputs are the same "
               "object; distinct physical quantities require distinct outputs "
               "and nothing was written");
    }

    Eigen::MatrixXd spatial_jacobian(6, velocity_count);
    const auto& body_frame =
        model.tree_.get_link(model.tree_body_[body_ordinal]).body_frame();
    model.tree_.CalcJacobianSpatialVelocity(
        context.implementation_->tree_context(),
        drake::multibody::JacobianWrtVariable::kV, body_frame,
        point_position_in_body_frame, model.tree_.world_frame(),
        model.tree_.world_frame(), &spatial_jacobian);
    *angular_velocity_jacobian = spatial_jacobian.topRows<3>();
    *point_translational_velocity_jacobian =
        spatial_jacobian.bottomRows<3>();
}

// --- Dynamics --------------------------------------------------------------

void MultibodyModel::CalcGeneralizedMassMatrix(
    const MultibodyEvaluationContext& context,
    Eigen::MatrixXd& generalized_mass_matrix) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate the generalized mass matrix");
    Implementation::RequireOwnContext(
        model, &context, "calculate the generalized mass matrix from");
    const int velocity_count = model.tree_.num_velocities();
    if (generalized_mass_matrix.rows() != velocity_count ||
        generalized_mass_matrix.cols() != velocity_count) {
        Reject("generalized mass matrix output must be " +
               std::to_string(velocity_count) + " x " +
               std::to_string(velocity_count) + ", but it is " +
               std::to_string(generalized_mass_matrix.rows()) + " x " +
               std::to_string(generalized_mass_matrix.cols()) +
               "; nothing was written");
    }

    model.tree_.CalcMassMatrix(context.implementation_->tree_context(),
                               &generalized_mass_matrix);
}

namespace {

void RequireDynamicsVector(const Eigen::VectorXd& vector, int expected_size,
                           const char* name) {
    RequireCoordinateCount(static_cast<int>(vector.size()), expected_size, name);
    RequireFiniteVector(vector, name);
}

void RequireDynamicsOutput(Eigen::VectorXd& output, int expected_size,
                           const char* name) {
    RequireCoordinateCount(static_cast<int>(output.size()), expected_size, name);
}

void AssembleModelAppliedForces(
    const drake::multibody::internal::MultibodyTree<double>& tree,
    const rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext&
        tree_context,
    drake::multibody::MultibodyForces<double>* forces) {
    tree.CalcForceElementsContribution(
        tree_context, tree.EvalPositionKinematics(tree_context),
        tree.EvalVelocityKinematics(tree_context), forces);
}

}  // namespace

void MultibodyModel::CalcRequiredGeneralizedForces(
    const MultibodyEvaluationContext& context,
    const Eigen::VectorXd& generalized_velocity_derivatives,
    Eigen::VectorXd& required_generalized_forces) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate required generalized forces");
    Implementation::RequireOwnContext(
        model, &context, "calculate required generalized forces from");
    const int nv = model.tree_.num_velocities();
    RequireDynamicsVector(generalized_velocity_derivatives, nv,
                          "generalized velocity derivatives");
    RequireDynamicsOutput(required_generalized_forces, nv,
                          "required generalized forces");
    if (&generalized_velocity_derivatives == &required_generalized_forces) {
        Reject("generalized velocity derivatives and required generalized "
               "forces are the same object; nothing was written");
    }

    auto& workspace = *context.implementation_;
    auto& forces = workspace.mutable_forces();
    AssembleModelAppliedForces(model.tree_, workspace.tree_context(), &forces);
    model.tree_.CalcInverseDynamics(
        workspace.tree_context(), generalized_velocity_derivatives,
        forces.body_forces(), forces.generalized_forces(),
        &workspace.mutable_accelerations(), &workspace.mutable_reactions(),
        &required_generalized_forces);
}

void MultibodyModel::CalcVelocityBiasGeneralizedForces(
    const MultibodyEvaluationContext& context,
    Eigen::VectorXd& velocity_bias_generalized_forces) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate velocity-bias generalized forces");
    Implementation::RequireOwnContext(
        model, &context, "calculate velocity-bias generalized forces from");
    const int nv = model.tree_.num_velocities();
    RequireDynamicsOutput(velocity_bias_generalized_forces, nv,
                          "velocity-bias generalized forces");

    auto& workspace = *context.implementation_;
    auto& forces = workspace.mutable_forces();
    forces.SetZero();
    model.tree_.CalcInverseDynamics(
        workspace.tree_context(), workspace.zero_velocity_derivatives(),
        forces.body_forces(), forces.generalized_forces(),
        &workspace.mutable_accelerations(), &workspace.mutable_reactions(),
        &velocity_bias_generalized_forces);
}

void MultibodyModel::CalcGravityAppliedGeneralizedForces(
    const MultibodyEvaluationContext& context,
    Eigen::VectorXd& gravity_applied_generalized_forces) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate gravity generalized forces");
    Implementation::RequireOwnContext(
        model, &context, "calculate gravity generalized forces from");
    const int nv = model.tree_.num_velocities();
    RequireDynamicsOutput(gravity_applied_generalized_forces, nv,
                          "gravity generalized forces");

    auto& workspace = *context.implementation_;
    auto& forces = workspace.mutable_forces();
    AssembleModelAppliedForces(model.tree_, workspace.tree_context(), &forces);
    gravity_applied_generalized_forces.setZero();
    model.tree_.CalcSystemJacobianTransposeTimesF(
        workspace.tree_context(), &forces.mutable_body_forces(),
        &gravity_applied_generalized_forces);
}

void MultibodyModel::CalcJointDampingAppliedGeneralizedForces(
    const MultibodyEvaluationContext& context,
    Eigen::VectorXd& damping_applied_generalized_forces) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate joint-damping generalized forces");
    Implementation::RequireOwnContext(
        model, &context, "calculate joint-damping generalized forces from");
    const int nv = model.tree_.num_velocities();
    RequireDynamicsOutput(damping_applied_generalized_forces, nv,
                          "joint-damping generalized forces");

    auto& workspace = *context.implementation_;
    auto& forces = workspace.mutable_forces();
    AssembleModelAppliedForces(model.tree_, workspace.tree_context(), &forces);
    damping_applied_generalized_forces = forces.generalized_forces();
}

std::unique_ptr<ForwardDynamicsWorkspace>
MultibodyModel::CreateForwardDynamicsWorkspace() const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("create a forward-dynamics workspace");
    auto implementation =
        std::make_unique<ForwardDynamicsWorkspace::Implementation>(
            model.identity_, model.tree_);
    return std::unique_ptr<ForwardDynamicsWorkspace>(
        new ForwardDynamicsWorkspace(std::move(implementation)));
}

const Eigen::VectorXd& MultibodyModel::EvaluateForwardDynamics(
    const MultibodyEvaluationContext& context,
    std::span<const AppliedBodyWrench> body_wrenches,
    std::span<const AppliedRevoluteJointTorque> revolute_joint_torques,
    std::span<const AppliedPrismaticJointForce> prismatic_joint_forces,
    ForwardDynamicsWorkspace& workspace) const {
    const Implementation& model = *implementation_;
    model.ThrowIfNotFinalized("calculate forward dynamics");
    Implementation::RequireOwnContext(model, &context,
                                      "calculate forward dynamics from");
    if (workspace.implementation_->issuer_ != model.identity_) {
        Reject("forward-dynamics workspace belongs to a different model");
    }

    for (const AppliedBodyWrench& wrench : body_wrenches) {
        model.Resolve(wrench.body, static_cast<int>(model.body_names_.size()),
                      "rigid body");
        model.Resolve(wrench.expressed_in_frame,
                      static_cast<int>(model.frame_names_.size()), "frame");
        if (!wrench.point_position_in_body_frame_meters.allFinite() ||
            !wrench.torque_about_point_newton_metres.allFinite() ||
            !wrench.force_newtons.allFinite()) {
            Reject("an applied body wrench contains a non-finite value");
        }
    }
    for (const AppliedRevoluteJointTorque& effort : revolute_joint_torques) {
        const int ordinal = model.Resolve(
            effort.joint, static_cast<int>(model.joint_names_.size()), "joint");
        if (model.joint_kind_[ordinal] !=
            Implementation::JointKind::kRevolute) {
            Reject("a revolute torque was applied to joint '" +
                   model.joint_names_[ordinal] + "', which is not revolute");
        }
        if (!std::isfinite(effort.torque_newton_metres)) {
            Reject("an applied revolute-joint torque is not finite");
        }
    }
    for (const AppliedPrismaticJointForce& effort : prismatic_joint_forces) {
        const int ordinal = model.Resolve(
            effort.joint, static_cast<int>(model.joint_names_.size()), "joint");
        if (model.joint_kind_[ordinal] !=
            Implementation::JointKind::kPrismatic) {
            Reject("a prismatic force was applied to joint '" +
                   model.joint_names_[ordinal] + "', which is not prismatic");
        }
        if (!std::isfinite(effort.force_newtons)) {
            Reject("an applied prismatic-joint force is not finite");
        }
    }

    const auto& tree_context = context.implementation_->tree_context();
    auto& computation = *workspace.implementation_;
    model.tree_.CalcForceElementsContribution(
        tree_context, model.tree_.EvalPositionKinematics(tree_context),
        model.tree_.EvalVelocityKinematics(tree_context), &computation.forces_);

    for (const AppliedBodyWrench& wrench : body_wrenches) {
        const int body_ordinal = HandleOrdinal(wrench.body);
        const int frame_ordinal = HandleOrdinal(wrench.expressed_in_frame);
        const auto& body =
            model.tree_.get_link(model.tree_body_[body_ordinal]);
        const auto& frame_E =
            model.tree_.get_frame(model.tree_frame_[frame_ordinal]);
        const Eigen::Matrix3d R_WB =
            body.body_frame().CalcRotationMatrixInWorld(tree_context).matrix();
        const Eigen::Matrix3d R_WE =
            frame_E.CalcRotationMatrixInWorld(tree_context).matrix();
        const Eigen::Vector3d p_BoQ_E =
            R_WE.transpose() * R_WB *
            wrench.point_position_in_body_frame_meters;
        body.AddInForce(
            tree_context, p_BoQ_E,
            drake::multibody::SpatialForce<double>(
                wrench.torque_about_point_newton_metres,
                wrench.force_newtons),
            frame_E, &computation.forces_);
    }
    for (const AppliedRevoluteJointTorque& effort : revolute_joint_torques) {
        const int ordinal = HandleOrdinal(effort.joint);
        const auto& joint = static_cast<const drake::multibody::RevoluteJoint<
            double>&>(model.tree_.get_joint(model.tree_joint_[ordinal]));
        joint.AddInTorque(tree_context.state(), effort.torque_newton_metres,
                          &computation.forces_);
    }
    for (const AppliedPrismaticJointForce& effort : prismatic_joint_forces) {
        const int ordinal = HandleOrdinal(effort.joint);
        const auto& joint = static_cast<const drake::multibody::PrismaticJoint<
            double>&>(model.tree_.get_joint(model.tree_joint_[ordinal]));
        joint.AddInForce(tree_context.state(), effort.force_newtons,
                         &computation.forces_);
    }

    model.tree_.CalcArticulatedBodyForceCache(
        tree_context, computation.forces_, &computation.aba_force_);
    model.tree_.CalcArticulatedBodyAccelerations(
        tree_context, computation.aba_force_, &computation.acceleration_);
    return computation.acceleration_.get_vdot();
}

void MultibodyModel::CalcGeneralizedVelocityDerivatives(
    const MultibodyEvaluationContext& context,
    std::span<const AppliedBodyWrench> body_wrenches,
    std::span<const AppliedRevoluteJointTorque> revolute_joint_torques,
    std::span<const AppliedPrismaticJointForce> prismatic_joint_forces,
    ForwardDynamicsWorkspace& workspace,
    Eigen::VectorXd& generalized_velocity_derivatives) const {
    const int nv = implementation_->tree_.num_velocities();
    RequireDynamicsOutput(generalized_velocity_derivatives, nv,
                          "generalized velocity derivatives");
    generalized_velocity_derivatives = EvaluateForwardDynamics(
        context, body_wrenches, revolute_joint_torques,
        prismatic_joint_forces, workspace);
}

void MultibodyModel::CalcStateTimeDerivatives(
    const MultibodyEvaluationContext& context,
    std::span<const AppliedBodyWrench> body_wrenches,
    std::span<const AppliedRevoluteJointTorque> revolute_joint_torques,
    std::span<const AppliedPrismaticJointForce> prismatic_joint_forces,
    ForwardDynamicsWorkspace& workspace,
    Eigen::VectorXd& state_time_derivatives) const {
    const int nq = implementation_->tree_.num_positions();
    const int nv = implementation_->tree_.num_velocities();
    RequireDynamicsOutput(state_time_derivatives, nq + nv,
                          "state time derivatives");
    const Eigen::VectorXd& vdot = EvaluateForwardDynamics(
        context, body_wrenches, revolute_joint_torques,
        prismatic_joint_forces, workspace);
    auto& qdot = workspace.implementation_->position_derivatives_;
    const auto& state = context.implementation_->tree_context().state();
    implementation_->tree_.MapVelocityToQDot(
        state, state.generalized_velocities(), &qdot);
    state_time_derivatives.head(nq) = qdot;
    state_time_derivatives.tail(nv) = vdot;
}

}  // namespace orvd::multibody_model
