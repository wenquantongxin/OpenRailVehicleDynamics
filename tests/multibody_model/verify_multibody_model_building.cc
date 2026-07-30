// What the modelling facade accepts, what it refuses, and when it refuses it.
//
// The refusals are the substance. A model that quietly took a duplicate name, a
// handle from another model, or a relation that closes a loop would produce
// something that builds and evaluates and is not what the caller described — and
// the difference would surface as a dynamics discrepancy a long way from the
// line that caused it.
//
// The one refusal that is not about the caller's mistake is the unrelated body.
// The rigid tree underneath will give any body without a relation a floating one
// to the world at finalization: six degrees of freedom nobody asked for. So the
// relation is stated here rather than inferred, and stating it twice, or stating
// it for a body already related, is itself refused.
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

// A handle is held and given back, and that is all. Exposing the ordinal would
// invite `handle + 1`, or a loop over a range the caller assembled, and both
// reach past the model into a layout that belongs to it.
static_assert(!std::is_convertible_v<RigidBodyHandle, int>);
static_assert(!std::is_convertible_v<int, RigidBodyHandle>);
static_assert(!std::is_constructible_v<RigidBodyHandle, int>);
static_assert(!std::is_constructible_v<
              RigidBodyHandle,
              orvd::multibody_model::internal::ModelIdentity, int>);
template <typename T>
concept HasArithmetic = requires(T a, T b) { a + b; };
static_assert(!HasArithmetic<RigidBodyHandle>);
template <typename T>
concept HasOrdering = requires(T a, T b) { a < b; };
static_assert(!HasOrdering<RigidBodyHandle>);

// Three distinct types, not three names for the same two integers. A frame
// ordinal passed where a body ordinal is expected would be a mistake the model
// could not detect: both are small non-negative numbers naming something real.
static_assert(!std::is_same_v<RigidBodyHandle, FrameHandle>);
static_assert(!std::is_convertible_v<RigidBodyHandle, FrameHandle>);
static_assert(!std::is_convertible_v<FrameHandle, JointHandle>);

// A model owns its elements and hands out handles that name it.
static_assert(!std::is_copy_constructible_v<MultibodyModel>);
static_assert(!std::is_move_constructible_v<MultibodyModel>);

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

template <typename Attempt>
bool WasRefused(Attempt&& attempt) {
    try {
        attempt();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

void ExpectRefused(bool refused, const std::string& what) {
    ExpectTrue(refused, what + ": expected a refusal, but it was accepted");
}

/// Runs `attempt` and reports whether it was refused with a message containing
/// `fragment`.
///
/// The roadmap's failure semantics ask for explicit failure, and explicit means
/// the caller is told which element and what about it. A check that only asked
/// "was it refused" would pass for a refusal that named nothing.
template <typename Attempt>
bool RefusalMentions(Attempt&& attempt, std::string_view fragment) {
    try {
        attempt();
    } catch (const std::invalid_argument& why) {
        return std::string_view(why.what()).find(fragment) !=
               std::string_view::npos;
    }
    return false;
}

void ExpectAccepted(bool refused, const std::string& what) {
    ExpectTrue(!refused, what + ": expected acceptance, but it was refused");
}

RigidBodyInertiaParameters SolidishBody(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.unit_inertia_moments = Eigen::Vector3d(0.01, 0.02, 0.02);
    return inertia;
}

const Eigen::Vector3d kZAxis = Eigen::Vector3d::UnitZ();

void CheckAModelCanBeDescribed() {
    MultibodyModel model;

    const RigidBodyHandle first = model.AddRigidBody("first", SolidishBody(1.0));
    const RigidBodyHandle second =
        model.AddRigidBody("second", SolidishBody(2.0));
    ExpectTrue(first.is_valid() && second.is_valid(),
               "adding bodies returns usable semantic handles");

    const JointHandle shoulder = model.AddRevoluteJoint(
        "shoulder", model.world_frame(), model.body_frame(first), kZAxis);
    const JointHandle elbow = model.AddRevoluteJoint(
        "elbow", model.body_frame(first), model.body_frame(second), kZAxis);
    ExpectTrue(shoulder.is_valid() && elbow.is_valid(),
               "adding joints returns usable semantic handles");
}

void CheckNamesMustBeUsableAndUnique() {
    MultibodyModel model;
    const RigidBodyHandle body = model.AddRigidBody("link", SolidishBody(1.0));

    ExpectRefused(WasRefused([&] { model.AddRigidBody("", SolidishBody(1.0)); }),
                  "a body with no name");
    ExpectRefused(
        WasRefused([&] { model.AddRigidBody("link", SolidishBody(1.0)); }),
        "a second body named 'link'");
    ExpectRefused(
        WasRefused([&] {
            FixedFramePoseParameters pose;
            model.AddFixedFrame("link", body, pose);
        }),
        "a frame taking a name a body already brought");

    FixedFramePoseParameters pose;
    model.AddFixedFrame("future_body", body, pose);
    ExpectTrue(
        RefusalMentions(
            [&] { model.AddRigidBody("future_body", SolidishBody(1.0)); },
            "frame named 'future_body'"),
        "a later body cannot bring a second frame with an existing name");

    model.AddRevoluteJoint("j", model.world_frame(), model.body_frame(body),
                           kZAxis);
    const RigidBodyHandle other = model.AddRigidBody("other", SolidishBody(1.0));
    ExpectRefused(WasRefused([&] {
                      model.AddRevoluteJoint("j", model.world_frame(),
                                             model.body_frame(other), kZAxis);
                  }),
                  "a second joint named 'j'");
}

void CheckHandlesFromElsewhereAreRefused() {
    MultibodyModel first;
    MultibodyModel second;
    const RigidBodyHandle in_first =
        first.AddRigidBody("body", SolidishBody(1.0));
    // The second model is given a body too, so the foreign handle's ordinal is
    // a real one there. Without this the handle would be caught for being out
    // of range, and the check would pass while saying nothing about whether the
    // model's identity is consulted at all.
    second.AddRigidBody("body", SolidishBody(1.0));

    // The ordinal would be a perfectly good index into the second model — which
    // is exactly why a bare integer would not do. The handle carries which model
    // it came from, so the mistake is caught instead of silently naming whatever
    // sits at that position.
    ExpectTrue(RefusalMentions([&] { (void)second.body_frame(in_first); },
                               "different model"),
               "a body handle from another model is refused as foreign, not as "
               "out of range");

    const RigidBodyHandle names_nothing;
    ExpectTrue(!names_nothing.is_valid(), "a default handle names nothing");
    ExpectRefused(WasRefused([&] { (void)first.body_frame(names_nothing); }),
                  "a handle that names nothing");
}

void CheckRelationsThatNoModelCanMean() {
    MultibodyModel model;
    const RigidBodyHandle first = model.AddRigidBody("first", SolidishBody(1.0));
    const RigidBodyHandle second =
        model.AddRigidBody("second", SolidishBody(1.0));

    // A body related to itself is refused, and the refusal names the body. That
    // it is refused is also what the loop check would do — a body always reaches
    // itself — so what distinguishes this case is that the caller is told which
    // body they related to itself rather than being told about a loop they did
    // not draw.
    ExpectTrue(RefusalMentions(
                   [&] {
                       model.AddRevoluteJoint("self", model.body_frame(first),
                                              model.body_frame(first), kZAxis);
                   },
                   "to itself"),
               "a joint from a body to itself is refused as such, not as a loop");

    ExpectRefused(WasRefused([&] {
                      model.AddRevoluteJoint("zero_axis", model.world_frame(),
                                             model.body_frame(first),
                                             Eigen::Vector3d::Zero());
                  }),
                  "a joint about a zero axis");

    model.AddRevoluteJoint("a", model.world_frame(), model.body_frame(first),
                           kZAxis);
    model.AddRevoluteJoint("b", model.body_frame(first),
                           model.body_frame(second), kZAxis);

    // first and second already reach each other, so another relation between
    // them closes a loop.
    ExpectRefused(WasRefused([&] {
                      model.AddRevoluteJoint("loop", model.body_frame(second),
                                             model.body_frame(first), kZAxis);
                  }),
                  "a second relation between two bodies already connected");
    ExpectRefused(WasRefused([&] {
                      model.AddWeldJoint("loop_to_world", model.world_frame(),
                                         model.body_frame(second));
                  }),
                  "a relation to the world from a body that already reaches it");
}

void CheckBranchingAndRepeatedBodiesAreFine() {
    // A body may appear in any number of relations, and a model may branch. The
    // constraint is one path to the world, not one relation per body: which side
    // of a joint the caller called the parent fixes the joint's coordinate and
    // the sign of its angle, and says nothing about which body the rigid tree
    // will treat as inboard.
    MultibodyModel model;
    const RigidBodyHandle hub = model.AddRigidBody("hub", SolidishBody(1.0));
    const RigidBodyHandle left = model.AddRigidBody("left", SolidishBody(1.0));
    const RigidBodyHandle right = model.AddRigidBody("right", SolidishBody(1.0));

    ExpectAccepted(WasRefused([&] {
                       model.AddRevoluteJoint("to_hub", model.world_frame(),
                                              model.body_frame(hub), kZAxis);
                   }),
                   "a joint from the world to the hub");
    ExpectAccepted(WasRefused([&] {
                       model.AddRevoluteJoint("hub_left",
                                              model.body_frame(hub),
                                              model.body_frame(left), kZAxis);
                   }),
                   "a first branch off the hub");
    ExpectAccepted(WasRefused([&] {
                       model.AddPrismaticJoint("hub_right",
                                               model.body_frame(hub),
                                               model.body_frame(right), kZAxis);
                   }),
                   "a second branch off the same hub");

    // The world as the child of a joint: legitimate, and the arrangement the
    // rigid tree grows from it is its own business.
    MultibodyModel backwards;
    const RigidBodyHandle body =
        backwards.AddRigidBody("attached_backwards", SolidishBody(1.0));
    ExpectAccepted(WasRefused([&] {
                       backwards.AddRevoluteJoint(
                           "towards_world", backwards.body_frame(body),
                           backwards.world_frame(), kZAxis);
                   }),
                   "a joint declared with the world as its child");
}

void CheckPublicNamesDoNotCollideWithImplementationFrames() {
    // The landed tree's body-plus-offset convenience overload would create
    // hidden frames named after the joint. A public FrameHandle must instead
    // denote the one real frame in the tree, or this otherwise valid body name
    // collides with an implementation detail.
    MultibodyModel model;
    const RigidBodyHandle child =
        model.AddRigidBody("hinge_child", SolidishBody(1.0));
    ExpectAccepted(
        WasRefused([&] {
            model.AddRevoluteJoint("hinge", model.world_frame(),
                                   model.body_frame(child), kZAxis);
        }),
        "a body name that resembles an implementation joint-frame name");
}

void CheckFreedomIsStatedRatherThanInferred() {
    MultibodyModel model;
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    const RigidBodyHandle jointed =
        model.AddRigidBody("jointed", SolidishBody(1.0));

    model.DeclareFreeBody(floating);

    ExpectRefused(WasRefused([&] { model.DeclareFreeBody(floating); }),
                  "declaring the same body free twice");

    model.AddRevoluteJoint("j", model.world_frame(),
                           model.body_frame(jointed), kZAxis);
    ExpectRefused(WasRefused([&] { model.DeclareFreeBody(jointed); }),
                  "declaring a body free when a joint already relates it");

    // A free body is a root, not a leaf: things may hang off it.
    const RigidBodyHandle hanging =
        model.AddRigidBody("hanging", SolidishBody(1.0));
    ExpectAccepted(WasRefused([&] {
                       model.AddRevoluteJoint("off_the_free_body",
                                              model.body_frame(floating),
                                              model.body_frame(hanging),
                                              kZAxis);
                   }),
                   "a joint outboard of a free body");

    // An internal quaternion relation has no public JointHandle and therefore
    // cannot reserve a caller's joint name. This component is not yet attached
    // to World, so declaring its root free is topologically valid.
    MultibodyModel collision;
    const RigidBodyHandle root =
        collision.AddRigidBody("root", SolidishBody(1.0));
    const RigidBodyHandle child =
        collision.AddRigidBody("child", SolidishBody(1.0));
    collision.AddRevoluteJoint("free_root", collision.body_frame(root),
                               collision.body_frame(child), kZAxis);
    ExpectAccepted(WasRefused([&] { collision.DeclareFreeBody(root); }),
                   "a free declaration after a caller used its obvious name");
}

void CheckPhysicallyImpossibleDescriptionsAreRefused() {
    MultibodyModel model;

    RigidBodyInertiaParameters impossible = SolidishBody(1.0);
    // Principal moments of (-9, 1, 11): no distribution of mass has one that is
    // negative.
    impossible.unit_inertia_moments = Eigen::Vector3d(1.0, 1.0, 1.0);
    impossible.unit_inertia_products = Eigen::Vector3d(10.0, 0.0, 0.0);
    ExpectRefused(
        WasRefused([&] { model.AddRigidBody("impossible", impossible); }),
        "a body whose inertia no real mass distribution has");

    const RigidBodyHandle body = model.AddRigidBody("body", SolidishBody(1.0));
    FixedFramePoseParameters not_a_rotation;
    not_a_rotation.R_PF(0, 1) = 0.5;
    ExpectRefused(
        WasRefused([&] { model.AddFixedFrame("f", body, not_a_rotation); }),
        "a fixed frame whose rotation is not a rotation");

    FixedFramePoseParameters infinite_translation;
    infinite_translation.p_PoFo_P.x() =
        std::numeric_limits<double>::infinity();
    ExpectRefused(
        WasRefused(
            [&] { model.AddFixedFrame("infinite", body, infinite_translation); }),
        "a fixed frame with a non-finite translation");
}

void CheckEveryFiniteNonzeroAxisIsAUsableDirection() {
    MultibodyModel model;
    const RigidBodyHandle large_axis_body =
        model.AddRigidBody("large_axis_body", SolidishBody(1.0));
    const RigidBodyHandle small_axis_body =
        model.AddRigidBody("small_axis_body", SolidishBody(1.0));
    const double largest = std::numeric_limits<double>::max();
    ExpectAccepted(
        WasRefused([&] {
            model.AddRevoluteJoint(
                "large_axis", model.world_frame(),
                model.body_frame(large_axis_body),
                Eigen::Vector3d(largest, largest, 0.0));
        }),
        "a large finite nonzero axis whose direct norm would overflow");
    const double smallest = std::numeric_limits<double>::denorm_min();
    ExpectAccepted(
        WasRefused([&] {
            model.AddPrismaticJoint(
                "small_axis", model.world_frame(),
                model.body_frame(small_axis_body),
                Eigen::Vector3d(smallest, 0.0, 0.0));
        }),
        "a small finite nonzero axis whose direct norm would underflow");
}

void CheckAFixedFrameIsUsableAsAJointEndpoint() {
    MultibodyModel model;
    const RigidBodyHandle body = model.AddRigidBody("body", SolidishBody(1.0));
    FixedFramePoseParameters offset;
    offset.p_PoFo_P = Eigen::Vector3d(0.3, 0.0, 0.0);
    const FrameHandle mount = model.AddFixedFrame("mount", body, offset);
    ExpectTrue(mount.is_valid(), "adding a fixed frame returns a usable handle");

    ExpectAccepted(WasRefused([&] {
                       model.AddRevoluteJoint("at_the_mount",
                                              model.world_frame(), mount,
                                              kZAxis);
                   }),
                   "a joint attached at a fixed frame");
}

}  // namespace

int main() {
    CheckAModelCanBeDescribed();
    CheckNamesMustBeUsableAndUnique();
    CheckHandlesFromElsewhereAreRefused();
    CheckRelationsThatNoModelCanMean();
    CheckBranchingAndRepeatedBodiesAreFine();
    CheckPublicNamesDoNotCollideWithImplementationFrames();
    CheckFreedomIsStatedRatherThanInferred();
    CheckPhysicallyImpossibleDescriptionsAreRefused();
    CheckEveryFiniteNonzeroAxisIsAUsableDirection();
    CheckAFixedFrameIsUsableAsAJointEndpoint();

    if (failure_count > 0) {
        std::printf("%d model building check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "the modelling facade takes what a caller describes and refuses what no"
        " model can mean, naming the element it refused\n");
    return 0;
}
