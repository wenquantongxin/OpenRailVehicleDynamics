// Where the model says things are, checked against where they have to be.
//
// Every expected pose here is written out by hand from the relations that were
// stated — a product of elementary rotations and offsets, composed in the order
// the chain was described. Nothing is compared against a second call into the
// same code, and no number was read out of an implementation and pasted back
// in: an expectation obtained that way agrees with whatever the code does,
// including the wrong thing.
//
// The sign checks are the reason this file has to exist at all. Which frame a
// caller calls the parent fixes the sign of the joint's coordinate, and until
// there is a public way to see where a body ended up, that statement cannot be
// checked from outside — a model with the two frames swapped builds, finalizes
// and reports the same counts.
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyEvaluationContext;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_model::RigidPose;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

// A pose is produced by the model, never supplied to it. There is no
// configuration a caller can name that the model did not compute.
static_assert(!std::is_default_constructible_v<RigidPose>);
static_assert(!std::is_constructible_v<RigidPose, Eigen::Matrix3d,
                                       Eigen::Vector3d>);

// Only a pose the caller is holding hands out a reference. Every rvalue
// category returns by value, because a query returns a pose by value and
// `const auto& R = model.CalcPoseInWorld(...).rotation();` would otherwise
// reference a member of something that is already gone. `const&&` is not
// redundant with `&&`: without it a `const` rvalue resolves to the `const&`
// overload and dangles exactly as before.
//
// Named one overload at a time, by taking a pointer to each. Asking what a call
// returns cannot tell three overloads from two: delete the `&&` one and an
// rvalue simply binds to `const&&`, which also returns by value, so every
// return type stays what it was and nothing notices that an overload went
// missing. A pointer to a particular ref-qualified member does notice.
// The cast is the check: naming an overload that is not there is what fails,
// and it fails at compile time. Comparing the resulting pointer against null
// would add nothing — a pointer obtained this way cannot be null, which GCC
// says out loud — so nothing is compared.
[[maybe_unused]] constexpr auto kRotationOnLvalue =
    static_cast<const Eigen::Matrix3d& (RigidPose::*)() const&>(
        &RigidPose::rotation);
[[maybe_unused]] constexpr auto kRotationOnRvalue =
    static_cast<Eigen::Matrix3d (RigidPose::*)() &&>(&RigidPose::rotation);
[[maybe_unused]] constexpr auto kRotationOnConstRvalue =
    static_cast<Eigen::Matrix3d (RigidPose::*)() const&&>(&RigidPose::rotation);
[[maybe_unused]] constexpr auto kTranslationOnLvalue =
    static_cast<const Eigen::Vector3d& (RigidPose::*)() const&>(
        &RigidPose::translation);
[[maybe_unused]] constexpr auto kTranslationOnRvalue =
    static_cast<Eigen::Vector3d (RigidPose::*)() &&>(&RigidPose::translation);
[[maybe_unused]] constexpr auto kTranslationOnConstRvalue =
    static_cast<Eigen::Vector3d (RigidPose::*)() const&&>(
        &RigidPose::translation);

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

// A composition of a few elementary rotations and offsets carries a few units
// of round-off and no more. This is the same bound the pose-composition
// contract uses; it is not a fitted number and nothing here needs it loosened.
constexpr double kFewUlps = 64 * std::numeric_limits<double>::epsilon();

void ExpectPoseIs(const RigidPose& actual, const Eigen::Matrix3d& rotation,
                  const Eigen::Vector3d& translation, const std::string& what) {
    const double rotation_error = (actual.rotation() - rotation).cwiseAbs().maxCoeff();
    const double translation_error =
        (actual.translation() - translation).cwiseAbs().maxCoeff();
    // Scaled by how big the answer is: a translation of ten metres cannot be
    // asked to agree to the same absolute figure as one of a millimetre.
    const double translation_bound =
        kFewUlps * std::max(1.0, translation.cwiseAbs().maxCoeff());
    ExpectTrue(rotation_error <= kFewUlps,
               what + ": rotation is off by " + std::to_string(rotation_error));
    ExpectTrue(translation_error <= translation_bound,
               what + ": translation is off by " +
                   std::to_string(translation_error));
}

template <typename Attempt>
bool RefusalMentions(Attempt&& attempt, std::string_view fragment) {
    try {
        attempt();
    } catch (const std::exception& why) {
        return std::string_view(why.what()).find(fragment) !=
               std::string_view::npos;
    }
    return false;
}

RigidBodyInertiaParameters SolidishBody(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.unit_inertia_moments = Eigen::Vector3d(0.01, 0.02, 0.02);
    return inertia;
}

Eigen::Matrix3d RotationAboutX(double angle) {
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0, 0.0, 0.0, std::cos(angle), -std::sin(angle), 0.0,
        std::sin(angle), std::cos(angle);
    return rotation;
}

Eigen::Matrix3d RotationAboutY(double angle) {
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), 0.0, std::sin(angle), 0.0, 1.0, 0.0,
        -std::sin(angle), 0.0, std::cos(angle);
    return rotation;
}

Eigen::Matrix3d RotationAboutZ(double angle) {
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), -std::sin(angle), 0.0, std::sin(angle),
        std::cos(angle), 0.0, 0.0, 0.0, 1.0;
    return rotation;
}

const Eigen::Vector3d kXAxis = Eigen::Vector3d::UnitX();
const Eigen::Vector3d kZAxis = Eigen::Vector3d::UnitZ();

/// Puts one value into the whole-model q, at the place the model says it goes.
void PlaceJointPosition(const MultibodyModel& model, JointHandle joint,
                        double value, Eigen::VectorXd* positions) {
    (*positions)[model.GetJointPositionRange(joint).start()] = value;
}

// --- An analytic chain -------------------------------------------------------

void CheckAChainOfStatedRelations() {
    // world --revolute z--> arm --(fixed frame at +x)--> elbow
    //       --revolute z--> fore --prismatic x--> slider --weld--> tip
    //
    // Every pose below is the product of what was stated, in the order it was
    // stated. Nothing is read back from the model to build an expectation.
    constexpr double kArmLength = 0.4;
    constexpr double kFirstAngle = 0.37;
    constexpr double kSecondAngle = -0.81;
    constexpr double kSlide = 0.23;

    MultibodyModel model;
    const RigidBodyHandle arm = model.AddRigidBody("arm", SolidishBody(1.0));
    const RigidBodyHandle fore = model.AddRigidBody("fore", SolidishBody(1.0));
    const RigidBodyHandle slider =
        model.AddRigidBody("slider", SolidishBody(1.0));
    const RigidBodyHandle tip = model.AddRigidBody("tip", SolidishBody(1.0));

    const JointHandle shoulder = model.AddRevoluteJoint(
        "shoulder", model.world_frame(), model.body_frame(arm), kZAxis);

    FixedFramePoseParameters elbow_pose;
    elbow_pose.p_PoFo_P = Eigen::Vector3d(kArmLength, 0.0, 0.0);
    const FrameHandle elbow = model.AddFixedFrame("elbow", arm, elbow_pose);

    const JointHandle forearm =
        model.AddRevoluteJoint("forearm", elbow, model.body_frame(fore), kZAxis);
    const JointHandle rail = model.AddPrismaticJoint(
        "rail", model.body_frame(fore), model.body_frame(slider), kXAxis);
    model.AddWeldJoint("tip_weld", model.body_frame(slider),
                       model.body_frame(tip));
    model.Finalize();

    const std::unique_ptr<MultibodyEvaluationContext> context =
        model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    PlaceJointPosition(model, shoulder, kFirstAngle, &positions);
    PlaceJointPosition(model, forearm, kSecondAngle, &positions);
    PlaceJointPosition(model, rail, kSlide, &positions);
    model.SetGeneralizedPositions(context.get(), positions);

    const Eigen::Matrix3d R_W_arm = RotationAboutZ(kFirstAngle);
    ExpectPoseIs(model.CalcPoseInWorld(*context, arm), R_W_arm,
                 Eigen::Vector3d::Zero(), "the arm turns about the world's z");

    const Eigen::Vector3d p_W_elbow = R_W_arm * elbow_pose.p_PoFo_P;
    ExpectPoseIs(model.CalcPoseInWorld(*context, elbow), R_W_arm, p_W_elbow,
                 "the fixed frame rides the arm it is fixed to");

    const Eigen::Matrix3d R_W_fore = R_W_arm * RotationAboutZ(kSecondAngle);
    ExpectPoseIs(model.CalcPoseInWorld(*context, fore), R_W_fore, p_W_elbow,
                 "the second link turns about the frame it hangs from");

    const Eigen::Vector3d p_W_slider =
        p_W_elbow + R_W_fore * Eigen::Vector3d(kSlide, 0.0, 0.0);
    ExpectPoseIs(model.CalcPoseInWorld(*context, slider), R_W_fore, p_W_slider,
                 "the slider slides along its parent's x, not the world's");

    ExpectPoseIs(model.CalcPoseInWorld(*context, tip), R_W_fore, p_W_slider,
                 "a weld holds the two frames together exactly");

    // The world frame is where everything else is measured from.
    ExpectPoseIs(model.CalcPoseInWorld(*context, model.world_frame()),
                 Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                 "the world frame's own pose");

    // Reading straight off the returned temporary, which is how a caller will
    // most often write it. The value has to survive the end of the statement.
    const Eigen::Matrix3d rotation_from_temporary =
        model.CalcPoseInWorld(*context, arm).rotation();
    const Eigen::Vector3d translation_from_temporary =
        model.CalcPoseInWorld(*context, slider).translation();
    ExpectTrue(rotation_from_temporary == R_W_arm,
               "a rotation read off the returned temporary is the rotation");
    ExpectTrue(translation_from_temporary == p_W_slider,
               "and a translation read off one is the translation");
}

void CheckANonIdentityFixedFrameRotationEntersTheChain() {
    // A mount that is turned, not merely displaced. Everything outboard of it
    // inherits that turn — including the direction its joint axis points, which
    // is stated in the mount's frame and therefore is not the world's z.
    constexpr double kMountTilt = 0.53;
    constexpr double kHingeAngle = 0.29;
    const Eigen::Vector3d kMountOffset(0.11, -0.22, 0.33);

    MultibodyModel model;
    const RigidBodyHandle base = model.AddRigidBody("base", SolidishBody(1.0));
    const RigidBodyHandle hung = model.AddRigidBody("hung", SolidishBody(1.0));
    model.AddWeldJoint("base_weld", model.world_frame(),
                       model.body_frame(base));

    FixedFramePoseParameters mount_pose;
    mount_pose.R_PF = RotationAboutY(kMountTilt);
    mount_pose.p_PoFo_P = kMountOffset;
    const FrameHandle mount = model.AddFixedFrame("mount", base, mount_pose);

    const JointHandle hinge =
        model.AddRevoluteJoint("hinge", mount, model.body_frame(hung), kZAxis);
    model.Finalize();

    const std::unique_ptr<MultibodyEvaluationContext> context =
        model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    PlaceJointPosition(model, hinge, kHingeAngle, &positions);
    model.SetGeneralizedPositions(context.get(), positions);

    const Eigen::Matrix3d R_W_mount = RotationAboutY(kMountTilt);
    ExpectPoseIs(model.CalcPoseInWorld(*context, mount), R_W_mount, kMountOffset,
                 "the mount carries the rotation it was given");

    // The hinge turns about the mount's z, which in the world is a tilted axis.
    // Had the rotation been dropped anywhere along the way, this would come out
    // as a turn about the world's z instead — a different matrix entirely.
    const Eigen::Matrix3d R_W_hung = R_W_mount * RotationAboutZ(kHingeAngle);
    ExpectPoseIs(model.CalcPoseInWorld(*context, hung), R_W_hung, kMountOffset,
                 "the hinge turns about its parent frame's z, tilted and all");
    ExpectTrue((R_W_hung - RotationAboutZ(kHingeAngle)).cwiseAbs().maxCoeff() >
                   0.1,
               "and that is a different rotation from a turn about the world's "
               "z, so the check is not vacuous");
}

// --- The sign of a coordinate ------------------------------------------------

void CheckWhichFrameIsTheParentFixesTheSign() {
    // The same physical relation, stated both ways round. A positive coordinate
    // has to move the two models in opposite senses; if it did not, the two
    // arguments would be interchangeable and the documented meaning of parent
    // and child would be a statement about nothing.
    constexpr double kAngle = 0.41;
    constexpr double kSlide = 0.17;

    const auto pose_of_pair = [](bool swapped, bool prismatic, double value) {
        MultibodyModel model;
        const RigidBodyHandle first =
            model.AddRigidBody("first", SolidishBody(1.0));
        const RigidBodyHandle second =
            model.AddRigidBody("second", SolidishBody(1.0));
        model.AddWeldJoint("anchor", model.world_frame(),
                           model.body_frame(first));
        const FrameHandle parent = swapped ? model.body_frame(second)
                                           : model.body_frame(first);
        const FrameHandle child = swapped ? model.body_frame(first)
                                          : model.body_frame(second);
        const JointHandle joint =
            prismatic ? model.AddPrismaticJoint("relation", parent, child, kXAxis)
                      : model.AddRevoluteJoint("relation", parent, child, kZAxis);
        model.Finalize();
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd positions =
            Eigen::VectorXd::Zero(model.num_generalized_positions());
        PlaceJointPosition(model, joint, value, &positions);
        model.SetGeneralizedPositions(context.get(), positions);
        return model.CalcPoseInWorld(*context, second);
    };

    // Revolute: as written, `second` turns by +angle about z. Swapped, the
    // relation says `second` is the parent, so the forest reaches `second`
    // through the joint from the child side and the same positive coordinate
    // turns it the other way.
    const RigidPose as_written = pose_of_pair(false, false, kAngle);
    const RigidPose swapped = pose_of_pair(true, false, kAngle);
    ExpectPoseIs(as_written, RotationAboutZ(kAngle), Eigen::Vector3d::Zero(),
                 "revolute as written: a positive angle turns the child "
                 "positively");
    ExpectPoseIs(swapped, RotationAboutZ(-kAngle), Eigen::Vector3d::Zero(),
                 "revolute swapped: the same positive angle turns it the other "
                 "way, because the tree now traverses the joint in reverse");

    const RigidPose slid = pose_of_pair(false, true, kSlide);
    const RigidPose slid_swapped = pose_of_pair(true, true, kSlide);
    ExpectPoseIs(slid, Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d(kSlide, 0.0, 0.0),
                 "prismatic as written: a positive displacement moves the child "
                 "along +x");
    ExpectPoseIs(slid_swapped, Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d(-kSlide, 0.0, 0.0),
                 "prismatic swapped: the same positive displacement moves it "
                 "along −x");
}

void CheckARelationStatedTowardsTheWorldIsTraversedInReverse() {
    // The world as the child. There is nowhere for the forest to start but the
    // world, so it has to grow through this joint backwards — the case that
    // makes a mobilizer reversed, and the one the sign convention has to
    // survive rather than be defined by.
    constexpr double kAngle = 0.63;

    MultibodyModel model;
    const RigidBodyHandle backwards =
        model.AddRigidBody("backwards", SolidishBody(1.0));
    const JointHandle joint =
        model.AddRevoluteJoint("towards_world", model.body_frame(backwards),
                               model.world_frame(), kZAxis);
    model.Finalize();

    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    PlaceJointPosition(model, joint, kAngle, &positions);
    model.SetGeneralizedPositions(context.get(), positions);

    // The joint says: the world, as seen from `backwards`, is turned by +angle
    // about `backwards`'s z. So `backwards`, as seen from the world, is turned
    // by −angle.
    ExpectPoseIs(model.CalcPoseInWorld(*context, backwards),
                 RotationAboutZ(-kAngle), Eigen::Vector3d::Zero(),
                 "a relation stated towards the world puts the body at minus "
                 "the coordinate");
}

void CheckANegativeAxisKeepsItsDirection() {
    // An axis is a direction, not merely a line. Normalising it may change its
    // magnitude but must not discard its sign: with -z as the stated axis, a
    // positive coordinate is a negative turn about z.
    constexpr double kAngle = 0.47;

    MultibodyModel model;
    const RigidBodyHandle arm =
        model.AddRigidBody("negative_axis_arm", SolidishBody(1.0));
    const JointHandle hinge = model.AddRevoluteJoint(
        "negative_axis_hinge", model.world_frame(), model.body_frame(arm),
        -kZAxis);
    model.Finalize();

    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    PlaceJointPosition(model, hinge, kAngle, &positions);
    model.SetGeneralizedPositions(context.get(), positions);

    ExpectPoseIs(model.CalcPoseInWorld(*context, arm),
                 RotationAboutZ(-kAngle), Eigen::Vector3d::Zero(),
                 "a negative joint axis keeps its direction during "
                 "normalisation");
}

// --- Free bodies and quaternions ---------------------------------------------

void CheckAFreeBodyIsPlacedByItsOwnCoordinates() {
    constexpr double kTilt = 0.71;
    const Eigen::Vector3d kWhere(1.5, -2.25, 0.75);

    MultibodyModel model;
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    model.DeclareFreeBody(floating);
    model.Finalize();

    auto context = model.CreateDefaultContext();
    const auto range = model.GetFreeBodyPositionRange(floating);
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    // w, x, y, z then the origin's place in the world — the layout the public
    // header states.
    positions[range.start() + 0] = std::cos(kTilt / 2.0);
    positions[range.start() + 1] = std::sin(kTilt / 2.0);
    positions.segment<3>(range.start() + 4) = kWhere;
    model.SetGeneralizedPositions(context.get(), positions);

    ExpectPoseIs(model.CalcPoseInWorld(*context, floating), RotationAboutX(kTilt),
                 kWhere,
                 "a free body sits where its own seven coordinates put it");
}

void CheckASafeNonUnitQuaternionIsUsedAndNotRewritten() {
    constexpr double kTilt = 0.44;
    constexpr double kScale = 3.0;

    MultibodyModel model;
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    model.DeclareFreeBody(floating);
    model.Finalize();

    auto context = model.CreateDefaultContext();
    const auto range = model.GetFreeBodyPositionRange(floating);
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    positions[range.start() + 0] = kScale * std::cos(kTilt / 2.0);
    positions[range.start() + 1] = kScale * std::sin(kTilt / 2.0);
    model.SetGeneralizedPositions(context.get(), positions);

    ExpectPoseIs(model.CalcPoseInWorld(*context, floating), RotationAboutX(kTilt),
                 Eigen::Vector3d::Zero(),
                 "a quaternion scaled by three denotes the same rotation");

    // And the numbers that were written are the numbers that are there. An
    // implementation that normalised in place would give the same pose and a
    // caller would never learn that what they read back is not what they wrote.
    ExpectTrue(context->generalized_positions()[range.start()] ==
                   kScale * std::cos(kTilt / 2.0),
               "the quaternion was stored exactly as written, not normalised "
               "back into the state");
}

// --- What a write does and does not disturb ----------------------------------

void CheckAVelocityWriteLeavesThePositionsWhereTheyWere() {
    constexpr double kAngle = 0.35;

    MultibodyModel model;
    const RigidBodyHandle arm = model.AddRigidBody("arm", SolidishBody(1.0));
    const JointHandle hinge = model.AddRevoluteJoint(
        "hinge", model.world_frame(), model.body_frame(arm), kZAxis);
    model.Finalize();

    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    PlaceJointPosition(model, hinge, kAngle, &positions);
    model.SetGeneralizedPositions(context.get(), positions);

    const RigidPose before = model.CalcPoseInWorld(*context, arm);
    Eigen::VectorXd velocities =
        Eigen::VectorXd::Constant(model.num_generalized_velocities(), 2.5);
    model.SetGeneralizedVelocities(context.get(), velocities);
    const RigidPose after = model.CalcPoseInWorld(*context, arm);

    // Bit for bit, not near: a velocity has no bearing on where anything is, so
    // there is nothing here for round-off to come from.
    ExpectTrue(before.rotation() == after.rotation() &&
                   before.translation() == after.translation(),
               "writing velocities leaves the positions' answer identical");

    // And a position write does change it, so the check above is not passing
    // because nothing ever changes.
    PlaceJointPosition(model, hinge, kAngle + 0.2, &positions);
    model.SetGeneralizedPositions(context.get(), positions);
    const RigidPose moved = model.CalcPoseInWorld(*context, arm);
    ExpectTrue(moved.rotation() != before.rotation(),
               "and a position write does move it");
}

// --- Ownership ---------------------------------------------------------------

void CheckAContextFromAnotherModelIsRefused() {
    MultibodyModel described;
    const RigidBodyHandle body =
        described.AddRigidBody("body", SolidishBody(1.0));
    described.DeclareFreeBody(body);
    described.Finalize();

    MultibodyModel other;
    const RigidBodyHandle other_body =
        other.AddRigidBody("body", SolidishBody(1.0));
    other.DeclareFreeBody(other_body);
    other.Finalize();
    const std::unique_ptr<MultibodyEvaluationContext> foreign =
        other.CreateDefaultContext();

    // Same shape, same names, same coordinate count: nothing but the identity
    // distinguishes them, which is exactly the case a size check would miss.
    ExpectTrue(RefusalMentions(
                   [&] { (void)described.CalcPoseInWorld(*foreign, body); },
                   "issued by a different model"),
               "reading a pose from another model's context is refused as "
               "foreign");
    ExpectTrue(RefusalMentions(
                   [&] {
                       Eigen::VectorXd positions = Eigen::VectorXd::Zero(
                           described.num_generalized_positions());
                       positions[0] = 1.0;
                       described.SetGeneralizedPositions(foreign.get(),
                                                         positions);
                   },
                   "issued by a different model"),
               "writing positions into another model's context is refused");
}

void CheckACoordinateVectorOfTheWrongSizeIsRefused() {
    MultibodyModel model;
    const RigidBodyHandle arm = model.AddRigidBody("arm", SolidishBody(1.0));
    model.AddRevoluteJoint("hinge", model.world_frame(),
                           model.body_frame(arm), kZAxis);
    model.Finalize();
    auto context = model.CreateDefaultContext();

    ExpectTrue(RefusalMentions(
                   [&] {
                       model.SetGeneralizedPositions(context.get(),
                                                     Eigen::VectorXd::Zero(4));
                   },
                   "nothing was written"),
               "a q of the wrong size is refused and says nothing was written");
    ExpectTrue(RefusalMentions(
                   [&] {
                       model.SetGeneralizedVelocities(context.get(),
                                                      Eigen::VectorXd::Zero(7));
                   },
                   "nothing was written"),
               "a v of the wrong size is refused");
}

}  // namespace

int main() {
    CheckAChainOfStatedRelations();
    CheckANonIdentityFixedFrameRotationEntersTheChain();
    CheckWhichFrameIsTheParentFixesTheSign();
    CheckARelationStatedTowardsTheWorldIsTraversedInReverse();
    CheckANegativeAxisKeepsItsDirection();
    CheckAFreeBodyIsPlacedByItsOwnCoordinates();
    CheckASafeNonUnitQuaternionIsUsedAndNotRewritten();
    CheckAVelocityWriteLeavesThePositionsWhereTheyWere();
    CheckAContextFromAnotherModelIsRefused();
    CheckACoordinateVectorOfTheWrongSizeIsRefused();

    if (failure_count > 0) {
        std::printf("%d position-kinematics check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("the model's poses are the poses its relations describe\n");
    return 0;
}
