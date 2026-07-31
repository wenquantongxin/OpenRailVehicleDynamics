// Finalization, and what a finalized model will answer.
//
// A model has two lives, and the line between them is the substance here. While
// it is described it takes elements and answers nothing; once finalized it takes
// nothing and answers everything. Both halves of that are checked, because a
// query that worked mid-description would be answering "what has been said so
// far" — a different question, whose answer the next call could change — and an
// Add that worked after finalization would be adding to a model whose
// coordinates had already been handed out.
//
// The load-bearing check is the tiling one. Every generalized coordinate has to
// belong to exactly one thing the caller declared. The rigid tree underneath
// will happily give an unrelated body a floating joint and carry on; if it ever
// did, the extra six velocities would belong to no public joint and no declared
// free body, and the tiling check is what notices. The scenarios it runs over
// are the topology contract's own — serial, branched, welded, free, mixed and
// reversed — restated through the public facade, so the counts G14 pins at the
// forest level are the counts a caller sees.
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::GeneralizedPositionRange;
using orvd::multibody_model::GeneralizedVelocityRange;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyEvaluationContext;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

// Two contexts on one model are meant to be independent, and a context is meant
// to come from a model. Neither is a convention: both are the type's.
static_assert(!std::is_copy_constructible_v<MultibodyEvaluationContext>);
static_assert(!std::is_move_constructible_v<MultibodyEvaluationContext>);
static_assert(!std::is_default_constructible_v<MultibodyEvaluationContext>);

// A position range is not a velocity range. They are not even the same size for
// the same element: a free body has seven positions and six velocities.
static_assert(!std::is_same_v<GeneralizedPositionRange,
                              GeneralizedVelocityRange>);
static_assert(
    !std::is_convertible_v<GeneralizedPositionRange, GeneralizedVelocityRange>);
// And neither is something a caller assembles: a range a caller could build
// would name a span of q or v that the model never assigned to anything.
static_assert(!std::is_constructible_v<GeneralizedPositionRange, int, int>);
static_assert(!std::is_default_constructible_v<GeneralizedPositionRange>);

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

void ExpectEqual(const std::string& what, int actual, int expected) {
    ExpectTrue(actual == expected, what + ": got " + std::to_string(actual) +
                                       ", expected " +
                                       std::to_string(expected));
}

/// Runs `attempt` and reports whether it was refused with a message containing
/// `fragment`.
///
/// The fragment matters. A model that refused everything with one blank message
/// would pass a check that only asked whether an exception came out, and the
/// caller would still not know which element was wrong or why.
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

template <typename Attempt>
bool WasRefused(Attempt&& attempt) {
    try {
        attempt();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

RigidBodyInertiaParameters SolidishBody(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.unit_inertia_moments = Eigen::Vector3d(0.01, 0.02, 0.02);
    return inertia;
}

const Eigen::Vector3d kZAxis = Eigen::Vector3d::UnitZ();
const Eigen::Vector3d kXAxis = Eigen::Vector3d::UnitX();

/// Checks that the declared elements' coordinate ranges tile q and v exactly.
///
/// Three ways this can fail, and each is a different defect. A coordinate
/// claimed twice means two elements were given the same place. A coordinate
/// claimed by nobody means the model has a degree of freedom no caller asked
/// for — which is precisely what the rigid tree does to an unrelated body, and
/// precisely what the facade exists to prevent. A claim outside the range means
/// the model reported a place that is not in its own state.
void ExpectCoordinatesAreTiled(const MultibodyModel& model,
                               const std::string& scenario) {
    std::vector<std::string> position_owner(
        static_cast<std::size_t>(model.num_generalized_positions()));
    std::vector<std::string> velocity_owner(
        static_cast<std::size_t>(model.num_generalized_velocities()));

    const auto claim = [&](std::vector<std::string>& owners, int start,
                           int size, const std::string& who) {
        // Checked before the loop, because for a weld there is no loop: a
        // zero-length range still reports a place, and an unchecked one could
        // be anything at all. One past the end is legitimate — nothing begins
        // there — and anything outside that is a place this model does not have.
        ExpectTrue(start >= 0 && start <= static_cast<int>(owners.size()),
                   scenario + ": " + who + " begins at " +
                       std::to_string(start) +
                       ", which is not a place in this model's " +
                       std::to_string(owners.size()));
        for (int offset = 0; offset < size; ++offset) {
            const int at = start + offset;
            if (at < 0 || at >= static_cast<int>(owners.size())) {
                ExpectTrue(false, scenario + ": " + who + " claims coordinate " +
                                      std::to_string(at) +
                                      ", which is outside this model's " +
                                      std::to_string(owners.size()));
                continue;
            }
            std::string& owner = owners[static_cast<std::size_t>(at)];
            ExpectTrue(owner.empty(), scenario + ": coordinate " +
                                          std::to_string(at) +
                                          " is claimed by both " + owner +
                                          " and " + who);
            owner = who;
        }
    };

    for (int index = 0; index < model.num_joints(); ++index) {
        const JointHandle joint = model.GetJoint(index);
        const std::string who =
            "joint '" + std::string(model.GetJointName(joint)) + "'";
        const GeneralizedPositionRange positions =
            model.GetJointPositionRange(joint);
        const GeneralizedVelocityRange velocities =
            model.GetJointVelocityRange(joint);
        claim(position_owner, positions.start(), positions.size(), who);
        claim(velocity_owner, velocities.start(), velocities.size(), who);
    }

    for (int index = 0; index < model.num_rigid_bodies(); ++index) {
        const RigidBodyHandle body = model.GetRigidBody(index);
        if (!model.IsFreeBody(body)) continue;
        const std::string who =
            "free body '" + std::string(model.GetRigidBodyName(body)) + "'";
        const GeneralizedPositionRange positions =
            model.GetFreeBodyPositionRange(body);
        const GeneralizedVelocityRange velocities =
            model.GetFreeBodyVelocityRange(body);
        ExpectEqual(scenario + ": " + who + " position count",
                    positions.size(), 7);
        ExpectEqual(scenario + ": " + who + " velocity count",
                    velocities.size(), 6);
        claim(position_owner, positions.start(), positions.size(), who);
        claim(velocity_owner, velocities.start(), velocities.size(), who);
    }

    for (std::size_t at = 0; at < position_owner.size(); ++at) {
        ExpectTrue(!position_owner[at].empty(),
                   scenario + ": generalized position " + std::to_string(at) +
                       " belongs to no joint and no declared free body, so the "
                       "model was given a degree of freedom nobody asked for");
    }
    for (std::size_t at = 0; at < velocity_owner.size(); ++at) {
        ExpectTrue(!velocity_owner[at].empty(),
                   scenario + ": generalized velocity " + std::to_string(at) +
                       " belongs to no joint and no declared free body, so the "
                       "model was given a degree of freedom nobody asked for");
    }
}

void ExpectScenario(MultibodyModel* model, const std::string& scenario,
                    int expected_positions, int expected_velocities) {
    model->Finalize();
    ExpectTrue(model->is_finalized(), scenario + ": the model reports itself "
                                                 "finalized");
    ExpectEqual(scenario + ": num_generalized_positions",
                model->num_generalized_positions(), expected_positions);
    ExpectEqual(scenario + ": num_generalized_velocities",
                model->num_generalized_velocities(), expected_velocities);
    ExpectCoordinatesAreTiled(*model, scenario);
}

// --- The topology contract, restated through the public facade ---------------

void CheckSerialChain() {
    MultibodyModel model;
    const RigidBodyHandle first = model.AddRigidBody("first", SolidishBody(1.0));
    const RigidBodyHandle second =
        model.AddRigidBody("second", SolidishBody(1.0));
    const RigidBodyHandle third = model.AddRigidBody("third", SolidishBody(1.0));
    model.AddRevoluteJoint("world_to_first", model.world_frame(),
                           model.body_frame(first), kZAxis, 0.0);
    model.AddRevoluteJoint("first_to_second", model.body_frame(first),
                           model.body_frame(second), kZAxis, 0.0);
    model.AddRevoluteJoint("second_to_third", model.body_frame(second),
                           model.body_frame(third), kZAxis, 0.0);
    ExpectScenario(&model, "serial chain", 3, 3);
}

void CheckBranchedTree() {
    MultibodyModel model;
    const RigidBodyHandle shoulder =
        model.AddRigidBody("shoulder", SolidishBody(1.0));
    const RigidBodyHandle deep = model.AddRigidBody("deep", SolidishBody(1.0));
    const RigidBodyHandle shallow =
        model.AddRigidBody("shallow", SolidishBody(1.0));
    const RigidBodyHandle tip = model.AddRigidBody("tip", SolidishBody(1.0));
    model.AddRevoluteJoint("world_to_shoulder", model.world_frame(),
                           model.body_frame(shoulder), kZAxis, 0.0);
    model.AddRevoluteJoint("shoulder_to_deep", model.body_frame(shoulder),
                           model.body_frame(deep), kZAxis, 0.0);
    model.AddRevoluteJoint("deep_to_tip", model.body_frame(deep),
                           model.body_frame(tip), kZAxis, 0.0);
    model.AddRevoluteJoint("shoulder_to_shallow", model.body_frame(shoulder),
                           model.body_frame(shallow), kZAxis, 0.0);
    ExpectScenario(&model, "branched tree", 4, 4);
}

void CheckAWeldConsumesNoCoordinates() {
    MultibodyModel model;
    const RigidBodyHandle hinged =
        model.AddRigidBody("hinged", SolidishBody(1.0));
    const RigidBodyHandle welded =
        model.AddRigidBody("welded", SolidishBody(1.0));
    model.AddRevoluteJoint("world_to_hinged", model.world_frame(),
                           model.body_frame(hinged), kZAxis, 0.0);
    model.AddWeldJoint("the_weld", model.body_frame(hinged),
                       model.body_frame(welded));
    ExpectScenario(&model, "weld", 1, 1);

    // A weld still reports a place — one past the hinge's — with nothing in it.
    // A caller that treated a zero-length range as absent and fell back to
    // "wherever the previous element ended" would read the hinge's coordinate.
    const JointHandle weld = model.GetJointByName("the_weld");
    ExpectEqual("weld: the weld's position count",
                model.GetJointPositionRange(weld).size(), 0);
    ExpectEqual("weld: the weld's velocity count",
                model.GetJointVelocityRange(weld).size(), 0);
    ExpectEqual("weld: the weld's position start",
                model.GetJointPositionRange(weld).start(), 1);
    ExpectEqual("weld: the weld's velocity start",
                model.GetJointVelocityRange(weld).start(), 1);
}

void CheckTwoFreeBodiesEachGetTheirOwnRelation() {
    // Two, because a loop that handled only the first would leave the second to
    // the rigid tree — which would give it a floating relation of its own, and
    // the counts would come out right while the model held a relation nobody
    // stated. It is also the only shape in which the injection loop's own
    // bounds are exercised.
    MultibodyModel model;
    const RigidBodyHandle first = model.AddRigidBody("first", SolidishBody(1.0));
    const RigidBodyHandle second =
        model.AddRigidBody("second", SolidishBody(1.0));
    model.DeclareFreeBody(first);
    model.DeclareFreeBody(second);
    ExpectScenario(&model, "two free bodies", 14, 12);

    // Each has its own place, and the two places are different ones.
    const GeneralizedPositionRange first_positions =
        model.GetFreeBodyPositionRange(first);
    const GeneralizedPositionRange second_positions =
        model.GetFreeBodyPositionRange(second);
    ExpectTrue(first_positions.start() != second_positions.start(),
               "two free bodies do not share a place in q");
    ExpectTrue(model.GetFreeBodyVelocityRange(first).start() !=
                   model.GetFreeBodyVelocityRange(second).start(),
               "nor in v");
}

void CheckAPrivateNameCollisionSurvivesFinalization() {
    // The private joint's name is chosen against the complete set of public
    // joint names, and that choice happens during finalization. Checking only
    // that the modelling calls were accepted stops one step short of the thing
    // being claimed: the model has to come out the other side with the public
    // joint still answering to the name and the free body still holding a
    // relation of its own.
    for (const bool free_first : {true, false}) {
        const std::string order =
            free_first ? "free declared first" : "public joint added first";
        MultibodyModel model;
        const RigidBodyHandle root = model.AddRigidBody("root",
                                                        SolidishBody(1.0));
        const RigidBodyHandle child =
            model.AddRigidBody("child", SolidishBody(1.0));
        if (free_first) model.DeclareFreeBody(root);
        // The name the private joint would reach for first.
        const JointHandle public_joint = model.AddRevoluteJoint(
            "__orvd_free_body_1", model.body_frame(root),
            model.body_frame(child), kZAxis, 0.0);
        if (!free_first) model.DeclareFreeBody(root);

        // Stated as its own claim: the private name is chosen here, and if it
        // ever stopped being chosen against the public names this is where it
        // would show — as a refusal to finalize a model nobody described wrong.
        if (WasRefused([&] { model.Finalize(); })) {
            ExpectTrue(false,
                       order + ": finalization was refused although the only "
                               "collision is with a name the model chooses for "
                               "itself");
            continue;
        }
        ExpectTrue(model.GetJointByName("__orvd_free_body_1") == public_joint,
                   order + ": the caller's joint still answers to the name it "
                           "was given");
        ExpectEqual(order + ": one revolute and one free body",
                    model.num_generalized_positions(), 8);
        ExpectEqual(order + ": velocities", model.num_generalized_velocities(),
                    7);
        ExpectEqual(order + ": the free body still has its own seven positions",
                    model.GetFreeBodyPositionRange(root).size(), 7);
        ExpectCoordinatesAreTiled(model, order);
    }
}

void CheckAFreeBodyGetsSevenPositionsAndSixVelocities() {
    MultibodyModel model;
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    model.DeclareFreeBody(floating);
    ExpectScenario(&model, "free body", 7, 6);
}

void CheckMixedJointsAndAFreeBody() {
    MultibodyModel model;
    const RigidBodyHandle hinged =
        model.AddRigidBody("hinged", SolidishBody(1.0));
    const RigidBodyHandle sliding =
        model.AddRigidBody("sliding", SolidishBody(1.0));
    const RigidBodyHandle welded =
        model.AddRigidBody("welded", SolidishBody(1.0));
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    model.AddRevoluteJoint("world_to_hinged", model.world_frame(),
                           model.body_frame(hinged), kZAxis, 0.0);
    model.AddPrismaticJoint("hinged_to_sliding", model.body_frame(hinged),
                            model.body_frame(sliding), kXAxis, 0.0);
    model.AddWeldJoint("sliding_to_welded", model.body_frame(sliding),
                       model.body_frame(welded));
    model.DeclareFreeBody(floating);
    ExpectScenario(&model, "mixed joints", 9, 8);
}

void CheckARelationStatedTowardsTheWorld() {
    // The world is the child. The forest has to grow through this joint from
    // the child side, which is the only way its mobilizer comes out reversed —
    // and the coordinate count must come out the same either way, because which
    // side the caller called the parent fixes the sign of the coordinate, not
    // how many there are.
    MultibodyModel model;
    const RigidBodyHandle backwards =
        model.AddRigidBody("backwards", SolidishBody(1.0));
    model.AddRevoluteJoint("towards_world", model.body_frame(backwards),
                           model.world_frame(), kZAxis, 0.0);
    ExpectScenario(&model, "reversed relation", 1, 1);
}

void CheckPositionsAndVelocitiesAreIndexedSeparately() {
    // A joint outboard of a free body. The free body contributes seven
    // positions but only six velocities, so from here on the two indexings
    // diverge — and this is the only shape in which they can, because every
    // joint this facade offers has as many positions as velocities.
    //
    // Without a case like this, the position and velocity accessors could be
    // each other and nothing would notice: for a revolute, a prismatic and a
    // weld alike, the two answers are identical.
    MultibodyModel model;
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    const RigidBodyHandle hanging =
        model.AddRigidBody("hanging", SolidishBody(1.0));
    model.DeclareFreeBody(floating);
    model.AddRevoluteJoint("outboard", model.body_frame(floating),
                           model.body_frame(hanging), kZAxis, 0.0);
    ExpectScenario(&model, "joint outboard of a free body", 8, 7);

    const JointHandle outboard = model.GetJointByName("outboard");
    const int position_start = model.GetJointPositionRange(outboard).start();
    const int velocity_start = model.GetJointVelocityRange(outboard).start();
    ExpectEqual("exactly one free body precedes this joint, so its position "
                "start runs one ahead of its velocity start",
                position_start - velocity_start, 1);

    const GeneralizedPositionRange free_positions =
        model.GetFreeBodyPositionRange(floating);
    const GeneralizedVelocityRange free_velocities =
        model.GetFreeBodyVelocityRange(floating);
    ExpectEqual("the free body's positions run up to where the joint's begin",
                free_positions.start() + free_positions.size(), position_start);
    ExpectEqual("and its velocities up to where the joint's velocities begin",
                free_velocities.start() + free_velocities.size(),
                velocity_start);
}

// --- The finalization state machine ------------------------------------------

void CheckABodyThatReachesNothingIsNamedAndNothingIsChanged() {
    MultibodyModel model;
    const RigidBodyHandle attached =
        model.AddRigidBody("attached", SolidishBody(1.0));
    const RigidBodyHandle adrift =
        model.AddRigidBody("adrift", SolidishBody(1.0));
    const RigidBodyHandle also_adrift =
        model.AddRigidBody("also_adrift", SolidishBody(1.0));
    model.AddRevoluteJoint("hinge", model.world_frame(),
                           model.body_frame(attached), kZAxis, 0.0);

    ExpectTrue(RefusalMentions([&] { model.Finalize(); }, "'adrift'"),
               "an unreachable body is named in the refusal");
    ExpectTrue(RefusalMentions([&] { model.Finalize(); }, "'also_adrift'"),
               "every unreachable body is named, not just the first one found");
    if (model.is_finalized()) {
        ExpectTrue(false,
                   "a model with two bodies that reach nothing finalized "
                   "anyway, so the rigid tree gave them relations the caller "
                   "never stated");
        return;
    }

    // The precheck writes nothing, so the model is exactly what it was: saying
    // what was missing has to be enough. This is the half of the state machine
    // that is not terminal, and it is only sound because nothing was committed.
    model.DeclareFreeBody(adrift);
    model.AddRevoluteJoint("second_hinge", model.body_frame(attached),
                           model.body_frame(also_adrift), kZAxis, 0.0);
    model.Finalize();
    ExpectTrue(model.is_finalized(),
               "a model that failed its precheck finalizes once the missing "
               "relations are stated");
    ExpectCoordinatesAreTiled(model, "recovered from a failed precheck");
}

void CheckFinalizationHappensOnce() {
    MultibodyModel model;
    const RigidBodyHandle body = model.AddRigidBody("body", SolidishBody(1.0));
    model.DeclareFreeBody(body);
    model.Finalize();
    ExpectTrue(RefusalMentions([&] { model.Finalize(); }, "is finalized"),
               "a second Finalize() is refused and says the model is finalized");
}

void CheckAFinalizedModelRefusesToBeChanged() {
    MultibodyModel model;
    const RigidBodyHandle body = model.AddRigidBody("body", SolidishBody(1.0));
    const RigidBodyHandle other =
        model.AddRigidBody("other", SolidishBody(1.0));
    model.DeclareFreeBody(body);
    model.AddRevoluteJoint("hinge", model.body_frame(body),
                           model.body_frame(other), kZAxis, 0.0);
    model.Finalize();

    // Each refusal has to come from this layer's own gate, named for what was
    // attempted. The rigid tree underneath refuses post-finalize changes too,
    // and a check that only asked whether something was thrown would pass on a
    // model with no gate of its own — reporting the arrangement's complaint
    // instead of the model's, about an element the caller named differently.
    ExpectTrue(RefusalMentions([&] { model.AddRigidBody("late",
                                                        SolidishBody(1.0)); },
                               "cannot add a rigid body"),
               "adding a rigid body to a finalized model is refused by the "
               "model, in its own words");
    ExpectTrue(RefusalMentions([&] { model.AddFixedFrame("late_frame", body,
                                                          {}); },
                               "cannot add a fixed frame"),
               "adding a fixed frame to a finalized model is refused by the "
               "model");
    ExpectTrue(RefusalMentions(
                   [&] {
                       model.AddRevoluteJoint("late_hinge", model.world_frame(),
                                              model.body_frame(other), kZAxis, 0.0);
                   },
                   "cannot add a revolute joint"),
               "adding a joint to a finalized model is refused by the model");
    ExpectTrue(RefusalMentions(
                   [&] {
                       model.AddPrismaticJoint("late_slider",
                                               model.world_frame(),
                                               model.body_frame(other), kXAxis, 0.0);
                   },
                   "cannot add a prismatic joint"),
               "adding a prismatic joint to a finalized model is refused by the "
               "model");
    ExpectTrue(RefusalMentions(
                   [&] {
                       model.AddWeldJoint("late_weld", model.world_frame(),
                                          model.body_frame(other));
                   },
                   "cannot add a weld joint"),
               "adding a weld joint to a finalized model is refused by the "
               "model");
    ExpectTrue(RefusalMentions([&] { model.DeclareFreeBody(other); },
                               "cannot declare a rigid body free"),
               "declaring a body free in a finalized model is refused by the "
               "model");
    ExpectTrue(RefusalMentions(
                   [&] { model.SetGravityVector(Eigen::Vector3d::Zero()); },
                   "cannot set the gravity vector"),
               "changing gravity in a finalized model is refused by the model");
}

void CheckQueriesNeedAFinalizedModel() {
    MultibodyModel model;
    const RigidBodyHandle body = model.AddRigidBody("body", SolidishBody(1.0));
    model.DeclareFreeBody(body);

    ExpectTrue(RefusalMentions([&] { (void)model.num_rigid_bodies(); },
                               "before Finalize()"),
               "a count before finalization is refused and says so");
    ExpectTrue(WasRefused([&] { (void)model.num_generalized_positions(); }),
               "the position count before finalization is refused");
    ExpectTrue(WasRefused([&] { (void)model.GetRigidBody(0); }),
               "enumeration before finalization is refused");
    ExpectTrue(WasRefused([&] { (void)model.GetRigidBodyByName("body"); }),
               "lookup by name before finalization is refused");
    ExpectTrue(WasRefused([&] { (void)model.GetFreeBodyPositionRange(body); }),
               "a coordinate range before finalization is refused");
    ExpectTrue(WasRefused([&] { (void)model.CreateDefaultContext(); }),
               "creating a context before finalization is refused");
}

// --- What a finalized model says it contains ---------------------------------

void CheckWhatTheModelSaysItContains() {
    MultibodyModel model;
    const RigidBodyHandle carrier =
        model.AddRigidBody("carrier", SolidishBody(2.0));
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    const FrameHandle mount = model.AddFixedFrame("mount", carrier, {});
    model.DeclareFreeBody(floating);
    model.AddRevoluteJoint("hinge", mount, model.body_frame(floating), kZAxis, 0.0);
    model.Finalize();

    ExpectEqual("the world is not one of the model's rigid bodies",
                model.num_rigid_bodies(), 2);
    // World frame, two body frames, one fixed frame.
    ExpectEqual("frames are the world's, the bodies' and the added ones",
                model.num_frames(), 4);
    // The joint implementing the free relation is not one of them.
    ExpectEqual("only the joints a caller added are counted",
                model.num_joints(), 1);

    ExpectTrue(model.GetRigidBodyName(model.GetRigidBody(0)) == "carrier",
               "rigid bodies enumerate in the order they were added");
    ExpectTrue(model.GetRigidBodyName(model.GetRigidBody(1)) == "floating",
               "and the world does not take the first place");
    ExpectTrue(model.GetFrameName(model.GetFrame(0)) == "world",
               "the world frame is the model's first frame");
    ExpectTrue(model.GetJointName(model.GetJoint(0)) == "hinge",
               "the only public joint is the one that was added");

    ExpectTrue(model.GetRigidBodyByName("carrier") == model.GetRigidBody(0),
               "a body found by name is the same handle as the enumerated one");
    ExpectTrue(model.GetFrameByName("mount") == mount,
               "a fixed frame is found by the name it was added under");

    ExpectTrue(RefusalMentions([&] { (void)model.GetRigidBodyByName("world"); },
                               "names the world"),
               "asking for the world among the rigid bodies is refused by name");
    ExpectTrue(WasRefused([&] { (void)model.GetJointByName("__orvd_free_body_2"); }),
               "the joint implementing a free relation is not publicly named");
    ExpectTrue(WasRefused([&] { (void)model.GetRigidBody(2); }),
               "an index past the end is refused");
    ExpectTrue(WasRefused([&] { (void)model.GetJoint(-1); }),
               "a negative index is refused");

    ExpectTrue(model.IsFreeBody(model.GetRigidBody(1)),
               "the body that was declared free reports itself free");
    ExpectTrue(!model.IsFreeBody(model.GetRigidBody(0)),
               "and the one that was not, does not");
    ExpectTrue(RefusalMentions(
                   [&] { (void)model.GetFreeBodyPositionRange(carrier); },
                   "was not declared free"),
               "asking for an unfree body's own coordinates is refused by name "
               "rather than answered with an empty range");
}

void CheckHandlesFromAnotherModelAreRefused() {
    MultibodyModel described;
    const RigidBodyHandle body =
        described.AddRigidBody("body", SolidishBody(1.0));
    described.DeclareFreeBody(body);
    described.Finalize();

    // Deliberately a different size, so a foreign handle cannot be caught by a
    // bounds check standing in for the identity check.
    MultibodyModel other;
    const RigidBodyHandle first = other.AddRigidBody("first", SolidishBody(1.0));
    const RigidBodyHandle second =
        other.AddRigidBody("second", SolidishBody(1.0));
    other.DeclareFreeBody(first);
    other.AddRevoluteJoint("hinge", other.body_frame(first),
                           other.body_frame(second), kZAxis, 0.0);
    other.Finalize();

    ExpectTrue(RefusalMentions([&] { (void)described.IsFreeBody(second); },
                               "from a different model"),
               "a handle from another model is refused as foreign, not "
               "resolved against whatever sits at that ordinal");
    ExpectTrue(
        RefusalMentions([&] { (void)described.GetRigidBodyName(first); },
                        "from a different model"),
        "including when the ordinal is in range in both models");
}

// --- Gravity -----------------------------------------------------------------

void CheckGravityIsAModelConstant() {
    MultibodyModel model;
    // A model that never mentions gravity still has it, pointing down. A silent
    // zero would make every static result look like a model in free fall.
    ExpectTrue(model.gravity_vector().allFinite(),
               "an unstated gravity vector is finite");
    ExpectTrue(model.gravity_vector().z() < 0.0,
               "an unstated gravity vector points down");
    ExpectTrue(model.gravity_vector().norm() > 9.0 &&
                   model.gravity_vector().norm() < 10.0,
               "an unstated gravity vector has Earth's magnitude");

    const Eigen::Vector3d stated(0.0, 1.5, -3.0);
    model.SetGravityVector(stated);
    ExpectTrue(model.gravity_vector() == stated,
               "a stated gravity vector is kept exactly");

    ExpectTrue(RefusalMentions(
                   [&] {
                       model.SetGravityVector(Eigen::Vector3d(
                           0.0, 0.0,
                           std::numeric_limits<double>::infinity()));
                   },
                   "not finite"),
               "a gravity vector that is not finite is refused");
    ExpectTrue(model.gravity_vector() == stated,
               "and the refused value did not displace the stated one");
}

// --- Contexts ----------------------------------------------------------------

void CheckAContextComesFromTheModelWithItsDefaultsInstalled() {
    MultibodyModel model;
    const RigidBodyHandle floating =
        model.AddRigidBody("floating", SolidishBody(1.0));
    const RigidBodyHandle hinged =
        model.AddRigidBody("hinged", SolidishBody(1.0));
    model.DeclareFreeBody(floating);
    model.AddRevoluteJoint("hinge", model.body_frame(floating),
                           model.body_frame(hinged), kZAxis, 0.0);
    model.Finalize();

    const std::unique_ptr<MultibodyEvaluationContext> context =
        model.CreateDefaultContext();
    ExpectTrue(context != nullptr, "the model issued a context");
    ExpectEqual("the context's q is the model's q",
                static_cast<int>(context->generalized_positions().size()),
                model.num_generalized_positions());
    ExpectEqual("the context's v is the model's v",
                static_cast<int>(context->generalized_velocities().size()),
                model.num_generalized_velocities());

    // The defaults were installed, not left as the state store's zeros. A zero
    // quaternion is the tell: it is what a zero-filled state holds and it is not
    // a rotation, so finding a unit one here is finding that something wrote it.
    const GeneralizedPositionRange free_positions =
        model.GetFreeBodyPositionRange(floating);
    const Eigen::Vector4d quaternion =
        context->generalized_positions().segment<4>(free_positions.start());
    ExpectTrue(std::abs(quaternion.norm() - 1.0) < 1e-15,
               "the free body's default quaternion is a unit one, so the "
               "model's defaults reached the context rather than the state "
               "store's zeros");

    ExpectTrue(context->generalized_velocities().isZero(0.0),
               "the default velocities are zero");

    const std::unique_ptr<MultibodyEvaluationContext> second =
        model.CreateDefaultContext();
    ExpectTrue(second.get() != context.get(),
               "each call issues its own context");
}

}  // namespace

int main() {
    CheckSerialChain();
    CheckBranchedTree();
    CheckAWeldConsumesNoCoordinates();
    CheckAFreeBodyGetsSevenPositionsAndSixVelocities();
    CheckTwoFreeBodiesEachGetTheirOwnRelation();
    CheckMixedJointsAndAFreeBody();
    CheckARelationStatedTowardsTheWorld();
    CheckPositionsAndVelocitiesAreIndexedSeparately();
    CheckAPrivateNameCollisionSurvivesFinalization();

    CheckABodyThatReachesNothingIsNamedAndNothingIsChanged();
    CheckFinalizationHappensOnce();
    CheckAFinalizedModelRefusesToBeChanged();
    CheckQueriesNeedAFinalizedModel();

    CheckWhatTheModelSaysItContains();
    CheckHandlesFromAnotherModelAreRefused();
    CheckGravityIsAModelConstant();
    CheckAContextComesFromTheModelWithItsDefaultsInstalled();

    if (failure_count > 0) {
        std::printf("%d finalization check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("finalization and the finalized model's queries hold\n");
    return 0;
}
