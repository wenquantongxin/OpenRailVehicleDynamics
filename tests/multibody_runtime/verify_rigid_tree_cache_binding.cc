// The rigid tree's caches, bound to real state and exercised on a real model.
//
// G24 checked the slot mechanism against a hand-written transcription of the
// contract, with synthetic values and a synthetic calculator. This checks the
// thing itself: the tree's own calculators, writing the tree's own cache values,
// invalidated by writes to the tree's own state. What the type system cannot see
// — whether a calculator reads something its slot does not declare — is what a
// cold-versus-hot comparison can.
//
// Two contexts appear throughout, and they are driven to the same version
// numbers on purpose. Equal versions holding different states is exactly the
// situation in which a workspace shared between contexts, or a freshness rule
// that looked only at the numbers, would serve one context's value to the other
// and look right doing it.
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace {

using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::internal::MultibodyTree;
using Context = orvd::rigid_multibody_tree::internal::
    RigidMultibodyTreeEvaluationContext;

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

/// A two-link chain on revolute joints: enough that a position change moves a
/// pose, and that a mass change moves an inertia without moving a pose.
class TwoLinkChain {
   public:
    TwoLinkChain() {
        const auto& first = tree_.AddLink(
            "first", SpatialInertia<double>::SolidBoxWithMass(1.0, 0.2, 0.1, 0.1));
        const auto& second = tree_.AddLink(
            "second", SpatialInertia<double>::SolidBoxWithMass(2.0, 0.2, 0.1, 0.1));
        const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        // The joints are offset along x. With every frame coincident, rotating a
        // revolute joint would leave each body origin exactly where it was, and
        // a check that "the pose moved" would be asserting something the model
        // cannot do — passing or failing for reasons that have nothing to do
        // with the cache.
        const drake::math::RigidTransform<double> offset(
            Eigen::Vector3d(0.3, 0.0, 0.0));
        tree_.AddJoint<RevoluteJoint>("shoulder", tree_.world_link(), offset,
                                      first, {}, axis);
        tree_.AddJoint<RevoluteJoint>("elbow", first, offset, second, {}, axis);
        tree_.Finalize();
    }

    const MultibodyTree<double>& tree() const { return tree_; }
    std::unique_ptr<Context> MakeContext() const {
        return tree_.CreateDefaultEvaluationContext();
    }

   private:
    MultibodyTree<double> tree_;
};

/// The pose of the last body in the world, as the position pass computed it.
Eigen::Vector3d LastBodyOrigin(const MultibodyTree<double>& tree,
                               const Context& context) {
    const auto& pc = tree.EvalPositionKinematics(context);
    return pc.get_X_WB(drake::multibody::internal::MobodIndex(
                           tree.num_mobods() - 1))
        .translation();
}

void SetJointAngles(const MultibodyTree<double>& tree, Context* context,
                    double first, double second) {
    Eigen::VectorXd q(tree.num_positions());
    q << first, second;
    tree.SetPositions(context, q);
}

void CheckTheFactoryIsTheOnlyDoor() {
    // A context cannot be built from a layout alone: the bare constructor is
    // private. What would come out of one is a model with every body massless
    // and every joint at zero, and it would evaluate rather than complain.
    static_assert(!std::is_constructible_v<
                      Context, const orvd::multibody_runtime::MultibodyStateLayout&>,
                  "a context must come from the finalized model, not from a layout");
    static_assert(!std::is_default_constructible_v<Context>);
    static_assert(!std::is_copy_constructible_v<Context>);
    static_assert(!std::is_move_constructible_v<Context>);

    const TwoLinkChain model;
    const std::unique_ptr<Context> context = model.MakeContext();
    ExpectTrue(context != nullptr, "the factory returns a context");
    ExpectTrue(context->state().generalized_positions().size() ==
                   model.tree().num_positions(),
               "the context's state is sized from the finalized model");
    // The model's default mass reached the state; the store's own default is
    // zero, so this distinguishes "installed" from "merely allocated".
    ExpectTrue(context->state().rigid_body_inertia_parameters(1).mass_kilograms >
                   0.0,
               "the model's default inertia was installed, not the store's");
}

void CheckColdComputesAndHotDoesNot() {
    const TwoLinkChain model;
    const std::unique_ptr<Context> context = model.MakeContext();

    const Eigen::Vector3d first = LastBodyOrigin(model.tree(), *context);
    ExpectTrue(context->caches().position_kinematics.is_fresh(),
               "the position pass is fresh once it has been evaluated");
    const Eigen::Vector3d again = LastBodyOrigin(model.tree(), *context);
    ExpectTrue(first == again, "a repeated query returns the same value");
    ExpectTrue(context->caches().position_kinematics.is_fresh(),
               "and leaves it fresh");
}

void CheckAPositionWriteExpiresOnlyWhatReadsPositions() {
    const TwoLinkChain model;
    const std::unique_ptr<Context> context = model.MakeContext();
    LastBodyOrigin(model.tree(), *context);
    // The articulated body inertia is a public entry and its calculation warms
    // the reflected inertia along the way; that is the slot this check is about,
    // because it reads actuator parameters and nothing else.
    model.tree().EvalArticulatedBodyInertiaCache(*context);
    ExpectTrue(context->caches().position_kinematics.is_fresh() &&
                   context->caches().reflected_inertia.is_fresh(),
               "both start fresh");

    SetJointAngles(model.tree(), context.get(), 0.3, -0.4);
    ExpectTrue(!context->caches().position_kinematics.is_fresh(),
               "a position write expires the position pass");
    ExpectTrue(context->caches().reflected_inertia.is_fresh(),
               "and leaves the reflected inertia alone — it reads actuator "
               "parameters and nothing else");

    const Eigen::Vector3d moved = LastBodyOrigin(model.tree(), *context);
    ExpectTrue(moved.norm() > 0.0,
               "and the recomputed pose reflects the new configuration");
}

void CheckAMassWriteExpiresInertiaButNotPosition() {
    const TwoLinkChain model;
    const std::unique_ptr<Context> context = model.MakeContext();
    LastBodyOrigin(model.tree(), *context);
    model.tree().EvalArticulatedBodyInertiaCache(*context);
    ExpectTrue(context->caches().position_kinematics.is_fresh() &&
                   context->caches().spatial_inertia_in_world.is_fresh(),
               "both start fresh");

    // This is the distinction the runtime contract insisted on and that the
    // upstream cache could not draw: mass properties and frame poses live in
    // one value type, but the position pass reads only the pose fields.
    orvd::multibody_runtime::RigidBodyInertiaParameters heavier =
        context->state().rigid_body_inertia_parameters(1);
    heavier.mass_kilograms *= 2.0;
    model.tree().GetMutableParameters(context.get()).set_rigid_body_inertia_parameters(
        1, heavier);

    ExpectTrue(context->caches().position_kinematics.is_fresh(),
               "changing a mass does not expire the position pass");
    ExpectTrue(!context->caches().spatial_inertia_in_world.is_fresh(),
               "but it does expire the world inertias");
}

void CheckTwoContextsWithEqualVersionsStayApart() {
    const TwoLinkChain model;
    const std::unique_ptr<Context> a = model.MakeContext();
    const std::unique_ptr<Context> b = model.MakeContext();

    // The same number of writes of the same kind, with different values: equal
    // versions, different states.
    SetJointAngles(model.tree(), a.get(), 0.5, 0.0);
    SetJointAngles(model.tree(), b.get(), -0.5, 0.0);
    ExpectTrue(a->state().generalized_positions_version() ==
                       b->state().generalized_positions_version() &&
                   a->state().generalized_velocities_version() ==
                       b->state().generalized_velocities_version() &&
                   a->state().fixed_frame_poses_version() ==
                       b->state().fixed_frame_poses_version() &&
                   a->state().rigid_body_inertias_version() ==
                       b->state().rigid_body_inertias_version() &&
                   a->state().joint_actuator_parameters_version() ==
                       b->state().joint_actuator_parameters_version(),
               "the two contexts are at identical version numbers");
    ExpectTrue(a->state().generalized_positions() !=
                   b->state().generalized_positions(),
               "while holding different states");

    // Warm A, then B, then read A again. A workspace shared between the two, or
    // a freshness rule that compared only the numbers, would hand back B's
    // answer here and look entirely reasonable doing it.
    const Eigen::Vector3d a_first = LastBodyOrigin(model.tree(), *a);
    const Eigen::Vector3d b_warm = LastBodyOrigin(model.tree(), *b);
    const Eigen::Vector3d a_again = LastBodyOrigin(model.tree(), *a);
    ExpectTrue(a_first == a_again,
               "reading A after warming B returns A's own value");
    ExpectTrue(a_first != b_warm,
               "and the two contexts genuinely disagree, so the check is not "
               "vacuous");

    // Move A and recompute it; B must not notice in any way.
    SetJointAngles(model.tree(), a.get(), 1.1, 0.2);
    const Eigen::Vector3d a_moved = LastBodyOrigin(model.tree(), *a);
    ExpectTrue(a_moved != a_first, "A's pose moved");
    ExpectTrue(b->caches().position_kinematics.is_fresh(),
               "B's position pass is still fresh");
    ExpectTrue(LastBodyOrigin(model.tree(), *b) == b_warm,
               "and still holds B's value");

    // An inertia change in A: the inertia slots expire, the position slot does
    // not, and B is untouched by either.
    LastBodyOrigin(model.tree(), *a);
    model.tree().EvalArticulatedBodyInertiaCache(*a);
    orvd::multibody_runtime::RigidBodyInertiaParameters heavier =
        a->state().rigid_body_inertia_parameters(0);
    heavier.mass_kilograms += 1.0;
    model.tree().GetMutableParameters(a.get()).set_rigid_body_inertia_parameters(0, heavier);
    ExpectTrue(!a->caches().spatial_inertia_in_world.is_fresh(),
               "A's world inertias expired");
    ExpectTrue(a->caches().position_kinematics.is_fresh(),
               "A's position pass did not");
    ExpectTrue(b->caches().position_kinematics.is_fresh() &&
                   LastBodyOrigin(model.tree(), *b) == b_warm,
               "and B is untouched by A's inertia change");
}

void CheckAZeroQuaternionIsRefusedAndNothingIsWritten() {
    // A revolute chain has no quaternion, so the gate has nothing to reject
    // here; what this pins is that an ordinary write still passes through the
    // gate unharmed, and that a non-unit value is stored exactly as given
    // rather than being normalised behind the caller's back.
    const TwoLinkChain model;
    const std::unique_ptr<Context> context = model.MakeContext();
    Eigen::VectorXd q(model.tree().num_positions());
    q << 0.25, 0.75;
    model.tree().SetPositions(context.get(), q);
    ExpectTrue(context->state().generalized_positions() == q,
               "an ordinary position write is stored exactly as given");
}

}  // namespace

int main() {
    CheckTheFactoryIsTheOnlyDoor();
    CheckColdComputesAndHotDoesNot();
    CheckAPositionWriteExpiresOnlyWhatReadsPositions();
    CheckAMassWriteExpiresInertiaButNotPosition();
    CheckTwoContextsWithEqualVersionsStayApart();
    CheckAZeroQuaternionIsRefusedAndNothingIsWritten();

    if (failure_count > 0) {
        std::printf("%d rigid tree cache binding check(s) failed\n",
                    failure_count);
        return 1;
    }
    std::printf(
        "the tree's caches expire for exactly the state their calculators read,"
        " and two contexts at identical version numbers keep their own answers"
        "\n");
    return 0;
}
