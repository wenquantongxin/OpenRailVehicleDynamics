// The smallest thing that uses the rigid tree the way a consumer will.
//
// Build a model, finalize it, make its own evaluation context, put it in a
// configuration and ask for the kinematics. It links only the product archive,
// so what it demonstrates is that the archive is consumable on its own — that
// its transitive dependencies are declared and that nothing it needs was left
// in a target the archive does not carry.
//
// The checks are explicit `if`s rather than `assert()`. An assertion compiled
// out in Release would leave a program that runs, prints, exits zero and
// verifies nothing, and Release is one of the configurations this is registered
// in.
//
// The joints are offset along x, which is what makes the checks non-vacuous:
// with every frame coincident, rotating a revolute joint leaves each body origin
// exactly where it was, and "the pose changed" would be asserting something the
// model cannot do.
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace {

using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::internal::MobodIndex;
using drake::multibody::internal::MultibodyTree;

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

}  // namespace

int main() {
    MultibodyTree<double> tree;

    // A body with a finite, physically realisable inertia — not a massless
    // frame carrier, so the mass properties reaching the state are something
    // the store had to accept rather than something it defaults to.
    const auto& first = tree.AddLink(
        "first", SpatialInertia<double>::SolidBoxWithMass(1.5, 0.2, 0.1, 0.1));
    const auto& second = tree.AddLink(
        "second", SpatialInertia<double>::SolidBoxWithMass(2.5, 0.2, 0.1, 0.1));
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    const drake::math::RigidTransform<double> offset(
        Eigen::Vector3d(0.3, 0.0, 0.0));
    tree.AddJoint<RevoluteJoint>("shoulder", tree.world_link(), offset, first,
                                 {}, axis);
    tree.AddJoint<RevoluteJoint>("elbow", first, offset, second, {}, axis);

    tree.Finalize();
    ExpectTrue(tree.num_positions() == 2 && tree.num_velocities() == 2,
               "the finalized model has the two degrees of freedom it was built "
               "with");

    const std::unique_ptr<orvd::rigid_multibody_tree::internal::
                              RigidMultibodyTreeEvaluationContext>
        context = tree.CreateDefaultEvaluationContext();
    ExpectTrue(context != nullptr, "the model produced an evaluation context");

    // The default configuration, for something to compare against. A check that
    // only looked at the value after the write could not tell a computed pose
    // from a constant.
    const Eigen::Vector3d default_origin =
        tree.EvalPositionKinematics(*context)
            .get_X_WB(MobodIndex(tree.num_mobods() - 1))
            .translation();
    ExpectTrue(default_origin.allFinite(),
               "the default configuration's pose is finite");

    Eigen::VectorXd q(tree.num_positions());
    q << 0.7, -0.4;
    tree.SetPositions(context.get(), q);
    ExpectTrue(context->state().generalized_positions() == q,
               "the positions were stored exactly as written");

    const Eigen::Vector3d moved_origin =
        tree.EvalPositionKinematics(*context)
            .get_X_WB(MobodIndex(tree.num_mobods() - 1))
            .translation();
    ExpectTrue(moved_origin.allFinite(),
               "the recomputed pose is finite");
    ExpectTrue(moved_origin != default_origin,
               "and differs from the default one, so the kinematics were "
               "actually recomputed for this configuration");

    // Diagnostic only. Nothing downstream compares this text, and no golden copy
    // of it is kept: a frozen stdout would turn a formatting change into a
    // failure and an arithmetic change into a passing test with a stale record.
    std::printf("last body origin in world: (%.6f, %.6f, %.6f)\n",
                moved_origin.x(), moved_origin.y(), moved_origin.z());

    if (failure_count > 0) {
        std::printf("%d minimal model check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "the product archive builds a model, finalizes it, creates its own"
        " evaluation context and computes kinematics from it\n");
    return 0;
}
