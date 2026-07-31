// The rigid tree's caches, bound to real state and exercised on a real model.
//
// G24 checked the slot mechanism against a hand-written transcription of the
// contract, with synthetic values and a synthetic calculator. This checks the
// thing itself: the tree's own calculators, writing the tree's own cache values,
// invalidated by writes to the tree's own state. Representative position,
// velocity and inertia chains are compared cold versus hot. The full
// calculator-input map remains the source audit in the runtime contract; this
// test does not pretend to exhaustively compare every field of every cache.
//
// Two contexts appear throughout, and they are driven to the same version
// numbers on purpose. Equal versions holding different states is exactly the
// situation in which a workspace shared between contexts, or a freshness rule
// that looked only at the numbers, would serve one context's value to the other
// and look right doing it.
#include <array>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orvd/rigid_multibody_tree/spatial_inertia_parameter_conversion.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace {

using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::internal::MultibodyTree;
using Context = orvd::rigid_multibody_tree::internal::
    RigidMultibodyTreeEvaluationContext;
using RigidMultibodyTreeCacheWorkspace =
    orvd::rigid_multibody_tree::internal::RigidMultibodyTreeCacheWorkspace;
using MultibodyStateInstance =
    orvd::multibody_runtime::MultibodyStateInstance;
using SpanningForest = drake::multibody::internal::SpanningForest;
using VersionSource =
    orvd::multibody_runtime::MultibodyStateVersionSource;

template <typename T>
concept PubliclyExposesStateLayout =
    requires(const T& value) { value.state_layout(); };
static_assert(!PubliclyExposesStateLayout<MultibodyTree<double>>);

inline constexpr auto kQ = VersionSource::kGeneralizedPositions;
inline constexpr auto kV = VersionSource::kGeneralizedVelocities;
inline constexpr auto kFramePoses = VersionSource::kFixedFramePoses;
inline constexpr auto kInertias = VersionSource::kRigidBodyInertias;
inline constexpr auto kActuators =
    VersionSource::kJointActuatorParameters;

template <typename Value, VersionSource... Sources>
using ExpectedSlot =
    orvd::multibody_runtime::VersionedCacheSlot<Value, Sources...>;

template <typename Actual, typename Expected>
inline constexpr bool IsExpectedSlot =
    std::is_same_v<std::remove_cvref_t<Actual>, Expected>;

// This table is intentionally independent of the declarations in the
// workspace. It is the G20 contract transcribed into compile-time checks, so a
// hand-edited source set in a real slot cannot make its own expected answer.
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .frame_body_poses),
              ExpectedSlot<
                  drake::multibody::internal::FrameBodyPoseCache<double>,
                  kFramePoses, kInertias>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .position_kinematics),
              ExpectedSlot<
                  drake::multibody::internal::PositionKinematicsCache<double>,
                  kQ, kFramePoses>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .across_node_jacobian),
              ExpectedSlot<std::vector<drake::Vector6<double>>, kQ,
                           kFramePoses>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .velocity_kinematics),
              ExpectedSlot<
                  drake::multibody::internal::VelocityKinematicsCache<double>,
                  kQ, kV, kFramePoses>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .spatial_inertia_in_world),
              ExpectedSlot<
                  std::vector<drake::multibody::SpatialInertia<double>>, kQ,
                  kFramePoses, kInertias>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .composite_body_inertia_in_world),
              ExpectedSlot<
                  std::vector<drake::multibody::SpatialInertia<double>>, kQ,
                  kFramePoses, kInertias>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .dynamic_bias),
              ExpectedSlot<
                  std::vector<drake::multibody::SpatialForce<double>>, kQ, kV,
                  kFramePoses, kInertias>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .spatial_acceleration_bias),
              ExpectedSlot<
                  std::vector<drake::multibody::SpatialAcceleration<double>>,
                  kQ, kV, kFramePoses>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .reflected_inertia),
              ExpectedSlot<drake::VectorX<double>, kActuators>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .articulated_body_inertia),
              ExpectedSlot<
                  drake::multibody::internal::
                      ArticulatedBodyInertiaCache<double>,
                  kQ, kFramePoses, kInertias, kActuators>>);
static_assert(IsExpectedSlot<
              decltype(std::declval<RigidMultibodyTreeCacheWorkspace&>()
                           .articulated_body_force_bias),
              ExpectedSlot<
                  std::vector<drake::multibody::SpatialForce<double>>, kQ, kV,
                  kFramePoses, kInertias, kActuators>>);

template <typename T>
concept PubliclyExposesMutableCaches =
    requires(T& value) { value.mutable_caches(); };
static_assert(!PubliclyExposesMutableCaches<Context>);
static_assert(std::is_same_v<decltype(std::declval<const Context&>().caches()),
                             const RigidMultibodyTreeCacheWorkspace&>);
// The workspace is not an independently assembled object. Even an lvalue state
// may belong to a shorter-lived scope than the returned workspace, so deleting
// only rvalue construction would leave the same eleven dangling bindings
// reachable through a helper function. The context's private ownership pair is
// the sole construction path.
static_assert(!std::is_constructible_v<
              RigidMultibodyTreeCacheWorkspace, MultibodyStateInstance&,
              const SpanningForest&, int, int>);
static_assert(!std::is_constructible_v<
              RigidMultibodyTreeCacheWorkspace, const MultibodyStateInstance&,
              const SpanningForest&, int, int>);
static_assert(!std::is_constructible_v<
              RigidMultibodyTreeCacheWorkspace, MultibodyStateInstance&&,
              const SpanningForest&, int, int>);
static_assert(!std::is_constructible_v<
              RigidMultibodyTreeCacheWorkspace, const MultibodyStateInstance&&,
              const SpanningForest&, int, int>);

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
        const auto first_inertia =
            SpatialInertia<double>::MakeFromCentralInertia(
                1.2, Eigen::Vector3d(0.08, -0.04, 0.03),
                drake::multibody::RotationalInertia<double>(
                    0.025, 0.032, 0.041, 0.002, -0.001, 0.003));
        const auto second_inertia =
            SpatialInertia<double>::MakeFromCentralInertia(
                1.8, Eigen::Vector3d(-0.05, 0.07, 0.04),
                drake::multibody::RotationalInertia<double>(
                    0.038, 0.047, 0.056, -0.002, 0.003, 0.001));
        const auto& first = tree_.AddLink(
            "first", first_inertia);
        const auto& second = tree_.AddLink(
            "second", second_inertia);
        // The joints are offset along x. With every frame coincident, rotating a
        // revolute joint would leave each body origin exactly where it was, and
        // a check that "the pose moved" would be asserting something the model
        // cannot do — passing or failing for reasons that have nothing to do
        // with the cache.
        const drake::math::RigidTransform<double> offset(
            Eigen::Vector3d(0.3, 0.0, 0.0));
        const auto& shoulder =
            tree_.AddJoint<RevoluteJoint>("shoulder", tree_.world_link(),
                                          offset, first, {},
                                          Eigen::Vector3d::UnitZ());
        const auto& elbow =
            tree_.AddJoint<RevoluteJoint>("elbow", first, offset, second, {},
                                          Eigen::Vector3d::UnitY());
        // The overload above creates a FixedOffsetFrame for its non-identity
        // parent offset. Keep that real mobilizer frame: changing it must move
        // the kinematics, unlike an otherwise unused frame attached to a body.
        fixed_frame_ =
            &dynamic_cast<const drake::multibody::FixedOffsetFrame<double>&>(
                shoulder.frame_on_parent());
        actuator_ = &tree_.AddJointActuator("elbow_motor", elbow);
        auto& mutable_actuator =
            tree_.get_mutable_joint_actuator(actuator_->index());
        mutable_actuator.set_default_rotor_inertia(0.2);
        mutable_actuator.set_default_gear_ratio(2.0);
        tree_.Finalize();
    }

    const MultibodyTree<double>& tree() const { return tree_; }
    const drake::multibody::RigidBody<double>& first_body() const {
        return tree_.GetLinkByName("first");
    }
    const drake::multibody::RigidBody<double>& second_body() const {
        return tree_.GetLinkByName("second");
    }
    const drake::multibody::FixedOffsetFrame<double>& fixed_frame() const {
        return *fixed_frame_;
    }
    const drake::multibody::JointActuator<double>& actuator() const {
        return *actuator_;
    }
    std::unique_ptr<Context> MakeContext() const {
        return tree_.CreateDefaultEvaluationContext();
    }

   private:
    MultibodyTree<double> tree_;
    const drake::multibody::FixedOffsetFrame<double>* fixed_frame_{};
    const drake::multibody::JointActuator<double>* actuator_{};
};

/// The pose of the last body in the world, as the position pass computed it.
Eigen::Vector3d LastBodyOrigin(const MultibodyTree<double>& tree,
                               const Context& context) {
    const auto& pc = tree.EvalPositionKinematics(context);
    return pc.get_X_WB(drake::multibody::internal::MobodIndex(
                           tree.num_mobods() - 1))
        .translation();
}

drake::math::RigidTransform<double> LastBodyPose(
    const MultibodyTree<double>& tree, const Context& context) {
    return tree.EvalPositionKinematics(context).get_X_WB(
        drake::multibody::internal::MobodIndex(tree.num_mobods() - 1));
}

drake::Vector6<double> LastBodySpatialVelocity(
    const MultibodyTree<double>& tree, const Context& context) {
    return tree
        .EvalVelocityKinematics(context)
        .get_V_WB(drake::multibody::internal::MobodIndex(
            tree.num_mobods() - 1))
        .get_coeffs();
}

Eigen::MatrixXd MassMatrix(const MultibodyTree<double>& tree,
                           const Context& context) {
    Eigen::MatrixXd result(tree.num_velocities(), tree.num_velocities());
    tree.CalcMassMatrix(context, &result);
    return result;
}

void SetJointAngles(const MultibodyTree<double>& tree, Context* context,
                    double first, double second) {
    Eigen::VectorXd q(tree.num_positions());
    q << first, second;
    tree.SetPositions(context, q);
}

struct CacheFreshness {
    bool frame_body_poses;
    bool position_kinematics;
    bool across_node_jacobian;
    bool velocity_kinematics;
    bool spatial_inertia_in_world;
    bool composite_body_inertia_in_world;
    bool dynamic_bias;
    bool spatial_acceleration_bias;
    bool reflected_inertia;
    bool articulated_body_inertia;
    bool articulated_body_force_bias;
};

void ExpectCacheFreshness(const Context& context,
                          const CacheFreshness& expected,
                          const std::string& reason) {
    const auto& caches = context.caches();
    const std::array<std::pair<bool, const char*>, 11> observations{{
        {caches.frame_body_poses.is_fresh(), "frame/body poses"},
        {caches.position_kinematics.is_fresh(), "position kinematics"},
        {caches.across_node_jacobian.is_fresh(), "across-node Jacobian"},
        {caches.velocity_kinematics.is_fresh(), "velocity kinematics"},
        {caches.spatial_inertia_in_world.is_fresh(), "world inertias"},
        {caches.composite_body_inertia_in_world.is_fresh(),
         "composite inertias"},
        {caches.dynamic_bias.is_fresh(), "dynamic bias"},
        {caches.spatial_acceleration_bias.is_fresh(),
         "spatial acceleration bias"},
        {caches.reflected_inertia.is_fresh(), "reflected inertia"},
        {caches.articulated_body_inertia.is_fresh(),
         "articulated body inertia"},
        {caches.articulated_body_force_bias.is_fresh(),
         "articulated body force bias"},
    }};
    const std::array<bool, 11> expected_values{{
        expected.frame_body_poses,
        expected.position_kinematics,
        expected.across_node_jacobian,
        expected.velocity_kinematics,
        expected.spatial_inertia_in_world,
        expected.composite_body_inertia_in_world,
        expected.dynamic_bias,
        expected.spatial_acceleration_bias,
        expected.reflected_inertia,
        expected.articulated_body_inertia,
        expected.articulated_body_force_bias,
    }};
    for (std::size_t index = 0; index < observations.size(); ++index) {
        ExpectTrue(
            observations[index].first == expected_values[index],
            reason + ": " + observations[index].second +
                (expected_values[index] ? " should remain fresh"
                                        : " should be stale"));
    }
}

void WarmEveryRetainedCache(const MultibodyTree<double>& tree,
                            const Context& context) {
    tree.EvalFrameBodyPoses(context);
    tree.EvalPositionKinematics(context);
    tree.EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
    tree.EvalVelocityKinematics(context);
    tree.EvalArticulatedBodyInertiaCache(context);

    Eigen::MatrixXd mass_matrix(tree.num_velocities(), tree.num_velocities());
    tree.CalcMassMatrix(context, &mass_matrix);
    Eigen::VectorXd bias(tree.num_velocities());
    tree.CalcBiasTerm(context, &bias);
    drake::multibody::MultibodyForces<double> forces(tree);
    drake::multibody::internal::ArticulatedBodyForceCache<double>
        force_workspace(tree.forest());
    tree.CalcArticulatedBodyForceCache(context, forces, &force_workspace);

    ExpectCacheFreshness(
        context,
        {/*frame_body_poses=*/true,
         /*position_kinematics=*/true,
         /*across_node_jacobian=*/true,
         /*velocity_kinematics=*/true,
         /*spatial_inertia_in_world=*/true,
         /*composite_body_inertia_in_world=*/true,
         /*dynamic_bias=*/true,
         /*spatial_acceleration_bias=*/true,
         /*reflected_inertia=*/true,
         /*articulated_body_inertia=*/true,
         /*articulated_body_force_bias=*/true},
        "warming the complete retained directory");
}

struct AbaObservations {
    Eigen::VectorXd dynamic_bias;
    Eigen::VectorXd articulated_body_inertia;
    Eigen::VectorXd spatial_acceleration_bias;
    Eigen::VectorXd articulated_body_force_bias;
};

Eigen::VectorXd FlattenArticulatedBodyInertia(
    const MultibodyTree<double>& tree,
    const drake::multibody::internal::ArticulatedBodyInertiaCache<double>&
        cache) {
    Eigen::VectorXd values((tree.num_mobods() - 1) * (36 + 36 + 6));
    int cursor = 0;
    for (int index = 1; index < tree.num_mobods(); ++index) {
        const Eigen::Matrix<double, 6, 6> matrix =
            cache
                .get_P_B_W(
                    drake::multibody::internal::MobodIndex(index))
                .CopyToFullMatrix6();
        values.segment<36>(cursor) =
            Eigen::Map<const Eigen::Matrix<double, 36, 1>>(matrix.data());
        cursor += 36;
        const Eigen::Matrix<double, 6, 6> outboard_matrix =
            cache
                .get_Pplus_PB_W(
                    drake::multibody::internal::MobodIndex(index))
                .CopyToFullMatrix6();
        values.segment<36>(cursor) =
            Eigen::Map<const Eigen::Matrix<double, 36, 1>>(
                outboard_matrix.data());
        cursor += 36;
        const auto& gain =
            cache.get_g_PB_W(drake::multibody::internal::MobodIndex(index));
        ExpectTrue(gain.cols() == 1,
                   "the ABA dependency fixture uses one-velocity mobilizers");
        values.segment<6>(cursor) = gain.col(0);
        cursor += 6;
    }
    return values;
}

template <typename SpatialQuantity>
Eigen::VectorXd FlattenSpatialQuantities(
    const std::vector<SpatialQuantity>& quantities) {
    Eigen::VectorXd values(
        static_cast<Eigen::Index>((quantities.size() - 1) * 6));
    int cursor = 0;
    for (std::size_t index = 1; index < quantities.size(); ++index) {
        values.segment<6>(cursor) = quantities[index].get_coeffs();
        cursor += 6;
    }
    return values;
}

AbaObservations ObserveAbaRetainedCaches(const MultibodyTree<double>& tree,
                                         const Context& context) {
    AbaObservations observations;
    Eigen::VectorXd projected_bias(tree.num_velocities());
    tree.CalcBiasTerm(context, &projected_bias);
    observations.dynamic_bias =
        FlattenSpatialQuantities(context.caches().dynamic_bias.value());
    observations.articulated_body_inertia =
        FlattenArticulatedBodyInertia(
            tree, tree.EvalArticulatedBodyInertiaCache(context));

    drake::multibody::MultibodyForces<double> forces(tree);
    forces.SetZero();
    drake::multibody::internal::ArticulatedBodyForceCache<double>
        force_workspace(tree.forest());
    tree.CalcArticulatedBodyForceCache(context, forces, &force_workspace);
    // The public ABA force calculation evaluates #13, whose calculator in turn
    // evaluates #12. Read the retained values only after that real consumer has
    // made both slots fresh; no test-only evaluation hook is needed.
    observations.spatial_acceleration_bias = FlattenSpatialQuantities(
        context.caches().spatial_acceleration_bias.value());
    observations.articulated_body_force_bias =
        FlattenSpatialQuantities(
            context.caches().articulated_body_force_bias.value());
    return observations;
}

enum class StateMutation {
    kGeneralizedPositions,
    kGeneralizedVelocities,
    kFixedFramePose,
    kRigidBodyInertia,
    kJointActuatorParameters,
};

void EstablishNondegenerateState(const TwoLinkChain& model,
                                 Context* context) {
    SetJointAngles(model.tree(), context, 0.21, -0.37);
    Eigen::VectorXd velocities(model.tree().num_velocities());
    velocities << 0.63, -0.46;
    model.tree().SetVelocities(context, velocities);
}

void ApplyMutation(const TwoLinkChain& model, StateMutation mutation,
                   Context* context) {
    switch (mutation) {
        case StateMutation::kGeneralizedPositions:
            SetJointAngles(model.tree(), context, -0.42, 0.58);
            return;
        case StateMutation::kGeneralizedVelocities: {
            Eigen::VectorXd velocities(model.tree().num_velocities());
            velocities << -0.31, 0.77;
            model.tree().SetVelocities(context, velocities);
            return;
        }
        case StateMutation::kFixedFramePose:
            model.tree().SetFixedFramePoseInParentFrame(
                context, model.fixed_frame(),
                drake::math::RigidTransform<double>(
                    drake::math::RotationMatrix<double>::MakeXRotation(0.29),
                    Eigen::Vector3d(0.24, -0.13, 0.19)));
            return;
        case StateMutation::kRigidBodyInertia:
            model.tree().SetRigidBodySpatialInertiaInBodyFrame(
                context, model.second_body(),
                SpatialInertia<double>::MakeFromCentralInertia(
                    2.6, Eigen::Vector3d(-0.09, 0.06, 0.11),
                    drake::multibody::RotationalInertia<double>(
                        0.051, 0.064, 0.079, -0.004, 0.002, 0.005)));
            return;
        case StateMutation::kJointActuatorParameters:
            model.tree().SetJointActuatorRotorInertia(
                context, model.actuator(), 0.83);
            model.tree().SetJointActuatorGearRatio(
                context, model.actuator(), -3.4);
            return;
    }
}

bool SameValues(const Eigen::VectorXd& first,
                const Eigen::VectorXd& second) {
    const double tolerance =
        256.0 * std::numeric_limits<double>::epsilon();
    return first.isApprox(second, tolerance);
}

void CheckAbaRetainedCacheDependenciesAgainstColdContexts() {
    const TwoLinkChain model;
    struct ExpectedDependencies {
        StateMutation source;
        const char* source_name;
        bool dynamic_bias;
        bool articulated_body_inertia;
        bool spatial_acceleration_bias;
        bool articulated_body_force_bias;
    };
    const std::array<ExpectedDependencies, 5> expectations{{
        {StateMutation::kGeneralizedPositions, "generalized positions", true,
         true, true, true},
        {StateMutation::kGeneralizedVelocities, "generalized velocities", true,
         false, true, true},
        {StateMutation::kFixedFramePose, "fixed-frame pose", true, true, true,
         true},
        {StateMutation::kRigidBodyInertia, "rigid-body inertia", true, true,
         false, true},
        {StateMutation::kJointActuatorParameters,
         "joint-actuator parameters", false, true, false, true},
    }};

    for (const ExpectedDependencies& expected : expectations) {
        const std::unique_ptr<Context> hot = model.MakeContext();
        const std::unique_ptr<Context> cold = model.MakeContext();
        EstablishNondegenerateState(model, hot.get());
        EstablishNondegenerateState(model, cold.get());
        const AbaObservations before =
            ObserveAbaRetainedCaches(model.tree(), *hot);

        ApplyMutation(model, expected.source, hot.get());
        ApplyMutation(model, expected.source, cold.get());
        const AbaObservations hot_after =
            ObserveAbaRetainedCaches(model.tree(), *hot);
        const AbaObservations cold_after =
            ObserveAbaRetainedCaches(model.tree(), *cold);

        const auto check = [&](const Eigen::VectorXd& previous,
                               const Eigen::VectorXd& hot_value,
                               const Eigen::VectorXd& cold_value,
                               bool should_depend,
                               const char* cache_name) {
            ExpectTrue(
                SameValues(hot_value, cold_value),
                std::string(cache_name) +
                    " agrees between invalidated-hot and cold contexts after "
                    "changing " +
                    expected.source_name);
            ExpectTrue(
                SameValues(previous, cold_value) != should_depend,
                std::string(cache_name) +
                    (should_depend ? " is observably sensitive to "
                                   : " is observably independent of ") +
                    expected.source_name);
        };
        check(before.dynamic_bias, hot_after.dynamic_bias,
              cold_after.dynamic_bias, expected.dynamic_bias,
              "dynamic bias (#10)");
        check(before.articulated_body_inertia,
              hot_after.articulated_body_inertia,
              cold_after.articulated_body_inertia,
              expected.articulated_body_inertia,
              "articulated-body inertia (#11)");
        check(before.spatial_acceleration_bias,
              hot_after.spatial_acceleration_bias,
              cold_after.spatial_acceleration_bias,
              expected.spatial_acceleration_bias,
              "spatial acceleration bias (#12)");
        check(before.articulated_body_force_bias,
              hot_after.articulated_body_force_bias,
              cold_after.articulated_body_force_bias,
              expected.articulated_body_force_bias,
              "articulated-body force bias (#13)");
    }
}

void CheckTheFiveVersionSourcesExpireExactlyTheirDeclaredSlots() {
    const TwoLinkChain model;

    {
        const std::unique_ptr<Context> context = model.MakeContext();
        WarmEveryRetainedCache(model.tree(), *context);
        SetJointAngles(model.tree(), context.get(), 0.3, -0.4);
        ExpectCacheFreshness(
            *context,
            {/*frame_body_poses=*/true,
             /*position_kinematics=*/false,
             /*across_node_jacobian=*/false,
             /*velocity_kinematics=*/false,
             /*spatial_inertia_in_world=*/false,
             /*composite_body_inertia_in_world=*/false,
             /*dynamic_bias=*/false,
             /*spatial_acceleration_bias=*/false,
             /*reflected_inertia=*/true,
             /*articulated_body_inertia=*/false,
             /*articulated_body_force_bias=*/false},
            "a generalized-position write");
    }

    {
        const std::unique_ptr<Context> context = model.MakeContext();
        WarmEveryRetainedCache(model.tree(), *context);
        Eigen::VectorXd velocities(model.tree().num_velocities());
        velocities << 0.7, -0.2;
        model.tree().SetVelocities(context.get(), velocities);
        ExpectCacheFreshness(
            *context,
            {/*frame_body_poses=*/true,
             /*position_kinematics=*/true,
             /*across_node_jacobian=*/true,
             /*velocity_kinematics=*/false,
             /*spatial_inertia_in_world=*/true,
             /*composite_body_inertia_in_world=*/true,
             /*dynamic_bias=*/false,
             /*spatial_acceleration_bias=*/false,
             /*reflected_inertia=*/true,
             /*articulated_body_inertia=*/true,
             /*articulated_body_force_bias=*/false},
            "a generalized-velocity write");
        const std::unique_ptr<Context> cold_context = model.MakeContext();
        model.tree().SetVelocities(cold_context.get(), velocities);
        const drake::Vector6<double> hot_value =
            LastBodySpatialVelocity(model.tree(), *context);
        const drake::Vector6<double> cold_value =
            LastBodySpatialVelocity(model.tree(), *cold_context);
        ExpectTrue(hot_value.norm() > 0.0 && hot_value.isApprox(cold_value),
                   "the invalidated velocity cache agrees with a cold context "
                   "at the same state");
    }

    {
        const std::unique_ptr<Context> context = model.MakeContext();
        WarmEveryRetainedCache(model.tree(), *context);
        const Eigen::Vector3d origin_before =
            LastBodyOrigin(model.tree(), *context);
        const drake::math::RigidTransform<double> changed_pose(
            drake::math::RotationMatrix<double>::MakeYRotation(0.4),
            Eigen::Vector3d(0.2, -0.1, 0.3));
        model.tree().SetFixedFramePoseInParentFrame(
            context.get(), model.fixed_frame(), changed_pose);
        ExpectCacheFreshness(
            *context,
            {/*frame_body_poses=*/false,
             /*position_kinematics=*/false,
             /*across_node_jacobian=*/false,
             /*velocity_kinematics=*/false,
             /*spatial_inertia_in_world=*/false,
             /*composite_body_inertia_in_world=*/false,
             /*dynamic_bias=*/false,
             /*spatial_acceleration_bias=*/false,
             /*reflected_inertia=*/true,
             /*articulated_body_inertia=*/false,
             /*articulated_body_force_bias=*/false},
            "a fixed-frame-pose write");
        const auto& frame_poses = model.tree().EvalFrameBodyPoses(*context);
        ExpectTrue(
            frame_poses.get_X_LF(model.fixed_frame().index())
                    .rotation()
                    .matrix()
                    .isApprox(changed_pose.rotation().matrix()) &&
                frame_poses.get_X_LF(model.fixed_frame().index())
                    .translation()
                    .isApprox(changed_pose.translation()),
            "the real frame-pose calculator reads the complete updated fixed "
            "frame pose, including its non-identity rotation");
        const std::unique_ptr<Context> cold_context = model.MakeContext();
        model.tree().SetFixedFramePoseInParentFrame(
            cold_context.get(), model.fixed_frame(), changed_pose);
        const drake::math::RigidTransform<double> hot_pose =
            LastBodyPose(model.tree(), *context);
        const drake::math::RigidTransform<double> cold_pose =
            LastBodyPose(model.tree(), *cold_context);
        ExpectTrue(
            !hot_pose.translation().isApprox(origin_before) &&
                hot_pose.GetAsMatrix34().isApprox(cold_pose.GetAsMatrix34()),
                   "the frame-dependent kinematics agrees with a cold context "
                   "in rotation and translation, and differs from its old value");
    }

    {
        const std::unique_ptr<Context> context = model.MakeContext();
        WarmEveryRetainedCache(model.tree(), *context);
        const Eigen::MatrixXd mass_before =
            MassMatrix(model.tree(), *context);
        model.tree().SetRigidBodySpatialInertiaInBodyFrame(
            context.get(), model.first_body(),
            SpatialInertia<double>::SolidBoxWithMass(3.0, 0.2, 0.1, 0.1));
        ExpectCacheFreshness(
            *context,
            {/*frame_body_poses=*/false,
             /*position_kinematics=*/true,
             /*across_node_jacobian=*/true,
             /*velocity_kinematics=*/true,
             /*spatial_inertia_in_world=*/false,
             /*composite_body_inertia_in_world=*/false,
             /*dynamic_bias=*/false,
             /*spatial_acceleration_bias=*/true,
             /*reflected_inertia=*/true,
             /*articulated_body_inertia=*/false,
             /*articulated_body_force_bias=*/false},
            "a rigid-body-inertia write");
        const std::unique_ptr<Context> cold_context = model.MakeContext();
        model.tree().SetRigidBodySpatialInertiaInBodyFrame(
            cold_context.get(), model.first_body(),
            SpatialInertia<double>::SolidBoxWithMass(3.0, 0.2, 0.1, 0.1));
        const Eigen::MatrixXd mass_hot = MassMatrix(model.tree(), *context);
        const Eigen::MatrixXd mass_cold =
            MassMatrix(model.tree(), *cold_context);
        ExpectTrue(!mass_hot.isApprox(mass_before) &&
                       mass_hot.isApprox(mass_cold),
                   "the invalidated inertia chain agrees with a cold context "
                   "and differs from its old value");
    }

    {
        const std::unique_ptr<Context> context = model.MakeContext();
        WarmEveryRetainedCache(model.tree(), *context);
        const Eigen::MatrixXd mass_before =
            MassMatrix(model.tree(), *context);
        model.tree().SetJointActuatorRotorInertia(
            context.get(), model.actuator(), 0.8);
        model.tree().SetJointActuatorGearRatio(
            context.get(), model.actuator(), 3.0);
        ExpectCacheFreshness(
            *context,
            {/*frame_body_poses=*/true,
             /*position_kinematics=*/true,
             /*across_node_jacobian=*/true,
             /*velocity_kinematics=*/true,
             /*spatial_inertia_in_world=*/true,
             /*composite_body_inertia_in_world=*/true,
             /*dynamic_bias=*/true,
             /*spatial_acceleration_bias=*/true,
             /*reflected_inertia=*/false,
             /*articulated_body_inertia=*/false,
             /*articulated_body_force_bias=*/false},
            "a joint-actuator-parameter write");
        model.tree().EvalArticulatedBodyInertiaCache(*context);
        ExpectTrue(context->caches().reflected_inertia.is_fresh(),
                   "the ABI chain really recomputes reflected inertia");
        ExpectTrue(context->caches().spatial_inertia_in_world.is_fresh(),
                   "the ABI chain really evaluates world inertias");
        const std::unique_ptr<Context> cold_context = model.MakeContext();
        model.tree().SetJointActuatorRotorInertia(
            cold_context.get(), model.actuator(), 0.8);
        model.tree().SetJointActuatorGearRatio(
            cold_context.get(), model.actuator(), 3.0);
        const Eigen::MatrixXd mass_hot = MassMatrix(model.tree(), *context);
        const Eigen::MatrixXd mass_cold =
            MassMatrix(model.tree(), *cold_context);
        ExpectTrue(!mass_hot.isApprox(mass_before) &&
                       mass_hot.isApprox(mass_cold),
                   "the actuator-dependent inertia agrees with a cold context "
                   "and differs from its old value");
    }
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

void CheckColdAndRepeatedReadsReturnTheSameRealValue() {
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
    const Eigen::Vector3d origin_before =
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

    const std::unique_ptr<Context> cold_context = model.MakeContext();
    SetJointAngles(model.tree(), cold_context.get(), 0.3, -0.4);
    const Eigen::Vector3d moved = LastBodyOrigin(model.tree(), *context);
    const Eigen::Vector3d cold_value =
        LastBodyOrigin(model.tree(), *cold_context);
    ExpectTrue(!moved.isApprox(origin_before) && moved.isApprox(cold_value),
               "and the recomputed pose changes and agrees with a cold "
               "context at the same state");
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
    model.tree().SetRigidBodySpatialInertiaInBodyFrame(
        context.get(), model.first_body(),
        SpatialInertia<double>::SolidBoxWithMass(2.0, 0.2, 0.1, 0.1));

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
    model.tree().EvalArticulatedBodyInertiaCache(*b);
    const Eigen::MatrixXd b_mass_before = MassMatrix(model.tree(), *b);
    const auto b_q_version_before =
        b->state().generalized_positions_version();
    const auto b_v_version_before =
        b->state().generalized_velocities_version();
    const auto b_frame_version_before =
        b->state().fixed_frame_poses_version();
    const auto b_inertia_version_before =
        b->state().rigid_body_inertias_version();
    const auto b_actuator_version_before =
        b->state().joint_actuator_parameters_version();
    model.tree().SetRigidBodySpatialInertiaInBodyFrame(
        a.get(), model.first_body(),
        SpatialInertia<double>::SolidBoxWithMass(2.0, 0.2, 0.1, 0.1));
    ExpectTrue(!a->caches().spatial_inertia_in_world.is_fresh(),
               "A's world inertias expired");
    ExpectTrue(a->caches().position_kinematics.is_fresh(),
               "A's position pass did not");
    ExpectTrue(b->caches().position_kinematics.is_fresh() &&
                   LastBodyOrigin(model.tree(), *b) == b_warm,
               "and B is untouched by A's inertia change");
    ExpectTrue(b->caches().spatial_inertia_in_world.is_fresh() &&
                   b->caches().articulated_body_inertia.is_fresh() &&
                   MassMatrix(model.tree(), *b).isApprox(b_mass_before),
               "B's warmed inertia chain and value remain unchanged");
    ExpectTrue(
        b->state().generalized_positions_version() == b_q_version_before &&
            b->state().generalized_velocities_version() == b_v_version_before &&
            b->state().fixed_frame_poses_version() ==
                b_frame_version_before &&
            b->state().rigid_body_inertias_version() ==
                b_inertia_version_before &&
            b->state().joint_actuator_parameters_version() ==
                b_actuator_version_before,
        "B's five state versions remain unchanged");
}

class QuaternionFloatingBody {
   public:
    explicit QuaternionFloatingBody(bool zero_default_quaternion = false) {
        const auto& body = tree_.AddLink(
            "floating",
            SpatialInertia<double>::SolidBoxWithMass(1.0, 0.2, 0.1, 0.1));
        tree_.AddJoint<drake::multibody::QuaternionFloatingJoint>(
            "floating_joint", tree_.world_link(), {}, body, {});
        if (zero_default_quaternion) {
            tree_
                .GetMutableJointByName<
                    drake::multibody::QuaternionFloatingJoint>(
                    "floating_joint")
                .set_default_quaternion(Eigen::Quaterniond(0.0, 0.0, 0.0,
                                                            0.0));
        }
        tree_.Finalize();
    }

    const MultibodyTree<double>& tree() const { return tree_; }
    std::unique_ptr<Context> MakeContext() const {
        return tree_.CreateDefaultEvaluationContext();
    }

   private:
    MultibodyTree<double> tree_;
};

void CheckTheQuaternionCommitGate() {
    const QuaternionFloatingBody model;
    const std::unique_ptr<Context> context = model.MakeContext();

    Eigen::VectorXd non_unit = context->state().generalized_positions();
    non_unit.template head<4>() << 2.0, 0.0, 0.0, 0.0;
    non_unit.template tail<3>() << 0.3, -0.4, 0.5;
    model.tree().SetPositions(context.get(), non_unit);
    ExpectTrue(context->state().generalized_positions() == non_unit,
               "a nonzero non-unit quaternion is stored exactly, not "
               "silently normalized");
    const Eigen::Matrix3d non_unit_rotation =
        LastBodyPose(model.tree(), *context).rotation().matrix();
    ExpectTrue(non_unit_rotation.allFinite() &&
                   non_unit_rotation.isApprox(Eigen::Matrix3d::Identity()),
               "a safely scaled identity quaternion evaluates to the same "
               "finite rotation");

    for (const double safe_scale : {1e-150, 1e150}) {
        Eigen::VectorXd safely_scaled = non_unit;
        safely_scaled.template head<4>() << safe_scale, safe_scale, 0.0, 0.0;
        model.tree().SetPositions(context.get(), safely_scaled);
        ExpectTrue(
            context->state().generalized_positions() == safely_scaled &&
                LastBodyPose(model.tree(), *context).rotation().matrix().allFinite(),
            "a finite quaternion inside the formula's numeric domain is "
            "accepted without an arbitrary magnitude policy");
    }

    model.tree().SetPositions(context.get(), non_unit);
    const Eigen::VectorXd before_zero =
        context->state().generalized_positions();
    const auto version_before_zero =
        context->state().generalized_positions_version();
    Eigen::VectorXd zero = before_zero;
    zero.template head<4>().setZero();
    bool zero_was_rejected = false;
    try {
        model.tree().SetPositions(context.get(), zero);
    } catch (const std::invalid_argument&) {
        zero_was_rejected = true;
    }
    ExpectTrue(zero_was_rejected,
               "a real quaternion mobilizer rejects an exactly zero "
               "quaternion");
    ExpectTrue(context->state().generalized_positions() == before_zero &&
                   context->state().generalized_positions_version() ==
                       version_before_zero,
               "a refused zero quaternion changes neither q nor its version");

    for (const double unsafe_scale :
         {std::numeric_limits<double>::max(),
          std::numeric_limits<double>::min()}) {
        Eigen::VectorXd unsafe = before_zero;
        unsafe.template head<4>() << unsafe_scale, unsafe_scale, 0.0, 0.0;
        bool unsafe_was_rejected = false;
        try {
            model.tree().SetPositions(context.get(), unsafe);
        } catch (const std::invalid_argument&) {
            unsafe_was_rejected = true;
        }
        ExpectTrue(
            unsafe_was_rejected,
            "a finite quaternion whose squared norm cannot support the "
            "rotation formula is refused");
        ExpectTrue(
            context->state().generalized_positions() == before_zero &&
                context->state().generalized_positions_version() ==
                    version_before_zero,
            "a refused numerically unusable quaternion changes neither q nor "
            "its version");
    }

    Eigen::VectorXd short_positions(model.tree().num_positions() - 1);
    short_positions.setZero();
    bool short_vector_was_rejected = false;
    try {
        model.tree().SetPositions(context.get(), short_positions);
    } catch (const std::invalid_argument&) {
        short_vector_was_rejected = true;
    }
    ExpectTrue(short_vector_was_rejected,
               "a short q is rejected before quaternion segments are read");
    ExpectTrue(context->state().generalized_positions() == before_zero &&
                   context->state().generalized_positions_version() ==
                       version_before_zero,
               "a refused short q changes neither q nor its version");

    const QuaternionFloatingBody invalid_default(
        /*zero_default_quaternion=*/true);
    bool invalid_default_was_rejected = false;
    try {
        const auto invalid_context = invalid_default.MakeContext();
        (void)invalid_context;
    } catch (const std::invalid_argument&) {
        invalid_default_was_rejected = true;
    }
    ExpectTrue(
        invalid_default_was_rejected,
        "the model-aware factory routes default q through the quaternion gate");
}

void CheckSpatialInertiaParameterConversionPreservesEveryField() {
    using orvd::multibody_runtime::RigidBodyInertiaParameters;
    using orvd::rigid_multibody_tree::internal::ToInertiaParameters;
    using orvd::rigid_multibody_tree::internal::ToSpatialInertia;

    const Eigen::Vector3d center_of_mass(0.12, -0.23, 0.31);
    Eigen::Matrix3d central_unit_inertia;
    central_unit_inertia << 0.42, 0.017, -0.026, 0.017, 0.53, 0.031, -0.026,
        0.031, 0.64;
    const Eigen::Matrix3d shift =
        center_of_mass.squaredNorm() * Eigen::Matrix3d::Identity() -
        center_of_mass * center_of_mass.transpose();
    const Eigen::Matrix3d about_body_origin = central_unit_inertia + shift;

    RigidBodyInertiaParameters original;
    original.mass_kilograms = 3.7;
    original.center_of_mass_in_body_frame = center_of_mass;
    original.unit_inertia_moments = about_body_origin.diagonal();
    original.unit_inertia_products =
        Eigen::Vector3d(about_body_origin(0, 1), about_body_origin(0, 2),
                        about_body_origin(1, 2));

    const SpatialInertia<double> spatial = ToSpatialInertia(original);
    const RigidBodyInertiaParameters round_trip =
        ToInertiaParameters(spatial);
    const double tolerance =
        64.0 * std::numeric_limits<double>::epsilon();
    ExpectTrue(round_trip.mass_kilograms == original.mass_kilograms,
               "inertia conversion preserves a nontrivial mass");
    ExpectTrue(
        round_trip.center_of_mass_in_body_frame.isApprox(
            original.center_of_mass_in_body_frame, tolerance),
        "inertia conversion preserves all three nonzero centre-of-mass "
        "components");
    ExpectTrue(
        round_trip.unit_inertia_moments.isApprox(
            original.unit_inertia_moments, tolerance),
        "inertia conversion preserves three distinct unit-inertia moments");
    ExpectTrue(
        round_trip.unit_inertia_products.isApprox(
            original.unit_inertia_products, tolerance),
        "inertia conversion preserves three distinct nonzero products in "
        "Ixy, Ixz, Iyz order");
}

void CheckPhysicalParameterDoorsRejectAContextFromAnotherModel() {
    const TwoLinkChain first_model;
    const TwoLinkChain second_model;
    const std::unique_ptr<Context> second_context = second_model.MakeContext();
    bool foreign_context_was_rejected = false;
    try {
        first_model.tree().SetRigidBodySpatialInertiaInBodyFrame(
            second_context.get(), first_model.first_body(),
            SpatialInertia<double>::SolidBoxWithMass(2.0, 0.2, 0.1, 0.1));
    } catch (const std::invalid_argument&) {
        foreign_context_was_rejected = true;
    }
    ExpectTrue(foreign_context_was_rejected,
               "a physical parameter door rejects a context from another "
               "finalized model");
}

}  // namespace

int main() {
    CheckTheFactoryIsTheOnlyDoor();
    CheckColdAndRepeatedReadsReturnTheSameRealValue();
    CheckTheFiveVersionSourcesExpireExactlyTheirDeclaredSlots();
    CheckAbaRetainedCacheDependenciesAgainstColdContexts();
    CheckAPositionWriteExpiresOnlyWhatReadsPositions();
    CheckAMassWriteExpiresInertiaButNotPosition();
    CheckTwoContextsWithEqualVersionsStayApart();
    CheckTheQuaternionCommitGate();
    CheckSpatialInertiaParameterConversionPreservesEveryField();
    CheckPhysicalParameterDoorsRejectAContextFromAnotherModel();

    if (failure_count > 0) {
        std::printf("%d rigid tree cache binding check(s) failed\n",
                    failure_count);
        return 1;
    }
    std::printf(
        "the eleven real cache slots match the runtime contract, model-aware "
        "writes preserve their gates, and equal-version contexts keep their "
        "own answers\n");
    return 0;
}
