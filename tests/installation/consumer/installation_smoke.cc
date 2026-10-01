#include <algorithm>
#include <array>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <cmath>
#include <cstdio>
#include <exception>

#include <Eigen/Dense>

#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "orvd/track_geometry/track_geometry.h"

#if __has_include("orvd/integrators/cvode_continuous_state_advancer.h")
#error "Concrete CVODE backend must not be installed"
#endif

namespace {

using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemContinuousStateAdvancer;
using orvd::integrators::ContinuousStateNumericalFailure;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackScalarProfile;
using orvd::track_geometry::TrackScalarSegment;
using orvd::track_geometry::TrackScalarSegmentShape;
using orvd::track_geometry::TrackVerticalProfile;

bool Near(double measured, double expected) {
    return std::abs(measured - expected) <=
           1.0e-8 *
               std::max({1.0, std::abs(measured), std::abs(expected)});
}

int RunInstalledNumericalFailureSmoke() {
    using Reason = ContinuousStateNumericalFailure::Reason;
    // Existing serialized classifications keep their values; single-attempt
    // basic methods have distinct appended reasons, not fictitious retries.
    static_assert(static_cast<int>(Reason::kAdvanceWorkBudgetExhausted) == 0);
    static_assert(static_cast<int>(Reason::kNonFiniteLinearSystem) == 7);
    for (const auto reason : {Reason::kNonFiniteState,
                              Reason::kNonlinearConvergenceFailure,
                              Reason::kSingularLinearSystem}) {
        try {
            throw ContinuousStateNumericalFailure(reason, 19, "installed failure classification");
        } catch (const ContinuousStateNumericalFailure& failure) {
            if (failure.reason() != reason || failure.backend_code() != 19) return 1;
        }
    }
    return 0;
}

// The installed line layer, exercised through its own public header rather than
// merely linked: a straight, level stretch whose centerline and track frame
// have closed-form values, so a consumer that compiled against a stale header
// or an empty archive would not reach the end of this function.
int RunInstalledLineSmoke() {
    TrackScalarSegment level;
    level.length_meters = 100.0;
    level.shape = TrackScalarSegmentShape::kConstant;
    level.start_value = 0.0;
    level.end_value = 0.0;
    const TrackGeometry line(TrackScalarProfile(0.0, {level}, {}),
                             TrackScalarProfile(0.0, {level}, {}),
                             TrackVerticalProfile(
                                 0.0,
                                 {orvd::track_geometry::ConstantGradeSegment{
                                     100.0, 0.0}},
                                 {}),
                             1.5, 1.0);
    const auto kinematics = line.EvaluateTrackFrame(40.0);
    const Eigen::Vector3d origin = kinematics.pose().origin_in_inertial_meters();
    const Eigen::Matrix3d rotation =
        kinematics.pose().rotation_inertial_from_track();
    if (!Near(origin.x(), 40.0) || !Near(origin.y(), 0.0) ||
        !Near(origin.z(), 0.0) ||
        (rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1.0e-12) {
        std::fprintf(stderr,
                     "installed ORVD line smoke produced origin (% .17g, % .17g,"
                     " % .17g) on a straight level line\n",
                     origin.x(), origin.y(), origin.z());
        return 1;
    }
    return 0;
}

// The installed multibody facade must carry the public Ball-RPY symbol and its
// link-only rigid-tree implementation. This is a real kinematics and dynamics
// use, not a header or archive-presence check.
int RunInstalledBallRpySmoke() {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 23.958904;
    inertia.unit_inertia_moments = Eigen::Vector3d(0.058, 0.001, 0.058);

    MultibodyModel model;
    const auto bar = model.AddRigidBody("longitudinal_bar", inertia);
    const Eigen::Vector3d default_angles(0.012, -0.009,
                                         -4.57347190844519);
    const auto ball = model.AddBallRpyJoint(
        "axle_bridge_ball", model.world_frame(), model.body_frame(bar),
        default_angles);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();

    const auto q_range = model.GetJointPositionRange(ball);
    const auto v_range = model.GetJointVelocityRange(ball);
    auto context = model.CreateDefaultContext();
    const auto pose = model.CalcPoseInWorld(*context, bar);
    Eigen::MatrixXd mass_matrix(3, 3);
    model.CalcGeneralizedMassMatrix(*context, mass_matrix);
    if (q_range.size() != 3 || v_range.size() != 3 ||
        !(context->generalized_positions().segment<3>(q_range.start()) ==
          default_angles) ||
        !pose.rotation().allFinite() || !mass_matrix.allFinite() ||
        mass_matrix.determinant() <= 0.0) {
        std::fprintf(stderr,
                     "installed ORVD Ball-RPY smoke did not preserve its "
                     "3q/3v state and finite dynamics\n");
        return 1;
    }

    // Exercise the newly installed second-coordinate-derivative symbol with a
    // nonzero velocity bias. The reference differentiates omega = T(q) qdot.
    const Eigen::Vector3d omega(0.23, -0.17, 0.31);
    const Eigen::Vector3d alpha(0.07, 0.12, -0.09);
    Eigen::VectorXd velocities = omega;
    Eigen::VectorXd accelerations = alpha;
    model.SetGeneralizedVelocities(context.get(), velocities);
    Eigen::VectorXd qddot(3);
    model.MapGeneralizedVelocityDerivativesToPositionSecondDerivatives(
        *context, accelerations, &qddot);
    const double cp = std::cos(default_angles[1]);
    const double sp = std::sin(default_angles[1]);
    const double cy = std::cos(default_angles[2]);
    const double sy = std::sin(default_angles[2]);
    Eigen::Matrix3d velocity_map;
    velocity_map << cy * cp, -sy, 0.0,
                   sy * cp,  cy, 0.0,
                       -sp, 0.0, 1.0;
    const Eigen::Vector3d rates = velocity_map.fullPivLu().solve(omega);
    Eigen::Matrix3d map_derivative = Eigen::Matrix3d::Zero();
    map_derivative(0, 0) = -sy * rates[2] * cp - cy * sp * rates[1];
    map_derivative(0, 1) = -cy * rates[2];
    map_derivative(1, 0) = cy * rates[2] * cp - sy * sp * rates[1];
    map_derivative(1, 1) = -sy * rates[2];
    map_derivative(2, 0) = -cp * rates[1];
    const Eigen::Vector3d expected_qddot =
        velocity_map.fullPivLu().solve(alpha - map_derivative * rates);
    if (!qddot.allFinite() ||
        (qddot - expected_qddot).cwiseAbs().maxCoeff() > 1e-12 ||
        !(context->generalized_positions() == default_angles) ||
        !(context->generalized_velocities() == velocities)) {
        std::fprintf(stderr,
                     "installed ORVD second-coordinate-derivative mapping "
                     "failed its Ball-RPY acceleration or state-purity check\n");
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    try {
        constexpr double kMassKilograms = 2.0;
        constexpr double kUnitInertia = 0.5;
        constexpr double kDamping = 0.4;
        constexpr double kInitialPosition = 0.2;
        constexpr double kInitialVelocity = 1.4;
        constexpr double kTargetTime = 0.1;

        RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = kMassKilograms;
        inertia.center_of_mass_in_body_frame.setZero();
        inertia.unit_inertia_moments.setConstant(kUnitInertia);
        inertia.unit_inertia_products.setZero();

        MultibodyModel model;
        const auto rotor = model.AddRigidBody("rotor", inertia);
        const auto bearing = model.AddRevoluteJoint(
            "bearing", model.world_frame(), model.body_frame(rotor),
            Eigen::Vector3d::UnitZ(), kDamping);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
        if (model.GetJointType(bearing) != orvd::multibody_model::JointType::kRevolute) {
            throw std::runtime_error("installed joint type query mismatch");
        }

        const SystemAssemblyDescription description(model);
        const SystemInstance system(description);
        const CompiledSystemPlan plan(system);
        using namespace orvd::integrators;
        static_assert(!std::is_constructible_v<SystemContinuousStateAdvancer,
            const SystemInstance&, const CompiledSystemPlan&,
            orvd::system_assembly::SystemRuntimeContext&,
            ContinuousStateErrorTolerances, NoCallTimeAppliedForces>);
        const auto tolerances = [] {
            return ContinuousStateErrorTolerances(
                1.0e-10, Eigen::VectorXd::Constant(2, 1.0e-12));
        };
        constexpr double h = 1e-4;
        const NewmarkConfiguration newmark{h, {12,
            {1e-11, 1e-11, 1e-11, 1.0}, {1e-11, 1e-11, 1e-11, 1.0},
            {1e-11, 1e-11, 1e-11, 1.0}, {1e-11, 1e-11, 1.0}}};
        const std::array<SystemIntegrationMethodConfiguration, 5> methods{
            CvodeBdf2Configuration{tolerances()}, CvodeBdf5Configuration{tolerances()},
            Radau5Configuration{tolerances()}, newmark, ZhaiConfiguration{h}};
        const std::array<std::string_view, 5> identifiers{
            "cvode_bdf2", "cvode_bdf5", "radau5", "newmark", "zhai"};
        const double decay_rate = kDamping / (kMassKilograms * kUnitInertia);
        const double expected_velocity = kInitialVelocity * std::exp(-decay_rate * kTargetTime);
        const double expected_position = kInitialPosition + kInitialVelocity *
            (1.0 - std::exp(-decay_rate * kTargetTime)) / decay_rate;
        for (std::size_t index = 0; index < methods.size(); ++index) {
            auto context = system.CreateDefaultRuntimeContext(0.0);
            system.SetContinuousState(*context, Eigen::Vector2d(kInitialPosition, kInitialVelocity));
            SystemContinuousStateAdvancer advancer(
                system, plan, *context, SystemIntegrationConfiguration{methods[index]},
                NoCallTimeAppliedForces{});
            advancer.AdvanceTo(kTargetTime);
            Eigen::VectorXd observed(system.continuous_state_size());
            system.CopyContinuousState(*context, observed);
            if (advancer.method_identifier() != identifiers[index] ||
                context->time_seconds() != kTargetTime || !observed.allFinite() ||
                !Near(observed[0], expected_position) || !Near(observed[1], expected_velocity) ||
                advancer.integration_statistics().successful_internal_step_count == 0) {
                std::fprintf(stderr, "installed method %.*s failed analytic rotor smoke\n",
                    static_cast<int>(identifiers[index].size()), identifiers[index].data());
                return 1;
            }
        }
        if (RunInstalledLineSmoke() != 0) {
            return 1;
        }
        if (RunInstalledBallRpySmoke() != 0) {
            return 1;
        }
        if (RunInstalledNumericalFailureSmoke() != 0) {
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed ORVD smoke failed: %s\n", error.what());
        return 1;
    }
    return 0;
}
