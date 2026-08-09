// G53/G54: the installed GZ18 contact personality enters the vehicle RHS, and
// the same real scenario advances its local projection branches at CVODE's
// accepted internal endpoints.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <omp.h>

#include "orvd/configuration/assembled_gz18_contact_scenario.h"
#include "orvd/configuration/gz18_wheel_rail_contact.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/wheel_rail_contact/contact_wrench.h"
#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"
#include "wheel_rail_contact/allocation_probe.h"

namespace {

using orvd::configuration::AssembledGz18ContactScenario;
using orvd::configuration::AssembleGz18ContactScenario;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::LoadTrackGeometryFromJsonFile;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::configuration::ResolvedWheelsetPlacement;
using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemContinuousStateAdvancer;
using orvd::multibody_model::AppliedBodyWrench;
using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackScalarProfile;
using orvd::track_geometry::TrackScalarSegment;
using orvd::track_geometry::TrackScalarSegmentShape;
using orvd::track_geometry::TrackStationRegion;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "GZ18 contact RHS: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

double RelativeTolerance(double scale) {
    return 2.0e-10 * std::max(1.0, scale);
}

bool Near(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected) {
    return (actual - expected).lpNorm<Eigen::Infinity>() <=
           RelativeTolerance(expected.lpNorm<Eigen::Infinity>());
}

bool Near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
          double relative = 2.0e-10) {
    return (actual - expected).lpNorm<Eigen::Infinity>() <=
           relative * std::max(1.0, expected.lpNorm<Eigen::Infinity>());
}

ContinuousStateErrorTolerances MakeGz18Tolerances() {
    Eigen::VectorXd absolute = Eigen::VectorXd::Constant(109, 1.0e-7);
    absolute.head(57).setConstant(1.0e-8);
    absolute.tail(2).setConstant(1.0);
    return ContinuousStateErrorTolerances(1.0e-6, std::move(absolute));
}

TrackScalarProfile ConstantProfile(double length_meters, double value) {
    return TrackScalarProfile(
        0.0,
        {TrackScalarSegment{length_meters, TrackScalarSegmentShape::kConstant,
                            value, value}},
        {});
}

TrackGeometry MakeConstantGradeLine(double grade) {
    constexpr double kLengthMeters = 2000.0;
    return TrackGeometry(ConstantProfile(kLengthMeters, 0.0),
                         ConstantProfile(kLengthMeters, 0.0),
                         ConstantProfile(kLengthMeters, grade), 1.5, 0.5);
}

bool SameWrenchComponents(const AppliedBodyWrench& actual,
                          const AppliedBodyWrench& expected) {
    return actual.point_position_in_body_frame_meters ==
               expected.point_position_in_body_frame_meters &&
           actual.force_newtons == expected.force_newtons &&
           actual.torque_about_point_newton_metres ==
               expected.torque_about_point_newton_metres;
}

bool SameObservation(
    const orvd::forces::WheelRailContactInterfaceObservation& actual,
    const orvd::forces::WheelRailContactInterfaceObservation& expected) {
    return actual.contact_patch_count == expected.contact_patch_count &&
           actual.vertical_support_force_on_wheel_newtons ==
               expected.vertical_support_force_on_wheel_newtons &&
           actual.normal_force_newtons == expected.normal_force_newtons &&
           actual.longitudinal_force_on_wheel_newtons ==
               expected.longitudinal_force_on_wheel_newtons &&
           actual.lateral_force_on_wheel_newtons ==
               expected.lateral_force_on_wheel_newtons;
}

std::vector<AppliedBodyWrench> EvaluateContactForces(
    const AssembledGz18ContactScenario& scenario,
    orvd::forces::WheelRailContactForceWorkspace& workspace) {
    const auto& assembled = scenario.vehicle_system();
    const auto* plan = assembled.contact_force_plan();
    std::vector<AppliedBodyWrench> wrenches(
        static_cast<std::size_t>(plan->body_wrench_count()));
    auto view = assembled.system().GetMultibodyComponentView(
        scenario.initial_context().context(),
        assembled.system().multibody_component());
    plan->CalcAppliedForces(
        view.context(), workspace,
        scenario.initial_context()
            .context()
            .wheel_rail_projection_station_hints_meters(),
        wrenches);
    return wrenches;
}

orvd::configuration::FreeBodyStartupState& MutableBodyState(
    orvd::configuration::ResolvedStartupState& startup,
    std::string_view name) {
    for (auto& state : startup.free_body_startup_states) {
        if (state.body_name == name) {
            return state;
        }
    }
    throw std::logic_error("test fixture has no free body named " +
                           std::string(name));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr,
                     "usage: verify_gz18_contact_rhs VEHICLE STARTUP LINE "
                     "DATA_ROOT\n");
        return 2;
    }

    const auto vehicle = LoadVehicleDefinitionFromJsonFile(argv[1]);
    const auto startup = LoadResolvedStartupStateFromJsonFile(argv[2]);
    auto line = LoadTrackGeometryFromJsonFile(argv[3]);
    constexpr double kReferenceStationMeters = 0.0;
    constexpr double kProjectionHalfWidthMeters = 0.01;
    std::unique_ptr<AssembledGz18ContactScenario> scenario =
        AssembleGz18ContactScenario(
            vehicle, startup, std::move(line), std::filesystem::path(argv[4]),
            kReferenceStationMeters, kProjectionHalfWidthMeters);

    const auto& assembled = scenario->vehicle_system();
    const auto* contact_plan = assembled.contact_force_plan();
    Require(contact_plan != nullptr, "the assembled system lost its contact plan");
    if (contact_plan == nullptr) {
        return 1;
    }
    Require(contact_plan->carrier_count() == 4,
            "the GZ18 plan does not have four shared wheelset carriers");
    Require(contact_plan->interface_count() == 8 &&
                contact_plan->body_wrench_count() == 8,
            "the GZ18 plan does not have eight fixed contact interfaces");
    Require(assembled.system().vehicle_body_wrench_count() == 56 &&
                assembled.system().contact_body_wrench_count() == 8,
            "the two force families do not occupy the expected fixed slices");
    Require(assembled.system().continuous_state_size() == 109,
            "contact changed the GZ18 continuous-state dimension");

    auto& runtime_context = scenario->initial_context().context();

    // Exercise the combined accepted-state transaction directly on the real
    // GZ18 context. Invalid projection hints must be rejected before a valid,
    // deliberately changed time or state can be partially committed.
    Eigen::VectorXd transaction_baseline_state(109);
    assembled.system().CopyContinuousState(runtime_context,
                                           transaction_baseline_state);
    const double transaction_baseline_time = runtime_context.time_seconds();
    const std::vector<double> transaction_baseline_hints(
        runtime_context.wheel_rail_projection_station_hints_meters().begin(),
        runtime_context.wheel_rail_projection_station_hints_meters().end());
    Eigen::VectorXd transaction_candidate_state = transaction_baseline_state;
    transaction_candidate_state.tail(2) += Eigen::Vector2d(1.0, -1.0);
    const auto RequireProjectionTransactionRefusal =
        [&](std::span<const double> invalid_hints, std::string_view message) {
            bool rejected = false;
            try {
                assembled.system()
                    .SetTimeContinuousStateAndWheelRailProjectionHints(
                        runtime_context, transaction_baseline_time + 0.125,
                        transaction_candidate_state, invalid_hints);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            Eigen::VectorXd state_after_refusal(109);
            assembled.system().CopyContinuousState(runtime_context,
                                                   state_after_refusal);
            Require(rejected &&
                        runtime_context.time_seconds() ==
                            transaction_baseline_time &&
                        state_after_refusal == transaction_baseline_state &&
                        std::equal(
                            transaction_baseline_hints.begin(),
                            transaction_baseline_hints.end(),
                            runtime_context
                                .wheel_rail_projection_station_hints_meters()
                                .begin()),
                    message);
        };
    std::vector<double> wrong_count_hints = transaction_baseline_hints;
    wrong_count_hints.pop_back();
    RequireProjectionTransactionRefusal(
        wrong_count_hints,
        "wrong-sized projection hints partially committed accepted state");
    std::vector<double> nonfinite_hints = transaction_baseline_hints;
    nonfinite_hints.front() = std::numeric_limits<double>::quiet_NaN();
    RequireProjectionTransactionRefusal(
        nonfinite_hints,
        "non-finite projection hints partially committed accepted state");

    std::unordered_map<std::string, double> resolved_station;
    std::unordered_map<std::string, const ResolvedWheelsetPlacement*>
        resolved_placement;
    for (const ResolvedWheelsetPlacement& placement :
         scenario->initial_context().wheelset_placements()) {
        resolved_station.emplace(placement.wheelset_body_name,
                                 placement.track_station_meters);
        resolved_placement.emplace(placement.wheelset_body_name, &placement);
    }
    struct ExpectedWheelsetStation {
        std::string_view body_name;
        double station_meters;
        TrackStationRegion region;
    };
    constexpr std::array<ExpectedWheelsetStation, 4> kExpectedWheelsetStations{
        ExpectedWheelsetStation{"front_leading_wheelset", 9.1,
                                TrackStationRegion::kWithinDefinedInterval},
        ExpectedWheelsetStation{"front_trailing_wheelset", 6.6,
                                TrackStationRegion::kWithinDefinedInterval},
        ExpectedWheelsetStation{"rear_leading_wheelset", -6.6,
                                TrackStationRegion::kBeforeDefinedInterval},
        ExpectedWheelsetStation{"rear_trailing_wheelset", -9.1,
                                TrackStationRegion::kBeforeDefinedInterval},
    };
    for (const ExpectedWheelsetStation& expected :
         kExpectedWheelsetStations) {
        const std::string name(expected.body_name);
        Require(resolved_station.contains(name) &&
                    resolved_station.at(name) == expected.station_meters,
                "the formal zero-reference GZ18 scene lost an authoritative "
                "wheelset station");
        Require(contact_plan->track_geometry().ClassifyTrackStation(
                    expected.station_meters) == expected.region,
                "a formal GZ18 wheelset is in the wrong base-track definition "
                "region");
    }
    for (int carrier = 0; carrier < contact_plan->carrier_count(); ++carrier) {
        const std::string name(contact_plan->carrier_name(carrier));
        Require(resolved_station.contains(name),
                "a frozen carrier has no resolved wheelset placement");
        if (resolved_station.contains(name)) {
            Require(contact_plan->initial_projection_station_meters(carrier) ==
                        resolved_station.at(name),
                    "a projection anchor differs from its resolved wheelset "
                    "station");
        }
        Require(runtime_context
                        .wheel_rail_projection_station_hints_meters()[
                            static_cast<std::size_t>(carrier)] ==
                    contact_plan->initial_projection_station_meters(carrier),
                "a runtime projection history was not seeded from its G53 "
                "anchor");
        Require(contact_plan->interface_name(2 * carrier) == name + ".right" &&
                    contact_plan->interface_name(2 * carrier + 1) ==
                        name + ".left",
                "a carrier does not own one right and one left interface");
    }

    Eigen::VectorXd compiled_derivatives(109);
    assembled.compiled_plan().CalcStateTimeDerivatives(
        runtime_context, compiled_derivatives);
    Require(compiled_derivatives.allFinite(),
            "the first contact-enabled direct RHS is not finite");

    // Rebuild the same static evaluation order through the public typed plans.
    // This is independent of CompiledSystemPlan's wrench slicing and catches a
    // missing source, overlapping slices or forward dynamics evaluated early.
    auto component = assembled.system().GetMultibodyComponentView(
        runtime_context, assembled.system().multibody_component());
    std::vector<AppliedBodyWrench> all_wrenches(64);
    Eigen::VectorXd series_derivatives(2);
    assembled.force_plan().CalcAppliedForces(
        component.context(), runtime_context.series_spring_damper_forces(),
        runtime_context.nominal_forces(),
        std::span(all_wrenches).first(56), series_derivatives);
    auto independent_contact_workspace = contact_plan->CreateWorkspace();
    contact_plan->CalcAppliedForces(
        component.context(), *independent_contact_workspace,
        runtime_context.wheel_rail_projection_station_hints_meters(),
        std::span(all_wrenches).subspan(56, 8));
    Eigen::VectorXd multibody_derivatives(107);
    component.model().CalcStateTimeDerivatives(
        component.context(), all_wrenches, {}, {},
        component.forward_dynamics_workspace(), multibody_derivatives);
    Eigen::VectorXd independently_rebuilt(109);
    independently_rebuilt.head(107) = multibody_derivatives;
    independently_rebuilt.tail(2) = series_derivatives;
    Require(Near(compiled_derivatives, independently_rebuilt),
            "the compiled direct RHS differs from its independently rebuilt "
            "typed-force order");

    // One binary and one real GZ18 state request 1/2/4/8 workers through the
    // standard OpenMP process setting. The runtime may apply a tighter resource
    // limit. Each wheel owns a fixed output slot and no
    // cross-wheel floating-point reduction exists, so anything short of
    // bitwise equality would expose shared scratch or nondeterministic publish
    // order rather than an acceptable parallel rounding difference.
    const int original_openmp_dynamic = omp_get_dynamic();
    const int original_openmp_max_threads = omp_get_max_threads();
    omp_set_dynamic(0);
    std::array<AppliedBodyWrench, 8> reference_parallel_wrenches;
    std::array<orvd::forces::WheelRailContactInterfaceObservation, 8>
        reference_parallel_observations;
    Eigen::VectorXd reference_parallel_rhs(109);
    bool have_parallel_reference = false;
    for (const int requested_threads : {1, 2, 4, 8}) {
        omp_set_num_threads(requested_threads);
        std::array<AppliedBodyWrench, 8> candidate_wrenches;
        std::array<orvd::forces::WheelRailContactInterfaceObservation, 8>
            candidate_observations;
        contact_plan->CalcAppliedForcesAndObservations(
            component.context(), *independent_contact_workspace,
            runtime_context.wheel_rail_projection_station_hints_meters(),
            candidate_wrenches, candidate_observations);
        Eigen::VectorXd candidate_rhs(109);
        assembled.compiled_plan().CalcStateTimeDerivatives(runtime_context,
                                                            candidate_rhs);
        if (!have_parallel_reference) {
            reference_parallel_wrenches = candidate_wrenches;
            reference_parallel_observations = candidate_observations;
            reference_parallel_rhs = candidate_rhs;
            have_parallel_reference = true;
            continue;
        }
        for (std::size_t ordinal = 0; ordinal < candidate_wrenches.size();
             ++ordinal) {
            Require(candidate_wrenches[ordinal].body ==
                            reference_parallel_wrenches[ordinal].body &&
                        candidate_wrenches[ordinal].expressed_in_frame ==
                            reference_parallel_wrenches[ordinal]
                                .expressed_in_frame &&
                        SameWrenchComponents(
                            candidate_wrenches[ordinal],
                            reference_parallel_wrenches[ordinal]),
                    "changing the requested OpenMP worker count changed an interface "
                    "wrench");
            Require(SameObservation(
                        candidate_observations[ordinal],
                        reference_parallel_observations[ordinal]),
                    "changing the requested OpenMP worker count changed an interface "
                    "observation");
        }
        Require(candidate_rhs == reference_parallel_rhs,
                "changing the requested OpenMP worker count changed the "
                "109-state RHS");
    }

    // The eight-worker request has now entered its OpenMP path and warmed every
    // interface workspace. Repeated calls must not reach the ordinary global
    // C++ allocation replacements; the probe intentionally does not claim to
    // intercept direct malloc or an OpenMP runtime's private allocator.
    std::size_t rhs_allocations = 0;
    {
        orvd::test::AllocationScope allocation_scope;
        assembled.compiled_plan().CalcStateTimeDerivatives(
            runtime_context, compiled_derivatives);
        assembled.compiled_plan().CalcStateTimeDerivatives(
            runtime_context, compiled_derivatives);
        rhs_allocations = allocation_scope.allocations();
    }
    Require(rhs_allocations == 0,
            "a warmed contact-enabled direct RHS called first-party ordinary "
            "C++ operator new/new[]");
    omp_set_num_threads(original_openmp_max_threads);
    omp_set_dynamic(original_openmp_dynamic);

    const auto baseline_contact = EvaluateContactForces(
        *scenario, *independent_contact_workspace);
    const Eigen::Matrix3d polar_mirror =
        (Eigen::Vector3d(1.0, -1.0, 1.0)).asDiagonal();
    const Eigen::Matrix3d axial_mirror =
        (Eigen::Vector3d(-1.0, 1.0, -1.0)).asDiagonal();
    double largest_support_arm_moment = 0.0;
    for (int carrier = 0; carrier < contact_plan->carrier_count(); ++carrier) {
        const auto& right = baseline_contact[2 * carrier];
        const auto& left = baseline_contact[2 * carrier + 1];
        const auto wheelset = assembled.model().GetRigidBodyByName(
            contact_plan->carrier_name(carrier));
        Require(right.body == wheelset && left.body == wheelset,
                "a contact wrench is bound to the wrong rigid body");
        Require(right.point_position_in_body_frame_meters.isZero() &&
                    left.point_position_in_body_frame_meters.isZero() &&
                    right.expressed_in_frame == assembled.model().world_frame() &&
                    left.expressed_in_frame == assembled.model().world_frame(),
                "a contact wrench is not reduced to the wheelset origin in "
                "the inertial frame");
        Require(Near(left.force_newtons, polar_mirror * right.force_newtons) &&
                    Near(left.torque_about_point_newton_metres,
                         axial_mirror *
                             right.torque_about_point_newton_metres),
                "the straight-track left/right contact wrenches are not mirror "
                "images");
        largest_support_arm_moment = std::max(
            {largest_support_arm_moment,
             right.torque_about_point_newton_metres.norm(),
             left.torque_about_point_newton_metres.norm()});
        const auto& placement =
            *resolved_placement.at(std::string(contact_plan->carrier_name(
                carrier)));
        Require(std::abs(-right.force_newtons.z() -
                         placement.right_support_force_newtons) < 5.0 &&
                    std::abs(-left.force_newtons.z() -
                             placement.left_support_force_newtons) < 5.0,
                "the resolved start-up support load does not reach the direct "
                "contact wrench");
    }
    Require(largest_support_arm_moment > 1000.0,
            "the wrench fixture has no observable contact support-arm moment");

    // Independently reduce the eight spatial wrenches to generalized force,
    // then check both virtual power and the resulting acceleration increment.
    const int nv = assembled.model().num_generalized_velocities();
    const int nq = assembled.model().num_generalized_positions();
    Eigen::VectorXd contact_generalized_force = Eigen::VectorXd::Zero(nv);
    double spatial_power = 0.0;
    Eigen::MatrixXd angular_jacobian(3, nv);
    Eigen::MatrixXd translational_jacobian(3, nv);
    for (const AppliedBodyWrench& wrench : baseline_contact) {
        assembled.model()
            .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                component.context(), wrench.body,
                wrench.point_position_in_body_frame_meters, &angular_jacobian,
                &translational_jacobian);
        contact_generalized_force.noalias() +=
            angular_jacobian.transpose() *
                wrench.torque_about_point_newton_metres +
            translational_jacobian.transpose() * wrench.force_newtons;
        const auto velocity =
            assembled.model()
                .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    component.context(), wrench.body);
        spatial_power +=
            wrench.torque_about_point_newton_metres.dot(
                velocity.angular_velocity_radians_per_second()) +
            wrench.force_newtons.dot(
                velocity
                    .translational_velocity_at_frame_origin_meters_per_second());
    }
    Require(std::abs(contact_generalized_force.dot(
                         runtime_context.generalized_velocities()) -
                     spatial_power) <= RelativeTolerance(std::abs(spatial_power)),
            "contact generalized force and spatial-wrench virtual power "
            "disagree");

    Eigen::VectorXd vehicle_only_derivatives(107);
    component.model().CalcStateTimeDerivatives(
        component.context(), std::span(all_wrenches).first(56), {}, {},
        component.forward_dynamics_workspace(), vehicle_only_derivatives);
    Eigen::MatrixXd mass_matrix(nv, nv);
    component.model().CalcGeneralizedMassMatrix(component.context(),
                                                mass_matrix);
    const Eigen::VectorXd recovered_contact_force =
        mass_matrix *
        (independently_rebuilt.segment(nq, nv) -
         vehicle_only_derivatives.segment(nq, nv));
    Require(Near(recovered_contact_force, contact_generalized_force),
            "the acceleration increment does not equal M^-1 times the eight "
            "contact wrenches");

    // Lift one wheelset by a mechanical 50 mm. Its two fixed interface slots
    // remain present and become valid zero wrenches; the other three wheelsets
    // must be unaffected. This rejects output compression and stale shared
    // workspace state without treating an ordinary loss of contact as an error.
    Eigen::VectorXd original_state(assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(runtime_context, original_state);
    Eigen::VectorXd lifted_state = original_state;
    const auto lifted_body = assembled.model().GetRigidBodyByName(
        contact_plan->carrier_name(0));
    const auto lifted_range =
        assembled.model().GetFreeBodyPositionRange(lifted_body);
    lifted_state[lifted_range.start() + 6] -= 0.05;
    assembled.system().SetContinuousState(runtime_context, lifted_state);
    const auto lifted_contact = EvaluateContactForces(
        *scenario, *independent_contact_workspace);
    for (int interface = 0; interface < 2; ++interface) {
        Require(lifted_contact[interface].force_newtons.isZero() &&
                    lifted_contact[interface]
                        .torque_about_point_newton_metres.isZero(),
                "a lifted wheelset did not retain two zero-wrench interface "
                "slots");
    }
    for (std::size_t interface = 2; interface < lifted_contact.size();
         ++interface) {
        Require(lifted_contact[interface].force_newtons ==
                        baseline_contact[interface].force_newtons &&
                    lifted_contact[interface].torque_about_point_newton_metres ==
                        baseline_contact[interface]
                            .torque_about_point_newton_metres,
                "lifting one carrier changed another carrier's contact "
                "wrench");
    }
    assembled.system().SetContinuousState(runtime_context, original_state);

    // A non-zero-yaw, non-zero-grade, curved-irregularity fixture checks both
    // stages of the frozen WRL/GZ18 station selection at the real force-plan
    // call site. The shared carrier station first supplies the lateral slope
    // that corrects yaw; the corrected yaw then chooses the side-specific
    // profile station. Both series have visibly different slopes at the shared,
    // right and left stations, so sampling a fixed station cannot masquerade as
    // the two-stage contract. The grade also separates planar ds/dt from the
    // three-dimensional path speed.
    auto yawed_startup = startup;
    constexpr double kYawRadians = 0.04;
    constexpr double kAccumulatedSpinRadians = 0.31;
    const std::string yawed_wheelset =
        vehicle.mechanical_track_station_layout.wheelset_body_names.front();
    auto& yawed_body = MutableBodyState(yawed_startup, yawed_wheelset);
    yawed_body.rotation_local_track_from_body = Eigen::Quaterniond(
        Eigen::AngleAxisd(kYawRadians, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(kAccumulatedSpinRadians,
                          Eigen::Vector3d::UnitY()));
    yawed_body
        .additional_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second =
        Eigen::Vector3d(0.02, 0.0, -0.01);
    constexpr std::array<double, 9> fixture_stations{
        -20.0, 0.0, 8.8, 9.0, 9.1, 9.2, 9.4, 20.0, 40.0};
    constexpr std::array<double, 9> fixture_lateral{
        -8.0e-6, 6.0e-6, -4.0e-6, 3.0e-6, 9.0e-6,
        -5.0e-6, 2.0e-6, -3.0e-6, 5.0e-6};
    constexpr std::array<double, 9> fixture_vertical{
        7.0e-6, -5.0e-6, 5.0e-6, -7.0e-6, 1.0e-6,
        8.0e-6, -4.0e-6, 4.0e-6, -6.0e-6};
    const orvd::wheel_rail_contact::TrackIrregularityField fixture_oracle(
        fixture_stations, fixture_lateral, fixture_stations,
        fixture_vertical);
    auto fixture_irregularity = std::make_unique<
        orvd::wheel_rail_contact::TrackIrregularityField>(
        fixture_stations, fixture_lateral, fixture_stations,
        fixture_vertical);
    constexpr double kFixtureGrade = 0.02;
    const auto yawed_scenario = AssembleGz18ContactScenario(
        vehicle, yawed_startup, MakeConstantGradeLine(kFixtureGrade),
        std::filesystem::path(argv[4]), kReferenceStationMeters,
        kProjectionHalfWidthMeters, std::move(fixture_irregularity));
    const auto yawed_zero_field_scenario = AssembleGz18ContactScenario(
        vehicle, yawed_startup, MakeConstantGradeLine(kFixtureGrade),
        std::filesystem::path(argv[4]), kReferenceStationMeters,
        kProjectionHalfWidthMeters);
    const auto* yawed_plan =
        yawed_scenario->vehicle_system().contact_force_plan();
    auto yawed_workspace = yawed_plan->CreateWorkspace();
    const auto yawed_wrenches =
        EvaluateContactForces(*yawed_scenario, *yawed_workspace);
    auto yawed_zero_field_workspace =
        yawed_zero_field_scenario->vehicle_system()
            .contact_force_plan()
            ->CreateWorkspace();
    const auto yawed_zero_field_wrenches = EvaluateContactForces(
        *yawed_zero_field_scenario, *yawed_zero_field_workspace);

    const auto independent_personality =
        orvd::configuration::AssembleGz18WheelRailContact(
            argv[4], yawed_startup.wheel_rail_binding,
            yawed_startup.rail_profile_reference_vertical_offset_meters);
    using orvd::wheel_rail_contact::BuildContactPoseScalars;
    using orvd::wheel_rail_contact::PlaceRailProfileLongitudinalOrigin;
    using orvd::wheel_rail_contact::RailProfileFrame;
    using orvd::wheel_rail_contact::ResolveRollYawPitch;
    using orvd::wheel_rail_contact::ResolveRollYawPitchRates;
    using orvd::wheel_rail_contact::SafeAtan2Ratio;
    using orvd::wheel_rail_contact::SpatialWrench;
    using orvd::wheel_rail_contact::TrackIrregularity;
    using orvd::wheel_rail_contact::WheelProfileRigidMotion;
    using orvd::wheel_rail_contact::WheelRailContactInput;
    using orvd::wheel_rail_contact::WheelRailPoseInput;
    using orvd::wheel_rail_contact::WheelSide;

    const auto& yawed_assembled = yawed_scenario->vehicle_system();
    auto yawed_component = yawed_assembled.system().GetMultibodyComponentView(
        yawed_scenario->initial_context().context(),
        yawed_assembled.system().multibody_component());
    const auto wheel_body =
        yawed_assembled.model().GetRigidBodyByName(yawed_wheelset);
    const auto body_pose = yawed_assembled.model().CalcPoseInWorld(
        yawed_component.context(), wheel_body);
    const auto body_velocity = yawed_assembled.model()
                                   .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                                       yawed_component.context(), wheel_body);
    const double shared_station =
        yawed_plan->initial_projection_station_meters(0);
    const auto shared_track =
        yawed_plan->track_geometry().EvaluateTrackFrame(shared_station);
    const Eigen::Matrix3d rotation_track_from_inertial =
        shared_track.pose().rotation_inertial_from_track().transpose();
    const Eigen::Vector3d body_origin_in_track =
        rotation_track_from_inertial *
        (body_pose.translation() -
         shared_track.pose().origin_in_inertial_meters());
    const Eigen::Vector3d body_velocity_in_track =
        rotation_track_from_inertial *
        body_velocity
            .translational_velocity_at_frame_origin_meters_per_second();
    const Eigen::Vector3d body_omega_in_track =
        rotation_track_from_inertial *
        body_velocity.angular_velocity_radians_per_second();
    const auto angles = ResolveRollYawPitch(
        rotation_track_from_inertial * body_pose.rotation());
    const auto rates = ResolveRollYawPitchRates(
        body_omega_in_track, angles.roll_radians, angles.yaw_radians);
    const Eigen::Vector3d centerline_derivative_in_track =
        rotation_track_from_inertial *
        shared_track.centerline_derivative_in_inertial_meters_per_meter();
    const Eigen::Vector3d track_rotation_rate_in_track =
        rotation_track_from_inertial *
        shared_track.track_frame_rotation_rate_in_inertial_radians_per_meter();
    const double station_rate =
        body_velocity_in_track.x() /
        (centerline_derivative_in_track.x() +
         track_rotation_rate_in_track.cross(body_origin_in_track).x());
    const double path_rate =
        shared_track.centerline_derivative_in_inertial_meters_per_meter().norm() *
        station_rate;
    Require(std::abs(path_rate - station_rate) > 1.0e-3,
            "the graded fixture did not separate path speed from planar "
            "station rate");
    const double shared_lateral_slope =
        fixture_oracle.LateralSlopeMetersPerMeter(shared_station);
    const double effective_yaw =
        angles.yaw_radians -
        SafeAtan2Ratio(shared_lateral_slope * station_rate, station_rate);
    const Eigen::Matrix3d rotation_track_from_wheel_profile =
        Eigen::AngleAxisd(angles.roll_radians, Eigen::Vector3d::UnitX())
            .toRotationMatrix() *
        Eigen::AngleAxisd(angles.yaw_radians, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Require(std::abs(angles.pitch_radians - kAccumulatedSpinRadians) < 1.0e-14,
            "the integration fixture lost its non-zero accumulated wheel spin");
    struct DirectInterfaceResult {
        SpatialWrench wrench_in_inertial;
        double effective_station_meters{0.0};
        double lateral_slope_meters_per_meter{0.0};
        double vertical_slope_meters_per_meter{0.0};
    };
    const auto RebuildInterface = [&](WheelSide side) {
        const auto& constants = independent_personality->pose_constants(side);
        const double effective_offset =
            -std::sin(effective_yaw) *
            constants.wheel_lateral_datum_meters;
        const double effective_lateral =
            body_origin_in_track.y() +
            std::cos(effective_yaw) *
                constants.wheel_lateral_datum_meters;
        const double effective_station =
            shared_station +
            effective_offset /
                (1.0 - yawed_plan->track_geometry().CurvatureRadiansPerMeter(
                           shared_station) *
                           effective_lateral);
        const auto profile_track =
            yawed_plan->track_geometry().EvaluateTrackFrame(effective_station);
        const double irregularity_lateral =
            fixture_oracle.LateralDisplacementMeters(effective_station);
        const double irregularity_vertical =
            fixture_oracle.VerticalDisplacementMeters(effective_station);
        const double lateral_slope =
            fixture_oracle.LateralSlopeMetersPerMeter(effective_station);
        const double vertical_slope =
            fixture_oracle.VerticalSlopeMetersPerMeter(effective_station);

        const Eigen::Vector3d wheel_profile_offset =
            rotation_track_from_wheel_profile *
            Eigen::Vector3d(0.0, constants.wheel_lateral_datum_meters, 0.0);
        Require(body_omega_in_track.cross(wheel_profile_offset).norm() >
                    1.0e-3,
                "the integration fixture lost its non-collinear "
                "profile-datum velocity");
        WheelProfileRigidMotion wheel_motion;
        wheel_motion.origin_track_meters =
            body_origin_in_track + wheel_profile_offset;
        wheel_motion.rotation_track_from_profile =
            rotation_track_from_wheel_profile;
        wheel_motion.origin_velocity_track_meters_per_second =
            body_velocity_in_track +
            body_omega_in_track.cross(wheel_profile_offset);
        wheel_motion.angular_velocity_track_radians_per_second =
            body_omega_in_track;
        wheel_motion.arc_rate_meters_per_second = path_rate;
        wheel_motion.wheel_pitch_rate_radians_per_second =
            rates.pitch_rate_radians_per_second;

        RailProfileFrame rail_frame;
        const Eigen::Vector3d rail_origin_in_inertial =
            profile_track.pose().origin_in_inertial_meters() +
            profile_track.pose().rotation_inertial_from_track() *
                Eigen::Vector3d(
                    0.0,
                    constants.rail_lateral_datum_meters +
                        irregularity_lateral,
                    constants.rail_vertical_datum_meters +
                        irregularity_vertical);
        rail_frame.origin_track_meters =
            rotation_track_from_inertial *
            (rail_origin_in_inertial -
             shared_track.pose().origin_in_inertial_meters());
        rail_frame.rotation_track_from_profile =
            rotation_track_from_inertial *
            profile_track.pose().rotation_inertial_from_track() *
            Eigen::AngleAxisd(std::atan2(lateral_slope, 1.0),
                              Eigen::Vector3d::UnitZ())
                .toRotationMatrix() *
            Eigen::AngleAxisd(-std::atan2(vertical_slope, 1.0),
                              Eigen::Vector3d::UnitY())
                .toRotationMatrix() *
            Eigen::AngleAxisd(constants.rail_roll_radians,
                              Eigen::Vector3d::UnitX())
                .toRotationMatrix();
        rail_frame.origin_track_meters = PlaceRailProfileLongitudinalOrigin(
            independent_personality->rail_profile_origin_mode(),
            rail_frame.origin_track_meters,
            rail_frame.rotation_track_from_profile, body_origin_in_track,
            effective_offset);
        const double actual_profile_coordinate =
            (rail_frame.rotation_track_from_profile.transpose() *
             (rail_frame.origin_track_meters - body_origin_in_track))
                .x();
        Require(std::abs(actual_profile_coordinate - effective_offset) <
                    1.0e-14,
                "the non-zero-yaw rail origin does not satisfy the profile "
                "coordinate invariant");

        WheelRailPoseInput pose_input;
        pose_input.placement = {
            body_origin_in_track.y(), body_origin_in_track.z(),
            angles.roll_radians, angles.yaw_radians};
        pose_input.track_station_rate_meters_per_second = station_rate;
        pose_input.irregularity = TrackIrregularity{
            irregularity_lateral, irregularity_vertical,
            lateral_slope * station_rate, vertical_slope * station_rate};
        WheelRailContactInput contact_input;
        contact_input.pose =
            BuildContactPoseScalars(constants, pose_input, {});
        contact_input.rail_frame = rail_frame;
        contact_input.wheel = wheel_motion;
        orvd::wheel_rail_contact::WheelRailContactWorkspace contact_workspace;
        independent_personality->model(side).PrepareWorkspace(contact_workspace);
        const auto contact_result =
            independent_personality->model(side).Evaluate(contact_input,
                                                           contact_workspace);
        SpatialWrench direct_wrench;
        for (std::size_t patch = 0; patch < contact_result.count; ++patch) {
            const auto at_body = orvd::wheel_rail_contact::TransportWrench(
                contact_result.patches[patch].wrench.rail_on_wheel,
                contact_result.patches[patch].wrench.contact_point_meters,
                body_origin_in_track);
            direct_wrench.force_newtons += at_body.force_newtons;
            direct_wrench.moment_newton_meters += at_body.moment_newton_meters;
        }
        direct_wrench = orvd::wheel_rail_contact::RotateWrench(
            direct_wrench,
            shared_track.pose().rotation_inertial_from_track());
        return DirectInterfaceResult{direct_wrench, effective_station,
                                     lateral_slope, vertical_slope};
    };
    const DirectInterfaceResult direct_right =
        RebuildInterface(WheelSide::kRight);
    const DirectInterfaceResult direct_left =
        RebuildInterface(WheelSide::kLeft);
    Require(std::abs(direct_right.effective_station_meters - shared_station) >
                    1.0e-3 &&
                std::abs(direct_left.effective_station_meters - shared_station) >
                    1.0e-3 &&
                std::abs(direct_right.effective_station_meters -
                         direct_left.effective_station_meters) > 4.0e-2,
            "the fixture did not separate the shared, right and left "
            "sampling stations");
    Require(std::abs(shared_lateral_slope -
                     direct_right.lateral_slope_meters_per_meter) > 1.0e-8 &&
                std::abs(shared_lateral_slope -
                         direct_left.lateral_slope_meters_per_meter) > 1.0e-8 &&
                std::abs(direct_right.lateral_slope_meters_per_meter -
                         direct_left.lateral_slope_meters_per_meter) > 1.0e-8 &&
                std::abs(direct_right.vertical_slope_meters_per_meter -
                         direct_left.vertical_slope_meters_per_meter) > 1.0e-8,
            "the nonlinear field did not distinguish the three slope "
            "sampling stations");
    for (const auto& [interface, direct] :
         {std::pair{0, &direct_right}, std::pair{1, &direct_left}}) {
        Require(Near(yawed_wrenches[interface].force_newtons,
                     direct->wrench_in_inertial.force_newtons) &&
                    Near(yawed_wrenches[interface]
                             .torque_about_point_newton_metres,
                         direct->wrench_in_inertial.moment_newton_meters),
                "the force plan's non-zero-yaw irregularity wrench does not "
                "use the two-stage profile-station and rigid-profile "
                "contract");
    }
    for (int interface = 4; interface < 8; ++interface) {
        Require(SameWrenchComponents(
                    yawed_wrenches[interface],
                    yawed_zero_field_wrenches[interface]),
                "an interface on the left straight continuation consumed "
                "irregularity outside the base-track definition interval");
    }
    Require(std::abs(fixture_oracle.LateralDisplacementMeters(-6.6)) +
                        std::abs(fixture_oracle.VerticalDisplacementMeters(
                            -9.1)) >
                    1.0e-6,
            "the rear-interface suppression fixture has no non-zero raw "
            "field outside the base-track definition interval");
    std::array<AppliedBodyWrench, 8> irregularity_hot_wrenches;
    std::size_t irregularity_allocations = 0;
    {
        orvd::test::AllocationScope allocation_scope;
        yawed_plan->CalcAppliedForces(
            yawed_component.context(), *yawed_workspace,
            yawed_scenario->initial_context()
                .context()
                .wheel_rail_projection_station_hints_meters(),
            irregularity_hot_wrenches);
        yawed_plan->CalcAppliedForces(
            yawed_component.context(), *yawed_workspace,
            yawed_scenario->initial_context()
                .context()
                .wheel_rail_projection_station_hints_meters(),
            irregularity_hot_wrenches);
        irregularity_allocations = allocation_scope.allocations();
    }
    Require(irregularity_allocations == 0,
            "the warmed non-zero-irregularity contact path allocated heap "
            "storage");

    // A contact-enabled system owns one line.  The lower-level research overload
    // remains available for systems without contact, but it must not create a
    // start-up context against a second line while the contact plan evaluates
    // another one.
    bool separate_line_was_rejected = false;
    try {
        const auto separate_line = LoadTrackGeometryFromJsonFile(argv[3]);
        (void)orvd::configuration::AssembleResolvedInitialContext(
            assembled, startup, separate_line, kReferenceStationMeters);
    } catch (const std::invalid_argument& error) {
        separate_line_was_rejected =
            std::string_view(error.what()).find("already owns the line") !=
            std::string_view::npos;
    }
    Require(separate_line_was_rejected,
            "a contact-enabled system accepted a second line for start-up "
            "assembly");

    // The real 60 km/h GZ18 advances farther than the deliberately narrow
    // local window. Some ordinary CVODE trial steps also reach beyond the
    // latest accepted 1 cm branch window. Those trial states are recoverable:
    // CVODE reduces the step, while every successful endpoint advances the
    // accepted projection history. Static G53 anchors cannot do either job.
    std::vector<double> initial_hints(
        runtime_context.wheel_rail_projection_station_hints_meters().begin(),
        runtime_context.wheel_rail_projection_station_hints_meters().end());
    SystemContinuousStateAdvancer advancer(
        assembled.system(), assembled.compiled_plan(), runtime_context,
        MakeGz18Tolerances(), NoCallTimeAppliedForces{});
    constexpr double kRecoverableProjectionTargetSeconds = 3.0e-2;
    advancer.AdvanceTo(kRecoverableProjectionTargetSeconds);
    Require(runtime_context.time_seconds() ==
                kRecoverableProjectionTargetSeconds,
            "the contact-enabled public advance did not reach its target");
    const auto advanced_hints =
        runtime_context.wheel_rail_projection_station_hints_meters();
    for (std::size_t ordinal = 0; ordinal < initial_hints.size(); ++ordinal) {
        Require(advanced_hints[ordinal] > initial_hints[ordinal] +
                    kProjectionHalfWidthMeters,
                "a GZ18 projection branch did not advance beyond the static "
                "G53 window");
    }
    Eigen::VectorXd advanced_rhs(109);
    assembled.compiled_plan().CalcStateTimeDerivatives(runtime_context,
                                                       advanced_rhs);
    Require(advanced_rhs.allFinite(),
            "the accepted GZ18 projection history does not feed a finite RHS");

    // Put the same real vehicle just inside the base track's right definition
    // boundary.
    // The accepted vehicle then crosses onto TrackGeometry's native straight
    // continuation. The finite JSON interval stays observable, but is not a
    // wall in the vehicle dynamics.
    auto boundary_line = LoadTrackGeometryFromJsonFile(argv[3]);
    const double boundary_end_station =
        boundary_line.end_track_station_meters();
    double foremost_offset = -std::numeric_limits<double>::infinity();
    for (const ResolvedWheelsetPlacement& placement :
         scenario->initial_context().wheelset_placements()) {
        foremost_offset =
            std::max(foremost_offset,
                     placement.track_station_meters - kReferenceStationMeters);
    }
    const double boundary_reference_station =
        boundary_line.end_track_station_meters() - foremost_offset -
        3.5 * kProjectionHalfWidthMeters;
    auto boundary_scenario = AssembleGz18ContactScenario(
        vehicle, startup, std::move(boundary_line),
        std::filesystem::path(argv[4]), boundary_reference_station,
        kProjectionHalfWidthMeters);
    const auto& boundary_system = boundary_scenario->vehicle_system();
    auto& boundary_context = boundary_scenario->initial_context().context();
    const std::vector<double> initial_boundary_hints(
        boundary_context.wheel_rail_projection_station_hints_meters().begin(),
        boundary_context.wheel_rail_projection_station_hints_meters().end());
    SystemContinuousStateAdvancer boundary_advancer(
        boundary_system.system(), boundary_system.compiled_plan(),
        boundary_context, MakeGz18Tolerances(), NoCallTimeAppliedForces{});
    constexpr double kAcceptedHistoryTargetSeconds = 1.0e-3;
    boundary_advancer.AdvanceTo(kAcceptedHistoryTargetSeconds);
    Require(boundary_context.time_seconds() == kAcceptedHistoryTargetSeconds,
            "the boundary scenario did not establish an accepted history");
    const std::vector<double> accepted_boundary_hints(
        boundary_context.wheel_rail_projection_station_hints_meters().begin(),
        boundary_context.wheel_rail_projection_station_hints_meters().end());
    for (std::size_t ordinal = 0; ordinal < initial_boundary_hints.size();
         ++ordinal) {
        Require(accepted_boundary_hints[ordinal] >
                    initial_boundary_hints[ordinal] +
                        kProjectionHalfWidthMeters,
                "the boundary scenario did not move its accepted projection "
                "history beyond the construction anchors");
    }
    const std::array<double, 5> boundary_sample_times{
        kAcceptedHistoryTargetSeconds, 1.5e-3, 2.0e-3, 2.5e-3, 3.0e-3};
    const Eigen::MatrixXd boundary_samples =
        boundary_advancer.AdvanceToWithDenseStateSamples(
            boundary_sample_times.back(), boundary_sample_times);
    Require(boundary_context.time_seconds() == boundary_sample_times.back() &&
                boundary_samples.rows() == 109 &&
                boundary_samples.cols() ==
                    static_cast<Eigen::Index>(boundary_sample_times.size()) &&
                boundary_samples.allFinite(),
            "the GZ18 right-boundary crossing did not produce one finite "
            "accepted state batch");
    const auto continued_hints =
        boundary_context.wheel_rail_projection_station_hints_meters();
    Require(std::any_of(continued_hints.begin(), continued_hints.end(),
                        [&](double station) {
                            return station > boundary_end_station;
                        }),
            "no GZ18 carrier crossed onto the right straight continuation");
    Eigen::VectorXd continued_rhs(109);
    boundary_system.compiled_plan().CalcStateTimeDerivatives(boundary_context,
                                                             continued_rhs);
    Require(continued_rhs.allFinite(),
            "the right straight continuation did not feed a finite vehicle "
            "RHS");

    if (failures != 0) {
        return 1;
    }
    std::puts("GZ18 contact direct RHS verified");
    return 0;
}
