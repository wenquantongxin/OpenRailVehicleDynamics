// G69: the frozen H3 IRW state drives eight independently rotating wheel
// contacts through one complete q81/v74/z2 vehicle RHS.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <omp.h>

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "wheel_rail_contact/allocation_probe.h"

namespace {

using orvd::configuration::AssembleIrwContactScenario;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::LoadTrackGeometryFromJsonFile;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::forces::WheelRailContactInterfaceObservation;
using orvd::multibody_model::AppliedBodyWrench;

constexpr std::size_t kCarrierCount = 4;
constexpr std::size_t kInterfaceCount = 8;
constexpr int kPositionCount = 81;
constexpr int kVelocityCount = 74;
constexpr int kSeriesStateCount = 2;
constexpr int kContinuousStateCount = 157;
constexpr int kVehicleWrenchCount = 96;

constexpr std::array<std::string_view, kCarrierCount> kCarrierNames{
    "axlebridge_ff", "axlebridge_fr", "axlebridge_rf", "axlebridge_rr"};
constexpr std::array<std::string_view, kInterfaceCount> kInterfaceNames{
    "wheel_ff_l", "wheel_ff_r", "wheel_fr_l", "wheel_fr_r",
    "wheel_rf_l", "wheel_rf_r", "wheel_rr_l", "wheel_rr_r"};
constexpr std::array<std::string_view, kInterfaceCount> kWheelJointNames{
    "rev_wheel_ff_l", "rev_wheel_ff_r", "rev_wheel_fr_l",
    "rev_wheel_fr_r", "rev_wheel_rf_l", "rev_wheel_rf_r",
    "rev_wheel_rr_l", "rev_wheel_rr_r"};

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "IRW contact RHS: %.*s\n",
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

bool SameWrench(const AppliedBodyWrench& actual,
                const AppliedBodyWrench& expected) {
    return actual.body == expected.body &&
           actual.expressed_in_frame == expected.expressed_in_frame &&
           actual.point_position_in_body_frame_meters ==
               expected.point_position_in_body_frame_meters &&
           actual.force_newtons == expected.force_newtons &&
           actual.torque_about_point_newton_metres ==
               expected.torque_about_point_newton_metres;
}

bool SameObservation(const WheelRailContactInterfaceObservation& actual,
                     const WheelRailContactInterfaceObservation& expected) {
    if (actual.contact_patch_count != expected.contact_patch_count ||
        actual.rail_profile_reference_marker_track_station_meters !=
            expected.rail_profile_reference_marker_track_station_meters ||
        actual.vertical_support_force_on_wheel_newtons !=
            expected.vertical_support_force_on_wheel_newtons ||
        actual.normal_force_newtons != expected.normal_force_newtons ||
        actual.total_force_on_wheel_in_carrier_track_frame_newtons !=
            expected.total_force_on_wheel_in_carrier_track_frame_newtons) {
        return false;
    }
    for (std::size_t patch = 0; patch < actual.patches.size(); ++patch) {
        const auto& left = actual.patches[patch];
        const auto& right = expected.patches[patch];
        if (left.contact_point_in_carrier_track_frame_meters !=
                right.contact_point_in_carrier_track_frame_meters ||
            left.force_on_wheel_in_carrier_track_frame_newtons !=
                right.force_on_wheel_in_carrier_track_frame_newtons ||
            left.normal_force_newtons != right.normal_force_newtons ||
            left.longitudinal_force_on_wheel_in_contact_frame_newtons !=
                right.longitudinal_force_on_wheel_in_contact_frame_newtons ||
            left.lateral_force_on_wheel_in_contact_frame_newtons !=
                right.lateral_force_on_wheel_in_contact_frame_newtons ||
            left.contact_frame_angle_radians !=
                right.contact_frame_angle_radians) {
            return false;
        }
    }
    return true;
}

struct ContactBatch {
    std::array<AppliedBodyWrench, kInterfaceCount> wrenches;
    std::array<WheelRailContactInterfaceObservation, kInterfaceCount>
        observations;
};

ContactBatch EvaluateContactBatch(
    const orvd::configuration::AssembledVehicleSystem& assembled,
    orvd::system_assembly::SystemRuntimeContext& context,
    orvd::forces::WheelRailContactForceWorkspace& workspace) {
    ContactBatch batch;
    const auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());
    assembled.contact_force_plan()->CalcAppliedForcesAndObservations(
        component.context(), workspace,
        context.wheel_rail_projection_station_hints_meters(), batch.wrenches,
        batch.observations);
    return batch;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr,
                     "usage: verify_irw_contact_rhs VEHICLE STARTUP LINE "
                     "DATA_ROOT\n");
        return 2;
    }

    try {
        const auto vehicle = LoadVehicleDefinitionFromJsonFile(argv[1]);
        const auto startup = LoadResolvedStartupStateFromJsonFile(argv[2]);
        auto line = LoadTrackGeometryFromJsonFile(argv[3]);
        constexpr double kReferenceStationMeters = 0.0;
        constexpr double kProjectionHalfWidthMeters = 0.01;
        const auto scenario = AssembleIrwContactScenario(
            vehicle, startup, std::move(line), std::filesystem::path(argv[4]),
            kReferenceStationMeters, kProjectionHalfWidthMeters);
        const auto& assembled = scenario->vehicle_system();
        const auto* contact_plan = assembled.contact_force_plan();
        Require(contact_plan != nullptr,
                "the assembled system lost its IRW contact plan");
        if (contact_plan == nullptr) {
            return 1;
        }

        Require(assembled.model().num_generalized_positions() ==
                        kPositionCount &&
                    assembled.model().num_generalized_velocities() ==
                        kVelocityCount &&
                    assembled.system().continuous_state_size() ==
                        kContinuousStateCount,
                "the contact-enabled IRW is not q81/v74/z2");
        Require(assembled.force_plan().body_wrench_count() ==
                        kVehicleWrenchCount &&
                    assembled.system().vehicle_body_wrench_count() ==
                        kVehicleWrenchCount &&
                    assembled.system().contact_body_wrench_count() ==
                        static_cast<int>(kInterfaceCount),
                "the 96 vehicle and eight contact wrench slices are wrong");
        Require(contact_plan->carrier_count() ==
                        static_cast<int>(kCarrierCount) &&
                    contact_plan->interface_count() ==
                        static_cast<int>(kInterfaceCount) &&
                    contact_plan->body_wrench_count() ==
                        static_cast<int>(kInterfaceCount),
                "the IRW plan is not four carriers and eight interfaces");

        auto& context = scenario->initial_context().context();
        Require(context.generalized_positions().size() == kPositionCount &&
                    context.generalized_velocities().size() ==
                        kVelocityCount &&
                    context.series_spring_damper_forces().size() ==
                        kSeriesStateCount &&
                    context.wheel_rail_projection_station_hints_meters()
                            .size() == kCarrierCount,
                "the resolved H3 context has the wrong state or history size");

        std::unordered_map<std::string, double> resolved_stations;
        for (const auto& placement :
             scenario->initial_context().wheel_pair_placements()) {
            resolved_stations.emplace(placement.station_reference_body_name,
                                      placement.track_station_meters);
        }
        Require(resolved_stations.size() == kCarrierCount,
                "the H3 state did not resolve four axle-bridge stations");
        for (std::size_t ordinal = 0; ordinal < kCarrierCount; ++ordinal) {
            const std::string carrier(kCarrierNames[ordinal]);
            Require(contact_plan->carrier_name(static_cast<int>(ordinal)) ==
                            kCarrierNames[ordinal] &&
                        resolved_stations.contains(carrier) &&
                        contact_plan->initial_projection_station_meters(
                            static_cast<int>(ordinal)) ==
                            resolved_stations.at(carrier) &&
                        context
                                .wheel_rail_projection_station_hints_meters()[
                            ordinal] == resolved_stations.at(carrier),
                    "a frozen axle-bridge carrier or initial hint is wrong");
        }
        for (std::size_t ordinal = 0; ordinal < kInterfaceCount; ++ordinal) {
            Require(contact_plan->interface_name(static_cast<int>(ordinal)) ==
                        kInterfaceNames[ordinal],
                    "the frozen IRW interface order is wrong");
        }

        Eigen::VectorXd compiled_rhs(kContinuousStateCount);
        assembled.compiled_plan().CalcStateTimeDerivatives(context,
                                                            compiled_rhs);
        Require(compiled_rhs.allFinite(),
                "the real H3 contact-enabled 157-state RHS is not finite");

        const auto component = assembled.system().GetMultibodyComponentView(
            context, assembled.system().multibody_component());
        std::vector<AppliedBodyWrench> all_wrenches(
            kVehicleWrenchCount + kInterfaceCount);
        Eigen::VectorXd series_derivatives(kSeriesStateCount);
        assembled.force_plan().CalcAppliedForces(
            component.context(), context.series_spring_damper_forces(),
            context.nominal_forces(),
            std::span(all_wrenches).first(kVehicleWrenchCount),
            series_derivatives);
        auto contact_workspace = contact_plan->CreateWorkspace();
        contact_plan->CalcAppliedForces(
            component.context(), *contact_workspace,
            context.wheel_rail_projection_station_hints_meters(),
            std::span(all_wrenches).subspan(kVehicleWrenchCount,
                                             kInterfaceCount));
        Eigen::VectorXd multibody_derivatives(kPositionCount +
                                              kVelocityCount);
        component.model().CalcStateTimeDerivatives(
            component.context(), all_wrenches, {}, {},
            component.forward_dynamics_workspace(), multibody_derivatives);
        Eigen::VectorXd independently_rebuilt(kContinuousStateCount);
        independently_rebuilt.head(kPositionCount + kVelocityCount) =
            multibody_derivatives;
        independently_rebuilt.tail(kSeriesStateCount) = series_derivatives;
        Require(Near(compiled_rhs, independently_rebuilt),
                "the compiled 157-state RHS differs from its typed 96+8 "
                "wrench rebuild");

        const ContactBatch baseline =
            EvaluateContactBatch(assembled, context, *contact_workspace);
        for (std::size_t ordinal = 0; ordinal < kInterfaceCount; ++ordinal) {
            const auto expected_body = assembled.model().GetRigidBodyByName(
                std::string(kInterfaceNames[ordinal]));
            const auto& wrench = baseline.wrenches[ordinal];
            const auto& observation = baseline.observations[ordinal];
            Eigen::Vector3d patch_force_sum = Eigen::Vector3d::Zero();
            double patch_normal_force_sum = 0.0;
            for (std::size_t patch = 0;
                 patch < observation.contact_patch_count; ++patch) {
                patch_force_sum +=
                    observation.patches[patch]
                        .force_on_wheel_in_carrier_track_frame_newtons;
                patch_normal_force_sum +=
                    observation.patches[patch].normal_force_newtons;
            }
            Require(wrench.body == expected_body &&
                        wrench.body != assembled.model().GetRigidBodyByName(
                                           std::string(kCarrierNames[ordinal / 2])) &&
                        wrench.expressed_in_frame ==
                            assembled.model().world_frame() &&
                        wrench.point_position_in_body_frame_meters.isZero(),
                    "a contact wrench is not reduced onto its independent "
                    "wheel-body origin in the world frame");
            Require(
                patch_force_sum ==
                        observation
                            .total_force_on_wheel_in_carrier_track_frame_newtons &&
                    patch_normal_force_sum ==
                        observation.normal_force_newtons &&
                    observation.vertical_support_force_on_wheel_newtons ==
                        -observation
                             .total_force_on_wheel_in_carrier_track_frame_newtons
                             .z(),
                "an interface observation does not reconcile its patch and "
                "carrier-Track-T totals");
            Require(wrench.force_newtons.allFinite() &&
                        wrench.torque_about_point_newton_metres.allFinite() &&
                        observation.contact_patch_count == 1 &&
                        observation.normal_force_newtons > 0.0 &&
                        observation.vertical_support_force_on_wheel_newtons >
                            0.0 &&
                        observation
                            .total_force_on_wheel_in_carrier_track_frame_newtons
                            .allFinite() &&
                        std::isfinite(
                            observation.patches[0]
                                .longitudinal_force_on_wheel_in_contact_frame_newtons) &&
                        std::isfinite(
                            observation.patches[0]
                                .lateral_force_on_wheel_in_contact_frame_newtons),
                    "a real H3 wheel interface lost finite positive contact");
        }

        // Reduce the eight real wheel-body wrenches independently. This checks
        // that their application bodies and origin moments produce the same
        // generalized force and virtual power as the multibody projection,
        // rather than merely observing a finite forward-dynamics result.
        Eigen::VectorXd contact_generalized_force =
            Eigen::VectorXd::Zero(kVelocityCount);
        double spatial_power_watts = 0.0;
        Eigen::MatrixXd angular_jacobian(3, kVelocityCount);
        Eigen::MatrixXd translational_jacobian(3, kVelocityCount);
        for (const AppliedBodyWrench& wrench : baseline.wrenches) {
            assembled.model()
                .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                    component.context(), wrench.body,
                    wrench.point_position_in_body_frame_meters,
                    &angular_jacobian, &translational_jacobian);
            contact_generalized_force.noalias() +=
                angular_jacobian.transpose() *
                    wrench.torque_about_point_newton_metres +
                translational_jacobian.transpose() * wrench.force_newtons;
            const auto wheel_velocity =
                assembled.model()
                    .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        component.context(), wrench.body);
            spatial_power_watts +=
                wrench.torque_about_point_newton_metres.dot(
                    wheel_velocity.angular_velocity_radians_per_second()) +
                wrench.force_newtons.dot(
                    wheel_velocity
                        .translational_velocity_at_frame_origin_meters_per_second());
        }
        Require(std::abs(contact_generalized_force.dot(
                             context.generalized_velocities()) -
                         spatial_power_watts) <=
                    RelativeTolerance(std::abs(spatial_power_watts)),
                "wheel-body contact generalized force and spatial virtual "
                "power disagree");

        Eigen::VectorXd vehicle_only_derivatives(kPositionCount +
                                                 kVelocityCount);
        component.model().CalcStateTimeDerivatives(
            component.context(),
            std::span(all_wrenches).first(kVehicleWrenchCount), {}, {},
            component.forward_dynamics_workspace(), vehicle_only_derivatives);
        Eigen::MatrixXd mass_matrix(kVelocityCount, kVelocityCount);
        component.model().CalcGeneralizedMassMatrix(component.context(),
                                                    mass_matrix);
        const Eigen::VectorXd recovered_contact_generalized_force =
            mass_matrix *
            (independently_rebuilt.segment(kPositionCount, kVelocityCount) -
             vehicle_only_derivatives.segment(kPositionCount,
                                              kVelocityCount));
        Require(Near(recovered_contact_generalized_force,
                     contact_generalized_force),
                "the acceleration increment does not close against the eight "
                "independent-wheel contact wrenches");

        Eigen::VectorXd original_state(kContinuousStateCount);
        assembled.system().CopyContinuousState(context, original_state);
        Eigen::VectorXd perturbed_state = original_state;
        const auto perturbed_joint =
            assembled.model().GetJointByName(std::string(kWheelJointNames[0]));
        const auto perturbed_velocity_range =
            assembled.model().GetJointVelocityRange(perturbed_joint);
        Require(perturbed_velocity_range.size() == 1,
                "the selected independent-wheel joint is not one-dimensional");
        perturbed_state[kPositionCount + perturbed_velocity_range.start()] +=
            0.25;
        assembled.system().SetContinuousState(context, perturbed_state);
        const ContactBatch perturbed =
            EvaluateContactBatch(assembled, context, *contact_workspace);
        Require(!SameWrench(perturbed.wrenches[0], baseline.wrenches[0]) ||
                    !SameObservation(perturbed.observations[0],
                                     baseline.observations[0]),
                "changing one wheel's joint rate did not reach its contact");
        for (std::size_t ordinal = 1; ordinal < kInterfaceCount; ++ordinal) {
            Require(SameWrench(perturbed.wrenches[ordinal],
                               baseline.wrenches[ordinal]) &&
                        SameObservation(perturbed.observations[ordinal],
                                        baseline.observations[ordinal]),
                    "changing one wheel's joint rate changed another direct "
                    "contact interface");
        }
        Eigen::VectorXd perturbed_rhs(kContinuousStateCount);
        assembled.compiled_plan().CalcStateTimeDerivatives(context,
                                                            perturbed_rhs);
        Require(perturbed_rhs.allFinite() && perturbed_rhs != compiled_rhs,
                "the independent-wheel rate perturbation did not reach a "
                "finite complete RHS");
        assembled.system().SetContinuousState(context, original_state);

        const int original_openmp_dynamic = omp_get_dynamic();
        const int original_openmp_max_threads = omp_get_max_threads();
        omp_set_dynamic(0);
        ContactBatch parallel_reference;
        Eigen::VectorXd parallel_rhs_reference(kContinuousStateCount);
        bool have_parallel_reference = false;
        for (const int requested_threads : {1, 2, 4, 8}) {
            omp_set_num_threads(requested_threads);
            const ContactBatch candidate =
                EvaluateContactBatch(assembled, context, *contact_workspace);
            Eigen::VectorXd candidate_rhs(kContinuousStateCount);
            assembled.compiled_plan().CalcStateTimeDerivatives(
                context, candidate_rhs);
            if (!have_parallel_reference) {
                parallel_reference = candidate;
                parallel_rhs_reference = candidate_rhs;
                have_parallel_reference = true;
                continue;
            }
            for (std::size_t ordinal = 0; ordinal < kInterfaceCount;
                 ++ordinal) {
                Require(SameWrench(candidate.wrenches[ordinal],
                                   parallel_reference.wrenches[ordinal]) &&
                            SameObservation(
                                candidate.observations[ordinal],
                                parallel_reference.observations[ordinal]),
                        "changing the OpenMP worker request changed an IRW "
                        "contact result");
            }
            Require(candidate_rhs == parallel_rhs_reference,
                    "changing the OpenMP worker request changed the 157-state "
                    "RHS");
        }

        std::size_t rhs_allocations = 0;
        {
            orvd::test::AllocationScope allocation_scope;
            assembled.compiled_plan().CalcStateTimeDerivatives(context,
                                                                compiled_rhs);
            assembled.compiled_plan().CalcStateTimeDerivatives(context,
                                                                compiled_rhs);
            rhs_allocations = allocation_scope.allocations();
        }
        Require(rhs_allocations == 0,
                "a warmed IRW contact RHS called first-party ordinary C++ "
                "operator new/new[]");
        omp_set_num_threads(original_openmp_max_threads);
        omp_set_dynamic(original_openmp_dynamic);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW contact RHS threw: %s\n", error.what());
        return 1;
    }

    if (failures != 0) {
        std::fprintf(stderr, "%d IRW contact RHS assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts("IRW independent-wheel contact RHS verified");
    return 0;
}
