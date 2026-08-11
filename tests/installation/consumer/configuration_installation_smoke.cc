#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/load_irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_irregularity_field.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"

int main(int argc, char* argv[]) {
    try {
        if (argc != 7) {
            throw std::invalid_argument(
                "expected the installed track geometry, GZ18 vehicle "
                "definition, resolved start-up state, data root and IRW "
                "vehicle definition and resolved start-up state paths");
        }
        auto line =
            orvd::configuration::LoadTrackGeometryFromJsonFile(argv[1]);
        const auto point = line.CenterlinePositionInInertialMeters(40.0);
        if (std::abs(point.x() - 40.0) > 1.0e-12 || point.y() != 0.0 ||
            point.z() != 0.0) {
            std::fprintf(stderr,
                         "installed configuration smoke produced an invalid "
                         "straight-line point\n");
            return 1;
        }
        auto r300_line =
            orvd::configuration::LoadTrackGeometryFromJsonFile(
                std::filesystem::path(argv[4]) / "track_library" /
                "geometries" /
                "r300_centerline_superelevation_1150m.json");
        if (r300_line.start_track_station_meters() != 0.0 ||
            r300_line.end_track_station_meters() != 1150.0 ||
            r300_line.CurvatureRadiansPerMeter(150.0) != 1.0 / 300.0 ||
            r300_line.SuperelevationMeters(150.0) != 0.12 ||
            r300_line.superelevation_reference_baselength_meters() != 1.5) {
            std::fprintf(stderr,
                         "installed R300 line does not carry its qualified "
                         "geometry\n");
            return 1;
        }
        // The installed vehicle record, read from where the install put it and
        // assembled with only the installed public headers and export set.
        const auto vehicle =
            orvd::configuration::LoadVehicleDefinitionFromJsonFile(argv[2]);
        double total_mass_kilograms = 0.0;
        for (const auto& body : vehicle.rigid_bodies) {
            total_mass_kilograms += body.mass_kilograms;
        }
        const auto model =
            orvd::configuration::AssembleVehicleMultibodyModel(vehicle, 9.81);
        const auto force_plan =
            orvd::configuration::BuildVehicleForcePlan(vehicle, *model);
        if (!model->is_finalized() || total_mass_kilograms != 55695.0 ||
            model->num_generalized_positions() != 57 ||
            model->num_generalized_velocities() != 50 ||
            force_plan->translational_spring_damper_count() != 20 ||
            force_plan->series_spring_viscous_damper_count() != 2 ||
            force_plan->body_wrench_count() != 56) {
            std::fprintf(stderr,
                         "installed vehicle record did not assemble and "
                         "compile into the vehicle it describes\n");
            return 1;
        }

        // The installed IRW mechanical record, H3 state, R300 line and
        // S1002/UIC60 personality cross the relocated package boundary as one
        // contact-enabled scenario.
        const auto irw_vehicle =
            orvd::configuration::LoadVehicleDefinitionFromJsonFile(argv[5]);
        const auto irw_startup =
            orvd::configuration::LoadResolvedStartupStateFromJsonFile(argv[6]);
        const auto irw_torque_conditioner = orvd::configuration::
            LoadWheelDriveTorqueCommandConditionerFromJsonFile(
                std::filesystem::path(argv[4]) / "vehicle_library" / "irw" /
                "drive_torque_conditioners" /
                "irw_reference_wheel_drive_torque_conditioner.json");
        const auto irw_controller = orvd::configuration::
            LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
                std::filesystem::path(argv[4]) / "controller_library" /
                "irw" /
                "irw_r300_v60_full_state_wheel_speed_guidance_controller.json");
        orvd::control::IrwFullStateWheelSpeedGuidanceControllerInput
            controller_input;
        controller_input.axle_track_stations_meters.fill(100.0);
        controller_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
            .fill(-38.0);
        const auto controller_result = irw_controller.Step(
            controller_input,
            orvd::control::IrwFullStateWheelSpeedGuidanceControllerState{});
        bool controller_produced_nonzero_finite_request = false;
        for (const double request :
             controller_result.requested_wheel_torques_newton_metres) {
            if (!std::isfinite(request)) {
                std::fprintf(stderr,
                             "installed IRW controller produced a non-finite "
                             "request\n");
                return 1;
            }
            controller_produced_nonzero_finite_request =
                controller_produced_nonzero_finite_request || request != 0.0;
        }
        if (irw_controller.config().identifier !=
                "irw_r300_v60_full_state_wheel_speed_guidance_controller" ||
            irw_controller.config().sample_period_seconds != 0.01 ||
            !controller_result.next_state.initialized ||
            !controller_produced_nonzero_finite_request) {
            std::fprintf(stderr,
                         "installed IRW controller did not load and execute "
                         "its real active personality\n");
            return 1;
        }
        const std::array<double, 8> conditioner_requests{
            -194.0, -194.0, -194.0, -194.0,
            -194.0, -194.0, -194.0, -194.0};
        const std::array<double, 8> conditioner_speeds{
            -39.4, -39.4, -39.4, -39.4, -39.4, -39.4, -39.4, -39.4};
        const std::array<double, 8> conditioner_memory{};
        const auto conditioned = irw_torque_conditioner.Step(
            conditioner_requests, conditioner_speeds, conditioner_memory);
        if (irw_torque_conditioner.config().identifier !=
                "irw_reference_wheel_drive_torque_conditioner" ||
            irw_torque_conditioner.config().sample_period_seconds != 0.01 ||
            !(conditioned.wheel_dynamic_torque_limits_newton_metres[0] >
              std::abs(conditioner_requests[0])) ||
            std::abs(conditioned.actual_wheel_torques_newton_metres[0] -
                     conditioner_requests[0]) > 1.0e-12 ||
            !(conditioned.next_drive_side_torque_memory_newton_metres[0] >
              0.0)) {
            std::fprintf(stderr,
                         "installed IRW torque conditioner did not load and "
                         "evaluate its real active table\n");
            return 1;
        }
        auto irw_line =
            orvd::configuration::LoadTrackGeometryFromJsonFile(
                std::filesystem::path(argv[4]) / "track_library" /
                "geometries" /
                "r300_centerline_superelevation_1150m.json");
        auto irw_irregularity = std::make_unique<
            orvd::wheel_rail_contact::TrackIrregularityField>(
            orvd::configuration::LoadTrackIrregularityFieldFromDataRoot(
                argv[4], "irw_r300_aar5_reference_irregularity"));
        const auto irw_scenario =
            orvd::configuration::AssembleIrwContactScenario(
                irw_vehicle, irw_startup, std::move(irw_line), argv[4], 0.0,
                0.01, std::move(irw_irregularity));
        const auto& irw_system = irw_scenario->vehicle_system();
        const auto& irw_resolved = irw_scenario->initial_context();
        if (irw_system.model().num_rigid_bodies() != 25 ||
            irw_system.model().num_generalized_positions() != 81 ||
            irw_system.model().num_generalized_velocities() != 74 ||
            irw_system.force_plan().translational_spring_damper_count() !=
                36 ||
            irw_system.force_plan().roll_spring_damper_couple_count() != 2 ||
            irw_system.force_plan()
                    .half_angle_midpoint_roll_pitch_yaw_bushing_count() != 8 ||
            irw_system.force_plan()
                    .series_spring_viscous_damper_count() != 2 ||
            irw_system.force_plan().body_wrench_count() != 96 ||
            irw_system.system().continuous_state_size() != 157 ||
            irw_system.contact_force_plan() == nullptr ||
            irw_system.contact_force_plan()->carrier_count() != 4 ||
            irw_system.contact_force_plan()->interface_count() != 8 ||
            irw_system.active_torque_plan() == nullptr ||
            irw_system.active_torque_plan()->channel_count() != 8 ||
            irw_system.active_torque_plan()->body_wrench_count() != 16 ||
            irw_system.system().active_torque_body_wrench_count() != 16 ||
            irw_system.system().held_active_torque_count() != 8 ||
            irw_resolved.context().generalized_positions().size() != 81 ||
            irw_resolved.context().generalized_velocities().size() != 74 ||
            irw_resolved.context().series_spring_damper_forces().size() != 2 ||
            irw_resolved.context()
                    .held_independent_wheel_active_torques_newton_metres()
                    .size() != 8 ||
            !irw_resolved.context()
                 .held_independent_wheel_active_torques_newton_metres()
                 .isZero(0.0) ||
            irw_resolved.wheel_pair_placements().size() != 4) {
            std::fprintf(stderr,
                         "installed IRW assets did not assemble their complete "
                         "contact-enabled H3 system\n");
            return 1;
        }
        const std::array<double, 8> installed_active_torques{
            10.0, -20.0, 30.0, -40.0, 50.0, -60.0, 70.0, -80.0};
        irw_system.system().SetHeldIndependentWheelActiveTorques(
            irw_resolved.context(), installed_active_torques);
        Eigen::VectorXd irw_derivatives(157);
        irw_system.compiled_plan().CalcStateTimeDerivatives(
            irw_resolved.context(), irw_derivatives);
        auto irw_contact_workspace =
            irw_system.contact_force_plan()->CreateWorkspace();
        std::vector<orvd::multibody_model::AppliedBodyWrench>
            irw_contact_wrenches(8);
        std::vector<orvd::forces::WheelRailContactInterfaceObservation>
            irw_contact_observations(8);
        const auto irw_component =
            irw_system.system().GetMultibodyComponentView(
                irw_resolved.context(),
                irw_system.system().multibody_component());
        irw_system.contact_force_plan()->CalcAppliedForcesAndObservations(
            irw_component.context(), *irw_contact_workspace,
            irw_resolved.context()
                .wheel_rail_projection_station_hints_meters(),
            irw_contact_wrenches, irw_contact_observations);
        const std::vector<std::string> expected_wheels{
            "wheel_ff_l", "wheel_ff_r", "wheel_fr_l", "wheel_fr_r",
            "wheel_rf_l", "wheel_rf_r", "wheel_rr_l", "wheel_rr_r"};
        for (std::size_t ordinal = 0; ordinal < expected_wheels.size();
             ++ordinal) {
            if (irw_contact_wrenches[ordinal].body !=
                    irw_system.model().GetRigidBodyByName(
                        expected_wheels[ordinal]) ||
                irw_contact_observations[ordinal].contact_patch_count != 1 ||
                !(irw_contact_observations[ordinal].normal_force_newtons >
                  0.0) ||
                !std::isfinite(
                    irw_contact_observations[ordinal].normal_force_newtons)) {
                std::fprintf(stderr,
                             "installed IRW contact interface did not apply "
                             "one finite positive contact to its named wheel\n");
                return 1;
            }
        }
        if (!irw_derivatives.allFinite()) {
            std::fprintf(stderr,
                         "installed IRW contact scenario did not produce a "
                         "finite complete 157-state RHS\n");
            return 1;
        }

        // The installed start-up, line, contact assets and vehicle are one
        // transaction. The returned system owns the exact line and contact
        // personality its direct RHS consumes.
        const auto startup =
            orvd::configuration::LoadResolvedStartupStateFromJsonFile(argv[3]);
        const auto aar6_irregularity =
            orvd::configuration::LoadTrackIrregularityFieldFromDataRoot(
                argv[4], "gz18_aar6_reference_irregularity");
        for (const double station : {50.0, 100.0, 250.0, 300.0}) {
            if (!std::isfinite(
                    aar6_irregularity.LateralDisplacementMeters(station)) ||
                !std::isfinite(
                    aar6_irregularity.VerticalDisplacementMeters(station)) ||
                !std::isfinite(
                    aar6_irregularity.LateralSlopeMetersPerMeter(station)) ||
                !std::isfinite(
                    aar6_irregularity.VerticalSlopeMetersPerMeter(station))) {
                std::fprintf(
                    stderr,
                    "installed GZ18 track-irregularity asset did not produce "
                    "finite values and slopes\n");
                return 1;
            }
        }
        auto aar5_irregularity =
            orvd::configuration::LoadTrackIrregularityFieldFromDataRoot(
                argv[4], "gz18_r300_aar5_reference_irregularity");
        for (const double station : {60.0, 100.0, 960.0, 1000.0}) {
            if (!std::isfinite(
                    aar5_irregularity.LateralDisplacementMeters(station)) ||
                !std::isfinite(
                    aar5_irregularity.VerticalDisplacementMeters(station)) ||
                !std::isfinite(
                    aar5_irregularity.LateralSlopeMetersPerMeter(station)) ||
                !std::isfinite(
                    aar5_irregularity.VerticalSlopeMetersPerMeter(station))) {
                std::fprintf(
                    stderr,
                    "installed GZ18 R300 AAR5 asset did not produce finite "
                    "values and slopes\n");
                return 1;
            }
        }
        const auto scenario =
            orvd::configuration::AssembleGz18ContactScenario(
                vehicle, startup, std::move(r300_line), argv[4], 0.0, 2.0,
                std::make_unique<
                    orvd::wheel_rail_contact::TrackIrregularityField>(
                    std::move(aar5_irregularity)));
        const auto& system = scenario->vehicle_system();
        const auto& resolved = scenario->initial_context();
        Eigen::VectorXd derivatives(system.system().continuous_state_size());
        system.compiled_plan().CalcStateTimeDerivatives(resolved.context(),
                                                        derivatives);
        if (resolved.wheel_pair_placements().size() != 4 ||
            resolved.context().generalized_positions().size() != 57 ||
            resolved.context().series_spring_damper_forces().size() != 2 ||
            system.contact_force_plan() == nullptr ||
            system.contact_force_plan()->carrier_count() != 4 ||
            system.contact_force_plan()->interface_count() != 8 ||
            derivatives.size() != 109 || !derivatives.allFinite()) {
            std::fprintf(stderr,
                         "installed GZ18 contact scenario did not produce its "
                         "complete finite direct RHS\n");
            return 1;
        }

        // A finite RHS alone would also admit an installed profile asset that
        // had drifted into eight zero-contact interfaces. Consume the same
        // installed scenario through the observation surface and require the
        // stated moving start-up to retain its eight physical contacts.
        auto contact_workspace =
            system.contact_force_plan()->CreateWorkspace();
        std::vector<orvd::multibody_model::AppliedBodyWrench> contact_wrenches(
            8);
        std::vector<orvd::forces::WheelRailContactInterfaceObservation>
            contact_observations(8);
        const auto component = system.system().GetMultibodyComponentView(
            resolved.context(), system.system().multibody_component());
        system.contact_force_plan()->CalcAppliedForcesAndObservations(
            component.context(), *contact_workspace,
            resolved.context()
                .wheel_rail_projection_station_hints_meters(),
            contact_wrenches, contact_observations);
        for (const auto& observation : contact_observations) {
            if (observation.contact_patch_count != 1 ||
                !(observation.normal_force_newtons > 0.0) ||
                !std::isfinite(observation.normal_force_newtons)) {
                std::fprintf(
                    stderr,
                    "installed GZ18 contact assets did not produce their "
                    "stated single-patch positive contact\n");
                return 1;
            }
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed configuration smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
