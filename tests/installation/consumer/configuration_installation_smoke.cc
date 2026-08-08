#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/configuration/assemble_resolved_initial_context.h"
#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/gz18_wheel_rail_contact.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"

int main(int argc, char* argv[]) {
    try {
        if (argc != 5) {
            throw std::invalid_argument(
                "expected the installed track geometry, vehicle definition and "
                "resolved start-up state paths plus the installed data root");
        }
        const auto line =
            orvd::configuration::LoadTrackGeometryFromJsonFile(argv[1]);
        const auto point = line.CenterlinePositionInInertialMeters(40.0);
        if (std::abs(point.x() - 40.0) > 1.0e-12 || point.y() != 0.0 ||
            point.z() != 0.0) {
            std::fprintf(stderr,
                         "installed configuration smoke produced an invalid "
                         "straight-line point\n");
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

        // The installed start-up state, placed on the installed line. This is
        // the whole chain a consumer of the package walks: three installed
        // records in, one runtime context out.
        const auto startup =
            orvd::configuration::LoadResolvedStartupStateFromJsonFile(argv[3]);
        const auto system = orvd::configuration::AssembleVehicleSystem(
            vehicle,
            startup.gravitational_acceleration_meters_per_second_squared);
        const auto resolved =
            orvd::configuration::AssembleResolvedInitialContext(
                *system, startup, line, 20.0);
        if (resolved.wheelset_placements().size() != 4 ||
            resolved.context().generalized_positions().size() != 57 ||
            resolved.context().series_spring_damper_forces().size() != 2) {
            std::fprintf(stderr,
                         "installed start-up state did not assemble into the "
                         "context it describes\n");
            return 1;
        }

        // Resolve the installed logical profile identities under the relocated
        // data root, then run the complete contact core once. This is the
        // evidence that the installed JSON assets and the installed typed
        // personality meet, not merely that two files happened to be copied.
        const auto contact =
            orvd::configuration::AssembleGz18WheelRailContact(
                argv[4], startup.wheel_rail_binding,
                startup.rail_profile_reference_vertical_offset_meters);
        using orvd::wheel_rail_contact::WheelSide;
        const auto& constants = contact->pose_constants(WheelSide::kRight);
        orvd::wheel_rail_contact::WheelRailPoseInput pose_input;
        pose_input.placement.vertical_meters =
            -constants.nominal_rolling_radius_meters;
        pose_input.arc_rate_meters_per_second =
            startup.initial_longitudinal_speed_meters_per_second;

        orvd::wheel_rail_contact::WheelRailContactInput contact_input;
        contact_input.pose = orvd::wheel_rail_contact::BuildContactPoseScalars(
            constants, pose_input, {});
        contact_input.rail_frame.origin_track_meters = Eigen::Vector3d(
            0.0, constants.rail_lateral_datum_meters,
            constants.rail_vertical_datum_meters);
        contact_input.rail_frame.rotation_track_from_profile =
            Eigen::AngleAxisd(constants.rail_roll_radians,
                              Eigen::Vector3d::UnitX())
                .toRotationMatrix();
        contact_input.wheel.origin_track_meters = Eigen::Vector3d(
            0.0, constants.wheel_lateral_datum_meters,
            -constants.nominal_rolling_radius_meters);
        contact_input.wheel.origin_velocity_track_meters_per_second =
            Eigen::Vector3d(
                startup.initial_longitudinal_speed_meters_per_second, 0.0,
                0.0);
        const double wheel_spin =
            -startup.initial_longitudinal_speed_meters_per_second /
            startup.common_startup_effective_rolling_radius_meters;
        contact_input.wheel.angular_velocity_track_radians_per_second =
            Eigen::Vector3d(0.0, wheel_spin, 0.0);
        contact_input.wheel.arc_rate_meters_per_second =
            startup.initial_longitudinal_speed_meters_per_second;
        contact_input.wheel.wheel_pitch_rate_radians_per_second = wheel_spin;
        orvd::wheel_rail_contact::WheelRailContactWorkspace contact_workspace;
        contact->model(WheelSide::kRight).PrepareWorkspace(contact_workspace);
        const auto contact_result = contact->model(WheelSide::kRight).Evaluate(
            contact_input, contact_workspace);
        if (contact_result.count != 1 ||
            !contact_result.patches[0].normal.in_contact ||
            !(contact_result.patches[0].normal.normal_force_newtons > 0.0)) {
            std::fprintf(stderr,
                         "installed GZ18 contact assets did not produce the "
                         "contact they describe\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed configuration smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
