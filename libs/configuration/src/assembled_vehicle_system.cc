#include "orvd/configuration/assembled_vehicle_system.h"

#include <utility>

#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "vehicle_definition_invariants.h"

namespace orvd::configuration {
namespace {

VehicleBinding CaptureBinding(const VehicleDefinition& vehicle) {
    VehicleBinding binding;
    binding.vehicle_name = vehicle.vehicle_name;
    binding.mechanical_definition_identifier =
        vehicle.mechanical_definition_identifier;
    binding.reference_body_name =
        vehicle.mechanical_track_station_layout.reference_body_name;

    binding.free_body_station_offsets.reserve(
        vehicle.mechanical_track_station_layout.free_body_station_offsets
            .size());
    for (const VehicleFreeBodyStationOffsetDefinition& offset :
         vehicle.mechanical_track_station_layout.free_body_station_offsets) {
        binding.free_body_station_offsets.push_back(
            VehicleFreeBodyStationOffset{offset.body_name,
                                         offset.station_offset_meters});
    }
    binding.wheel_pair_station_reference_body_names =
        vehicle.mechanical_track_station_layout
            .wheel_pair_station_reference_body_names;

    // Revolute and Ball-RPY joints keep separate typed name families because
    // their coordinate blocks have different shapes. A weld's position and
    // velocity ranges are empty, so a start-up state that listed one would be
    // stating a coordinate nobody owns.
    binding.revolute_joint_names.reserve(
        vehicle.revolute_joints.size());
    for (const VehicleRevoluteJointDefinition& joint :
         vehicle.revolute_joints) {
        binding.revolute_joint_names.push_back(joint.name);
    }

    binding.ball_rpy_joint_names.reserve(vehicle.ball_rpy_joints.size());
    for (const VehicleBallRpyJointDefinition& joint :
         vehicle.ball_rpy_joints) {
        binding.ball_rpy_joint_names.push_back(joint.name);
    }

    binding.translational_spring_damper_names.reserve(
        vehicle.translational_spring_dampers.size());
    for (const VehicleTranslationalSpringDamperDefinition& element :
         vehicle.translational_spring_dampers) {
        binding.translational_spring_damper_names.push_back(element.name);
    }

    binding.series_spring_viscous_damper_names.reserve(
        vehicle.series_spring_viscous_dampers.size());
    for (const VehicleSeriesSpringViscousDamperDefinition& element :
         vehicle.series_spring_viscous_dampers) {
        binding.series_spring_viscous_damper_names.push_back(element.name);
    }
    return binding;
}

}  // namespace

AssembledVehicleSystem::AssembledVehicleSystem(
    std::unique_ptr<multibody_model::MultibodyModel> model,
    std::unique_ptr<forces::VehicleForcePlan> force_plan,
    std::unique_ptr<forces::WheelRailContactForcePlan> contact_force_plan,
    std::unique_ptr<forces::IndependentWheelActiveTorquePlan>
        active_torque_plan,
    std::unique_ptr<system_assembly::SystemInstance> system,
    std::unique_ptr<system_assembly::CompiledSystemPlan> compiled_plan,
    VehicleBinding binding)
    : model_(std::move(model)),
      force_plan_(std::move(force_plan)),
      contact_force_plan_(std::move(contact_force_plan)),
      active_torque_plan_(std::move(active_torque_plan)),
      system_(std::move(system)),
      compiled_plan_(std::move(compiled_plan)),
      binding_(std::move(binding)) {}

AssembledVehicleSystem::~AssembledVehicleSystem() = default;

std::unique_ptr<AssembledVehicleSystem> AssembleVehicleSystem(
    const VehicleDefinition& vehicle,
    double gravitational_acceleration_magnitude_meters_per_second_squared) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(
        vehicle, "vehicle definition");
    VehicleBinding binding = CaptureBinding(vehicle);

    std::unique_ptr<multibody_model::MultibodyModel> model =
        AssembleVehicleMultibodyModel(
            vehicle,
            gravitational_acceleration_magnitude_meters_per_second_squared);
    std::unique_ptr<forces::VehicleForcePlan> force_plan =
        BuildVehicleForcePlan(vehicle, *model);

    // The description is read by the system instance and then finished with: it
    // hands over the borrowed plan pointers and fixed layout counts, and keeps
    // no reference to itself. Holding it for the lifetime of the bundle would
    // suggest a borrow that does not exist.
    const system_assembly::SystemAssemblyDescription description(*model,
                                                                *force_plan);
    auto system =
        std::make_unique<system_assembly::SystemInstance>(description);
    auto compiled_plan =
        std::make_unique<system_assembly::CompiledSystemPlan>(*system);

    return std::unique_ptr<AssembledVehicleSystem>(new AssembledVehicleSystem(
        std::move(model), std::move(force_plan), nullptr, nullptr,
        std::move(system), std::move(compiled_plan), std::move(binding)));
}

std::unique_ptr<AssembledVehicleSystem>
AssembledVehicleSystem::AssembleWithWheelRailContact(
    const VehicleDefinition& vehicle,
    double gravitational_acceleration_magnitude_meters_per_second_squared,
    track_geometry::TrackGeometry line,
    std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality>
        personality,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField>
        track_irregularity,
    std::vector<forces::WheelRailContactCarrierDefinition> carriers,
    std::vector<forces::WheelRailContactInterfaceDefinition> interfaces,
    std::vector<forces::IndependentWheelActiveTorqueCoupleDefinition>
        active_torque_couples) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(
        vehicle, "vehicle definition");
    VehicleBinding binding = CaptureBinding(vehicle);

    auto model = AssembleVehicleMultibodyModel(
        vehicle,
        gravitational_acceleration_magnitude_meters_per_second_squared);
    auto force_plan = BuildVehicleForcePlan(vehicle, *model);
    auto contact_force_plan =
        std::make_unique<forces::WheelRailContactForcePlan>(
            *model, std::move(line), std::move(personality),
            std::move(track_irregularity),
            std::move(carriers), std::move(interfaces));
    std::unique_ptr<forces::IndependentWheelActiveTorquePlan>
        active_torque_plan;
    if (!active_torque_couples.empty()) {
        active_torque_plan =
            std::make_unique<forces::IndependentWheelActiveTorquePlan>(
                *model, std::move(active_torque_couples));
    }
    std::unique_ptr<system_assembly::SystemInstance> system;
    if (active_torque_plan != nullptr) {
        const system_assembly::SystemAssemblyDescription description(
            *model, *force_plan, *contact_force_plan, *active_torque_plan);
        system =
            std::make_unique<system_assembly::SystemInstance>(description);
    } else {
        const system_assembly::SystemAssemblyDescription description(
            *model, *force_plan, *contact_force_plan);
        system =
            std::make_unique<system_assembly::SystemInstance>(description);
    }
    auto compiled_plan =
        std::make_unique<system_assembly::CompiledSystemPlan>(*system);

    return std::unique_ptr<AssembledVehicleSystem>(new AssembledVehicleSystem(
        std::move(model), std::move(force_plan),
        std::move(contact_force_plan), std::move(active_torque_plan),
        std::move(system),
        std::move(compiled_plan), std::move(binding)));
}

}  // namespace orvd::configuration
