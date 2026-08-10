#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"
#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace orvd::configuration {

class AssembledVehicleContactScenario;
struct ResolvedStartupState;

// What one assembled vehicle system remembers about the description it was
// built from.
//
// The model and the force plan answer questions by name but do not enumerate
// the names they hold, so an all-and-only check against a start-up state has no
// other place to read the vehicle's own name sets from. This snapshot owns its
// strings: the description it came from stays modifiable, and a system already
// built from it must not change when it does.
//
// Equality of `mechanical_definition_identifier` is identity. Whether a user
// who edits a mechanical parameter also updates that identifier is the caller's
// responsibility; nothing here digests content, compares parameters element by
// element, or revises the identifier on its own.
struct VehicleFreeBodyStationOffset {
    std::string body_name;
    double station_offset_meters{0.0};
};

struct VehicleBinding {
    std::string vehicle_name;
    std::string mechanical_definition_identifier;
    std::string reference_body_name;
    std::vector<VehicleFreeBodyStationOffset> free_body_station_offsets;
    std::vector<std::string> revolute_joint_names;
    std::vector<std::string> ball_rpy_joint_names;
    std::vector<std::string> translational_spring_damper_names;
    std::vector<std::string> series_spring_viscous_damper_names;
    std::vector<std::string> wheel_pair_station_reference_body_names;
};

// One vehicle, assembled once, with everything downstream needs to reach it.
//
// The model, its typed force plans, the system instance and the compiled plan
// borrow one another and none of them may be moved, so holding them separately
// would make every caller responsible for a construction order and a
// destruction order it did not choose. This owns them in dependency order and
// releases them in reverse.
//
// It must outlive every runtime context created from it, every resolved
// start-up result that holds such a context, and every integrator advancing
// one.
class AssembledVehicleSystem {
   public:
    ~AssembledVehicleSystem();

    AssembledVehicleSystem(const AssembledVehicleSystem&) = delete;
    AssembledVehicleSystem& operator=(const AssembledVehicleSystem&) = delete;
    AssembledVehicleSystem(AssembledVehicleSystem&&) = delete;
    AssembledVehicleSystem& operator=(AssembledVehicleSystem&&) = delete;

    [[nodiscard]] const multibody_model::MultibodyModel& model() const {
        return *model_;
    }
    [[nodiscard]] const forces::VehicleForcePlan& force_plan() const {
        return *force_plan_;
    }
    [[nodiscard]] const forces::WheelRailContactForcePlan*
    contact_force_plan() const {
        return contact_force_plan_.get();
    }
    [[nodiscard]] const forces::IndependentWheelActiveTorquePlan*
    active_torque_plan() const {
        return active_torque_plan_.get();
    }
    [[nodiscard]] const system_assembly::SystemInstance& system() const {
        return *system_;
    }
    [[nodiscard]] const system_assembly::CompiledSystemPlan& compiled_plan()
        const {
        return *compiled_plan_;
    }
    [[nodiscard]] const VehicleBinding& binding() const { return binding_; }

   private:
    friend std::unique_ptr<AssembledVehicleSystem> AssembleVehicleSystem(
        const VehicleDefinition&, double);
    friend class AssembledVehicleContactScenario;
    friend std::unique_ptr<AssembledVehicleContactScenario>
    AssembleGz18ContactScenario(
        const VehicleDefinition&, const ResolvedStartupState&,
        track_geometry::TrackGeometry, const std::filesystem::path&, double,
        double,
        std::unique_ptr<wheel_rail_contact::TrackIrregularityField>);
    friend std::unique_ptr<AssembledVehicleContactScenario>
    AssembleIrwContactScenario(
        const VehicleDefinition&, const ResolvedStartupState&,
        track_geometry::TrackGeometry, const std::filesystem::path&, double,
        double,
        std::unique_ptr<wheel_rail_contact::TrackIrregularityField>);

    AssembledVehicleSystem(
        std::unique_ptr<multibody_model::MultibodyModel> model,
        std::unique_ptr<forces::VehicleForcePlan> force_plan,
        std::unique_ptr<forces::WheelRailContactForcePlan> contact_force_plan,
        std::unique_ptr<forces::IndependentWheelActiveTorquePlan>
            active_torque_plan,
        std::unique_ptr<system_assembly::SystemInstance> system,
        std::unique_ptr<system_assembly::CompiledSystemPlan> compiled_plan,
        VehicleBinding binding);

    static std::unique_ptr<AssembledVehicleSystem> AssembleWithWheelRailContact(
        const VehicleDefinition& vehicle,
        double gravitational_acceleration_magnitude_meters_per_second_squared,
        track_geometry::TrackGeometry line,
        std::unique_ptr<
            wheel_rail_contact::WheelRailContactRuntimePersonality>
            personality,
        std::unique_ptr<wheel_rail_contact::TrackIrregularityField>
            track_irregularity,
        std::vector<forces::WheelRailContactCarrierDefinition> carriers,
        std::vector<forces::WheelRailContactInterfaceDefinition> interfaces,
        std::vector<forces::IndependentWheelActiveTorqueCoupleDefinition>
            active_torque_couples,
        double projection_search_half_width_meters);

    // Declaration order is construction order; destruction runs in reverse, so
    // each object outlives the ones that borrow it.
    std::unique_ptr<multibody_model::MultibodyModel> model_;
    std::unique_ptr<forces::VehicleForcePlan> force_plan_;
    std::unique_ptr<forces::WheelRailContactForcePlan> contact_force_plan_;
    std::unique_ptr<forces::IndependentWheelActiveTorquePlan>
        active_torque_plan_;
    std::unique_ptr<system_assembly::SystemInstance> system_;
    std::unique_ptr<system_assembly::CompiledSystemPlan> compiled_plan_;
    VehicleBinding binding_;
};

// Assembles the model, the force plan, the system instance and the compiled
// plan from one call-time value of the description, and records the name sets a
// start-up state will be checked against.
//
// Gravity is the caller's to state for the same reason it is in
// `AssembleVehicleMultibodyModel`: the same vehicle is the same vehicle on any
// planet. A resolved start-up state, on the other hand, is solved under one
// gravity and states it, so the two are matched when a start-up context is
// assembled — not here.
//
// The description is read once and not retained.
//
// Throws whatever `AssembleVehicleMultibodyModel` and `BuildVehicleForcePlan`
// throw for a description this layer cannot accept.
[[nodiscard]] std::unique_ptr<AssembledVehicleSystem> AssembleVehicleSystem(
    const VehicleDefinition& vehicle,
    double gravitational_acceleration_magnitude_meters_per_second_squared);

}  // namespace orvd::configuration
