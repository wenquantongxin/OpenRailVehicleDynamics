#pragma once

/// @file
/// The immutable, model-neutral description of the first executable system.
///
/// The first consumer is one finalized multibody component.  Its continuous
/// state consists of q and v, its physical parameters remain the component's
/// typed context-local records, and its output is the time derivative of every
/// state block the description declares.
///
/// A vehicle's force elements may carry continuous state of their own.  Where
/// they do, the description states how many such components exist, by naming
/// the one constitutive family that has them rather than by opening a registry:
/// there is still no string lookup, no append operation, no generic parameter
/// bag and no runtime value here.  A second family with real state adds a
/// second named count when a real consumer for it exists.
///
/// The description carries no call-time applied force.  The system's admitted
/// force sources are its typed vehicle plan and, when present, its typed
/// wheel--rail contact and independent-wheel active-torque plans. A caller who
/// wants to apply an arbitrary wrench for research or for a unit test uses the
/// multibody facade one layer below.

namespace orvd::forces {
class IndependentWheelActiveTorquePlan;
class VehicleForcePlan;
class WheelRailContactForcePlan;
}

namespace orvd::multibody_model {
class MultibodyModel;
}

namespace orvd::system_assembly {

class SystemAssemblyDescription {
   public:
    /// Describes `model` without taking ownership or creating runtime state.
    /// The model must outlive this description and every system compiled from
    /// it.  This form declares no force-element state.
    ///
    /// @throws std::logic_error if the model has not been finalized.
    explicit SystemAssemblyDescription(
        const multibody_model::MultibodyModel& model);

    /// The same, for a vehicle that carries force elements.  The plan supplies
    /// both the number of state components and the number of nominal-force
    /// slots, so neither is a number a caller could get wrong: how much state
    /// there is follows from which constitutive families the vehicle has.
    ///
    /// The plan must have been compiled against the same model and must outlive
    /// this description and every `SystemInstance`, `CompiledSystemPlan` and
    /// advancer derived from it.
    ///
    /// @throws std::invalid_argument if the plan was compiled against a
    /// different model.
    SystemAssemblyDescription(const multibody_model::MultibodyModel& model,
                              const forces::VehicleForcePlan& force_plan);
    /// The same vehicle plus one frozen wheel--rail contact force plan.
    /// Both plans must have been compiled against `model` and must outlive
    /// every object derived from this description.
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel& model,
        const forces::VehicleForcePlan& vehicle_force_plan,
        const forces::WheelRailContactForcePlan& contact_force_plan);
    /// The same vehicle and contact graph plus one frozen active-torque plan.
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel& model,
        const forces::VehicleForcePlan& vehicle_force_plan,
        const forces::WheelRailContactForcePlan& contact_force_plan,
        const forces::IndependentWheelActiveTorquePlan& active_torque_plan);
    SystemAssemblyDescription(multibody_model::MultibodyModel&&,
                              const forces::VehicleForcePlan&) = delete;
    SystemAssemblyDescription(const multibody_model::MultibodyModel&&,
                              const forces::VehicleForcePlan&) = delete;
    SystemAssemblyDescription(const multibody_model::MultibodyModel&,
                              forces::VehicleForcePlan&&) = delete;
    SystemAssemblyDescription(const multibody_model::MultibodyModel&,
                              const forces::VehicleForcePlan&&) = delete;
    SystemAssemblyDescription(
        multibody_model::MultibodyModel&&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        forces::VehicleForcePlan&&,
        const forces::WheelRailContactForcePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&&,
        const forces::WheelRailContactForcePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&,
        forces::WheelRailContactForcePlan&&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&&) = delete;
    SystemAssemblyDescription(
        multibody_model::MultibodyModel&&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&,
        const forces::IndependentWheelActiveTorquePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&,
        const forces::IndependentWheelActiveTorquePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        forces::VehicleForcePlan&&,
        const forces::WheelRailContactForcePlan&,
        const forces::IndependentWheelActiveTorquePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&&,
        const forces::WheelRailContactForcePlan&,
        const forces::IndependentWheelActiveTorquePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&,
        forces::WheelRailContactForcePlan&&,
        const forces::IndependentWheelActiveTorquePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&&,
        const forces::IndependentWheelActiveTorquePlan&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&,
        forces::IndependentWheelActiveTorquePlan&&) = delete;
    SystemAssemblyDescription(
        const multibody_model::MultibodyModel&,
        const forces::VehicleForcePlan&,
        const forces::WheelRailContactForcePlan&,
        const forces::IndependentWheelActiveTorquePlan&&) = delete;
    SystemAssemblyDescription(multibody_model::MultibodyModel&&) = delete;
    SystemAssemblyDescription(const multibody_model::MultibodyModel&&) =
        delete;

    SystemAssemblyDescription(const SystemAssemblyDescription&) = delete;
    SystemAssemblyDescription& operator=(const SystemAssemblyDescription&) =
        delete;
    SystemAssemblyDescription(SystemAssemblyDescription&&) = delete;
    SystemAssemblyDescription& operator=(SystemAssemblyDescription&&) = delete;

    /// The sole component in the first assembly contract.
    [[nodiscard]] const multibody_model::MultibodyModel& multibody_model() const;

    /// The vehicle's force elements, or null when the system carries none.
    [[nodiscard]] const forces::VehicleForcePlan* force_plan() const {
        return force_plan_;
    }

    /// The wheel--rail contact force source, or null when none was assembled.
    [[nodiscard]] const forces::WheelRailContactForcePlan* contact_force_plan()
        const {
        return contact_force_plan_;
    }

    /// The named independent-wheel active-torque source, or null when absent.
    [[nodiscard]] const forces::IndependentWheelActiveTorquePlan*
    active_torque_plan() const {
        return active_torque_plan_;
    }

    /// Sizes of the distinct continuous-state blocks, in state order.
    [[nodiscard]] int generalized_position_count() const;
    [[nodiscard]] int generalized_velocity_count() const;
    [[nodiscard]] int series_spring_damper_force_state_count() const;

    /// Three components per translational element, all zero until a resolved
    /// start-up state fills them in a context.
    [[nodiscard]] int nominal_force_component_count() const;

    [[nodiscard]] int vehicle_body_wrench_count() const;
    [[nodiscard]] int contact_body_wrench_count() const;
    [[nodiscard]] int active_torque_body_wrench_count() const;
    [[nodiscard]] int held_active_torque_count() const;

    /// Size of the output [qdot; vdot; zdot].
    [[nodiscard]] int state_time_derivative_size() const;

   private:
    const multibody_model::MultibodyModel* multibody_model_;
    const forces::VehicleForcePlan* force_plan_{nullptr};
    const forces::WheelRailContactForcePlan* contact_force_plan_{nullptr};
    const forces::IndependentWheelActiveTorquePlan* active_torque_plan_{
        nullptr};
    int generalized_position_count_{};
    int generalized_velocity_count_{};
    int series_spring_damper_force_state_count_{};
    int nominal_force_component_count_{};
    int vehicle_body_wrench_count_{};
    int contact_body_wrench_count_{};
    int active_torque_body_wrench_count_{};
    int held_active_torque_count_{};
};

}  // namespace orvd::system_assembly
