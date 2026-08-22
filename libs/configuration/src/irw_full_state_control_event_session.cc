#include "orvd/configuration/irw_full_state_control_event_session.h"

#include <bit>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::configuration {
namespace {

constexpr double kFrozenSamplePeriodSeconds = 0.01;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("IRW full-state control event session: " +
                                detail);
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

}  // namespace

IrwFullStateControlEventSession::IrwFullStateControlEventSession(
    const AssembledVehicleSystem& assembled,
    control::IrwFullStateWheelSpeedGuidanceController controller,
    actuation::WheelDriveTorqueCommandConditioner conditioner)
    : assembled_(&assembled),
      observation_binding_(assembled),
      controller_(std::move(controller)),
      conditioner_(std::move(conditioner)),
      sample_period_seconds_(controller_.config().sample_period_seconds) {
    const double conditioner_period =
        conditioner_.config().sample_period_seconds;
    if (!SameBits(sample_period_seconds_, conditioner_period) ||
        !SameBits(sample_period_seconds_, kFrozenSamplePeriodSeconds)) {
        Reject("the controller and conditioner sample periods must be "
               "bitwise-equal frozen 0.01-second values");
    }
}

double IrwFullStateControlEventSession::next_periodic_event_time_seconds()
    const noexcept {
    return static_cast<double>(next_periodic_event_ordinal_) *
           sample_period_seconds_;
}

control::IrwFullStateWheelSpeedGuidanceControllerInput
IrwFullStateControlEventSession::ObserveMechanicalInput(
    system_assembly::SystemRuntimeContext& context) const {
    const auto observation = observation_binding_.Observe(context);
    return {
        .axle_lateral_displacements_meters =
            observation.mechanical_input.axle_lateral_displacements_meters,
        .axle_yaw_angles_radians =
            observation.mechanical_input.axle_yaw_angles_radians,
        .axle_track_stations_meters = observation.axle_track_stations_meters,
        .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second =
            observation.mechanical_input
                .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
    };
}

IrwFullStateControlEventAudit
IrwFullStateControlEventSession::ComputeCandidate(
    IrwFullStateControlEventKind kind, std::uint64_t ordinal,
    double event_time_seconds,
    system_assembly::SystemRuntimeContext& accepted_context) const {
    IrwFullStateControlEventAudit audit;
    audit.kind = kind;
    audit.periodic_event_ordinal = ordinal;
    audit.event_time_seconds = event_time_seconds;
    audit.mechanical_input = ObserveMechanicalInput(accepted_context);
    audit.controller_state_before = controller_state_;
    audit.conditioner_memory_before_newton_metres =
        conditioner_memory_newton_metres_;
    audit.controller_result =
        controller_.Step(audit.mechanical_input, controller_state_);
    audit.conditioning_result = conditioner_.Step(
        audit.controller_result.requested_wheel_torques_newton_metres,
        audit.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        conditioner_memory_newton_metres_);
    return audit;
}

IrwFullStateControlEventAudit
IrwFullStateControlEventSession::ApplyInitializationUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ != Phase::kInitializationRequired) {
        Reject("the initialization recurrence has already been committed");
    }
    if (!SameBits(accepted_context.time_seconds(), 0.0)) {
        Reject("the initialization recurrence requires accepted time zero");
    }
    IrwFullStateControlEventAudit audit = ComputeCandidate(
        IrwFullStateControlEventKind::kInitialization, 0, 0.0,
        accepted_context);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    phase_ = Phase::kPeriodicEventRequired;
    return audit;
}

IrwFullStateControlEventAudit
IrwFullStateControlEventSession::ApplyPeriodicUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ == Phase::kInitializationRequired) {
        Reject("the initialization recurrence has not been committed");
    }
    if (phase_ == Phase::kSynchronizationRequired) {
        Reject("the preceding event is committed but its backend is not "
               "synchronized");
    }
    const double expected_time = next_periodic_event_time_seconds();
    if (!SameBits(accepted_context.time_seconds(), expected_time)) {
        Reject("accepted time does not equal the next integer-grid event "
               "time");
    }

    // Once a valid boundary has been presented, any calculation or commit
    // failure leaves this event due and therefore blocks another advance.
    phase_ = Phase::kPeriodicEventRequired;
    IrwFullStateControlEventAudit audit = ComputeCandidate(
        IrwFullStateControlEventKind::kPeriodic,
        next_periodic_event_ordinal_, expected_time, accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    ++next_periodic_event_ordinal_;
    phase_ = Phase::kSynchronizationRequired;
    return audit;
}

bool IrwFullStateControlEventSession::synchronization_required() const
    noexcept {
    return phase_ == Phase::kSynchronizationRequired;
}

void IrwFullStateControlEventSession::ConfirmBackendSynchronized() {
    if (phase_ != Phase::kSynchronizationRequired) {
        Reject("there is no committed event awaiting backend "
               "synchronization");
    }
    phase_ = Phase::kReadyToAdvance;
}

void IrwFullStateControlEventSession::RequireReadyToAdvance() const {
    if (phase_ != Phase::kReadyToAdvance) {
        Reject("the next vehicle advance is blocked until the required "
               "control event and backend synchronization succeed");
    }
}

}  // namespace orvd::configuration
