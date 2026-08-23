#include "irw_guidance_control_event_session.h"

#include <bit>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "irw_guidance_control_transaction.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("IRW guidance control event session: " +
                                detail);
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

}  // namespace

IrwGuidanceControlEventSession::IrwGuidanceControlEventSession(
    const configuration::AssembledVehicleSystem& assembled,
    control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig recurrence_config,
    IrwOperatingPointEvaluator operating_point_evaluator,
    actuation::WheelDriveTorqueCommandConditioner conditioner)
    : assembled_(&assembled),
      observation_binding_(assembled),
      recurrence_(std::move(recurrence_config)),
      operating_point_evaluator_(std::move(operating_point_evaluator)),
      conditioner_(std::move(conditioner)),
      sample_period_seconds_(recurrence_.config().sample_period_seconds) {
    if (!operating_point_evaluator_) {
        Reject("the operating-point evaluator must be non-empty");
    }
    if (!SameBits(sample_period_seconds_, kControlSamplePeriodSeconds) ||
        !SameBits(sample_period_seconds_,
                  conditioner_.config().sample_period_seconds)) {
        Reject("the recurrence and conditioner sample periods must be "
               "bitwise-equal 0.01-second values");
    }
}

double IrwGuidanceControlEventSession::next_periodic_event_time_seconds()
    const noexcept {
    return static_cast<double>(next_periodic_event_ordinal_) *
           sample_period_seconds_;
}

configuration::IrwFullStateControlMechanicalObservation
IrwGuidanceControlEventSession::ObserveMechanicalInput(
    system_assembly::SystemRuntimeContext& context) const {
    return observation_binding_.Observe(context);
}

IrwGuidanceControlEventAudit
IrwGuidanceControlEventSession::ComputeCandidate(
    IrwGuidanceControlEventKind kind, std::uint64_t ordinal,
    double event_time_seconds,
    system_assembly::SystemRuntimeContext& accepted_context) const {
    IrwGuidanceControlEventAudit audit;
    audit.kind = kind;
    audit.periodic_event_ordinal = ordinal;
    audit.event_time_seconds = event_time_seconds;
    audit.mechanical_observation = ObserveMechanicalInput(accepted_context);
    audit.operating_point = operating_point_evaluator_(
        audit.mechanical_observation.axle_track_stations_meters);
    const auto transaction = ComputeIrwGuidanceControlTransaction(
        recurrence_, conditioner_,
        audit.mechanical_observation.mechanical_input, audit.operating_point,
        controller_state_, conditioner_memory_newton_metres_);
    audit.controller_state_before = transaction.controller_state_before;
    audit.conditioner_memory_before_newton_metres =
        transaction.conditioner_memory_before_newton_metres;
    audit.controller_result = transaction.controller_result;
    audit.common_mode_probe_requests_newton_metres =
        transaction.common_mode_probe_requests_newton_metres;
    audit.common_mode_conditioning_probe =
        transaction.common_mode_conditioning_probe;
    audit.prioritized_wheel_torque_requests_newton_metres =
        transaction.prioritized_wheel_torque_requests_newton_metres;
    audit.conditioning_result = transaction.conditioning_result;
    return audit;
}

IrwGuidanceControlEventAudit
IrwGuidanceControlEventSession::ApplyInitializationUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ != Phase::kInitializationRequired) {
        Reject("the initialization event has already been committed");
    }
    if (!SameBits(accepted_context.time_seconds(), 0.0)) {
        Reject("the initialization event requires accepted time zero");
    }

    IrwGuidanceControlEventAudit audit = ComputeCandidate(
        IrwGuidanceControlEventKind::kInitialization, 0, 0.0,
        accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    next_periodic_event_ordinal_ = 1;
    phase_ = Phase::kSynchronizationRequired;
    return audit;
}

IrwGuidanceControlEventAudit
IrwGuidanceControlEventSession::ApplyScheduledUpdate(
    IrwGuidanceControlEventKind kind,
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (kind == IrwGuidanceControlEventKind::kInitialization) {
        Reject("an initialization event cannot use the scheduled-update "
               "path");
    }
    if (phase_ == Phase::kInitializationRequired) {
        Reject("the initialization event has not been committed");
    }
    if (phase_ == Phase::kSynchronizationRequired) {
        Reject("the preceding event is committed but its backend is not "
               "synchronized");
    }
    if (phase_ == Phase::kTerminalEventCommitted) {
        Reject("the terminal event has already been committed");
    }
    const double expected_time = next_periodic_event_time_seconds();
    if (!SameBits(accepted_context.time_seconds(), expected_time)) {
        Reject("accepted time does not equal the next integer-grid event "
               "time");
    }

    // Once a valid boundary is presented, a failed calculation or context
    // commit leaves exactly this event due.  No controller or conditioner
    // memory changes until the complete candidate and held-torque vector have
    // both been accepted.
    phase_ = Phase::kPeriodicEventRequired;
    IrwGuidanceControlEventAudit audit = ComputeCandidate(
        kind, next_periodic_event_ordinal_, expected_time, accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    ++next_periodic_event_ordinal_;
    phase_ = kind == IrwGuidanceControlEventKind::kTerminal
                 ? Phase::kTerminalEventCommitted
                 : Phase::kSynchronizationRequired;
    return audit;
}

IrwGuidanceControlEventAudit
IrwGuidanceControlEventSession::ApplyPeriodicUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    return ApplyScheduledUpdate(IrwGuidanceControlEventKind::kPeriodic,
                                accepted_context);
}

IrwGuidanceControlEventAudit
IrwGuidanceControlEventSession::ApplyTerminalUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    return ApplyScheduledUpdate(IrwGuidanceControlEventKind::kTerminal,
                                accepted_context);
}

bool IrwGuidanceControlEventSession::synchronization_required() const
    noexcept {
    return phase_ == Phase::kSynchronizationRequired;
}

bool IrwGuidanceControlEventSession::terminal_event_committed() const
    noexcept {
    return phase_ == Phase::kTerminalEventCommitted;
}

void IrwGuidanceControlEventSession::ConfirmBackendSynchronized() {
    if (phase_ != Phase::kSynchronizationRequired) {
        Reject("there is no committed event awaiting backend "
               "synchronization");
    }
    phase_ = Phase::kReadyToAdvance;
}

void IrwGuidanceControlEventSession::RequireReadyToAdvance() const {
    if (phase_ != Phase::kReadyToAdvance) {
        Reject("the next vehicle advance is blocked until the required "
               "control event and backend synchronization succeed");
    }
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
