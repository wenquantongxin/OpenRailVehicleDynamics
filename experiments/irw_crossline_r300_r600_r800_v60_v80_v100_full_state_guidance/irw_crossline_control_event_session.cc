#include "irw_crossline_control_event_session.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

constexpr double kCommonModeProbeThresholdNewtonMetres = 1.0e-9;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("IRW cross-line control event session: " +
                                detail);
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

[[nodiscard]] actuation::WheelDriveTorqueChannelValues
MakeCommonModeProbeRequest(
    const actuation::WheelDriveTorqueChannelValues& raw_requests) {
    auto probe_requests = raw_requests;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount;
         ++axle) {
        const std::size_t left = 2 * axle;
        const std::size_t right = left + 1;
        const double common =
            0.5 * (raw_requests[left] + raw_requests[right]);
        if (std::abs(common) > kCommonModeProbeThresholdNewtonMetres) {
            probe_requests[left] = common;
            probe_requests[right] = common;
        }
    }
    return probe_requests;
}

[[nodiscard]] double SanitizeTorqueLimit(double limit) {
    if (!std::isfinite(limit)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, limit);
}

[[nodiscard]] double SignOrOne(double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

[[nodiscard]] actuation::WheelDriveTorqueChannelValues
ApplyLongitudinalCommonModePriority(
    const actuation::WheelDriveTorqueChannelValues& raw_requests,
    const actuation::WheelDriveTorqueChannelValues& dynamic_limits) {
    auto prioritized_requests = raw_requests;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount;
         ++axle) {
        const std::size_t left = 2 * axle;
        const std::size_t right = left + 1;

        double common = 0.5 * (raw_requests[left] + raw_requests[right]);
        double differential =
            0.5 * (raw_requests[left] - raw_requests[right]);
        const double left_limit = SanitizeTorqueLimit(dynamic_limits[left]);
        const double right_limit =
            SanitizeTorqueLimit(dynamic_limits[right]);

        if (!(std::isinf(left_limit) && std::isinf(right_limit))) {
            const double common_limit = std::min(left_limit, right_limit);
            if (std::abs(common) > common_limit) {
                common = SignOrOne(common) * common_limit;
                differential = 0.0;
            } else {
                const double lower_bound =
                    std::max(-left_limit - common, common - right_limit);
                const double upper_bound =
                    std::min(left_limit - common, common + right_limit);
                differential = lower_bound <= upper_bound
                                   ? std::clamp(differential, lower_bound,
                                                upper_bound)
                                   : 0.0;
            }
        }

        prioritized_requests[left] = common + differential;
        prioritized_requests[right] = common - differential;
    }
    return prioritized_requests;
}

}  // namespace

IrwCrosslineControlEventSession::IrwCrosslineControlEventSession(
    const configuration::AssembledVehicleSystem& assembled,
    IrwCrosslineOperatingPointSchedule schedule,
    actuation::WheelDriveTorqueCommandConditioner conditioner)
    : assembled_(&assembled),
      observation_binding_(assembled),
      schedule_(std::move(schedule)),
      recurrence_(schedule_.MakeRecurrenceConfig()),
      conditioner_(std::move(conditioner)),
      sample_period_seconds_(recurrence_.config().sample_period_seconds) {
    if (!SameBits(sample_period_seconds_, kControlSamplePeriodSeconds) ||
        !SameBits(sample_period_seconds_,
                  conditioner_.config().sample_period_seconds)) {
        Reject("the recurrence and conditioner sample periods must be "
               "bitwise-equal 0.01-second values");
    }
}

double IrwCrosslineControlEventSession::next_periodic_event_time_seconds()
    const noexcept {
    return static_cast<double>(next_periodic_event_ordinal_) *
           sample_period_seconds_;
}

configuration::IrwFullStateControlMechanicalObservation
IrwCrosslineControlEventSession::ObserveMechanicalInput(
    system_assembly::SystemRuntimeContext& context) const {
    return observation_binding_.Observe(context);
}

IrwCrosslineControlEventAudit
IrwCrosslineControlEventSession::ComputeCandidate(
    IrwCrosslineControlEventKind kind, std::uint64_t ordinal,
    double event_time_seconds,
    system_assembly::SystemRuntimeContext& accepted_context) const {
    IrwCrosslineControlEventAudit audit;
    audit.kind = kind;
    audit.periodic_event_ordinal = ordinal;
    audit.event_time_seconds = event_time_seconds;
    audit.mechanical_observation = ObserveMechanicalInput(accepted_context);
    audit.operating_point = schedule_.EvaluateOperatingPoint(
        audit.mechanical_observation.axle_track_stations_meters);
    audit.controller_state_before = controller_state_;
    audit.conditioner_memory_before_newton_metres =
        conditioner_memory_newton_metres_;
    audit.controller_result = recurrence_.Step(
        audit.mechanical_observation.mechanical_input, audit.operating_point,
        controller_state_);

    audit.common_mode_probe_requests_newton_metres =
        MakeCommonModeProbeRequest(
            audit.controller_result.requested_wheel_torques_newton_metres);
    audit.common_mode_conditioning_probe = conditioner_.Step(
        audit.common_mode_probe_requests_newton_metres,
        audit.mechanical_observation.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        conditioner_memory_newton_metres_);
    audit.prioritized_wheel_torque_requests_newton_metres =
        ApplyLongitudinalCommonModePriority(
            audit.controller_result.requested_wheel_torques_newton_metres,
            audit.common_mode_conditioning_probe
                .wheel_dynamic_torque_limits_newton_metres);
    audit.conditioning_result = conditioner_.Step(
        audit.prioritized_wheel_torque_requests_newton_metres,
        audit.mechanical_observation.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        conditioner_memory_newton_metres_);
    return audit;
}

IrwCrosslineControlEventAudit
IrwCrosslineControlEventSession::ApplyInitializationUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ != Phase::kInitializationRequired) {
        Reject("the initialization event has already been committed");
    }
    if (!SameBits(accepted_context.time_seconds(), 0.0)) {
        Reject("the initialization event requires accepted time zero");
    }

    IrwCrosslineControlEventAudit audit = ComputeCandidate(
        IrwCrosslineControlEventKind::kInitialization, 0, 0.0,
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

IrwCrosslineControlEventAudit
IrwCrosslineControlEventSession::ApplyScheduledUpdate(
    IrwCrosslineControlEventKind kind,
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (kind == IrwCrosslineControlEventKind::kInitialization) {
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
    IrwCrosslineControlEventAudit audit = ComputeCandidate(
        kind, next_periodic_event_ordinal_, expected_time, accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    ++next_periodic_event_ordinal_;
    phase_ = kind == IrwCrosslineControlEventKind::kTerminal
                 ? Phase::kTerminalEventCommitted
                 : Phase::kSynchronizationRequired;
    return audit;
}

IrwCrosslineControlEventAudit
IrwCrosslineControlEventSession::ApplyPeriodicUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    return ApplyScheduledUpdate(IrwCrosslineControlEventKind::kPeriodic,
                                accepted_context);
}

IrwCrosslineControlEventAudit
IrwCrosslineControlEventSession::ApplyTerminalUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    return ApplyScheduledUpdate(IrwCrosslineControlEventKind::kTerminal,
                                accepted_context);
}

bool IrwCrosslineControlEventSession::synchronization_required() const
    noexcept {
    return phase_ == Phase::kSynchronizationRequired;
}

bool IrwCrosslineControlEventSession::terminal_event_committed() const
    noexcept {
    return phase_ == Phase::kTerminalEventCommitted;
}

void IrwCrosslineControlEventSession::ConfirmBackendSynchronized() {
    if (phase_ != Phase::kSynchronizationRequired) {
        Reject("there is no committed event awaiting backend "
               "synchronization");
    }
    phase_ = Phase::kReadyToAdvance;
}

void IrwCrosslineControlEventSession::RequireReadyToAdvance() const {
    if (phase_ != Phase::kReadyToAdvance) {
        Reject("the next vehicle advance is blocked until the required "
               "control event and backend synchronization succeed");
    }
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
