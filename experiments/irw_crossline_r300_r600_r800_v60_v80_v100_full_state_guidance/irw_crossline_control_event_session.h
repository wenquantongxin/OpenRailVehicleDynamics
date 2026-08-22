#pragma once

/// @file
/// Atomic 100 Hz control transaction for the IRW cross-line experiment.

#include <cstdint>

#include "irw_crossline_operating_point_schedule.h"
#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/irw_full_state_control_observation_binding.h"
#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

enum class IrwCrosslineControlEventKind {
    kInitialization,
    kPeriodic,
    kTerminal,
};

/// Complete calculation record for one accepted control event.
///
/// The common-mode probe is audit-only.  Its proposed conditioner memory is
/// never committed.  `prioritized_wheel_torque_requests_newton_metres` is the
/// sole request passed through the final conditioner step, whose output and
/// memory are committed together with the controller recurrence.
struct IrwCrosslineControlEventAudit final {
    IrwCrosslineControlEventKind kind{
        IrwCrosslineControlEventKind::kInitialization};
    std::uint64_t periodic_event_ordinal{};
    double event_time_seconds{};
    configuration::IrwFullStateControlMechanicalObservation
        mechanical_observation;
    control::IrwFullStateWheelSpeedGuidanceOperatingPoint operating_point;
    control::IrwFullStateWheelSpeedGuidanceControllerState
        controller_state_before;
    actuation::WheelDriveTorqueChannelValues
        conditioner_memory_before_newton_metres{};
    control::IrwFullStateWheelSpeedGuidanceControllerResult controller_result;
    actuation::WheelDriveTorqueChannelValues
        common_mode_probe_requests_newton_metres{};
    actuation::WheelDriveTorqueConditioningResult
        common_mode_conditioning_probe;
    actuation::WheelDriveTorqueChannelValues
        prioritized_wheel_torque_requests_newton_metres{};
    actuation::WheelDriveTorqueConditioningResult conditioning_result;
};

/// Owns the finite control memory of the cross-line experiment.
///
/// Route scheduling and longitudinal-common-mode allocation remain local to
/// this experiment.  The public controller recurrence, mechanical binding and
/// per-wheel conditioner retain no knowledge of track station schedules.
/// Initialization computes and publishes U0 at accepted time zero.  Positive-
/// time events lie on the exact `ordinal * 0.01 s` grid.  A terminal update
/// commits its audit, held torque and both memories but deliberately requires
/// no subsequent numerical-backend synchronization because no further state
/// advance is permitted.
class IrwCrosslineControlEventSession final {
   public:
    IrwCrosslineControlEventSession(
        const configuration::AssembledVehicleSystem& assembled,
        IrwCrosslineOperatingPointSchedule schedule,
        actuation::WheelDriveTorqueCommandConditioner conditioner);

    IrwCrosslineControlEventSession(
        const IrwCrosslineControlEventSession&) = delete;
    IrwCrosslineControlEventSession& operator=(
        const IrwCrosslineControlEventSession&) = delete;
    IrwCrosslineControlEventSession(IrwCrosslineControlEventSession&&) =
        delete;
    IrwCrosslineControlEventSession& operator=(
        IrwCrosslineControlEventSession&&) = delete;

    [[nodiscard]] double sample_period_seconds() const noexcept {
        return sample_period_seconds_;
    }
    [[nodiscard]] std::uint64_t next_periodic_event_ordinal() const noexcept {
        return next_periodic_event_ordinal_;
    }
    [[nodiscard]] double next_periodic_event_time_seconds() const noexcept;

    [[nodiscard]] const
    control::IrwFullStateWheelSpeedGuidanceControllerState& controller_state()
        const noexcept {
        return controller_state_;
    }
    [[nodiscard]] const actuation::WheelDriveTorqueChannelValues&
    conditioner_memory_newton_metres() const noexcept {
        return conditioner_memory_newton_metres_;
    }

    /// Reads the current four axle stations and mechanical feedback without
    /// changing the context or either event memory.
    [[nodiscard]] configuration::IrwFullStateControlMechanicalObservation
    ObserveMechanicalInput(
        system_assembly::SystemRuntimeContext& context) const;

    /// Computes and publishes U0 at the accepted time-zero state.
    [[nodiscard]] IrwCrosslineControlEventAudit ApplyInitializationUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    /// Computes and commits the next positive-time integer-grid event.
    [[nodiscard]] IrwCrosslineControlEventAudit ApplyPeriodicUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    /// Commits the final integer-grid event without requesting a backend
    /// restart.  No subsequent event or vehicle advance is valid.
    [[nodiscard]] IrwCrosslineControlEventAudit ApplyTerminalUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    [[nodiscard]] bool synchronization_required() const noexcept;
    [[nodiscard]] bool terminal_event_committed() const noexcept;
    void ConfirmBackendSynchronized();
    void RequireReadyToAdvance() const;

   private:
    enum class Phase {
        kInitializationRequired,
        kPeriodicEventRequired,
        kSynchronizationRequired,
        kReadyToAdvance,
        kTerminalEventCommitted,
    };

    [[nodiscard]] IrwCrosslineControlEventAudit ComputeCandidate(
        IrwCrosslineControlEventKind kind, std::uint64_t ordinal,
        double event_time_seconds,
        system_assembly::SystemRuntimeContext& accepted_context) const;

    [[nodiscard]] IrwCrosslineControlEventAudit ApplyScheduledUpdate(
        IrwCrosslineControlEventKind kind,
        system_assembly::SystemRuntimeContext& accepted_context);

    const configuration::AssembledVehicleSystem* assembled_;
    configuration::IrwFullStateControlObservationBinding observation_binding_;
    IrwCrosslineOperatingPointSchedule schedule_;
    control::IrwFullStateWheelSpeedGuidanceRecurrence recurrence_;
    actuation::WheelDriveTorqueCommandConditioner conditioner_;
    double sample_period_seconds_{};
    control::IrwFullStateWheelSpeedGuidanceControllerState controller_state_;
    actuation::WheelDriveTorqueChannelValues
        conditioner_memory_newton_metres_{};
    std::uint64_t next_periodic_event_ordinal_{};
    Phase phase_{Phase::kInitializationRequired};
};

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
