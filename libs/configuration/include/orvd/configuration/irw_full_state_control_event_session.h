#pragma once

/// @file
/// Frozen IRW 100 Hz control observation binding and event transaction.

#include <array>
#include <cstdint>

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/multibody_model/multibody_coordinate_ranges.h"
#include "orvd/multibody_model/multibody_model_handles.h"

namespace orvd::configuration {

enum class IrwFullStateControlEventKind {
    kInitialization,
    kPeriodic,
};

/// Complete audit value produced by one frozen control recurrence.
///
/// An initialization audit advances both memories but does not publish its
/// torque to the vehicle. A periodic audit publishes the conditioner's actual
/// wheel torques as the eight context-local zero-order-held inputs.
struct IrwFullStateControlEventAudit final {
    IrwFullStateControlEventKind kind{
        IrwFullStateControlEventKind::kInitialization};
    std::uint64_t periodic_event_ordinal{};
    double event_time_seconds{};
    control::IrwFullStateWheelSpeedGuidanceControllerInput mechanical_input;
    control::IrwFullStateWheelSpeedGuidanceControllerState
        controller_state_before;
    actuation::WheelDriveTorqueChannelValues
        conditioner_memory_before_newton_metres{};
    control::IrwFullStateWheelSpeedGuidanceControllerResult controller_result;
    actuation::WheelDriveTorqueConditioningResult conditioning_result;
};

/// Caller-owned finite event session for the frozen P179 IRW personality.
///
/// Construction resolves the four axle-bridge bodies and eight independent-
/// wheel velocity ranges from the same closed table used by contact and active
/// torque assembly. Event evaluation thereafter reads only the accepted
/// context's four existing projection stations, body poses and generalized
/// velocities. It performs no name lookup, reprojection or contact solve.
///
/// The session owns the controller and conditioner memories. The accepted
/// `SystemRuntimeContext` remains the sole owner of the eight held torques. The
/// assembled system and every context passed to this object must outlive it.
class IrwFullStateControlEventSession final {
   public:
    IrwFullStateControlEventSession(
        const AssembledVehicleSystem& assembled,
        control::IrwFullStateWheelSpeedGuidanceController controller,
        actuation::WheelDriveTorqueCommandConditioner conditioner);

    IrwFullStateControlEventSession(
        const IrwFullStateControlEventSession&) = delete;
    IrwFullStateControlEventSession& operator=(
        const IrwFullStateControlEventSession&) = delete;
    IrwFullStateControlEventSession(IrwFullStateControlEventSession&&) =
        delete;
    IrwFullStateControlEventSession& operator=(
        IrwFullStateControlEventSession&&) = delete;

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

    /// Advances both memories once at the initial accepted mechanical state.
    /// Its output is audit-only and is not written to the held-torque vector.
    [[nodiscard]] IrwFullStateControlEventAudit ApplyInitializationUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    /// Computes and atomically commits the next periodic update.
    ///
    /// The accepted time must equal `ordinal * sample_period` for the next
    /// ordinal. All candidate controller and conditioner values are calculated
    /// before the eight held torques and the two memories are committed.
    [[nodiscard]] IrwFullStateControlEventAudit ApplyPeriodicUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    /// True after a periodic update changes the accepted held torque and until
    /// the caller confirms that its numerical backend was built or explicitly
    /// synchronized from that accepted context.
    [[nodiscard]] bool synchronization_required() const noexcept;

    /// Confirms successful backend construction or explicit synchronization.
    /// A failed synchronization must not call this method; retrying it therefore
    /// cannot repeat a controller recurrence.
    void ConfirmBackendSynchronized();

    /// Rejects an attempt to advance while initialization, an event retry or a
    /// synchronization is still required.
    void RequireReadyToAdvance() const;

   private:
    enum class Phase {
        kInitializationRequired,
        kPeriodicEventRequired,
        kSynchronizationRequired,
        kReadyToAdvance,
    };

    struct ResolvedBinding final {
        std::array<multibody_model::RigidBodyHandle,
                   control::kIrwGuidanceAxleCount>
            carrier_bodies;
        std::array<multibody_model::GeneralizedVelocityRange,
                   control::kIrwGuidanceWheelCount>
            wheel_velocity_ranges;
    };

    static ResolvedBinding ResolveBinding(
        const AssembledVehicleSystem& assembled);

    IrwFullStateControlEventSession(
        const AssembledVehicleSystem& assembled,
        control::IrwFullStateWheelSpeedGuidanceController controller,
        actuation::WheelDriveTorqueCommandConditioner conditioner,
        ResolvedBinding binding);

    [[nodiscard]] control::IrwFullStateWheelSpeedGuidanceControllerInput
    ObserveAcceptedMechanicalInput(
        system_assembly::SystemRuntimeContext& accepted_context) const;
    [[nodiscard]] IrwFullStateControlEventAudit ComputeCandidate(
        IrwFullStateControlEventKind kind, std::uint64_t ordinal,
        double event_time_seconds,
        system_assembly::SystemRuntimeContext& accepted_context) const;

    const AssembledVehicleSystem* assembled_;
    control::IrwFullStateWheelSpeedGuidanceController controller_;
    actuation::WheelDriveTorqueCommandConditioner conditioner_;
    std::array<multibody_model::RigidBodyHandle,
               control::kIrwGuidanceAxleCount>
        carrier_bodies_;
    std::array<multibody_model::GeneralizedVelocityRange,
               control::kIrwGuidanceWheelCount>
        wheel_velocity_ranges_;
    double sample_period_seconds_{};
    control::IrwFullStateWheelSpeedGuidanceControllerState controller_state_;
    actuation::WheelDriveTorqueChannelValues
        conditioner_memory_newton_metres_{};
    std::uint64_t next_periodic_event_ordinal_{};
    Phase phase_{Phase::kInitializationRequired};
};

}  // namespace orvd::configuration
