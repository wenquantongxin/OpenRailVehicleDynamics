#pragma once

/// @file
/// Atomic 100 Hz IRW longitudinal-cruise event binding.

#include <array>
#include <cstdint>

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/control/sampled_longitudinal_cruise_controller.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/system_instance.h"

namespace orvd::configuration {

enum class IrwLongitudinalCruiseEventKind {
    kInitialization,
    kPeriodic,
};

/// The two explicit wheel-rate conventions observed at one accepted event.
struct IrwLongitudinalCruiseWheelSpeedObservation final {
    actuation::WheelDriveTorqueChannelValues
        raw_joint_rates_radians_per_second{};
    actuation::WheelDriveTorqueChannelValues
        forward_wheel_circumferential_speeds_meters_per_second{};
    actuation::WheelDriveTorqueChannelValues
        conditioner_scalar_wheel_speeds_radians_per_second{};
    double common_forward_wheel_circumferential_speed_meters_per_second{};
};

/// Complete calculation and pre-commit memory for one accepted event.
struct IrwLongitudinalCruiseEventAudit final {
    IrwLongitudinalCruiseEventKind kind{};
    std::uint64_t periodic_event_ordinal{};
    double event_time_seconds{};
    IrwLongitudinalCruiseWheelSpeedObservation wheel_speed_observation;
    control::SampledLongitudinalCruiseControllerState controller_state_before;
    actuation::WheelDriveTorqueChannelValues
        conditioner_memory_before_newton_metres{};
    control::SampledLongitudinalCruiseControllerResult controller_result;
    actuation::WheelDriveTorqueChannelValues
        requested_wheel_torques_newton_metres{};
    actuation::WheelDriveTorqueConditioningResult conditioning_result;
};

/// Binds one scalar cruise PI to the closed eight-wheel IRW torque topology.
///
/// Raw revolute rates are converted to forward circumferential speeds with the
/// supplied nominal radius and explicit per-wheel signs, then averaged into
/// the controller's sole feedback value. The single requested torque is copied
/// identically to all eight conditioner inputs. The conditioner retains its
/// separate frozen scalar convention, which is the negative of the raw ORVD
/// revolute rate for every channel in this topology.
///
/// Initialization commits U0 once at accepted time zero. Every successful
/// event commits all eight held torques, the single PI state and conditioner
/// memory together, then blocks advancement until the numerical backend has
/// been constructed or synchronized from that accepted context.
class IrwLongitudinalCruiseEventSession final {
   public:
    IrwLongitudinalCruiseEventSession(
        const AssembledVehicleSystem& assembled,
        control::SampledLongitudinalCruiseController controller,
        double nominal_rolling_radius_meters,
        actuation::WheelDriveTorqueChannelValues forward_joint_rate_signs,
        actuation::WheelDriveTorqueCommandConditioner conditioner);

    IrwLongitudinalCruiseEventSession(
        const IrwLongitudinalCruiseEventSession&) = delete;
    IrwLongitudinalCruiseEventSession& operator=(
        const IrwLongitudinalCruiseEventSession&) = delete;
    IrwLongitudinalCruiseEventSession(
        IrwLongitudinalCruiseEventSession&&) = delete;
    IrwLongitudinalCruiseEventSession& operator=(
        IrwLongitudinalCruiseEventSession&&) = delete;

    [[nodiscard]] double sample_period_seconds() const noexcept {
        return sample_period_seconds_;
    }
    [[nodiscard]] double nominal_rolling_radius_meters() const noexcept {
        return nominal_rolling_radius_meters_;
    }
    [[nodiscard]] std::uint64_t next_periodic_event_ordinal() const noexcept {
        return next_periodic_event_ordinal_;
    }
    [[nodiscard]] double next_periodic_event_time_seconds() const noexcept;
    [[nodiscard]] const control::SampledLongitudinalCruiseControllerState&
    controller_state() const noexcept {
        return controller_state_;
    }
    [[nodiscard]] const actuation::WheelDriveTorqueChannelValues&
    conditioner_memory_newton_metres() const noexcept {
        return conditioner_memory_newton_metres_;
    }

    /// Reads wheel rates from one compatible context without changing the
    /// context or either controller. The assembled system must outlive this
    /// session; each context need only remain valid for its call and belong to
    /// that assembled system.
    [[nodiscard]] IrwLongitudinalCruiseWheelSpeedObservation ObserveWheelSpeeds(
        const system_assembly::SystemRuntimeContext& context) const;

    /// Computes and commits U0 at the accepted time-zero state.
    [[nodiscard]] IrwLongitudinalCruiseEventAudit ApplyInitializationUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    /// Computes and commits the next positive-time integer-grid event.
    [[nodiscard]] IrwLongitudinalCruiseEventAudit ApplyPeriodicUpdate(
        system_assembly::SystemRuntimeContext& accepted_context);

    [[nodiscard]] bool synchronization_required() const noexcept;
    void ConfirmBackendSynchronized();
    void RequireReadyToAdvance() const;

   private:
    struct ResolvedBinding final {
        std::array<multibody_model::GeneralizedVelocityRange,
                   actuation::kWheelDriveTorqueChannelCount>
            wheel_velocity_ranges;
    };

    static ResolvedBinding ResolveBinding(
        const AssembledVehicleSystem& assembled);

    IrwLongitudinalCruiseEventSession(
        const AssembledVehicleSystem& assembled,
        control::SampledLongitudinalCruiseController controller,
        double nominal_rolling_radius_meters,
        actuation::WheelDriveTorqueChannelValues forward_joint_rate_signs,
        actuation::WheelDriveTorqueCommandConditioner conditioner,
        ResolvedBinding binding);

    [[nodiscard]] IrwLongitudinalCruiseEventAudit ComputeCandidate(
        IrwLongitudinalCruiseEventKind kind, std::uint64_t ordinal,
        double event_time_seconds,
        const system_assembly::SystemRuntimeContext& accepted_context) const;

    enum class Phase {
        kInitializationRequired,
        kPeriodicEventRequired,
        kSynchronizationRequired,
        kReadyToAdvance,
    };

    const AssembledVehicleSystem* assembled_;
    control::SampledLongitudinalCruiseController controller_;
    actuation::WheelDriveTorqueCommandConditioner conditioner_;
    std::array<multibody_model::GeneralizedVelocityRange,
               actuation::kWheelDriveTorqueChannelCount>
        wheel_velocity_ranges_;
    actuation::WheelDriveTorqueChannelValues forward_joint_rate_signs_{};
    double nominal_rolling_radius_meters_{};
    double sample_period_seconds_{};
    control::SampledLongitudinalCruiseControllerState controller_state_;
    actuation::WheelDriveTorqueChannelValues
        conditioner_memory_newton_metres_{};
    std::uint64_t next_periodic_event_ordinal_{};
    Phase phase_{Phase::kInitializationRequired};
};

}  // namespace orvd::configuration
