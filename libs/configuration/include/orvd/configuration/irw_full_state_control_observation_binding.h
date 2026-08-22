#pragma once

/// @file
/// Stable binding from an assembled IRW vehicle to full-state control input.

#include <array>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/multibody_model/multibody_coordinate_ranges.h"
#include "orvd/multibody_model/multibody_model_handles.h"

namespace orvd::configuration {

/// One accepted-context observation, separated from any route schedule.
struct IrwFullStateControlMechanicalObservation final {
    control::IrwGuidanceAxleValues axle_track_stations_meters{};
    control::IrwFullStateWheelSpeedGuidanceMechanicalInput mechanical_input;
};

/// Resolves and observes the closed four-axle, eight-wheel IRW control ports.
///
/// Construction validates the contact, active-torque and multibody identities
/// once. `Observe()` thereafter reads only the context's
/// existing projection stations, body poses and generalized velocities; it
/// performs no name lookup, reprojection, contact solve or state mutation.
///
/// The assembled system must outlive this binding. A context only needs to
/// remain valid for the observation call and must belong to that system.
class IrwFullStateControlObservationBinding final {
   public:
    explicit IrwFullStateControlObservationBinding(
        const AssembledVehicleSystem& assembled);

    [[nodiscard]] IrwFullStateControlMechanicalObservation Observe(
        system_assembly::SystemRuntimeContext& context) const;

   private:
    struct ResolvedBinding final {
        std::array<multibody_model::RigidBodyHandle,
                   control::kIrwGuidanceAxleCount>
            carrier_bodies;
        std::array<multibody_model::GeneralizedVelocityRange,
                   control::kIrwGuidanceWheelCount>
            wheel_velocity_ranges;
    };

    [[nodiscard]] static ResolvedBinding ResolveBinding(
        const AssembledVehicleSystem& assembled);

    IrwFullStateControlObservationBinding(
        const AssembledVehicleSystem& assembled, ResolvedBinding binding);

    const AssembledVehicleSystem* assembled_;
    std::array<multibody_model::RigidBodyHandle,
               control::kIrwGuidanceAxleCount>
        carrier_bodies_;
    std::array<multibody_model::GeneralizedVelocityRange,
               control::kIrwGuidanceWheelCount>
        wheel_velocity_ranges_;
};

}  // namespace orvd::configuration
