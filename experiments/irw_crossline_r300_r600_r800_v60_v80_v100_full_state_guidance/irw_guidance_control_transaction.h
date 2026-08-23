#pragma once

/// @file
/// Backend-independent one-sample IRW guidance control transaction.

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

/// Complete pure calculation for one accepted control sample.
///
/// The caller owns both memories and decides when to commit them.  The same
/// transaction is used by the native ORVD event session and the SIMPACK
/// Realtime experiment, so their controller and torque-conditioning semantics
/// cannot drift apart.
struct IrwGuidanceControlTransactionResult final {
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

[[nodiscard]] IrwGuidanceControlTransactionResult
ComputeIrwGuidanceControlTransaction(
    const control::IrwFullStateWheelSpeedGuidanceRecurrence& recurrence,
    const actuation::WheelDriveTorqueCommandConditioner& conditioner,
    const control::IrwFullStateWheelSpeedGuidanceMechanicalInput&
        mechanical_input,
    const control::IrwFullStateWheelSpeedGuidanceOperatingPoint&
        operating_point,
    const control::IrwFullStateWheelSpeedGuidanceControllerState&
        controller_state,
    const actuation::WheelDriveTorqueChannelValues&
        conditioner_memory_newton_metres);

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
