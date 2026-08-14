#pragma once

#include <filesystem>

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/control/sampled_longitudinal_cruise_controller.h"

namespace orvd::configuration {

/// Typed values carried by one IRW longitudinal-cruise controller asset.
///
/// The controller owns only the topology-independent scalar PI calculation.
/// Rolling radius and raw-joint-rate signs remain explicit configuration-layer
/// authorities for the eight-wheel IRW observation binding.
struct IrwLongitudinalCruiseControllerAsset final {
    control::SampledLongitudinalCruiseController controller;
    double nominal_rolling_radius_meters{0.0};
    actuation::WheelDriveTorqueChannelValues forward_joint_rate_signs{};
};

/// Loads one strict IRW longitudinal-cruise controller asset.
///
/// Throws std::runtime_error when the file cannot be opened or read. Throws
/// std::invalid_argument for repeated, missing or unknown keys, wrong types,
/// malformed signs and every numeric-domain error reported by the controller.
/// The JSON document and parser types are not retained.
[[nodiscard]] IrwLongitudinalCruiseControllerAsset
LoadIrwLongitudinalCruiseControllerFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
