#pragma once

#include <filesystem>

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"

namespace orvd::configuration {

// Loads one immutable wheel-drive torque-command conditioner from the exact
// path supplied by the caller.
//
// The document carries only the parameters consumed by the frozen command-
// conditioning calculation. There is no format number, source-provenance
// payload, dormant feature switch, default insertion, path search or retained
// JSON storage. Optional dynamic-limit entries are written as JSON null and
// become typed no-value entries in the product configuration.
//
// Throws std::runtime_error when the file cannot be opened or read. Throws
// std::invalid_argument for every other refusal, including repeated, missing
// or unknown keys, wrong types, malformed fixed-size tables and numeric-domain
// errors reported by WheelDriveTorqueCommandConditioner.
[[nodiscard]] actuation::WheelDriveTorqueCommandConditioner
LoadWheelDriveTorqueCommandConditionerFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
