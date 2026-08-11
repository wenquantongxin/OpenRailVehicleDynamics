#pragma once

#include <filesystem>

#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::configuration {

// Loads one immutable IRW full-state wheel-speed guidance controller from the
// exact path supplied by the caller.
//
// The document carries only values consumed by the frozen 100 Hz calculation.
// It has one sample-period authority, one rolling-radius authority and four
// independent sign arrays. There is no format number, source-provenance
// payload, dormant switch, default insertion, path search or retained JSON.
//
// Throws std::runtime_error when the file cannot be opened or read. Throws
// std::invalid_argument for every other refusal, including repeated, missing
// or unknown keys, wrong types, malformed fixed-size arrays and numeric-domain
// errors reported by IrwFullStateWheelSpeedGuidanceController.
[[nodiscard]] control::IrwFullStateWheelSpeedGuidanceController
LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
