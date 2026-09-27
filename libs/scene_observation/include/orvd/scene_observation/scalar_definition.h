#pragma once

/// @file
/// Named scalar observations with their unit, frame, method and readiness.

#include <cstdint>
#include <string>
#include <vector>

namespace orvd::scene_observation {

/// Whether a scalar slot carries a measurement. A slot that is not ready or a
/// declared placeholder carries the value zero and must not be read as one.
enum class ScalarStatus : std::uint8_t {
    kNotReady = 0,
    kValid = 1,
    kPlaceholder = 2,
};

struct ScalarDefinition {
    std::string name;
    std::string unit;
    std::string reference_frame;
    std::string quantity;
    std::string method;
    std::string sample_semantics;
};

/// Values and statuses in definition order; both vectors have one entry per
/// definition.
struct ScalarValues {
    std::vector<double> values;
    std::vector<std::uint8_t> statuses;
};

}  // namespace orvd::scene_observation
