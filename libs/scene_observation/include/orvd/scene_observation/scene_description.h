#pragma once

/// @file
/// The renderer-independent description shared by scene recordings and live
/// scene observation: topology, scalar definitions and sampling cadence.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/scene_observation/scalar_definition.h"
#include "orvd/scene_observation/scene_topology.h"

namespace orvd::scene_observation {

inline constexpr char kSceneWorldFrame[] =
    "ORVD model world frame; a track scenario's world frame is the track "
    "inertial frame I: +x along increasing station at the line origin, "
    "+y to the right, +z downward";
inline constexpr char kSceneLengthUnit[] = "meter";
inline constexpr char kSceneTimeUnit[] = "second";
inline constexpr char kSceneAngleUnit[] = "radian";
inline constexpr char kSceneOrientation[] =
    "unit quaternion [w, x, y, z] mapping body coordinates to world "
    "coordinates; a wheel body's orientation includes its spin";

struct SceneDescription {
    std::string world_frame{kSceneWorldFrame};
    SceneTopology topology;
    std::vector<ScalarDefinition> scalar_definitions;
    /// Expected spacing of samples, when the producer has a nominal cadence.
    /// A supplied period is finite and strictly positive.
    std::optional<double> nominal_sample_period_seconds;
};

/// Encodes the common scene description as one JSON object. Names must be
/// unique within bodies, wheel interfaces and scalars; wheel and spin-scalar
/// references must resolve. A spin scalar has unit "rad" and sign +1 or -1.
/// Wheel placement geometry is finite, with a unit spin axis and positive
/// nominal rolling radius.
/// Invalid descriptions throw std::invalid_argument.
[[nodiscard]] std::string EncodeSceneDescription(
    const SceneDescription& description);

/// Decodes the same description and checks its required fields and semantic
/// references. Additional fields are ignored. Invalid input throws
/// std::invalid_argument.
[[nodiscard]] SceneDescription DecodeSceneDescription(std::string_view json);

}  // namespace orvd::scene_observation
