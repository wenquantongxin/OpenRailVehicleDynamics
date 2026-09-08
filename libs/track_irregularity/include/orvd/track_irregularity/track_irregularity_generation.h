#pragma once

/// @file
/// Common types and placement functions for finite station-domain
/// track-irregularity realizations.

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace orvd::track_irregularity {

inline constexpr std::string_view kTrackIrregularityPhaseGeneratorAlgorithm =
    "mt19937_64-u53-phase";
inline constexpr std::string_view
    kTrackIrregularityChannelSeedDerivationAlgorithm =
        "splitmix64-domain-separated-channel-seeds";
inline constexpr std::string_view kTrackIrregularityFrequencyGridAlgorithm =
    "linear-inclusive-uniform-delta";
inline constexpr std::string_view kTrackPlacementWindowAlgorithm =
    "smoothstep5";
inline constexpr std::string_view kTrackIrregularityCoordinateFrame =
    "track_lateral_right_vertical_down";
inline constexpr std::string_view kTrackIrregularityDisplacementUnit =
    "meter";
inline constexpr std::string_view kTrackIrregularityFrequencyVariable =
    "cycles_per_meter";
inline constexpr std::string_view kTrackIrregularityPsdSidedness =
    "one_sided";

enum class TrackIrregularityDirection {
    kLateral,
    kVertical,
};

/// Inclusive, equidistant positive spatial-frequency grid.
struct SpatialFrequencyGridSpec {
    double minimum_cycles_per_meter{0.0};
    double maximum_cycles_per_meter{0.0};
    std::size_t frequency_count{0};
};

/// Uniform output grid. Both stated endpoints are samples.
struct TrackStationGridSpec {
    double start_meters{0.0};
    double end_meters{0.0};
    double spacing_meters{0.0};
};

/// Finite activation interval and its independently configurable end windows.
///
/// The four stations `start`, `start + fade_in`, `end - fade_out` and `end`
/// must be knots of the output station grid. Both fade lengths are positive;
/// they may meet at one full-amplitude knot but may not overlap.
struct TrackIrregularityPlacementSpec {
    double start_meters{0.0};
    double end_meters{0.0};
    double fade_in_length_meters{0.0};
    double fade_out_length_meters{0.0};
};

/// Domain-separated channel seeds deterministically derived from one
/// realization identity. The two values are different for every root seed.
struct TrackIrregularityChannelSeeds {
    std::uint64_t lateral{0};
    std::uint64_t vertical{0};
};

struct TrackIrregularitySampleStatistics {
    std::size_t sample_count{0};
    double mean_meters{0.0};
    double root_mean_square_meters{0.0};
    double absolute_peak_meters{0.0};
};

/// Fifth-order unit step, clamped to zero below 0 and one above 1.
[[nodiscard]] double Smoothstep5(double unit_interval) noexcept;

/// Evaluates the validated finite placement envelope at one track station.
[[nodiscard]] double TrackIrregularityPlacementWeight(
    const TrackIrregularityPlacementSpec& placement,
    double track_station_meters);

}  // namespace orvd::track_irregularity
