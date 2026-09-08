#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "orvd/track_irregularity/track_irregularity_generation.h"

namespace orvd::track_irregularity::internal {

struct TrackIrregularityChannelCoreResult {
    double discrete_harmonic_variance_meters_squared{0.0};
    TrackIrregularitySampleStatistics full_amplitude_statistics;
    TrackIrregularitySampleStatistics complete_gated_statistics;
};

struct GeneratedTrackIrregularityCore {
    std::vector<double> track_station_meters;
    std::vector<double> lateral_displacement_meters;
    std::vector<double> vertical_displacement_meters;
    TrackIrregularityChannelSeeds channel_seeds;
    std::size_t station_sample_count{0};
    double frequency_spacing_cycles_per_meter{0.0};
    TrackIrregularityChannelCoreResult lateral;
    TrackIrregularityChannelCoreResult vertical;
};

[[noreturn]] void Reject(std::string_view diagnostic_prefix,
                         std::string_view detail);

void RequireFinite(std::string_view diagnostic_prefix, const char* name,
                   double value);

void ValidateTrackIrregularityDirection(
    std::string_view diagnostic_prefix,
    TrackIrregularityDirection direction);

void ValidateTrackIrregularityGenerationSpecification(
    std::string_view diagnostic_prefix,
    const SpatialFrequencyGridSpec& frequency_grid,
    const TrackStationGridSpec& station_grid,
    const TrackIrregularityPlacementSpec& placement);

[[nodiscard]] double FrequencySpacingCyclesPerMeter(
    const SpatialFrequencyGridSpec& frequency_grid) noexcept;

[[nodiscard]] TrackIrregularityChannelSeeds
DeriveTrackIrregularityChannelSeeds(std::uint64_t realization_seed) noexcept;

[[nodiscard]] GeneratedTrackIrregularityCore GenerateTrackIrregularityCore(
    std::string_view diagnostic_prefix,
    const SpatialFrequencyGridSpec& frequency_grid,
    const TrackStationGridSpec& station_grid,
    const TrackIrregularityPlacementSpec& placement,
    std::uint64_t realization_seed,
    std::span<const double> lateral_one_sided_psd,
    std::span<const double> vertical_one_sided_psd);

}  // namespace orvd::track_irregularity::internal
