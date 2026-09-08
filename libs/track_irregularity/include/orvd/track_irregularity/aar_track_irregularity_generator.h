#pragma once

/// @file
/// Reproducible finite station series from the one-sided AAR5/AAR6 spectra.

#include <cstdint>
#include <string_view>
#include <vector>

#include "orvd/track_irregularity/track_irregularity_generation.h"

namespace orvd::track_irregularity {

inline constexpr std::string_view kAarRandomPhaseRealizationAlgorithm =
    "orvd-aar-random-phase";
inline constexpr std::string_view kAarPhaseGeneratorAlgorithm =
    kTrackIrregularityPhaseGeneratorAlgorithm;
inline constexpr std::string_view kAarChannelSeedDerivationAlgorithm =
    kTrackIrregularityChannelSeedDerivationAlgorithm;
inline constexpr std::string_view kAarFrequencyGridAlgorithm =
    kTrackIrregularityFrequencyGridAlgorithm;

enum class AarTrackClass {
    kAar5,
    kAar6,
};

/// Complete caller-owned identity of one generated two-channel realization.
struct AarTrackIrregularityGenerationSpec {
    AarTrackClass track_class{AarTrackClass::kAar5};
    SpatialFrequencyGridSpec frequency_grid;
    TrackStationGridSpec station_grid;
    TrackIrregularityPlacementSpec placement;
    std::uint64_t realization_seed{0};
};

/// Parameters of one simplified FRA/AAR single-cutoff angular-wavenumber PSD.
struct AarSingleCutoffPsdParameters {
    // Traditional tabulation before the cm^2-to-m^2 conversion.
    double traditional_amplitude_square_centimeters_radians_per_meter{0.0};
    double scale_factor{0.0};
    double corner_angular_wavenumber_radians_per_meter{0.0};
    // S_omega(omega) = b0 / (a2*omega^2 + omega^4).
    double numerator_b0_si{0.0};
    double denominator_a2{0.0};
};

struct AarTrackIrregularityChannelGenerationMetadata {
    TrackIrregularityDirection direction{
        TrackIrregularityDirection::kLateral};
    std::uint64_t seed{0};
    AarSingleCutoffPsdParameters spectrum;
    double continuous_band_variance_meters_squared{0.0};
    double discrete_harmonic_variance_meters_squared{0.0};
    TrackIrregularitySampleStatistics full_amplitude_statistics;
    TrackIrregularitySampleStatistics complete_gated_statistics;
};

/// Sufficient numerical and algorithm identity to regenerate and audit output.
struct AarTrackIrregularityGenerationMetadata {
    AarTrackIrregularityGenerationSpec specification;
    std::size_t station_sample_count{0};
    double frequency_spacing_cycles_per_meter{0.0};
    double phase_origin_track_station_meters{0.0};
    std::string_view realization_algorithm{
        kAarRandomPhaseRealizationAlgorithm};
    std::string_view phase_generator_algorithm{
        kAarPhaseGeneratorAlgorithm};
    std::string_view channel_seed_derivation_algorithm{
        kAarChannelSeedDerivationAlgorithm};
    std::string_view frequency_grid_algorithm{kAarFrequencyGridAlgorithm};
    std::string_view placement_window_algorithm{
        kTrackPlacementWindowAlgorithm};
    std::string_view coordinate_frame{kTrackIrregularityCoordinateFrame};
    std::string_view displacement_unit{kTrackIrregularityDisplacementUnit};
    std::string_view spatial_frequency_variable{
        kTrackIrregularityFrequencyVariable};
    std::string_view psd_sidedness{kTrackIrregularityPsdSidedness};
    AarTrackIrregularityChannelGenerationMetadata lateral;
    AarTrackIrregularityChannelGenerationMetadata vertical;
};

struct GeneratedAarTrackIrregularity {
    std::vector<double> track_station_meters;
    std::vector<double> lateral_displacement_meters;
    std::vector<double> vertical_displacement_meters;
    AarTrackIrregularityGenerationMetadata metadata;
};

/// Returns the exact project parameters for one class and direction.
[[nodiscard]] AarSingleCutoffPsdParameters AarSingleCutoffPsdParametersFor(
    AarTrackClass track_class, TrackIrregularityDirection direction);

/// Applies the generator's fixed domain separation to a root realization seed.
[[nodiscard]] TrackIrregularityChannelSeeds
DeriveAarTrackIrregularityChannelSeeds(std::uint64_t realization_seed) noexcept;

/// Evaluates S_f(f) in m^2/(cycles/m), with f in cycles/m.
///
/// This is the one-sided cyclic-spatial-frequency form
/// `S_f(f) = 2*pi*S_omega(2*pi*f)`.
[[nodiscard]] double EvaluateAarOneSidedSpatialPsd(
    AarTrackClass track_class, TrackIrregularityDirection direction,
    double frequency_cycles_per_meter);

/// Generates both independent channels, applies placement, and reports all
/// audit metadata. The harmonic phase coordinate is local to placement.start.
[[nodiscard]] GeneratedAarTrackIrregularity GenerateAarTrackIrregularity(
    const AarTrackIrregularityGenerationSpec& specification);

}  // namespace orvd::track_irregularity
