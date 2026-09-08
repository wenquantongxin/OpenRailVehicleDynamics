#pragma once

/// @file
/// Reproducible finite station series from the one-sided ERRI B176
/// displacement spectra defined by the SIMPACK 2021x QCH implementation.

#include <cstdint>
#include <string_view>
#include <vector>

#include "orvd/track_irregularity/track_irregularity_generation.h"

namespace orvd::track_irregularity {

inline constexpr std::string_view kErriB176RandomPhaseRealizationAlgorithm =
    "orvd-erri-b176-random-phase";

/// QCH Low/High irregularity-amplitude level. `kHigh` denotes the larger PSD,
/// not a smoother, higher-quality track.
enum class ErriB176IrregularityLevel {
    kLow,
    kHigh,
};

/// Coefficients of one ERRI B176 horizontal or vertical displacement PSD.
///
/// `S_omega(omega) = b0 / (a0 + a2*omega^2 + a4*omega^4)`, with
/// `omega` in radians per meter and `S_omega` in m^2/(rad/m).
struct ErriB176DisplacementPsdParameters {
    double numerator_b0_si{0.0};
    double denominator_a0{0.0};
    double denominator_a2{0.0};
    double denominator_a4{0.0};
};

/// Complete caller-owned identity of one generated two-channel realization.
struct ErriB176TrackIrregularityGenerationSpec {
    ErriB176IrregularityLevel irregularity_level{
        ErriB176IrregularityLevel::kLow};
    SpatialFrequencyGridSpec frequency_grid;
    TrackStationGridSpec station_grid;
    TrackIrregularityPlacementSpec placement;
    std::uint64_t realization_seed{0};
};

struct ErriB176TrackIrregularityChannelGenerationMetadata {
    TrackIrregularityDirection direction{
        TrackIrregularityDirection::kLateral};
    std::uint64_t seed{0};
    ErriB176DisplacementPsdParameters spectrum;
    double continuous_band_variance_meters_squared{0.0};
    double discrete_harmonic_variance_meters_squared{0.0};
    TrackIrregularitySampleStatistics full_amplitude_statistics;
    TrackIrregularitySampleStatistics complete_gated_statistics;
};

struct ErriB176TrackIrregularityGenerationMetadata {
    ErriB176TrackIrregularityGenerationSpec specification;
    std::size_t station_sample_count{0};
    double frequency_spacing_cycles_per_meter{0.0};
    double phase_origin_track_station_meters{0.0};
    std::string_view realization_algorithm{
        kErriB176RandomPhaseRealizationAlgorithm};
    std::string_view phase_generator_algorithm{
        kTrackIrregularityPhaseGeneratorAlgorithm};
    std::string_view channel_seed_derivation_algorithm{
        kTrackIrregularityChannelSeedDerivationAlgorithm};
    std::string_view frequency_grid_algorithm{
        kTrackIrregularityFrequencyGridAlgorithm};
    std::string_view placement_window_algorithm{
        kTrackPlacementWindowAlgorithm};
    std::string_view coordinate_frame{kTrackIrregularityCoordinateFrame};
    std::string_view displacement_unit{kTrackIrregularityDisplacementUnit};
    std::string_view spatial_frequency_variable{
        kTrackIrregularityFrequencyVariable};
    std::string_view psd_sidedness{kTrackIrregularityPsdSidedness};
    ErriB176TrackIrregularityChannelGenerationMetadata lateral;
    ErriB176TrackIrregularityChannelGenerationMetadata vertical;
};

struct GeneratedErriB176TrackIrregularity {
    std::vector<double> track_station_meters;
    std::vector<double> lateral_displacement_meters;
    std::vector<double> vertical_displacement_meters;
    ErriB176TrackIrregularityGenerationMetadata metadata;
};

/// Returns the QCH-defined Low or High parameters for one displacement
/// direction. Crosslevel parameters are outside this two-channel interface.
[[nodiscard]] ErriB176DisplacementPsdParameters
ErriB176DisplacementPsdParametersFor(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction);

/// Evaluates the one-sided angular-wavenumber PSD in m^2/(rad/m).
/// Zero angular wavenumber is admitted and returns the finite ERRI limit.
[[nodiscard]] double EvaluateErriB176AngularWavenumberPsd(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction,
    double angular_wavenumber_radians_per_meter);

/// Evaluates `S_f(f) = 2*pi*S_omega(2*pi*f)` in m^2/(cycles/m).
/// Zero cyclic spatial frequency is admitted; generated grids remain positive.
[[nodiscard]] double EvaluateErriB176OneSidedSpatialPsd(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction,
    double frequency_cycles_per_meter);

[[nodiscard]] TrackIrregularityChannelSeeds
DeriveErriB176TrackIrregularityChannelSeeds(
    std::uint64_t realization_seed) noexcept;

[[nodiscard]] GeneratedErriB176TrackIrregularity
GenerateErriB176TrackIrregularity(
    const ErriB176TrackIrregularityGenerationSpec& specification);

}  // namespace orvd::track_irregularity
