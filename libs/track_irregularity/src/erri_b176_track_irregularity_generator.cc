#include "orvd/track_irregularity/erri_b176_track_irregularity_generator.h"

#include <cmath>
#include <numbers>
#include <string_view>
#include <utility>
#include <vector>

#include "track_irregularity_generation_internal.h"

namespace orvd::track_irregularity {
namespace {

constexpr std::string_view kDiagnosticPrefix =
    "ERRI B176 track-irregularity generator";
constexpr double kDenominatorA0 = 0.00028855;
constexpr double kDenominatorA2 = 0.6803895;
constexpr double kDenominatorA4 = 1.0;

void ValidateIrregularityLevel(
    ErriB176IrregularityLevel irregularity_level) {
    switch (irregularity_level) {
        case ErriB176IrregularityLevel::kLow:
        case ErriB176IrregularityLevel::kHigh:
            return;
    }
    internal::Reject(kDiagnosticPrefix,
                     "the ERRI B176 irregularity level is unsupported");
}

double NumeratorB0(ErriB176IrregularityLevel irregularity_level,
                   TrackIrregularityDirection direction) {
    ValidateIrregularityLevel(irregularity_level);
    internal::ValidateTrackIrregularityDirection(kDiagnosticPrefix,
                                                 direction);
    if (direction == TrackIrregularityDirection::kLateral) {
        return irregularity_level == ErriB176IrregularityLevel::kLow
                   ? 1.440846e-7
                   : 4.164787e-7;
    }
    return irregularity_level == ErriB176IrregularityLevel::kLow
               ? 2.741619e-7
               : 7.343623e-7;
}

double ContinuousBandVariance(
    const ErriB176DisplacementPsdParameters& spectrum,
    const SpatialFrequencyGridSpec& frequency_grid) {
    const double p = spectrum.denominator_a2 / spectrum.denominator_a4;
    const double q = spectrum.denominator_a0 / spectrum.denominator_a4;
    const double discriminant = p * p - 4.0 * q;
    const double larger_corner_squared =
        0.5 * (p + std::sqrt(discriminant));
    const double smaller_corner_squared = q / larger_corner_squared;
    const double omega_1 = std::sqrt(smaller_corner_squared);
    const double omega_2 = std::sqrt(larger_corner_squared);
    const double coefficient =
        spectrum.numerator_b0_si /
        (spectrum.denominator_a4 *
         (larger_corner_squared - smaller_corner_squared));
    const auto antiderivative = [omega_1, omega_2](double omega) {
        return std::atan(omega / omega_1) / omega_1 -
               std::atan(omega / omega_2) / omega_2;
    };
    const double minimum_angular_wavenumber =
        2.0 * std::numbers::pi * frequency_grid.minimum_cycles_per_meter;
    const double maximum_angular_wavenumber =
        2.0 * std::numbers::pi * frequency_grid.maximum_cycles_per_meter;
    return coefficient *
           (antiderivative(maximum_angular_wavenumber) -
            antiderivative(minimum_angular_wavenumber));
}

std::vector<double> SampleOneSidedSpatialPsd(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction,
    const SpatialFrequencyGridSpec& frequency_grid) {
    std::vector<double> sampled(frequency_grid.frequency_count);
    const double frequency_spacing =
        internal::FrequencySpacingCyclesPerMeter(frequency_grid);
    for (std::size_t frequency_index = 0;
         frequency_index < frequency_grid.frequency_count;
         ++frequency_index) {
        const double frequency =
            frequency_grid.minimum_cycles_per_meter +
            static_cast<double>(frequency_index) * frequency_spacing;
        sampled[frequency_index] = EvaluateErriB176OneSidedSpatialPsd(
            irregularity_level, direction, frequency);
    }
    return sampled;
}

}  // namespace

ErriB176DisplacementPsdParameters ErriB176DisplacementPsdParametersFor(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction) {
    return ErriB176DisplacementPsdParameters{
        NumeratorB0(irregularity_level, direction), kDenominatorA0,
        kDenominatorA2, kDenominatorA4};
}

double EvaluateErriB176AngularWavenumberPsd(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction,
    double angular_wavenumber_radians_per_meter) {
    if (!std::isfinite(angular_wavenumber_radians_per_meter) ||
        angular_wavenumber_radians_per_meter < 0.0) {
        internal::Reject(
            kDiagnosticPrefix,
            "PSD angular wavenumber must be finite and nonnegative");
    }
    const ErriB176DisplacementPsdParameters spectrum =
        ErriB176DisplacementPsdParametersFor(irregularity_level, direction);
    const double angular_wavenumber_squared =
        angular_wavenumber_radians_per_meter *
        angular_wavenumber_radians_per_meter;
    const double denominator =
        (spectrum.denominator_a4 * angular_wavenumber_squared +
         spectrum.denominator_a2) *
            angular_wavenumber_squared +
        spectrum.denominator_a0;
    return spectrum.numerator_b0_si / denominator;
}

double EvaluateErriB176OneSidedSpatialPsd(
    ErriB176IrregularityLevel irregularity_level,
    TrackIrregularityDirection direction,
    double frequency_cycles_per_meter) {
    if (!std::isfinite(frequency_cycles_per_meter) ||
        frequency_cycles_per_meter < 0.0) {
        internal::Reject(kDiagnosticPrefix,
                         "PSD frequency must be finite and nonnegative");
    }
    const double angular_wavenumber =
        2.0 * std::numbers::pi * frequency_cycles_per_meter;
    return 2.0 * std::numbers::pi *
           EvaluateErriB176AngularWavenumberPsd(
               irregularity_level, direction, angular_wavenumber);
}

TrackIrregularityChannelSeeds DeriveErriB176TrackIrregularityChannelSeeds(
    std::uint64_t realization_seed) noexcept {
    return internal::DeriveTrackIrregularityChannelSeeds(realization_seed);
}

GeneratedErriB176TrackIrregularity GenerateErriB176TrackIrregularity(
    const ErriB176TrackIrregularityGenerationSpec& specification) {
    ValidateIrregularityLevel(specification.irregularity_level);
    internal::ValidateTrackIrregularityGenerationSpecification(
        kDiagnosticPrefix, specification.frequency_grid,
        specification.station_grid, specification.placement);

    const std::vector<double> lateral_psd = SampleOneSidedSpatialPsd(
        specification.irregularity_level,
        TrackIrregularityDirection::kLateral,
        specification.frequency_grid);
    const std::vector<double> vertical_psd = SampleOneSidedSpatialPsd(
        specification.irregularity_level,
        TrackIrregularityDirection::kVertical,
        specification.frequency_grid);
    internal::GeneratedTrackIrregularityCore core =
        internal::GenerateTrackIrregularityCore(
            kDiagnosticPrefix, specification.frequency_grid,
            specification.station_grid, specification.placement,
            specification.realization_seed, lateral_psd, vertical_psd);

    const ErriB176DisplacementPsdParameters lateral_spectrum =
        ErriB176DisplacementPsdParametersFor(
            specification.irregularity_level,
            TrackIrregularityDirection::kLateral);
    const ErriB176DisplacementPsdParameters vertical_spectrum =
        ErriB176DisplacementPsdParametersFor(
            specification.irregularity_level,
            TrackIrregularityDirection::kVertical);

    GeneratedErriB176TrackIrregularity generated{
        std::move(core.track_station_meters),
        std::move(core.lateral_displacement_meters),
        std::move(core.vertical_displacement_meters),
        ErriB176TrackIrregularityGenerationMetadata{
            specification,
            core.station_sample_count,
            core.frequency_spacing_cycles_per_meter,
            specification.placement.start_meters,
            kErriB176RandomPhaseRealizationAlgorithm,
            kTrackIrregularityPhaseGeneratorAlgorithm,
            kTrackIrregularityChannelSeedDerivationAlgorithm,
            kTrackIrregularityFrequencyGridAlgorithm,
            kTrackPlacementWindowAlgorithm,
            kTrackIrregularityCoordinateFrame,
            kTrackIrregularityDisplacementUnit,
            kTrackIrregularityFrequencyVariable,
            kTrackIrregularityPsdSidedness,
            ErriB176TrackIrregularityChannelGenerationMetadata{
                TrackIrregularityDirection::kLateral,
                core.channel_seeds.lateral,
                lateral_spectrum,
                ContinuousBandVariance(lateral_spectrum,
                                       specification.frequency_grid),
                core.lateral.discrete_harmonic_variance_meters_squared,
                core.lateral.full_amplitude_statistics,
                core.lateral.complete_gated_statistics},
            ErriB176TrackIrregularityChannelGenerationMetadata{
                TrackIrregularityDirection::kVertical,
                core.channel_seeds.vertical,
                vertical_spectrum,
                ContinuousBandVariance(vertical_spectrum,
                                       specification.frequency_grid),
                core.vertical.discrete_harmonic_variance_meters_squared,
                core.vertical.full_amplitude_statistics,
                core.vertical.complete_gated_statistics}}};
    return generated;
}

}  // namespace orvd::track_irregularity
