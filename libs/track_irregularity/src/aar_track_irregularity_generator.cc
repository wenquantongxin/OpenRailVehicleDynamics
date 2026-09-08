#include "orvd/track_irregularity/aar_track_irregularity_generator.h"

#include <cmath>
#include <numbers>
#include <string_view>
#include <utility>
#include <vector>

#include "track_irregularity_generation_internal.h"

namespace orvd::track_irregularity {
namespace {

constexpr std::string_view kDiagnosticPrefix =
    "AAR track-irregularity generator";
constexpr double kSpectrumScaleFactor = 0.25;
constexpr double kCornerAngularWavenumberRadiansPerMeter = 0.8245;
constexpr double kSquareCentimetersToSquareMeters = 1.0e-4;

void ValidateTrackClass(AarTrackClass track_class) {
    switch (track_class) {
        case AarTrackClass::kAar5:
        case AarTrackClass::kAar6:
            return;
    }
    internal::Reject(kDiagnosticPrefix, "the AAR track class is unsupported");
}

double TraditionalAmplitude(
    AarTrackClass track_class, TrackIrregularityDirection direction) {
    ValidateTrackClass(track_class);
    internal::ValidateTrackIrregularityDirection(kDiagnosticPrefix,
                                                 direction);
    if (track_class == AarTrackClass::kAar5) {
        return direction == TrackIrregularityDirection::kLateral ? 0.0762
                                                                 : 0.2095;
    }
    return 0.0339;
}

double ContinuousBandVariance(
    const AarSingleCutoffPsdParameters& spectrum,
    const SpatialFrequencyGridSpec& frequency_grid) {
    const double corner =
        spectrum.corner_angular_wavenumber_radians_per_meter;
    const auto antiderivative = [corner](double angular_wavenumber) {
        return -1.0 / angular_wavenumber -
               std::atan(angular_wavenumber / corner) / corner;
    };
    const double minimum_angular_wavenumber =
        2.0 * std::numbers::pi * frequency_grid.minimum_cycles_per_meter;
    const double maximum_angular_wavenumber =
        2.0 * std::numbers::pi * frequency_grid.maximum_cycles_per_meter;
    const double scaled_traditional_amplitude =
        spectrum.scale_factor *
        spectrum.traditional_amplitude_square_centimeters_radians_per_meter *
        kSquareCentimetersToSquareMeters;
    return scaled_traditional_amplitude *
           (antiderivative(maximum_angular_wavenumber) -
            antiderivative(minimum_angular_wavenumber));
}

std::vector<double> SampleOneSidedSpatialPsd(
    AarTrackClass track_class, TrackIrregularityDirection direction,
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
        sampled[frequency_index] = EvaluateAarOneSidedSpatialPsd(
            track_class, direction, frequency);
    }
    return sampled;
}

}  // namespace

AarSingleCutoffPsdParameters AarSingleCutoffPsdParametersFor(
    AarTrackClass track_class, TrackIrregularityDirection direction) {
    const double traditional_amplitude =
        TraditionalAmplitude(track_class, direction);
    const double denominator_a2 =
        kCornerAngularWavenumberRadiansPerMeter *
        kCornerAngularWavenumberRadiansPerMeter;
    return AarSingleCutoffPsdParameters{
        traditional_amplitude, kSpectrumScaleFactor,
        kCornerAngularWavenumberRadiansPerMeter,
        kSpectrumScaleFactor * traditional_amplitude *
            kSquareCentimetersToSquareMeters * denominator_a2,
        denominator_a2};
}

TrackIrregularityChannelSeeds DeriveAarTrackIrregularityChannelSeeds(
    std::uint64_t realization_seed) noexcept {
    return internal::DeriveTrackIrregularityChannelSeeds(realization_seed);
}

double EvaluateAarOneSidedSpatialPsd(
    AarTrackClass track_class, TrackIrregularityDirection direction,
    double frequency_cycles_per_meter) {
    if (!std::isfinite(frequency_cycles_per_meter) ||
        !(frequency_cycles_per_meter > 0.0)) {
        internal::Reject(kDiagnosticPrefix,
                         "PSD frequency must be finite and positive");
    }
    const AarSingleCutoffPsdParameters spectrum =
        AarSingleCutoffPsdParametersFor(track_class, direction);
    const double angular_wavenumber =
        2.0 * std::numbers::pi * frequency_cycles_per_meter;
    const double angular_wavenumber_squared =
        angular_wavenumber * angular_wavenumber;
    const double angular_psd =
        spectrum.numerator_b0_si /
        (angular_wavenumber_squared *
         (angular_wavenumber_squared + spectrum.denominator_a2));
    return 2.0 * std::numbers::pi * angular_psd;
}

GeneratedAarTrackIrregularity GenerateAarTrackIrregularity(
    const AarTrackIrregularityGenerationSpec& specification) {
    ValidateTrackClass(specification.track_class);
    internal::ValidateTrackIrregularityGenerationSpecification(
        kDiagnosticPrefix, specification.frequency_grid,
        specification.station_grid, specification.placement);

    const std::vector<double> lateral_psd = SampleOneSidedSpatialPsd(
        specification.track_class, TrackIrregularityDirection::kLateral,
        specification.frequency_grid);
    const std::vector<double> vertical_psd = SampleOneSidedSpatialPsd(
        specification.track_class, TrackIrregularityDirection::kVertical,
        specification.frequency_grid);
    internal::GeneratedTrackIrregularityCore core =
        internal::GenerateTrackIrregularityCore(
            kDiagnosticPrefix, specification.frequency_grid,
            specification.station_grid, specification.placement,
            specification.realization_seed, lateral_psd, vertical_psd);

    const auto lateral_spectrum = AarSingleCutoffPsdParametersFor(
        specification.track_class, TrackIrregularityDirection::kLateral);
    const auto vertical_spectrum = AarSingleCutoffPsdParametersFor(
        specification.track_class, TrackIrregularityDirection::kVertical);

    GeneratedAarTrackIrregularity generated{
        std::move(core.track_station_meters),
        std::move(core.lateral_displacement_meters),
        std::move(core.vertical_displacement_meters),
        AarTrackIrregularityGenerationMetadata{
            specification,
            core.station_sample_count,
            core.frequency_spacing_cycles_per_meter,
            specification.placement.start_meters,
            kAarRandomPhaseRealizationAlgorithm,
            kAarPhaseGeneratorAlgorithm,
            kAarChannelSeedDerivationAlgorithm,
            kAarFrequencyGridAlgorithm,
            kTrackPlacementWindowAlgorithm,
            kTrackIrregularityCoordinateFrame,
            kTrackIrregularityDisplacementUnit,
            kTrackIrregularityFrequencyVariable,
            kTrackIrregularityPsdSidedness,
            AarTrackIrregularityChannelGenerationMetadata{
                TrackIrregularityDirection::kLateral,
                core.channel_seeds.lateral,
                lateral_spectrum,
                ContinuousBandVariance(lateral_spectrum,
                                       specification.frequency_grid),
                core.lateral.discrete_harmonic_variance_meters_squared,
                core.lateral.full_amplitude_statistics,
                core.lateral.complete_gated_statistics},
            AarTrackIrregularityChannelGenerationMetadata{
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
