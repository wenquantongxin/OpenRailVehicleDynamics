// Statistical recovery of the ERRI Low spectra used by the IRW V200 model.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <numbers>
#include <span>
#include <string_view>
#include <variant>
#include <vector>

#include "orvd/configuration/resolve_track_irregularity_field.h"
#include "orvd/track_irregularity/erri_b176_track_irregularity_generator.h"

namespace {

using orvd::configuration::FrozenTrackIrregularityFieldSource;
using orvd::configuration::GeneratedErriB176TrackIrregularityFieldSource;
using orvd::configuration::ResolveTrackIrregularityField;
using orvd::configuration::TrackIrregularityFieldSource;
using orvd::track_irregularity::ErriB176TrackIrregularityGenerationMetadata;
using orvd::track_irregularity::ErriB176TrackIrregularityGenerationSpec;
using orvd::track_irregularity::ErriB176IrregularityLevel;
using orvd::track_irregularity::TrackIrregularityDirection;
using orvd::wheel_rail_contact::TrackIrregularityField;

constexpr double kMinimumFrequencyCyclesPerMeter = 0.01;
constexpr double kMaximumFrequencyCyclesPerMeter = 0.333;
constexpr std::size_t kFrequencyCount = 1920;
constexpr double kStationSampleSpacingMeters = 0.1;
constexpr double kPlacementStartMeters = 50.0;
constexpr double kPlacementEndMeters = 500.0;
constexpr double kPlacementFadeMeters = 50.0;
constexpr std::uint64_t kOrvdRealizationSeed = 2026090704ULL;

constexpr double kAuditStartMeters = 100.0;
constexpr double kAuditEndExclusiveMeters = 450.0;
constexpr std::size_t kAuditSampleCount = 3500;
constexpr std::size_t kWelchSegmentSampleCount = 512;
constexpr std::size_t kWelchSegmentStrideSampleCount = 256;
constexpr std::size_t kFirstComparedFrequencyBin = 1;
constexpr std::size_t kLastComparedFrequencyBin = 17;
constexpr std::size_t kWelchSegmentCount =
    1 + (kAuditSampleCount - kWelchSegmentSampleCount) /
            kWelchSegmentStrideSampleCount;
static_assert(kWelchSegmentCount == 12);

constexpr double kDenominatorA0 = 0.00028855;
constexpr double kDenominatorA2 = 0.6803895;
constexpr double kDenominatorA4 = 1.0;
constexpr double kLateralLowNumeratorB0 = 1.440846e-7;
constexpr double kVerticalLowNumeratorB0 = 2.741619e-7;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "IRW V200 ERRI Low spectrum: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void RequireNear(double actual, double expected, double tolerance,
                 std::string_view message) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "IRW V200 ERRI Low spectrum: %.*s: got %.17g, "
                     "expected %.17g\n",
                     static_cast<int>(message.size()), message.data(), actual,
                     expected);
        ++failures;
    }
}

void RequireRange(double actual, double minimum, double maximum,
                  std::string_view source, std::string_view channel,
                  std::string_view metric) {
    if (!(actual >= minimum && actual <= maximum)) {
        std::fprintf(stderr,
                     "IRW V200 ERRI Low spectrum: %.*s %.*s %.*s %.17g is "
                     "outside [%.17g, %.17g]\n",
                     static_cast<int>(source.size()), source.data(),
                     static_cast<int>(channel.size()), channel.data(),
                     static_cast<int>(metric.size()), metric.data(), actual,
                     minimum, maximum);
        ++failures;
    }
}

void RequireAtMost(double actual, double maximum, std::string_view source,
                   std::string_view channel, std::string_view metric) {
    if (!(actual <= maximum)) {
        std::fprintf(stderr,
                     "IRW V200 ERRI Low spectrum: %.*s %.*s %.*s %.17g "
                     "exceeds %.17g\n",
                     static_cast<int>(source.size()), source.data(),
                     static_cast<int>(channel.size()), channel.data(),
                     static_cast<int>(metric.size()), metric.data(), actual,
                     maximum);
        ++failures;
    }
}

ErriB176TrackIrregularityGenerationSpec MakeIrwV200Specification() {
    // The frequency grid and placement mirror the V200 SIMPACK model. The
    // 0.1 m station grid is the shared ORVD/frozen-field observation grid, not
    // an intrinsic ERRI parameter.
    return ErriB176TrackIrregularityGenerationSpec{
        ErriB176IrregularityLevel::kLow,
        {kMinimumFrequencyCyclesPerMeter, kMaximumFrequencyCyclesPerMeter,
         kFrequencyCount},
        {0.0, kPlacementEndMeters, kStationSampleSpacingMeters},
        {kPlacementStartMeters, kPlacementEndMeters, kPlacementFadeMeters,
         kPlacementFadeMeters},
        kOrvdRealizationSeed};
}

double IndependentlyEvaluatedErriLowPsd(
    TrackIrregularityDirection direction,
    double frequency_cycles_per_meter) {
    const double numerator_b0 =
        direction == TrackIrregularityDirection::kLateral
            ? kLateralLowNumeratorB0
            : kVerticalLowNumeratorB0;
    const double angular_wavenumber =
        2.0 * std::numbers::pi * frequency_cycles_per_meter;
    const double squared = angular_wavenumber * angular_wavenumber;
    const double angular_psd =
        numerator_b0 /
        ((kDenominatorA4 * squared + kDenominatorA2) * squared +
         kDenominatorA0);
    return 2.0 * std::numbers::pi * angular_psd;
}

double ExpectedDiscreteVariance(TrackIrregularityDirection direction) {
    const double frequency_spacing =
        (kMaximumFrequencyCyclesPerMeter -
         kMinimumFrequencyCyclesPerMeter) /
        static_cast<double>(kFrequencyCount - 1);
    long double variance = 0.0L;
    for (std::size_t frequency_index = 0;
         frequency_index < kFrequencyCount; ++frequency_index) {
        const double frequency =
            kMinimumFrequencyCyclesPerMeter +
            static_cast<double>(frequency_index) * frequency_spacing;
        variance += static_cast<long double>(
                        IndependentlyEvaluatedErriLowPsd(direction,
                                                         frequency)) *
                    frequency_spacing;
    }
    return static_cast<double>(variance);
}

struct RecoveredWelchSpectrum {
    double frequency_spacing_cycles_per_meter{0.0};
    std::array<double, kLastComparedFrequencyBin + 1> one_sided_psd{};
};

RecoveredWelchSpectrum RecoverWelchSpectrum(
    std::span<const double> displacement_meters) {
    const std::size_t segment_count =
        1 + (displacement_meters.size() - kWelchSegmentSampleCount) /
                kWelchSegmentStrideSampleCount;
    Require(segment_count == kWelchSegmentCount,
            "the Welch recovery has the wrong segment count");
    std::vector<double> window(kWelchSegmentSampleCount, 0.0);
    long double window_square_sum = 0.0L;
    for (std::size_t sample_index = 0;
         sample_index < kWelchSegmentSampleCount; ++sample_index) {
        const double value =
            0.5 -
            0.5 * std::cos(2.0 * std::numbers::pi *
                           static_cast<double>(sample_index) /
                           static_cast<double>(kWelchSegmentSampleCount - 1));
        window[sample_index] = value;
        window_square_sum +=
            static_cast<long double>(value) * static_cast<long double>(value);
    }

    RecoveredWelchSpectrum recovered{
        1.0 / (kStationSampleSpacingMeters *
               static_cast<double>(kWelchSegmentSampleCount)),
        {}};
    for (std::size_t segment_index = 0; segment_index < segment_count;
         ++segment_index) {
        const std::size_t first_sample =
            segment_index * kWelchSegmentStrideSampleCount;
        long double mean = 0.0L;
        for (std::size_t sample_offset = 0;
             sample_offset < kWelchSegmentSampleCount; ++sample_offset) {
            mean += static_cast<long double>(
                displacement_meters[first_sample + sample_offset]);
        }
        mean /= static_cast<long double>(kWelchSegmentSampleCount);

        for (std::size_t frequency_bin = kFirstComparedFrequencyBin;
             frequency_bin <= kLastComparedFrequencyBin; ++frequency_bin) {
            long double real = 0.0L;
            long double imaginary = 0.0L;
            for (std::size_t sample_offset = 0;
                 sample_offset < kWelchSegmentSampleCount; ++sample_offset) {
                const long double windowed_sample =
                    (static_cast<long double>(displacement_meters[
                         first_sample + sample_offset]) -
                     mean) *
                    static_cast<long double>(window[sample_offset]);
                const double angle =
                    2.0 * std::numbers::pi *
                    static_cast<double>(frequency_bin * sample_offset) /
                    static_cast<double>(kWelchSegmentSampleCount);
                real += windowed_sample * std::cos(angle);
                imaginary -= windowed_sample * std::sin(angle);
            }
            const long double density =
                2.0L * (real * real + imaginary * imaginary) *
                kStationSampleSpacingMeters / window_square_sum;
            recovered.one_sided_psd[frequency_bin] +=
                static_cast<double>(density);
        }
    }
    for (double& density : recovered.one_sided_psd) {
        density /= static_cast<double>(segment_count);
    }
    return recovered;
}

double SampleVariance(std::span<const double> samples) {
    long double mean = 0.0L;
    for (const double sample : samples) {
        mean += static_cast<long double>(sample);
    }
    mean /= static_cast<long double>(samples.size());

    long double variance = 0.0L;
    for (const double sample : samples) {
        const long double centered = static_cast<long double>(sample) - mean;
        variance += centered * centered;
    }
    return static_cast<double>(variance /
                               static_cast<long double>(samples.size()));
}

std::array<std::vector<double>, 2> SampleFullAmplitudeField(
    const TrackIrregularityField& field) {
    std::array<std::vector<double>, 2> samples;
    for (auto& channel : samples) {
        channel.resize(kAuditSampleCount);
    }
    for (std::size_t sample_index = 0; sample_index < kAuditSampleCount;
         ++sample_index) {
        const double station =
            kAuditStartMeters +
            static_cast<double>(sample_index) * kStationSampleSpacingMeters;
        samples[0][sample_index] =
            field.LateralDisplacementMeters(station);
        samples[1][sample_index] =
            field.VerticalDisplacementMeters(station);
    }
    RequireNear(kAuditStartMeters +
                    static_cast<double>(kAuditSampleCount) *
                        kStationSampleSpacingMeters,
                kAuditEndExclusiveMeters, 0.0,
                "the full-amplitude audit interval has the wrong length");
    return samples;
}

void CheckRecoveredSpectrum(std::span<const double> samples,
                            TrackIrregularityDirection direction,
                            std::string_view source) {
    const std::string_view channel =
        direction == TrackIrregularityDirection::kLateral ? "lateral"
                                                          : "vertical";
    const double expected_discrete_variance =
        ExpectedDiscreteVariance(direction);
    const double complete_variance_ratio =
        SampleVariance(samples) / expected_discrete_variance;
    RequireRange(complete_variance_ratio, 0.70, 1.30, source, channel,
                 "sample-variance ratio");

    const RecoveredWelchSpectrum recovered =
        RecoverWelchSpectrum(samples);
    RequireNear(recovered.frequency_spacing_cycles_per_meter, 0.01953125,
                1.0e-15, "the Welch frequency spacing is wrong");

    long double estimated_band_variance = 0.0L;
    long double expected_band_variance = 0.0L;
    long double logarithmic_ratio_sum = 0.0L;
    long double logarithmic_ratio_square_sum = 0.0L;
    std::size_t compared_bin_count = 0;
    for (std::size_t frequency_bin = kFirstComparedFrequencyBin;
         frequency_bin <= kLastComparedFrequencyBin; ++frequency_bin) {
        const double frequency =
            static_cast<double>(frequency_bin) *
            recovered.frequency_spacing_cycles_per_meter;
        const double expected_psd =
            IndependentlyEvaluatedErriLowPsd(direction, frequency);
        const double estimated_psd =
            recovered.one_sided_psd[frequency_bin];
        estimated_band_variance +=
            static_cast<long double>(estimated_psd) *
            recovered.frequency_spacing_cycles_per_meter;
        expected_band_variance +=
            static_cast<long double>(expected_psd) *
            recovered.frequency_spacing_cycles_per_meter;
        const long double logarithmic_ratio =
            std::log(static_cast<long double>(estimated_psd) /
                     static_cast<long double>(expected_psd));
        logarithmic_ratio_sum += logarithmic_ratio;
        logarithmic_ratio_square_sum +=
            logarithmic_ratio * logarithmic_ratio;
        ++compared_bin_count;
    }

    const double band_variance_ratio = static_cast<double>(
        estimated_band_variance / expected_band_variance);
    const double geometric_psd_ratio = std::exp(static_cast<double>(
        logarithmic_ratio_sum /
        static_cast<long double>(compared_bin_count)));
    const double logarithmic_root_mean_square =
        std::sqrt(static_cast<double>(
            logarithmic_ratio_square_sum /
            static_cast<long double>(compared_bin_count)));
    RequireRange(band_variance_ratio, 0.65, 1.35, source, channel,
                 "Welch band-variance ratio");
    RequireRange(geometric_psd_ratio, 0.70, 1.30, source, channel,
                 "geometric PSD ratio");
    RequireAtMost(logarithmic_root_mean_square, 0.55, source, channel,
                  "logarithmic PSD-ratio RMS");
}

void CheckIrwV200ErriLowSpectrum(const std::filesystem::path& data_root) {
    const ErriB176TrackIrregularityGenerationSpec specification =
        MakeIrwV200Specification();
    const auto generated = ResolveTrackIrregularityField(
        {}, TrackIrregularityFieldSource{
                GeneratedErriB176TrackIrregularityFieldSource{
                    specification}});
    const auto frozen = ResolveTrackIrregularityField(
        data_root,
        TrackIrregularityFieldSource{FrozenTrackIrregularityFieldSource{
            "erri_low_irregularity"}});

    Require(generated.field != nullptr &&
                generated.generated_metadata.has_value(),
            "the generated V200 source returned no field or metadata");
    Require(frozen.field != nullptr &&
                !frozen.generated_metadata.has_value(),
            "the frozen SIMPACK field did not retain its asset identity");
    if (!generated.field || !generated.generated_metadata || !frozen.field) {
        return;
    }

    const auto* metadata =
        std::get_if<ErriB176TrackIrregularityGenerationMetadata>(
            &*generated.generated_metadata);
    Require(metadata != nullptr,
            "the generated V200 source returned the wrong metadata kind");
    if (!metadata) {
        return;
    }
    const auto& retained = metadata->specification;
    Require(retained.irregularity_level == ErriB176IrregularityLevel::kLow &&
                retained.frequency_grid.minimum_cycles_per_meter ==
                    kMinimumFrequencyCyclesPerMeter &&
                retained.frequency_grid.maximum_cycles_per_meter ==
                    kMaximumFrequencyCyclesPerMeter &&
                retained.frequency_grid.frequency_count == kFrequencyCount &&
                retained.station_grid.start_meters == 0.0 &&
                retained.station_grid.end_meters == kPlacementEndMeters &&
                retained.station_grid.spacing_meters ==
                    kStationSampleSpacingMeters &&
                retained.placement.start_meters == kPlacementStartMeters &&
                retained.placement.end_meters == kPlacementEndMeters &&
                retained.placement.fade_in_length_meters ==
                    kPlacementFadeMeters &&
                retained.placement.fade_out_length_meters ==
                    kPlacementFadeMeters &&
                retained.realization_seed == kOrvdRealizationSeed,
            "the generated source did not retain the V200 specification");
    Require(metadata->station_sample_count == 5001 &&
                metadata->phase_origin_track_station_meters ==
                    kPlacementStartMeters &&
                metadata->lateral.seed != metadata->vertical.seed,
            "the generated source lost its station, phase or seed identity");
    RequireNear(
        metadata->frequency_spacing_cycles_per_meter,
        (kMaximumFrequencyCyclesPerMeter -
         kMinimumFrequencyCyclesPerMeter) /
            static_cast<double>(kFrequencyCount - 1),
        1.0e-18, "the generated source retained the wrong frequency spacing");

    const auto generated_samples =
        SampleFullAmplitudeField(*generated.field);
    const auto frozen_samples = SampleFullAmplitudeField(*frozen.field);
    for (std::size_t channel_index = 0; channel_index < 2;
         ++channel_index) {
        const TrackIrregularityDirection direction =
            channel_index == 0 ? TrackIrregularityDirection::kLateral
                               : TrackIrregularityDirection::kVertical;
        CheckRecoveredSpectrum(generated_samples[channel_index], direction,
                               "generated ORVD realization");
        CheckRecoveredSpectrum(frozen_samples[channel_index], direction,
                               "frozen SIMPACK realization");
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <ORVD data root>\n", argv[0]);
        return 2;
    }
    CheckIrwV200ErriLowSpectrum(argv[1]);
    return failures == 0 ? 0 : 1;
}
