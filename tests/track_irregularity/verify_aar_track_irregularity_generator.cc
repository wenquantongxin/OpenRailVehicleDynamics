// The AAR spectrum, deterministic phase map, finite placement and an
// independent frequency-domain inversion of generated full-amplitude samples.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "orvd/track_irregularity/aar_track_irregularity_generator.h"
#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace {

using orvd::track_irregularity::AarSingleCutoffPsdParametersFor;
using orvd::track_irregularity::AarTrackClass;
using orvd::track_irregularity::AarTrackIrregularityGenerationSpec;
using orvd::track_irregularity::DeriveAarTrackIrregularityChannelSeeds;
using orvd::track_irregularity::GenerateAarTrackIrregularity;
using orvd::track_irregularity::GeneratedTrackIrregularity;
using orvd::track_irregularity::Smoothstep5;
using orvd::track_irregularity::SpatialFrequencyGridSpec;
using orvd::track_irregularity::TrackIrregularityDirection;
using orvd::track_irregularity::TrackIrregularityPlacementSpec;
using orvd::track_irregularity::TrackIrregularityPlacementWeight;
using orvd::track_irregularity::TrackStationGridSpec;
using orvd::wheel_rail_contact::TrackIrregularityField;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "AAR track-irregularity generator: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void RequireNear(double actual, double expected, double tolerance,
                 std::string_view message) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "AAR track-irregularity generator: %.*s: got %.17g, "
                     "expected %.17g\n",
                     static_cast<int>(message.size()), message.data(), actual,
                     expected);
        ++failures;
    }
}

void RequireRelativeNear(double actual, double expected,
                         double relative_tolerance,
                         std::string_view message) {
    const double scale = std::max(std::abs(expected),
                                  std::numeric_limits<double>::min());
    RequireNear(actual, expected, relative_tolerance * scale, message);
}

void ExpectInvalid(const std::function<void()>& operation,
                   std::string_view diagnostic_fragment) {
    try {
        operation();
    } catch (const std::invalid_argument& error) {
        if (std::string_view(error.what()).find(diagnostic_fragment) ==
            std::string_view::npos) {
            std::fprintf(stderr,
                         "AAR track-irregularity generator: invalid "
                         "diagnostic '%s' does not contain '%.*s'\n",
                         error.what(),
                         static_cast<int>(diagnostic_fragment.size()),
                         diagnostic_fragment.data());
            ++failures;
        }
        return;
    }
    Require(false, "an invalid specification was accepted");
}

AarTrackIrregularityGenerationSpec MakeAlignedFrequencyInversionSpec(
    AarTrackClass track_class, std::uint64_t realization_seed) {
    // The 100 m DFT window has 0.01 cycles/m bins. The nine generated
    // frequencies therefore occupy exact positive DFT bins 5 through 13.
    return AarTrackIrregularityGenerationSpec{
        track_class,
        SpatialFrequencyGridSpec{0.05, 0.13, 9},
        TrackStationGridSpec{0.0, 120.0, 0.25},
        TrackIrregularityPlacementSpec{0.0, 120.0, 10.0, 10.0},
        realization_seed};
}

double ExpectedNumeratorB0(AarTrackClass track_class,
                           TrackIrregularityDirection direction) {
    if (track_class == AarTrackClass::kAar5) {
        return direction == TrackIrregularityDirection::kLateral
                   ? 1.29501947625e-6
                   : 3.560453809375e-6;
    }
    return 5.76130711875e-7;
}

double IndependentlyEvaluatedAarPsd(
    AarTrackClass track_class, TrackIrregularityDirection direction,
    double frequency_cycles_per_meter) {
    constexpr double denominator_a2 = 0.67980025;
    const double angular_wavenumber =
        2.0 * std::numbers::pi * frequency_cycles_per_meter;
    const double angular_wavenumber_squared =
        angular_wavenumber * angular_wavenumber;
    const double angular_psd =
        ExpectedNumeratorB0(track_class, direction) /
        (angular_wavenumber_squared *
         (angular_wavenumber_squared + denominator_a2));
    return 2.0 * std::numbers::pi * angular_psd;
}

struct RecoveredHarmonic {
    double one_sided_psd{0.0};
    double phase_radians{0.0};
};

struct RecoveredWelchSpectrum {
    double frequency_spacing_cycles_per_meter{0.0};
    std::vector<double> one_sided_psd;
};

void TransformWelchSegment(std::vector<std::complex<double>>* samples) {
    const std::size_t sample_count = samples->size();
    for (std::size_t sample_index = 1, reversed_index = 0;
         sample_index < sample_count; ++sample_index) {
        std::size_t bit = sample_count >> 1U;
        while ((reversed_index & bit) != 0U) {
            reversed_index ^= bit;
            bit >>= 1U;
        }
        reversed_index ^= bit;
        if (sample_index < reversed_index) {
            std::swap((*samples)[sample_index], (*samples)[reversed_index]);
        }
    }

    for (std::size_t transform_length = 2;; transform_length <<= 1U) {
        const double angle =
            -2.0 * std::numbers::pi /
            static_cast<double>(transform_length);
        const std::complex<double> rotation(std::cos(angle), std::sin(angle));
        for (std::size_t first_sample = 0; first_sample < sample_count;
             first_sample += transform_length) {
            std::complex<double> phasor(1.0, 0.0);
            for (std::size_t offset = 0; offset < transform_length / 2;
                 ++offset) {
                const std::complex<double> even =
                    (*samples)[first_sample + offset];
                const std::complex<double> odd =
                    (*samples)[first_sample + offset + transform_length / 2] *
                    phasor;
                (*samples)[first_sample + offset] = even + odd;
                (*samples)[first_sample + offset + transform_length / 2] =
                    even - odd;
                phasor *= rotation;
            }
        }
        if (transform_length == sample_count) {
            break;
        }
    }
}

RecoveredWelchSpectrum RecoverWelchSpectrum(
    std::span<const double> displacement_meters,
    double sample_spacing_meters, std::size_t segment_sample_count,
    std::size_t segment_stride_sample_count) {
    Require(segment_sample_count >= 2 &&
                segment_stride_sample_count > 0 &&
                displacement_meters.size() >= segment_sample_count &&
                (segment_sample_count & (segment_sample_count - 1)) == 0,
            "the Welch recovery grid is invalid");
    const std::size_t segment_count =
        1 + (displacement_meters.size() - segment_sample_count) /
                segment_stride_sample_count;
    const std::size_t maximum_frequency_bin = segment_sample_count / 2;
    std::vector<double> window(segment_sample_count, 0.0);
    long double window_square_sum = 0.0L;
    for (std::size_t sample_index = 0;
         sample_index < segment_sample_count; ++sample_index) {
        const double value =
            0.5 -
            0.5 * std::cos(2.0 * std::numbers::pi *
                           static_cast<double>(sample_index) /
                           static_cast<double>(segment_sample_count - 1));
        window[sample_index] = value;
        window_square_sum +=
            static_cast<long double>(value) * static_cast<long double>(value);
    }

    RecoveredWelchSpectrum recovered{
        1.0 / (sample_spacing_meters *
               static_cast<double>(segment_sample_count)),
        std::vector<double>(maximum_frequency_bin + 1, 0.0)};
    for (std::size_t segment_index = 0; segment_index < segment_count;
         ++segment_index) {
        const std::size_t first_sample =
            segment_index * segment_stride_sample_count;
        long double mean = 0.0L;
        for (std::size_t sample_offset = 0;
             sample_offset < segment_sample_count; ++sample_offset) {
            mean += static_cast<long double>(
                displacement_meters[first_sample + sample_offset]);
        }
        mean /= static_cast<long double>(segment_sample_count);

        std::vector<std::complex<double>> transformed(segment_sample_count);
        for (std::size_t sample_offset = 0;
             sample_offset < segment_sample_count; ++sample_offset) {
            transformed[sample_offset] =
                (static_cast<double>(
                     displacement_meters[first_sample + sample_offset]) -
                 static_cast<double>(mean)) *
                window[sample_offset];
        }
        TransformWelchSegment(&transformed);
        for (std::size_t frequency_bin = 0;
             frequency_bin <= maximum_frequency_bin; ++frequency_bin) {
            long double one_sided_factor = 2.0L;
            if (frequency_bin == 0 ||
                (segment_sample_count % 2 == 0 &&
                 frequency_bin == segment_sample_count / 2)) {
                one_sided_factor = 1.0L;
            }
            const long double density =
                one_sided_factor *
                static_cast<long double>(
                    std::norm(transformed[frequency_bin])) *
                static_cast<long double>(sample_spacing_meters) /
                window_square_sum;
            recovered.one_sided_psd[frequency_bin] +=
                static_cast<double>(density);
        }
    }
    for (double& density : recovered.one_sided_psd) {
        density /= static_cast<double>(segment_count);
    }
    return recovered;
}

RecoveredHarmonic RecoverHarmonicByDirectProjection(
    const std::vector<double>& displacement_meters,
    const std::vector<double>& track_station_meters,
    std::size_t first_sample, std::size_t sample_count,
    double frequency_cycles_per_meter,
    double frequency_bin_width_cycles_per_meter) {
    long double cosine_projection = 0.0L;
    long double sine_projection = 0.0L;
    for (std::size_t sample_offset = 0; sample_offset < sample_count;
         ++sample_offset) {
        const std::size_t sample_index = first_sample + sample_offset;
        const double angle = 2.0 * std::numbers::pi *
                             frequency_cycles_per_meter *
                             track_station_meters[sample_index];
        cosine_projection +=
            static_cast<long double>(displacement_meters[sample_index]) *
            static_cast<long double>(std::cos(angle));
        sine_projection +=
            static_cast<long double>(displacement_meters[sample_index]) *
            static_cast<long double>(std::sin(angle));
    }
    const long double inverse_sample_count =
        1.0L / static_cast<long double>(sample_count);
    const long double amplitude_squared =
        4.0L * inverse_sample_count * inverse_sample_count *
        (cosine_projection * cosine_projection +
         sine_projection * sine_projection);
    double phase_radians = std::atan2(
        -static_cast<double>(sine_projection),
        static_cast<double>(cosine_projection));
    if (phase_radians < 0.0) {
        phase_radians += 2.0 * std::numbers::pi;
    }
    return RecoveredHarmonic{
        static_cast<double>(
            amplitude_squared /
            (2.0L * static_cast<long double>(
                        frequency_bin_width_cycles_per_meter))),
        phase_radians};
}

double IndependentlyMappedPhase(std::uint64_t engine_word) {
    const std::uint64_t significand = engine_word >> 11U;
    return 2.0 * std::numbers::pi *
           std::ldexp(static_cast<double>(significand), -53);
}

void CheckSpectrumParametersAndBandVariance() {
    struct ExpectedSpectrum {
        AarTrackClass track_class;
        TrackIrregularityDirection direction;
        double traditional_amplitude;
        double numerator_b0;
        double expected_rms_millimeters;
    };
    constexpr std::array<ExpectedSpectrum, 4> expected{{
        {AarTrackClass::kAar5, TrackIrregularityDirection::kLateral,
         0.0762, 1.29501947625e-6, 3.0624168},
        {AarTrackClass::kAar5, TrackIrregularityDirection::kVertical,
         0.2095, 3.560453809375e-6, 5.0778379},
        {AarTrackClass::kAar6, TrackIrregularityDirection::kLateral,
         0.0339, 5.76130711875e-7, 2.0426157},
        {AarTrackClass::kAar6, TrackIrregularityDirection::kVertical,
         0.0339, 5.76130711875e-7, 2.0426157},
    }};
    for (const auto& item : expected) {
        const auto parameters =
            AarSingleCutoffPsdParametersFor(item.track_class, item.direction);
        RequireNear(
            parameters
                .traditional_amplitude_square_centimeters_radians_per_meter,
            item.traditional_amplitude, 0.0,
            "the traditional AAR amplitude is wrong");
        RequireNear(parameters.scale_factor, 0.25, 0.0,
                    "the AAR spectrum scale factor is wrong");
        RequireNear(parameters.corner_angular_wavenumber_radians_per_meter,
                    0.8245, 0.0,
                    "the AAR corner angular wavenumber is wrong");
        RequireNear(parameters.denominator_a2, 0.67980025, 2.0e-16,
                    "the derived denominator coefficient is wrong");
        RequireNear(parameters.numerator_b0_si, item.numerator_b0, 2.0e-21,
                    "the derived numerator coefficient is wrong");

        const auto generated = GenerateAarTrackIrregularity(
            AarTrackIrregularityGenerationSpec{
                item.track_class,
                SpatialFrequencyGridSpec{0.024, 0.333, 32},
                TrackStationGridSpec{0.0, 100.0, 0.25},
                TrackIrregularityPlacementSpec{0.0, 100.0, 10.0, 10.0},
                17});
        const auto& channel =
            item.direction == TrackIrregularityDirection::kLateral
                ? generated.metadata.lateral
                : generated.metadata.vertical;
        RequireNear(
            1000.0 *
                std::sqrt(channel.continuous_band_variance_meters_squared),
            item.expected_rms_millimeters, 1.0e-7,
            "the analytic finite-band RMS is wrong");
    }

    ExpectInvalid(
        [] {
            static_cast<void>(AarSingleCutoffPsdParametersFor(
                static_cast<AarTrackClass>(99),
                TrackIrregularityDirection::kLateral));
        },
        "track class");
    ExpectInvalid(
        [] {
            static_cast<void>(orvd::track_irregularity::
                                  EvaluateAarOneSidedSpatialPsd(
                                      AarTrackClass::kAar5,
                                      TrackIrregularityDirection::kLateral,
                                      0.0));
        },
        "frequency");
}

void CheckSmoothPlacement() {
    RequireNear(Smoothstep5(0.0), 0.0, 0.0,
                "smoothstep5 is nonzero at its start");
    RequireNear(Smoothstep5(0.25), 0.103515625, 1.0e-16,
                "smoothstep5 quarter value is wrong");
    RequireNear(Smoothstep5(0.5), 0.5, 0.0,
                "smoothstep5 midpoint is wrong");
    RequireNear(Smoothstep5(0.75), 0.896484375, 1.0e-16,
                "smoothstep5 three-quarter value is wrong");
    RequireNear(Smoothstep5(1.0), 1.0, 0.0,
                "smoothstep5 is not full at its end");
    RequireNear(Smoothstep5(-1.0), 0.0, 0.0,
                "smoothstep5 did not clamp below its interval");
    RequireNear(Smoothstep5(2.0), 1.0, 0.0,
                "smoothstep5 did not clamp above its interval");

    // Independent five-point one-sided differences check the endpoint
    // derivatives asserted by the fifth-order transition definition.
    constexpr double derivative_step = 1.0e-3;
    std::array<double, 5> start_values{};
    std::array<double, 5> end_values{};
    for (std::size_t index = 0; index < start_values.size(); ++index) {
        start_values[index] =
            Smoothstep5(static_cast<double>(index) * derivative_step);
        end_values[index] =
            Smoothstep5(1.0 - static_cast<double>(index) * derivative_step);
    }
    const double start_first_derivative =
        (-25.0 * start_values[0] + 48.0 * start_values[1] -
         36.0 * start_values[2] + 16.0 * start_values[3] -
         3.0 * start_values[4]) /
        (12.0 * derivative_step);
    const double end_first_derivative =
        (25.0 * end_values[0] - 48.0 * end_values[1] +
         36.0 * end_values[2] - 16.0 * end_values[3] +
         3.0 * end_values[4]) /
        (12.0 * derivative_step);
    const double start_second_derivative =
        (35.0 * start_values[0] - 104.0 * start_values[1] +
         114.0 * start_values[2] - 56.0 * start_values[3] +
         11.0 * start_values[4]) /
        (12.0 * derivative_step * derivative_step);
    const double end_second_derivative =
        (35.0 * end_values[0] - 104.0 * end_values[1] +
         114.0 * end_values[2] - 56.0 * end_values[3] +
         11.0 * end_values[4]) /
        (12.0 * derivative_step * derivative_step);
    Require(std::abs(start_first_derivative) < 2.0e-10 &&
                std::abs(end_first_derivative) < 2.0e-10,
            "smoothstep5 does not close its first endpoint derivatives");
    Require(std::abs(start_second_derivative) < 1.0e-6 &&
                std::abs(end_second_derivative) < 1.0e-6,
            "smoothstep5 does not close its second endpoint derivatives");

    const TrackIrregularityPlacementSpec placement{2.0, 14.0, 4.0, 2.0};
    for (const auto& sample : std::array<std::pair<double, double>, 9>{
             std::pair{1.0, 0.0}, std::pair{2.0, 0.0},
             std::pair{3.0, 0.103515625}, std::pair{4.0, 0.5},
             std::pair{6.0, 1.0}, std::pair{12.0, 1.0},
             std::pair{13.0, 0.5}, std::pair{14.0, 0.0},
             std::pair{15.0, 0.0}}) {
        RequireNear(TrackIrregularityPlacementWeight(placement, sample.first),
                    sample.second, 2.0e-16,
                    "the finite placement window has the wrong shape");
    }
}

void CheckSeedDerivationAndReplay() {
    const auto first = DeriveAarTrackIrregularityChannelSeeds(1234);
    const auto repeated = DeriveAarTrackIrregularityChannelSeeds(1234);
    const auto other = DeriveAarTrackIrregularityChannelSeeds(1235);
    Require(first.lateral == repeated.lateral &&
                first.vertical == repeated.vertical,
            "one root seed did not reproduce its channel seeds");
    Require(first.lateral != first.vertical,
            "the channel seed domains are not separated");
    Require(first.lateral != other.lateral &&
                first.vertical != other.vertical,
            "a different root seed reused a channel seed");
    Require(first.lateral == 0x1e4978db0a45f847ULL &&
                first.vertical == 0xbb93a11c7ca8b4c6ULL,
            "the documented SplitMix64 channel-domain mapping drifted");

    const auto specification =
        MakeAlignedFrequencyInversionSpec(AarTrackClass::kAar5, 1234);
    const auto generated = GenerateAarTrackIrregularity(specification);
    const auto replayed = GenerateAarTrackIrregularity(specification);
    Require(generated.lateral_displacement_meters ==
                    replayed.lateral_displacement_meters &&
                generated.vertical_displacement_meters ==
                    replayed.vertical_displacement_meters,
            "one complete specification did not replay its realization");
    Require(generated.lateral_displacement_meters !=
                generated.vertical_displacement_meters,
            "independent channel seeds produced identical fields");

    const auto changed = GenerateAarTrackIrregularity(
        MakeAlignedFrequencyInversionSpec(AarTrackClass::kAar5, 1235));
    Require(generated.lateral_displacement_meters !=
                    changed.lateral_displacement_meters &&
                generated.vertical_displacement_meters !=
                    changed.vertical_displacement_meters,
            "a different realization seed reused a field");
    Require(generated.metadata.specification.realization_seed == 1234 &&
                generated.metadata.lateral.seed == first.lateral &&
                generated.metadata.vertical.seed == first.vertical,
            "metadata lost the root or derived channel seeds");
    Require(generated.metadata.channel_seed_derivation_algorithm ==
                    orvd::track_irregularity::
                        kAarChannelSeedDerivationAlgorithm &&
                generated.metadata.phase_generator_algorithm ==
                    orvd::track_irregularity::kAarPhaseGeneratorAlgorithm,
            "metadata lost a reproducibility algorithm identity");

    constexpr std::size_t first_plateau_sample = 40;
    constexpr std::size_t dft_sample_count = 400;
    const RecoveredHarmonic lateral_first_harmonic =
        RecoverHarmonicByDirectProjection(
            generated.lateral_displacement_meters,
            generated.track_station_meters, first_plateau_sample,
            dft_sample_count, 0.05, 0.01);
    const RecoveredHarmonic vertical_first_harmonic =
        RecoverHarmonicByDirectProjection(
            generated.vertical_displacement_meters,
            generated.track_station_meters, first_plateau_sample,
            dft_sample_count, 0.05, 0.01);
    const RecoveredHarmonic lateral_second_harmonic =
        RecoverHarmonicByDirectProjection(
            generated.lateral_displacement_meters,
            generated.track_station_meters, first_plateau_sample,
            dft_sample_count, 0.06, 0.01);
    const RecoveredHarmonic vertical_second_harmonic =
        RecoverHarmonicByDirectProjection(
            generated.vertical_displacement_meters,
            generated.track_station_meters, first_plateau_sample,
            dft_sample_count, 0.06, 0.01);
    // These are the first two published mt19937_64 words for each hard-coded
    // channel seed. Keeping the words literal prevents the phase oracle from
    // silently repeating the generator's engine construction and draw order.
    const double lateral_phase_error = std::remainder(
        lateral_first_harmonic.phase_radians -
            IndependentlyMappedPhase(0xfcb70feee67e12eeULL),
        2.0 * std::numbers::pi);
    const double vertical_phase_error = std::remainder(
        vertical_first_harmonic.phase_radians -
            IndependentlyMappedPhase(0x78a68186b9e63f88ULL),
        2.0 * std::numbers::pi);
    const double lateral_second_phase_error = std::remainder(
        lateral_second_harmonic.phase_radians -
            IndependentlyMappedPhase(0x062cfbb05f48eb4aULL),
        2.0 * std::numbers::pi);
    const double vertical_second_phase_error = std::remainder(
        vertical_second_harmonic.phase_radians -
            IndependentlyMappedPhase(0xdf429acbdad25f70ULL),
        2.0 * std::numbers::pi);
    Require(std::abs(lateral_phase_error) < 1.0e-8 &&
                std::abs(vertical_phase_error) < 1.0e-8 &&
                std::abs(lateral_second_phase_error) < 1.0e-8 &&
                std::abs(vertical_second_phase_error) < 1.0e-8,
            "direct projection did not recover the documented MT/U53 phases");
}

void CheckIndependentFrequencyDomainInversion() {
    for (const AarTrackClass track_class :
         {AarTrackClass::kAar5, AarTrackClass::kAar6}) {
        const auto generated = GenerateAarTrackIrregularity(
            MakeAlignedFrequencyInversionSpec(track_class, 987654321));
        constexpr std::size_t first_plateau_sample = 40;  // station 10 m.
        constexpr std::size_t dft_sample_count = 400;     // 100 m, end open.
        constexpr double frequency_bin_width = 0.01;
        RequireNear(generated.track_station_meters[first_plateau_sample],
                    10.0, 0.0,
                    "the PSD inversion window does not start on its plateau");
        RequireNear(generated.track_station_meters
                        [first_plateau_sample + dft_sample_count - 1],
                    109.75, 0.0,
                    "the PSD inversion window has the wrong length");

        for (const TrackIrregularityDirection direction :
             {TrackIrregularityDirection::kLateral,
              TrackIrregularityDirection::kVertical}) {
            const auto& displacement =
                direction == TrackIrregularityDirection::kLateral
                    ? generated.lateral_displacement_meters
                    : generated.vertical_displacement_meters;
            double expected_discrete_variance = 0.0;
            for (std::size_t frequency_index = 0; frequency_index < 9;
                 ++frequency_index) {
                const double frequency =
                    0.05 + static_cast<double>(frequency_index) *
                               frequency_bin_width;
                const double recovered_psd =
                    RecoverHarmonicByDirectProjection(
                        displacement, generated.track_station_meters,
                        first_plateau_sample, dft_sample_count, frequency,
                        frequency_bin_width)
                        .one_sided_psd;
                const double expected_psd = IndependentlyEvaluatedAarPsd(
                    track_class, direction, frequency);
                expected_discrete_variance +=
                    expected_psd * frequency_bin_width;
                RequireRelativeNear(
                    recovered_psd, expected_psd, 1.0e-8,
                    "direct DFT projection did not recover the target PSD");
            }
            const auto& metadata =
                direction == TrackIrregularityDirection::kLateral
                    ? generated.metadata.lateral
                    : generated.metadata.vertical;
            RequireRelativeNear(
                metadata.discrete_harmonic_variance_meters_squared,
                expected_discrete_variance, 1.0e-14,
                "metadata does not report the independently summed "
                "harmonic variance");
        }
    }
}

void CheckProductionBandStatisticalInversionAfterSpline() {
    constexpr double kGeneratedSampleSpacingMeters = 0.1;
    constexpr double kAuditSampleSpacingMeters = 0.05;
    constexpr double kAuditStartMeters = 100.025;
    constexpr double kAuditEndExclusiveMeters = 1000.025;
    constexpr std::size_t kAuditSampleCount = 18000;
    constexpr std::size_t kWelchSegmentSampleCount = 2048;
    constexpr std::size_t kWelchSegmentStrideSampleCount = 1024;
    constexpr std::size_t kFirstComparedFrequencyBin = 4;
    constexpr std::size_t kLastComparedFrequencyBin = 30;
    constexpr std::size_t kLastLowStopbandFrequencyBin = 1;
    constexpr std::size_t kFirstHighStopbandFrequencyBin = 37;

    for (const auto& qualification :
         std::array<std::pair<AarTrackClass, std::uint64_t>, 2>{
             std::pair{AarTrackClass::kAar5, 0x7f4a7c159e3779b9ULL},
             std::pair{AarTrackClass::kAar6, 0x94d049bb133111ebULL}}) {
        const auto generated = GenerateAarTrackIrregularity(
            AarTrackIrregularityGenerationSpec{
                qualification.first,
                SpatialFrequencyGridSpec{0.024, 0.333, 1920},
                TrackStationGridSpec{0.0, 1100.0,
                                     kGeneratedSampleSpacingMeters},
                TrackIrregularityPlacementSpec{50.0, 1050.0, 50.0, 50.0},
                qualification.second});
        constexpr std::size_t first_placement_sample = 500;
        constexpr std::size_t placement_sample_count = 10001;
        const auto station_span =
            std::span<const double>(generated.track_station_meters)
                .subspan(first_placement_sample, placement_sample_count);
        const TrackIrregularityField field(
            station_span,
            std::span<const double>(generated.lateral_displacement_meters)
                .subspan(first_placement_sample, placement_sample_count),
            station_span,
            std::span<const double>(generated.vertical_displacement_meters)
                .subspan(first_placement_sample, placement_sample_count));
        Require(
            std::abs(field.LateralSlopeMetersPerMeter(50.0)) <= 1.0e-8 &&
                std::abs(field.LateralSlopeMetersPerMeter(1050.0)) <= 1.0e-8 &&
                std::abs(field.VerticalSlopeMetersPerMeter(50.0)) <= 1.0e-8 &&
                std::abs(field.VerticalSlopeMetersPerMeter(1050.0)) <= 1.0e-8,
            "the production fade leaves an excessive spline endpoint slope");

        std::array<std::vector<double>, 2> spline_samples;
        for (auto& channel : spline_samples) {
            channel.resize(kAuditSampleCount);
        }
        for (std::size_t sample_index = 0; sample_index < kAuditSampleCount;
            ++sample_index) {
            const double station =
                kAuditStartMeters +
                static_cast<double>(sample_index) * kAuditSampleSpacingMeters;
            spline_samples[0][sample_index] =
                field.LateralDisplacementMeters(station);
            spline_samples[1][sample_index] =
                field.VerticalDisplacementMeters(station);
        }
        RequireNear(kAuditStartMeters +
                        static_cast<double>(kAuditSampleCount) *
                            kAuditSampleSpacingMeters,
                    kAuditEndExclusiveMeters, 0.0,
                    "the production-band audit window has the wrong length");

        for (std::size_t channel_index = 0;
             channel_index < spline_samples.size(); ++channel_index) {
            const TrackIrregularityDirection direction =
                channel_index == 0 ? TrackIrregularityDirection::kLateral
                                   : TrackIrregularityDirection::kVertical;
            const RecoveredWelchSpectrum recovered = RecoverWelchSpectrum(
                spline_samples[channel_index], kAuditSampleSpacingMeters,
                kWelchSegmentSampleCount,
                kWelchSegmentStrideSampleCount);
            RequireNear(recovered.frequency_spacing_cycles_per_meter,
                        0.009765625, 1.0e-15,
                        "the Welch frequency spacing is wrong");

            long double sample_mean = 0.0L;
            for (const double sample : spline_samples[channel_index]) {
                sample_mean += static_cast<long double>(sample);
            }
            sample_mean /= static_cast<long double>(kAuditSampleCount);
            long double sample_variance = 0.0L;
            for (const double sample : spline_samples[channel_index]) {
                const long double centered =
                    static_cast<long double>(sample) - sample_mean;
                sample_variance += centered * centered;
            }
            sample_variance /= static_cast<long double>(kAuditSampleCount);
            constexpr double production_frequency_spacing =
                (0.333 - 0.024) / 1919.0;
            long double expected_discrete_variance = 0.0L;
            for (std::size_t frequency_index = 0; frequency_index < 1920;
                 ++frequency_index) {
                const double frequency =
                    0.024 + static_cast<double>(frequency_index) *
                                production_frequency_spacing;
                expected_discrete_variance +=
                    static_cast<long double>(IndependentlyEvaluatedAarPsd(
                        qualification.first, direction, frequency)) *
                    production_frequency_spacing;
            }

            long double estimated_band_variance = 0.0L;
            long double expected_band_variance = 0.0L;
            long double recovered_total_variance = 0.0L;
            long double low_stopband_variance = 0.0L;
            long double high_stopband_variance = 0.0L;
            long double logarithmic_ratio_sum = 0.0L;
            long double logarithmic_ratio_square_sum = 0.0L;
            std::size_t compared_bin_count = 0;
            for (std::size_t frequency_bin = kFirstComparedFrequencyBin;
                 frequency_bin <= kLastComparedFrequencyBin;
                 ++frequency_bin) {
                const double frequency =
                    static_cast<double>(frequency_bin) *
                    recovered.frequency_spacing_cycles_per_meter;
                const double expected_psd = IndependentlyEvaluatedAarPsd(
                    qualification.first, direction, frequency);
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
            for (std::size_t frequency_bin = 0;
                 frequency_bin < recovered.one_sided_psd.size();
                 ++frequency_bin) {
                const long double bin_variance =
                    static_cast<long double>(
                        recovered.one_sided_psd[frequency_bin]) *
                    recovered.frequency_spacing_cycles_per_meter;
                recovered_total_variance += bin_variance;
                if (frequency_bin <= kLastLowStopbandFrequencyBin) {
                    low_stopband_variance += bin_variance;
                }
                if (frequency_bin >= kFirstHighStopbandFrequencyBin) {
                    high_stopband_variance += bin_variance;
                }
            }
            const double complete_variance_ratio = static_cast<double>(
                sample_variance / expected_discrete_variance);
            const double band_variance_ratio = static_cast<double>(
                estimated_band_variance / expected_band_variance);
            const double geometric_psd_ratio = std::exp(static_cast<double>(
                logarithmic_ratio_sum /
                static_cast<long double>(compared_bin_count)));
            const double logarithmic_root_mean_square =
                std::sqrt(static_cast<double>(
                    logarithmic_ratio_square_sum /
                    static_cast<long double>(compared_bin_count)));
            const double low_stopband_fraction = static_cast<double>(
                low_stopband_variance / recovered_total_variance);
            const double high_stopband_fraction = static_cast<double>(
                high_stopband_variance / recovered_total_variance);
            Require(complete_variance_ratio >= 0.70 &&
                        complete_variance_ratio <= 1.30,
                    "the complete spline realization has incompatible "
                    "variance");
            Require(band_variance_ratio >= 0.65 &&
                        band_variance_ratio <= 1.35,
                    "the spline realization has incompatible recovered band "
                    "variance");
            Require(geometric_psd_ratio >= 0.70 &&
                        geometric_psd_ratio <= 1.30 &&
                        logarithmic_root_mean_square <= 0.55,
                    "the spline realization does not recover the AAR PSD "
                    "shape within the statistical tolerance");
            Require(low_stopband_fraction <= 0.02,
                    "the spline realization has excessive energy below the "
                    "declared lower cutoff guard");
            Require(high_stopband_fraction <= 0.01,
                    "the spline realization has excessive energy above the "
                    "declared upper cutoff guard");
        }
    }
}

void CheckGridPlacementAndSplineClosure() {
    const AarTrackIrregularityGenerationSpec specification{
        AarTrackClass::kAar5,
        SpatialFrequencyGridSpec{0.05, 0.13, 9},
        TrackStationGridSpec{0.0, 12.0, 0.25},
        TrackIrregularityPlacementSpec{2.0, 10.0, 2.0, 2.0},
        42};
    const GeneratedTrackIrregularity generated =
        GenerateAarTrackIrregularity(specification);
    Require(generated.track_station_meters.size() == 49 &&
                generated.lateral_displacement_meters.size() == 49 &&
                generated.vertical_displacement_meters.size() == 49 &&
                generated.metadata.station_sample_count == 49,
            "the inclusive station grid has the wrong size");
    for (std::size_t index = 0; index <= 8; ++index) {
        Require(generated.lateral_displacement_meters[index] == 0.0 &&
                    generated.vertical_displacement_meters[index] == 0.0,
                "the generated field is nonzero before its placement");
    }
    for (std::size_t index = 40; index < 49; ++index) {
        Require(generated.lateral_displacement_meters[index] == 0.0 &&
                    generated.vertical_displacement_meters[index] == 0.0,
                "the generated field is nonzero after its placement");
    }

    // A natural spline is global. Consumers must therefore crop the generated
    // audit grid to the two placement boundary knots before constructing the
    // field; its wrapper then supplies strict zero-outside displacement/slope.
    constexpr std::size_t first_placement_index = 8;
    constexpr std::size_t placement_sample_count = 33;
    const auto station_span =
        std::span<const double>(generated.track_station_meters)
            .subspan(first_placement_index, placement_sample_count);
    const auto lateral_span =
        std::span<const double>(generated.lateral_displacement_meters)
            .subspan(first_placement_index, placement_sample_count);
    const auto vertical_span =
        std::span<const double>(generated.vertical_displacement_meters)
            .subspan(first_placement_index, placement_sample_count);
    const TrackIrregularityField field(station_span, lateral_span, station_span,
                                       vertical_span);
    for (const double outside_station : {1.999, 10.001}) {
        Require(field.LateralDisplacementMeters(outside_station) == 0.0 &&
                    field.VerticalDisplacementMeters(outside_station) == 0.0 &&
                    field.LateralSlopeMetersPerMeter(outside_station) == 0.0 &&
                    field.VerticalSlopeMetersPerMeter(outside_station) == 0.0,
                "the cropped spline field is not strictly zero outside its "
                "gate");
    }

    auto invalid = specification;
    invalid.frequency_grid.frequency_count = 1;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "frequency_count");
    invalid = specification;
    invalid.station_grid.spacing_meters = 0.7;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "divide");
    invalid = specification;
    invalid.station_grid.spacing_meters = 4.0;
    invalid.frequency_grid.maximum_cycles_per_meter = 0.125;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "Nyquist");
    invalid = specification;
    invalid.placement.fade_in_length_meters = 0.0;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "positive");
    invalid = specification;
    invalid.placement.fade_in_length_meters = 5.0;
    invalid.placement.fade_out_length_meters = 4.0;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "overlap");
    invalid = specification;
    invalid.placement.start_meters = 2.1;
    invalid.placement.fade_in_length_meters = 1.9;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "station-grid knot");
    invalid = specification;
    invalid.placement.end_meters = 12.25;
    ExpectInvalid([&] { static_cast<void>(GenerateAarTrackIrregularity(invalid)); },
                  "within");
}

}  // namespace

int main() {
    CheckSpectrumParametersAndBandVariance();
    CheckSmoothPlacement();
    CheckSeedDerivationAndReplay();
    CheckIndependentFrequencyDomainInversion();
    CheckProductionBandStatisticalInversionAfterSpline();
    CheckGridPlacementAndSplineClosure();
    return failures == 0 ? 0 : 1;
}
