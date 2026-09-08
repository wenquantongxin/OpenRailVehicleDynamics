// ERRI B176 spectra, deterministic generation and low-frequency inversion.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>

#include "orvd/track_irregularity/erri_b176_track_irregularity_generator.h"

namespace {

using orvd::track_irregularity::DeriveErriB176TrackIrregularityChannelSeeds;
using orvd::track_irregularity::ErriB176DisplacementPsdParameters;
using orvd::track_irregularity::ErriB176DisplacementPsdParametersFor;
using orvd::track_irregularity::ErriB176TrackIrregularityGenerationSpec;
using orvd::track_irregularity::ErriB176IrregularityLevel;
using orvd::track_irregularity::EvaluateErriB176AngularWavenumberPsd;
using orvd::track_irregularity::EvaluateErriB176OneSidedSpatialPsd;
using orvd::track_irregularity::GenerateErriB176TrackIrregularity;
using orvd::track_irregularity::TrackIrregularityDirection;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "ERRI B176 track-irregularity generator: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void RequireNear(double actual, double expected, double tolerance,
                 std::string_view message) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "ERRI B176 track-irregularity generator: %.*s: got "
                     "%.17g, expected %.17g\n",
                     static_cast<int>(message.size()), message.data(), actual,
                     expected);
        ++failures;
    }
}

void RequireRelativeNear(double actual, double expected,
                         double relative_tolerance,
                         std::string_view message) {
    const double scale =
        std::max(std::abs(expected), std::numeric_limits<double>::min());
    RequireNear(actual, expected, relative_tolerance * scale, message);
}

void ExpectInvalid(const std::function<void()>& operation,
                   std::string_view diagnostic_fragment) {
    try {
        operation();
    } catch (const std::invalid_argument& error) {
        Require(std::string_view(error.what()).find(diagnostic_fragment) !=
                    std::string_view::npos,
                "an invalid input returned the wrong diagnostic");
        return;
    }
    Require(false, "an invalid input was accepted");
}

double IndependentAngularPsd(
    const ErriB176DisplacementPsdParameters& parameters,
    double angular_wavenumber) {
    const double squared = angular_wavenumber * angular_wavenumber;
    return parameters.numerator_b0_si /
           (parameters.denominator_a0 +
            parameters.denominator_a2 * squared +
            parameters.denominator_a4 * squared * squared);
}

double IntegrateCyclicPsd(ErriB176IrregularityLevel irregularity_level,
                          TrackIrregularityDirection direction,
                          double minimum_frequency,
                          double maximum_frequency) {
    constexpr std::size_t kIntervalCount = 20000;
    static_assert(kIntervalCount % 2 == 0);
    const double spacing =
        (maximum_frequency - minimum_frequency) /
        static_cast<double>(kIntervalCount);
    long double weighted_sum =
        EvaluateErriB176OneSidedSpatialPsd(
            irregularity_level, direction, minimum_frequency) +
        EvaluateErriB176OneSidedSpatialPsd(
            irregularity_level, direction, maximum_frequency);
    for (std::size_t index = 1; index < kIntervalCount; ++index) {
        const double frequency =
            minimum_frequency + static_cast<double>(index) * spacing;
        const int weight = index % 2 == 0 ? 2 : 4;
        weighted_sum +=
            static_cast<long double>(weight) *
            EvaluateErriB176OneSidedSpatialPsd(irregularity_level, direction,
                                               frequency);
    }
    return static_cast<double>(weighted_sum * spacing / 3.0L);
}

ErriB176TrackIrregularityGenerationSpec MakeGenerationSpec(
    ErriB176IrregularityLevel irregularity_level,
    std::uint64_t realization_seed) {
    return ErriB176TrackIrregularityGenerationSpec{
        irregularity_level,
        {0.01, 0.03, 9},
        {0.0, 420.0, 0.25},
        {0.0, 420.0, 10.0, 10.0},
        realization_seed};
}

void CheckQchParametersAndPsd() {
    struct ParameterFixture {
        ErriB176IrregularityLevel irregularity_level;
        TrackIrregularityDirection direction;
        double numerator_b0;
    };
    constexpr std::array<ParameterFixture, 4> fixtures{{
        {ErriB176IrregularityLevel::kLow,
         TrackIrregularityDirection::kLateral, 1.440846e-7},
        {ErriB176IrregularityLevel::kHigh,
         TrackIrregularityDirection::kLateral, 4.164787e-7},
        {ErriB176IrregularityLevel::kLow,
         TrackIrregularityDirection::kVertical, 2.741619e-7},
        {ErriB176IrregularityLevel::kHigh,
         TrackIrregularityDirection::kVertical, 7.343623e-7},
    }};

    for (const ParameterFixture& fixture : fixtures) {
        const auto parameters = ErriB176DisplacementPsdParametersFor(
            fixture.irregularity_level, fixture.direction);
        Require(parameters.numerator_b0_si == fixture.numerator_b0 &&
                    parameters.denominator_a0 == 0.00028855 &&
                    parameters.denominator_a2 == 0.6803895 &&
                    parameters.denominator_a4 == 1.0,
                "a QCH coefficient set was not retained exactly");

        const double zero_limit = fixture.numerator_b0 / 0.00028855;
        RequireRelativeNear(EvaluateErriB176AngularWavenumberPsd(
                                fixture.irregularity_level, fixture.direction,
                                0.0),
                            zero_limit, 2.0e-16,
                            "the zero-wavenumber limit is wrong");
        for (const double angular_wavenumber : {0.02, 0.2, 2.0}) {
            RequireRelativeNear(
                EvaluateErriB176AngularWavenumberPsd(
                    fixture.irregularity_level, fixture.direction,
                    angular_wavenumber),
                IndependentAngularPsd(parameters, angular_wavenumber),
                3.0e-16, "the angular-wavenumber PSD is wrong");
        }

        constexpr double kFrequency = 0.125;
        const double angular_wavenumber =
            2.0 * std::numbers::pi * kFrequency;
        RequireRelativeNear(
            EvaluateErriB176OneSidedSpatialPsd(
                fixture.irregularity_level, fixture.direction, kFrequency),
            2.0 * std::numbers::pi *
                IndependentAngularPsd(parameters, angular_wavenumber),
            3.0e-16, "the cyclic-frequency PSD conversion is wrong");
    }

    ExpectInvalid(
        [] {
            static_cast<void>(EvaluateErriB176AngularWavenumberPsd(
                ErriB176IrregularityLevel::kLow,
                TrackIrregularityDirection::kLateral, -0.01));
        },
        "nonnegative");
    ExpectInvalid(
        [] {
            static_cast<void>(EvaluateErriB176OneSidedSpatialPsd(
                ErriB176IrregularityLevel::kLow,
                TrackIrregularityDirection::kLateral,
                std::numeric_limits<double>::infinity()));
        },
        "finite");
}

void CheckContinuousVarianceAndMetadata() {
    constexpr std::uint64_t kSeed = 2026090701ULL;
    for (const ErriB176IrregularityLevel irregularity_level :
         {ErriB176IrregularityLevel::kLow, ErriB176IrregularityLevel::kHigh}) {
        const auto generated = GenerateErriB176TrackIrregularity(
            MakeGenerationSpec(irregularity_level, kSeed));
        const auto expected_seeds =
            DeriveErriB176TrackIrregularityChannelSeeds(kSeed);
        Require(generated.metadata.specification.irregularity_level ==
                        irregularity_level &&
                    generated.metadata.lateral.seed == expected_seeds.lateral &&
                    generated.metadata.vertical.seed ==
                        expected_seeds.vertical &&
                    generated.metadata.lateral.seed !=
                        generated.metadata.vertical.seed &&
                    generated.metadata.phase_origin_track_station_meters ==
                        0.0,
                "generation metadata lost spectrum or realization identity");

        const auto& grid = generated.metadata.specification.frequency_grid;
        for (const TrackIrregularityDirection direction :
             {TrackIrregularityDirection::kLateral,
              TrackIrregularityDirection::kVertical}) {
            const double actual =
                direction == TrackIrregularityDirection::kLateral
                    ? generated.metadata.lateral
                          .continuous_band_variance_meters_squared
                    : generated.metadata.vertical
                          .continuous_band_variance_meters_squared;
            const double expected = IntegrateCyclicPsd(
                irregularity_level, direction, grid.minimum_cycles_per_meter,
                grid.maximum_cycles_per_meter);
            RequireRelativeNear(actual, expected, 2.0e-12,
                                "the analytic band variance is wrong");
        }
    }
}

void CheckLowHighScalingAndReplay() {
    constexpr std::uint64_t kSeed = 2026090702ULL;
    const auto low = GenerateErriB176TrackIrregularity(
        MakeGenerationSpec(ErriB176IrregularityLevel::kLow, kSeed));
    const auto high = GenerateErriB176TrackIrregularity(
        MakeGenerationSpec(ErriB176IrregularityLevel::kHigh, kSeed));
    const auto replayed = GenerateErriB176TrackIrregularity(
        MakeGenerationSpec(ErriB176IrregularityLevel::kLow, kSeed));
    const auto changed = GenerateErriB176TrackIrregularity(
        MakeGenerationSpec(ErriB176IrregularityLevel::kLow, kSeed + 1));

    const double lateral_scale = std::sqrt(4.164787e-7 / 1.440846e-7);
    const double vertical_scale = std::sqrt(7.343623e-7 / 2.741619e-7);
    bool different_seed_changed_a_sample = false;
    for (std::size_t index = 0; index < low.track_station_meters.size();
         ++index) {
        Require(low.track_station_meters[index] ==
                        replayed.track_station_meters[index] &&
                    low.lateral_displacement_meters[index] ==
                        replayed.lateral_displacement_meters[index] &&
                    low.vertical_displacement_meters[index] ==
                        replayed.vertical_displacement_meters[index],
                "the same ERRI specification did not replay exactly");
        RequireRelativeNear(high.lateral_displacement_meters[index],
                            lateral_scale *
                                low.lateral_displacement_meters[index],
                            2.0e-11,
                            "horizontal High did not scale from Low");
        RequireRelativeNear(high.vertical_displacement_meters[index],
                            vertical_scale *
                                low.vertical_displacement_meters[index],
                            2.0e-11,
                            "vertical High did not scale from Low");
        different_seed_changed_a_sample =
            different_seed_changed_a_sample ||
            low.lateral_displacement_meters[index] !=
                changed.lateral_displacement_meters[index] ||
            low.vertical_displacement_meters[index] !=
                changed.vertical_displacement_meters[index];
    }
    Require(different_seed_changed_a_sample,
            "a different root seed repeated both ERRI channels");
}

void CheckLowFrequencyDftInversion() {
    const auto generated = GenerateErriB176TrackIrregularity(
        MakeGenerationSpec(ErriB176IrregularityLevel::kLow, 2026090703ULL));
    constexpr double kSampleSpacing = 0.25;
    constexpr double kWindowStart = 10.0;
    constexpr double kWindowLength = 400.0;
    constexpr std::size_t kSampleCount =
        static_cast<std::size_t>(kWindowLength / kSampleSpacing);
    constexpr std::size_t kFirstSample =
        static_cast<std::size_t>(kWindowStart / kSampleSpacing);
    const double frequency_spacing =
        generated.metadata.frequency_spacing_cycles_per_meter;

    for (std::size_t frequency_index = 0; frequency_index < 9;
         ++frequency_index) {
        const double frequency =
            0.01 + static_cast<double>(frequency_index) * frequency_spacing;
        for (const TrackIrregularityDirection direction :
             {TrackIrregularityDirection::kLateral,
              TrackIrregularityDirection::kVertical}) {
            const auto& samples =
                direction == TrackIrregularityDirection::kLateral
                    ? generated.lateral_displacement_meters
                    : generated.vertical_displacement_meters;
            long double cosine_projection = 0.0L;
            long double sine_projection = 0.0L;
            for (std::size_t sample_index = 0;
                 sample_index < kSampleCount; ++sample_index) {
                const std::size_t generated_index =
                    kFirstSample + sample_index;
                const double station =
                    generated.track_station_meters[generated_index];
                const double angle =
                    2.0 * std::numbers::pi * frequency * station;
                cosine_projection +=
                    static_cast<long double>(samples[generated_index]) *
                    std::cos(angle);
                sine_projection +=
                    static_cast<long double>(samples[generated_index]) *
                    std::sin(angle);
            }
            const long double normalization =
                2.0L / static_cast<long double>(kSampleCount);
            const long double cosine_amplitude =
                normalization * cosine_projection;
            const long double sine_amplitude =
                normalization * sine_projection;
            const double recovered_psd = static_cast<double>(
                (cosine_amplitude * cosine_amplitude +
                 sine_amplitude * sine_amplitude) /
                (2.0L * frequency_spacing));
            const double expected_psd =
                EvaluateErriB176OneSidedSpatialPsd(
                    ErriB176IrregularityLevel::kLow, direction, frequency);
            RequireRelativeNear(recovered_psd, expected_psd, 2.0e-11,
                                "the low-frequency DFT did not recover the PSD");
        }
    }
}

void CheckGenerationBoundary() {
    auto invalid = MakeGenerationSpec(ErriB176IrregularityLevel::kLow, 42);
    invalid.frequency_grid.minimum_cycles_per_meter = 0.0;
    ExpectInvalid(
        [&] {
            static_cast<void>(GenerateErriB176TrackIrregularity(invalid));
        },
        "positive");
    invalid = MakeGenerationSpec(ErriB176IrregularityLevel::kLow, 42);
    invalid.irregularity_level = static_cast<ErriB176IrregularityLevel>(99);
    ExpectInvalid(
        [&] {
            static_cast<void>(GenerateErriB176TrackIrregularity(invalid));
        },
        "irregularity level");
}

}  // namespace

int main() {
    CheckQchParametersAndPsd();
    CheckContinuousVarianceAndMetadata();
    CheckLowHighScalingAndReplay();
    CheckLowFrequencyDftInversion();
    CheckGenerationBoundary();
    return failures == 0 ? 0 : 1;
}
