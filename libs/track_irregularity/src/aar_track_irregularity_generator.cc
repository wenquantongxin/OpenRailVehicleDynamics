#include "orvd/track_irregularity/aar_track_irregularity_generator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::track_irregularity {
namespace {

constexpr double kSpectrumScaleFactor = 0.25;
constexpr double kCornerAngularWavenumberRadiansPerMeter = 0.8245;
constexpr double kSquareCentimetersToSquareMeters = 1.0e-4;
constexpr std::size_t kOscillatorReanchorInterval = 256;

constexpr std::uint64_t kLateralSeedDomain = 0x4c41544552414c00ULL;
constexpr std::uint64_t kVerticalSeedDomain = 0x564552544943414cULL;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("AAR track-irregularity generator: " + detail);
}

void RequireFinite(const char* name, double value) {
    if (!std::isfinite(value)) {
        Reject(std::string(name) + " must be finite");
    }
}

void ValidateTrackClass(AarTrackClass track_class) {
    switch (track_class) {
        case AarTrackClass::kAar5:
        case AarTrackClass::kAar6:
            return;
    }
    Reject("the AAR track class is unsupported");
}

void ValidateDirection(TrackIrregularityDirection direction) {
    switch (direction) {
        case TrackIrregularityDirection::kLateral:
        case TrackIrregularityDirection::kVertical:
            return;
    }
    Reject("the track-irregularity direction is unsupported");
}

double TraditionalAmplitude(
    AarTrackClass track_class, TrackIrregularityDirection direction) {
    ValidateTrackClass(track_class);
    ValidateDirection(direction);
    if (track_class == AarTrackClass::kAar5) {
        return direction == TrackIrregularityDirection::kLateral ? 0.0762
                                                                 : 0.2095;
    }
    return 0.0339;
}

void ValidateFrequencyGrid(const SpatialFrequencyGridSpec& frequency_grid) {
    RequireFinite("minimum_cycles_per_meter",
                  frequency_grid.minimum_cycles_per_meter);
    RequireFinite("maximum_cycles_per_meter",
                  frequency_grid.maximum_cycles_per_meter);
    if (!(frequency_grid.minimum_cycles_per_meter > 0.0)) {
        Reject("minimum_cycles_per_meter must be positive");
    }
    if (!(frequency_grid.maximum_cycles_per_meter >
          frequency_grid.minimum_cycles_per_meter)) {
        Reject("maximum_cycles_per_meter must exceed the minimum");
    }
    if (frequency_grid.frequency_count < 2) {
        Reject("frequency_count must be at least two");
    }
}

void ValidateStationGrid(const TrackStationGridSpec& station_grid) {
    RequireFinite("station_grid.start_meters", station_grid.start_meters);
    RequireFinite("station_grid.end_meters", station_grid.end_meters);
    RequireFinite("station_grid.spacing_meters", station_grid.spacing_meters);
    if (!(station_grid.end_meters > station_grid.start_meters)) {
        Reject("station-grid end must exceed its start");
    }
    if (!(station_grid.spacing_meters > 0.0)) {
        Reject("station-grid spacing must be positive");
    }
}

void ValidatePlacementShape(
    const TrackIrregularityPlacementSpec& placement) {
    RequireFinite("placement.start_meters", placement.start_meters);
    RequireFinite("placement.end_meters", placement.end_meters);
    RequireFinite("placement.fade_in_length_meters",
                  placement.fade_in_length_meters);
    RequireFinite("placement.fade_out_length_meters",
                  placement.fade_out_length_meters);
    if (!(placement.end_meters > placement.start_meters)) {
        Reject("placement end must exceed its start");
    }
    if (!(placement.fade_in_length_meters > 0.0) ||
        !(placement.fade_out_length_meters > 0.0)) {
        Reject("both placement fade lengths must be positive");
    }
    const double active_length =
        placement.end_meters - placement.start_meters;
    if (placement.fade_in_length_meters +
            placement.fade_out_length_meters >
        active_length) {
        Reject("placement fade windows overlap");
    }
}

long double GridCoordinate(const TrackStationGridSpec& station_grid,
                           double track_station_meters) {
    return (static_cast<long double>(track_station_meters) -
            static_cast<long double>(station_grid.start_meters)) /
           static_cast<long double>(station_grid.spacing_meters);
}

bool IsGridKnot(const TrackStationGridSpec& station_grid,
                double track_station_meters) {
    const long double coordinate =
        GridCoordinate(station_grid, track_station_meters);
    const long double nearest = std::round(coordinate);
    const long double tolerance =
        128.0L * static_cast<long double>(std::numeric_limits<double>::epsilon()) *
        std::max(1.0L, std::abs(coordinate));
    return std::abs(coordinate - nearest) <= tolerance;
}

std::size_t StationSampleCount(const TrackStationGridSpec& station_grid) {
    const long double interval_count =
        GridCoordinate(station_grid, station_grid.end_meters);
    const long double nearest = std::round(interval_count);
    const long double tolerance =
        128.0L * static_cast<long double>(std::numeric_limits<double>::epsilon()) *
        std::max(1.0L, std::abs(interval_count));
    if (std::abs(interval_count - nearest) > tolerance) {
        Reject("station-grid spacing must divide the stated interval");
    }
    const long double maximum_interval_count =
        static_cast<long double>(std::numeric_limits<std::size_t>::max() - 1);
    if (!(nearest >= 1.0L) || nearest > maximum_interval_count) {
        Reject("station-grid sample count is not representable");
    }
    return static_cast<std::size_t>(nearest) + 1;
}

void ValidateSpecification(
    const AarTrackIrregularityGenerationSpec& specification) {
    ValidateTrackClass(specification.track_class);
    ValidateFrequencyGrid(specification.frequency_grid);
    ValidateStationGrid(specification.station_grid);
    ValidatePlacementShape(specification.placement);
    static_cast<void>(StationSampleCount(specification.station_grid));

    if (specification.station_grid.spacing_meters *
            specification.frequency_grid.maximum_cycles_per_meter >=
        0.5) {
        Reject("station-grid spacing violates the highest-frequency Nyquist "
               "limit");
    }
    if (specification.placement.start_meters <
            specification.station_grid.start_meters ||
        specification.placement.end_meters >
            specification.station_grid.end_meters) {
        Reject("placement must lie within the station grid");
    }

    const double full_amplitude_start =
        specification.placement.start_meters +
        specification.placement.fade_in_length_meters;
    const double full_amplitude_end =
        specification.placement.end_meters -
        specification.placement.fade_out_length_meters;
    for (const auto& connection : {
             std::pair{"placement start",
                       specification.placement.start_meters},
             std::pair{"fade-in end", full_amplitude_start},
             std::pair{"fade-out start", full_amplitude_end},
             std::pair{"placement end", specification.placement.end_meters}}) {
        if (!IsGridKnot(specification.station_grid, connection.second)) {
            Reject(std::string(connection.first) +
                   " must coincide with a station-grid knot");
        }
    }
}

double PlacementWeightUnchecked(
    const TrackIrregularityPlacementSpec& placement,
    double track_station_meters) noexcept {
    if (track_station_meters <= placement.start_meters ||
        track_station_meters >= placement.end_meters) {
        return 0.0;
    }
    const double full_amplitude_start =
        placement.start_meters + placement.fade_in_length_meters;
    if (track_station_meters < full_amplitude_start) {
        return Smoothstep5((track_station_meters - placement.start_meters) /
                           placement.fade_in_length_meters);
    }
    const double full_amplitude_end =
        placement.end_meters - placement.fade_out_length_meters;
    if (track_station_meters > full_amplitude_end) {
        return Smoothstep5((placement.end_meters - track_station_meters) /
                           placement.fade_out_length_meters);
    }
    return 1.0;
}

std::uint64_t SplitMix64(std::uint64_t value) noexcept {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

class ReproduciblePhaseGenerator {
   public:
    explicit ReproduciblePhaseGenerator(std::uint64_t seed) : engine_(seed) {}

    double NextPhaseRadians() {
        // The engine sequence is standardized. Mapping its high 53 bits
        // explicitly avoids the unspecified algorithm selected by
        // std::uniform_real_distribution.
        const std::uint64_t significand = engine_() >> 11U;
        const double unit_interval =
            std::ldexp(static_cast<double>(significand), -53);
        return 2.0 * std::numbers::pi * unit_interval;
    }

   private:
    std::mt19937_64 engine_;
};

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

double AccumulateHarmonicChannel(
    const AarTrackIrregularityGenerationSpec& specification,
    TrackIrregularityDirection direction, std::uint64_t seed,
    std::vector<double>* displacement_meters) {
    ReproduciblePhaseGenerator phase_generator(seed);
    const auto& frequency_grid = specification.frequency_grid;
    const double frequency_spacing =
        (frequency_grid.maximum_cycles_per_meter -
         frequency_grid.minimum_cycles_per_meter) /
        static_cast<double>(frequency_grid.frequency_count - 1);
    double discrete_variance = 0.0;

    for (std::size_t frequency_index = 0;
         frequency_index < frequency_grid.frequency_count;
         ++frequency_index) {
        const double frequency =
            frequency_grid.minimum_cycles_per_meter +
            static_cast<double>(frequency_index) * frequency_spacing;
        const double psd = EvaluateAarOneSidedSpatialPsd(
            specification.track_class, direction, frequency);
        const double amplitude =
            std::sqrt(2.0 * psd * frequency_spacing);
        discrete_variance += psd * frequency_spacing;

        const double phase = phase_generator.NextPhaseRadians();
        const double angular_station_increment =
            2.0 * std::numbers::pi * frequency *
            specification.station_grid.spacing_meters;
        const double initial_angle =
            phase + 2.0 * std::numbers::pi * frequency *
                        (specification.station_grid.start_meters -
                         specification.placement.start_meters);
        const double increment_cosine = std::cos(angular_station_increment);
        const double increment_sine = std::sin(angular_station_increment);
        double harmonic_cosine = std::cos(initial_angle);
        double harmonic_sine = std::sin(initial_angle);

        for (std::size_t station_index = 0;
             station_index < displacement_meters->size(); ++station_index) {
            (*displacement_meters)[station_index] +=
                amplitude * harmonic_cosine;
            if (station_index + 1 == displacement_meters->size()) {
                continue;
            }
            const std::size_t next_station_index = station_index + 1;
            if (next_station_index % kOscillatorReanchorInterval == 0) {
                const double angle =
                    initial_angle +
                    static_cast<double>(next_station_index) *
                        angular_station_increment;
                harmonic_cosine = std::cos(angle);
                harmonic_sine = std::sin(angle);
            } else {
                const double next_cosine =
                    harmonic_cosine * increment_cosine -
                    harmonic_sine * increment_sine;
                harmonic_sine = harmonic_sine * increment_cosine +
                                harmonic_cosine * increment_sine;
                harmonic_cosine = next_cosine;
            }
        }
    }
    return discrete_variance;
}

class SampleStatisticsAccumulator {
   public:
    void Add(double value) {
        ++sample_count_;
        sum_ += static_cast<long double>(value);
        sum_of_squares_ += static_cast<long double>(value) *
                           static_cast<long double>(value);
        absolute_peak_ = std::max(absolute_peak_, std::abs(value));
    }

    TrackIrregularitySampleStatistics Finish() const {
        if (sample_count_ == 0) {
            Reject("sample statistics have an empty interval");
        }
        const long double inverse_count =
            1.0L / static_cast<long double>(sample_count_);
        return TrackIrregularitySampleStatistics{
            sample_count_, static_cast<double>(sum_ * inverse_count),
            static_cast<double>(std::sqrt(sum_of_squares_ * inverse_count)),
            absolute_peak_};
    }

   private:
    std::size_t sample_count_{0};
    long double sum_{0.0L};
    long double sum_of_squares_{0.0L};
    double absolute_peak_{0.0};
};

struct GatedChannelStatistics {
    TrackIrregularitySampleStatistics full_amplitude;
    TrackIrregularitySampleStatistics complete;
};

GatedChannelStatistics ApplyPlacementAndMeasure(
    const TrackIrregularityPlacementSpec& placement,
    const std::vector<double>& track_station_meters,
    std::vector<double>* displacement_meters) {
    SampleStatisticsAccumulator full_amplitude;
    SampleStatisticsAccumulator complete;
    for (std::size_t station_index = 0;
         station_index < track_station_meters.size(); ++station_index) {
        const double weight = PlacementWeightUnchecked(
            placement, track_station_meters[station_index]);
        (*displacement_meters)[station_index] *= weight;
        complete.Add((*displacement_meters)[station_index]);
        if (weight == 1.0) {
            full_amplitude.Add((*displacement_meters)[station_index]);
        }
    }
    return GatedChannelStatistics{full_amplitude.Finish(),
                                  complete.Finish()};
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
    return TrackIrregularityChannelSeeds{
        SplitMix64(realization_seed ^ kLateralSeedDomain),
        SplitMix64(realization_seed ^ kVerticalSeedDomain)};
}

double EvaluateAarOneSidedSpatialPsd(
    AarTrackClass track_class, TrackIrregularityDirection direction,
    double frequency_cycles_per_meter) {
    if (!std::isfinite(frequency_cycles_per_meter) ||
        !(frequency_cycles_per_meter > 0.0)) {
        Reject("PSD frequency must be finite and positive");
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

double Smoothstep5(double unit_interval) noexcept {
    if (unit_interval <= 0.0) {
        return 0.0;
    }
    if (unit_interval >= 1.0) {
        return 1.0;
    }
    return unit_interval * unit_interval * unit_interval *
           (10.0 + unit_interval * (-15.0 + 6.0 * unit_interval));
}

double TrackIrregularityPlacementWeight(
    const TrackIrregularityPlacementSpec& placement,
    double track_station_meters) {
    ValidatePlacementShape(placement);
    RequireFinite("track_station_meters", track_station_meters);
    return PlacementWeightUnchecked(placement, track_station_meters);
}

GeneratedTrackIrregularity GenerateAarTrackIrregularity(
    const AarTrackIrregularityGenerationSpec& specification) {
    ValidateSpecification(specification);
    const std::size_t station_sample_count =
        StationSampleCount(specification.station_grid);
    GeneratedTrackIrregularity generated;
    generated.track_station_meters.resize(station_sample_count);
    generated.lateral_displacement_meters.assign(station_sample_count, 0.0);
    generated.vertical_displacement_meters.assign(station_sample_count, 0.0);

    for (std::size_t station_index = 0;
         station_index < station_sample_count; ++station_index) {
        generated.track_station_meters[station_index] =
            specification.station_grid.start_meters +
            static_cast<double>(station_index) *
                specification.station_grid.spacing_meters;
    }
    generated.track_station_meters.back() =
        specification.station_grid.end_meters;

    const TrackIrregularityChannelSeeds channel_seeds =
        DeriveAarTrackIrregularityChannelSeeds(
            specification.realization_seed);
    const double lateral_discrete_variance = AccumulateHarmonicChannel(
        specification, TrackIrregularityDirection::kLateral,
        channel_seeds.lateral, &generated.lateral_displacement_meters);
    const double vertical_discrete_variance = AccumulateHarmonicChannel(
        specification, TrackIrregularityDirection::kVertical,
        channel_seeds.vertical, &generated.vertical_displacement_meters);

    const GatedChannelStatistics lateral_statistics =
        ApplyPlacementAndMeasure(specification.placement,
                                 generated.track_station_meters,
                                 &generated.lateral_displacement_meters);
    const GatedChannelStatistics vertical_statistics =
        ApplyPlacementAndMeasure(specification.placement,
                                 generated.track_station_meters,
                                 &generated.vertical_displacement_meters);

    const auto lateral_spectrum = AarSingleCutoffPsdParametersFor(
        specification.track_class, TrackIrregularityDirection::kLateral);
    const auto vertical_spectrum = AarSingleCutoffPsdParametersFor(
        specification.track_class, TrackIrregularityDirection::kVertical);
    const double frequency_spacing =
        (specification.frequency_grid.maximum_cycles_per_meter -
         specification.frequency_grid.minimum_cycles_per_meter) /
        static_cast<double>(specification.frequency_grid.frequency_count - 1);
    generated.metadata = TrackIrregularityGenerationMetadata{
        specification,
        station_sample_count,
        frequency_spacing,
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
        TrackIrregularityChannelGenerationMetadata{
            TrackIrregularityDirection::kLateral, channel_seeds.lateral,
            lateral_spectrum,
            ContinuousBandVariance(lateral_spectrum,
                                   specification.frequency_grid),
            lateral_discrete_variance, lateral_statistics.full_amplitude,
            lateral_statistics.complete},
        TrackIrregularityChannelGenerationMetadata{
            TrackIrregularityDirection::kVertical, channel_seeds.vertical,
            vertical_spectrum,
            ContinuousBandVariance(vertical_spectrum,
                                   specification.frequency_grid),
            vertical_discrete_variance, vertical_statistics.full_amplitude,
            vertical_statistics.complete}};
    return generated;
}

}  // namespace orvd::track_irregularity
