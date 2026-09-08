#include "orvd/track_irregularity/track_irregularity_generation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "track_irregularity_generation_internal.h"

namespace orvd::track_irregularity {
namespace internal {
namespace {

constexpr std::size_t kOscillatorReanchorInterval = 256;
constexpr std::uint64_t kLateralSeedDomain = 0x4c41544552414c00ULL;
constexpr std::uint64_t kVerticalSeedDomain = 0x564552544943414cULL;

void ValidateFrequencyGrid(std::string_view diagnostic_prefix,
                           const SpatialFrequencyGridSpec& frequency_grid) {
    RequireFinite(diagnostic_prefix, "minimum_cycles_per_meter",
                  frequency_grid.minimum_cycles_per_meter);
    RequireFinite(diagnostic_prefix, "maximum_cycles_per_meter",
                  frequency_grid.maximum_cycles_per_meter);
    if (!(frequency_grid.minimum_cycles_per_meter > 0.0)) {
        Reject(diagnostic_prefix,
               "minimum_cycles_per_meter must be positive");
    }
    if (!(frequency_grid.maximum_cycles_per_meter >
          frequency_grid.minimum_cycles_per_meter)) {
        Reject(diagnostic_prefix,
               "maximum_cycles_per_meter must exceed the minimum");
    }
    if (frequency_grid.frequency_count < 2) {
        Reject(diagnostic_prefix, "frequency_count must be at least two");
    }
}

void ValidateStationGrid(std::string_view diagnostic_prefix,
                         const TrackStationGridSpec& station_grid) {
    RequireFinite(diagnostic_prefix, "station_grid.start_meters",
                  station_grid.start_meters);
    RequireFinite(diagnostic_prefix, "station_grid.end_meters",
                  station_grid.end_meters);
    RequireFinite(diagnostic_prefix, "station_grid.spacing_meters",
                  station_grid.spacing_meters);
    if (!(station_grid.end_meters > station_grid.start_meters)) {
        Reject(diagnostic_prefix, "station-grid end must exceed its start");
    }
    if (!(station_grid.spacing_meters > 0.0)) {
        Reject(diagnostic_prefix, "station-grid spacing must be positive");
    }
}

void ValidatePlacementShape(std::string_view diagnostic_prefix,
                            const TrackIrregularityPlacementSpec& placement) {
    RequireFinite(diagnostic_prefix, "placement.start_meters",
                  placement.start_meters);
    RequireFinite(diagnostic_prefix, "placement.end_meters",
                  placement.end_meters);
    RequireFinite(diagnostic_prefix, "placement.fade_in_length_meters",
                  placement.fade_in_length_meters);
    RequireFinite(diagnostic_prefix, "placement.fade_out_length_meters",
                  placement.fade_out_length_meters);
    if (!(placement.end_meters > placement.start_meters)) {
        Reject(diagnostic_prefix, "placement end must exceed its start");
    }
    if (!(placement.fade_in_length_meters > 0.0) ||
        !(placement.fade_out_length_meters > 0.0)) {
        Reject(diagnostic_prefix, "both placement fade lengths must be positive");
    }
    const double active_length = placement.end_meters - placement.start_meters;
    if (placement.fade_in_length_meters + placement.fade_out_length_meters >
        active_length) {
        Reject(diagnostic_prefix, "placement fade windows overlap");
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

std::size_t StationSampleCount(std::string_view diagnostic_prefix,
                               const TrackStationGridSpec& station_grid) {
    const long double interval_count =
        GridCoordinate(station_grid, station_grid.end_meters);
    const long double nearest = std::round(interval_count);
    const long double tolerance =
        128.0L * static_cast<long double>(std::numeric_limits<double>::epsilon()) *
        std::max(1.0L, std::abs(interval_count));
    if (std::abs(interval_count - nearest) > tolerance) {
        Reject(diagnostic_prefix,
               "station-grid spacing must divide the stated interval");
    }
    const long double maximum_interval_count =
        static_cast<long double>(std::numeric_limits<std::size_t>::max() - 1);
    if (!(nearest >= 1.0L) || nearest > maximum_interval_count) {
        Reject(diagnostic_prefix,
               "station-grid sample count is not representable");
    }
    return static_cast<std::size_t>(nearest) + 1;
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
        const std::uint64_t significand = engine_() >> 11U;
        const double unit_interval =
            std::ldexp(static_cast<double>(significand), -53);
        return 2.0 * std::numbers::pi * unit_interval;
    }

   private:
    std::mt19937_64 engine_;
};

double AccumulateHarmonicChannel(
    const SpatialFrequencyGridSpec& frequency_grid,
    const TrackStationGridSpec& station_grid,
    const TrackIrregularityPlacementSpec& placement, std::uint64_t seed,
    std::span<const double> one_sided_psd,
    std::vector<double>* displacement_meters) {
    ReproduciblePhaseGenerator phase_generator(seed);
    const double frequency_spacing =
        FrequencySpacingCyclesPerMeter(frequency_grid);
    double discrete_variance = 0.0;

    for (std::size_t frequency_index = 0;
         frequency_index < frequency_grid.frequency_count;
         ++frequency_index) {
        const double frequency =
            frequency_grid.minimum_cycles_per_meter +
            static_cast<double>(frequency_index) * frequency_spacing;
        const double psd = one_sided_psd[frequency_index];
        const double amplitude =
            std::sqrt(2.0 * psd * frequency_spacing);
        discrete_variance += psd * frequency_spacing;

        const double phase = phase_generator.NextPhaseRadians();
        const double angular_station_increment =
            2.0 * std::numbers::pi * frequency * station_grid.spacing_meters;
        const double initial_angle =
            phase + 2.0 * std::numbers::pi * frequency *
                        (station_grid.start_meters - placement.start_meters);
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
                    initial_angle + static_cast<double>(next_station_index) *
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

    TrackIrregularitySampleStatistics Finish(
        std::string_view diagnostic_prefix) const {
        if (sample_count_ == 0) {
            Reject(diagnostic_prefix, "sample statistics have an empty interval");
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

TrackIrregularityChannelCoreResult ApplyPlacementAndMeasure(
    std::string_view diagnostic_prefix,
    const TrackIrregularityPlacementSpec& placement,
    const TrackStationGridSpec& station_grid,
    const std::vector<double>& track_station_meters,
    double discrete_harmonic_variance_meters_squared,
    std::vector<double>* displacement_meters) {
    const std::size_t first_full_amplitude_index =
        static_cast<std::size_t>(std::round(GridCoordinate(
            station_grid,
            placement.start_meters + placement.fade_in_length_meters)));
    const std::size_t last_full_amplitude_index =
        static_cast<std::size_t>(std::round(GridCoordinate(
            station_grid,
            placement.end_meters - placement.fade_out_length_meters)));
    SampleStatisticsAccumulator full_amplitude;
    SampleStatisticsAccumulator complete;
    for (std::size_t station_index = 0;
         station_index < track_station_meters.size(); ++station_index) {
        const double ungated_displacement =
            (*displacement_meters)[station_index];
        const double weight = PlacementWeightUnchecked(
            placement, track_station_meters[station_index]);
        (*displacement_meters)[station_index] = ungated_displacement * weight;
        complete.Add((*displacement_meters)[station_index]);
        if (station_index >= first_full_amplitude_index &&
            station_index <= last_full_amplitude_index) {
            full_amplitude.Add(ungated_displacement);
        }
    }
    return TrackIrregularityChannelCoreResult{
        discrete_harmonic_variance_meters_squared,
        full_amplitude.Finish(diagnostic_prefix),
        complete.Finish(diagnostic_prefix)};
}

}  // namespace

[[noreturn]] void Reject(std::string_view diagnostic_prefix,
                         std::string_view detail) {
    throw std::invalid_argument(std::string(diagnostic_prefix) + ": " +
                                std::string(detail));
}

void RequireFinite(std::string_view diagnostic_prefix, const char* name,
                   double value) {
    if (!std::isfinite(value)) {
        Reject(diagnostic_prefix, std::string(name) + " must be finite");
    }
}

void ValidateTrackIrregularityDirection(
    std::string_view diagnostic_prefix,
    TrackIrregularityDirection direction) {
    switch (direction) {
        case TrackIrregularityDirection::kLateral:
        case TrackIrregularityDirection::kVertical:
            return;
    }
    Reject(diagnostic_prefix, "the track-irregularity direction is unsupported");
}

void ValidateTrackIrregularityGenerationSpecification(
    std::string_view diagnostic_prefix,
    const SpatialFrequencyGridSpec& frequency_grid,
    const TrackStationGridSpec& station_grid,
    const TrackIrregularityPlacementSpec& placement) {
    ValidateFrequencyGrid(diagnostic_prefix, frequency_grid);
    ValidateStationGrid(diagnostic_prefix, station_grid);
    ValidatePlacementShape(diagnostic_prefix, placement);
    static_cast<void>(StationSampleCount(diagnostic_prefix, station_grid));

    if (station_grid.spacing_meters *
            frequency_grid.maximum_cycles_per_meter >=
        0.5) {
        Reject(diagnostic_prefix,
               "station-grid spacing violates the highest-frequency Nyquist "
               "limit");
    }
    if (placement.start_meters < station_grid.start_meters ||
        placement.end_meters > station_grid.end_meters) {
        Reject(diagnostic_prefix, "placement must lie within the station grid");
    }

    const double full_amplitude_start =
        placement.start_meters + placement.fade_in_length_meters;
    const double full_amplitude_end =
        placement.end_meters - placement.fade_out_length_meters;
    for (const auto& connection : {
             std::pair{"placement start", placement.start_meters},
             std::pair{"fade-in end", full_amplitude_start},
             std::pair{"fade-out start", full_amplitude_end},
             std::pair{"placement end", placement.end_meters}}) {
        if (!IsGridKnot(station_grid, connection.second)) {
            Reject(diagnostic_prefix,
                   std::string(connection.first) +
                       " must coincide with a station-grid knot");
        }
    }
}

double FrequencySpacingCyclesPerMeter(
    const SpatialFrequencyGridSpec& frequency_grid) noexcept {
    return (frequency_grid.maximum_cycles_per_meter -
            frequency_grid.minimum_cycles_per_meter) /
           static_cast<double>(frequency_grid.frequency_count - 1);
}

TrackIrregularityChannelSeeds DeriveTrackIrregularityChannelSeeds(
    std::uint64_t realization_seed) noexcept {
    return TrackIrregularityChannelSeeds{
        SplitMix64(realization_seed ^ kLateralSeedDomain),
        SplitMix64(realization_seed ^ kVerticalSeedDomain)};
}

GeneratedTrackIrregularityCore GenerateTrackIrregularityCore(
    std::string_view diagnostic_prefix,
    const SpatialFrequencyGridSpec& frequency_grid,
    const TrackStationGridSpec& station_grid,
    const TrackIrregularityPlacementSpec& placement,
    std::uint64_t realization_seed,
    std::span<const double> lateral_one_sided_psd,
    std::span<const double> vertical_one_sided_psd) {
    if (lateral_one_sided_psd.size() != frequency_grid.frequency_count ||
        vertical_one_sided_psd.size() != frequency_grid.frequency_count) {
        Reject(diagnostic_prefix,
               "sampled PSD size does not match the frequency grid");
    }

    GeneratedTrackIrregularityCore generated;
    generated.station_sample_count =
        StationSampleCount(diagnostic_prefix, station_grid);
    generated.frequency_spacing_cycles_per_meter =
        FrequencySpacingCyclesPerMeter(frequency_grid);
    generated.track_station_meters.resize(generated.station_sample_count);
    generated.lateral_displacement_meters.assign(generated.station_sample_count,
                                                  0.0);
    generated.vertical_displacement_meters.assign(generated.station_sample_count,
                                                   0.0);

    for (std::size_t station_index = 0;
         station_index < generated.station_sample_count; ++station_index) {
        generated.track_station_meters[station_index] =
            station_grid.start_meters + static_cast<double>(station_index) *
                                            station_grid.spacing_meters;
    }
    generated.track_station_meters.back() = station_grid.end_meters;

    generated.channel_seeds =
        DeriveTrackIrregularityChannelSeeds(realization_seed);
    const double lateral_discrete_variance = AccumulateHarmonicChannel(
        frequency_grid, station_grid, placement,
        generated.channel_seeds.lateral, lateral_one_sided_psd,
        &generated.lateral_displacement_meters);
    const double vertical_discrete_variance = AccumulateHarmonicChannel(
        frequency_grid, station_grid, placement,
        generated.channel_seeds.vertical, vertical_one_sided_psd,
        &generated.vertical_displacement_meters);

    generated.lateral = ApplyPlacementAndMeasure(
        diagnostic_prefix, placement, station_grid,
        generated.track_station_meters, lateral_discrete_variance,
        &generated.lateral_displacement_meters);
    generated.vertical = ApplyPlacementAndMeasure(
        diagnostic_prefix, placement, station_grid,
        generated.track_station_meters, vertical_discrete_variance,
        &generated.vertical_displacement_meters);
    return generated;
}

}  // namespace internal

double Smoothstep5(double unit_interval) noexcept {
    if (unit_interval <= 0.0) {
        return 0.0;
    }
    if (unit_interval >= 1.0) {
        return 1.0;
    }
    const auto lower_half = [](double value) noexcept {
        return value * value * value *
               (10.0 + value * (-15.0 + 6.0 * value));
    };
    if (unit_interval <= 0.5) {
        return lower_half(unit_interval);
    }
    const double complement = 1.0 - unit_interval;
    return 1.0 - lower_half(complement);
}

double TrackIrregularityPlacementWeight(
    const TrackIrregularityPlacementSpec& placement,
    double track_station_meters) {
    constexpr std::string_view kDiagnosticPrefix =
        "track-irregularity placement";
    internal::ValidatePlacementShape(kDiagnosticPrefix, placement);
    internal::RequireFinite(kDiagnosticPrefix, "track_station_meters",
                            track_station_meters);
    return internal::PlacementWeightUnchecked(placement,
                                               track_station_meters);
}

}  // namespace orvd::track_irregularity
