#include "orvd/track_irregularity/composite_track_irregularity_generator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace orvd::track_irregularity {
namespace {

TrackIrregularitySampleStatistics Statistics(
    const std::vector<double>& samples, std::size_t begin, std::size_t end) {
    TrackIrregularitySampleStatistics result;
    result.sample_count = end - begin;
    double sum = 0.0;
    double sum_squares = 0.0;
    for (std::size_t index = begin; index < end; ++index) {
        const double value = samples[index];
        sum += value;
        sum_squares += value * value;
        result.absolute_peak_meters =
            std::max(result.absolute_peak_meters, std::abs(value));
    }
    if (result.sample_count != 0) {
        const double count = static_cast<double>(result.sample_count);
        result.mean_meters = sum / count;
        result.root_mean_square_meters = std::sqrt(sum_squares / count);
    }
    return result;
}

}  // namespace

GeneratedCompositeTrackIrregularity GenerateCompositeTrackIrregularity(
    const CompositeTrackIrregularityGenerationSpec& specification) {
    if (specification.components.empty() ||
        specification.transitions.size() + 1 != specification.components.size()) {
        throw std::invalid_argument(
            "composite irregularity requires one transition between adjacent components");
    }
    double previous_end = specification.station_grid.start_meters;
    for (const auto& transition : specification.transitions) {
        if (!std::isfinite(transition.start_meters) ||
            !std::isfinite(transition.end_meters) ||
            transition.start_meters < previous_end ||
            transition.end_meters <= transition.start_meters ||
            transition.end_meters > specification.station_grid.end_meters) {
            throw std::invalid_argument(
                "composite irregularity transitions must be ordered, nonoverlapping and inside the station grid");
        }
        previous_end = transition.end_meters;
    }
    GeneratedCompositeTrackIrregularity result;
    result.metadata.specification = specification;
    for (std::size_t component_index = 0;
         component_index < specification.components.size(); ++component_index) {
        const auto& component = specification.components[component_index];
        std::visit([&](const auto spectrum) {
            auto generated = [&]() {
                if constexpr (std::is_same_v<decltype(spectrum), const AarTrackClass>) {
                    return GenerateAarTrackIrregularity({
                        spectrum, component.frequency_grid, specification.station_grid,
                        specification.placement, specification.realization_seed});
                } else {
                    return GenerateErriB176TrackIrregularity({
                        spectrum, component.frequency_grid, specification.station_grid,
                        specification.placement, specification.realization_seed});
                }
            }();
            result.metadata.components.emplace_back(std::move(generated.metadata));
            if (component_index == 0) {
                result.track_station_meters = std::move(generated.track_station_meters);
                result.lateral_displacement_meters = std::move(generated.lateral_displacement_meters);
                result.vertical_displacement_meters = std::move(generated.vertical_displacement_meters);
                return;
            }
            const auto& transition = specification.transitions[component_index - 1];
            for (std::size_t sample = 0; sample < result.track_station_meters.size(); ++sample) {
                const double station = result.track_station_meters[sample];
                if (station <= transition.start_meters) {
                    continue;
                }
                if (station >= transition.end_meters) {
                    result.lateral_displacement_meters[sample] = generated.lateral_displacement_meters[sample];
                    result.vertical_displacement_meters[sample] = generated.vertical_displacement_meters[sample];
                    continue;
                }
                const double weight = Smoothstep5(
                    (station - transition.start_meters) /
                    (transition.end_meters - transition.start_meters));
                result.lateral_displacement_meters[sample] =
                    (1.0 - weight) * result.lateral_displacement_meters[sample] +
                    weight * generated.lateral_displacement_meters[sample];
                result.vertical_displacement_meters[sample] =
                    (1.0 - weight) * result.vertical_displacement_meters[sample] +
                    weight * generated.vertical_displacement_meters[sample];
            }
        }, component.spectrum);
    }
    const std::size_t sample_count = result.track_station_meters.size();
    result.metadata.station_sample_count = sample_count;
    result.metadata.lateral = Statistics(result.lateral_displacement_meters, 0, sample_count);
    result.metadata.vertical = Statistics(result.vertical_displacement_meters, 0, sample_count);
    std::vector<double> boundaries{specification.station_grid.start_meters};
    for (const auto& transition : specification.transitions) {
        boundaries.push_back(transition.start_meters);
        boundaries.push_back(transition.end_meters);
    }
    boundaries.push_back(specification.station_grid.end_meters);
    for (std::size_t interval = 0; interval + 1 < boundaries.size(); ++interval) {
        const auto first = std::lower_bound(result.track_station_meters.begin(),
            result.track_station_meters.end(), boundaries[interval]);
        const auto last = interval + 2 == boundaries.size()
            ? result.track_station_meters.end()
            : std::lower_bound(result.track_station_meters.begin(),
                result.track_station_meters.end(), boundaries[interval + 1]);
        const auto begin = static_cast<std::size_t>(first - result.track_station_meters.begin());
        const auto end = static_cast<std::size_t>(last - result.track_station_meters.begin());
        result.metadata.intervals.push_back({boundaries[interval], boundaries[interval + 1],
            Statistics(result.lateral_displacement_meters, begin, end),
            Statistics(result.vertical_displacement_meters, begin, end)});
    }
    return result;
}

}  // namespace orvd::track_irregularity
