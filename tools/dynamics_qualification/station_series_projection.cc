#include "station_series_projection.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace orvd::dynamics_qualification {
namespace {

void RequireFiniteStrictlyIncreasing(std::span<const double> values,
                                     const char* name) {
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!std::isfinite(values[index])) {
            throw std::invalid_argument(std::string(name) +
                                        " contains a non-finite value");
        }
        if (index != 0 && !(values[index] > values[index - 1])) {
            throw std::invalid_argument(std::string(name) +
                                        " must be strictly increasing");
        }
    }
}

}  // namespace

std::vector<double> ProjectMonotoneSeriesToStationGrid(
    std::span<const double> source_track_station_meters,
    std::span<const double> source_values,
    std::span<const double> target_track_station_meters) {
    if (source_track_station_meters.size() != source_values.size()) {
        throw std::invalid_argument(
            "station projection: source stations and values have different "
            "sizes");
    }
    if (source_track_station_meters.size() < 2) {
        throw std::invalid_argument(
            "station projection: at least two source samples are required");
    }
    if (target_track_station_meters.empty()) {
        throw std::invalid_argument(
            "station projection: the target station grid is empty");
    }
    RequireFiniteStrictlyIncreasing(source_track_station_meters,
                                    "source station sequence");
    RequireFiniteStrictlyIncreasing(target_track_station_meters,
                                    "target station sequence");
    for (double value : source_values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(
                "station projection: source values contain a non-finite "
                "value");
        }
    }
    if (target_track_station_meters.front() <
            source_track_station_meters.front() ||
        target_track_station_meters.back() >
            source_track_station_meters.back()) {
        throw std::invalid_argument(
            "station projection: the target grid is not covered by the "
            "source station interval");
    }

    std::vector<double> result;
    result.reserve(target_track_station_meters.size());
    std::size_t upper = 1;
    for (double target : target_track_station_meters) {
        while (source_track_station_meters[upper] < target) {
            ++upper;
        }
        if (target == source_track_station_meters[upper]) {
            result.push_back(source_values[upper]);
            continue;
        }
        const std::size_t lower = upper - 1;
        if (target == source_track_station_meters[lower]) {
            result.push_back(source_values[lower]);
            continue;
        }
        const double fraction =
            (target - source_track_station_meters[lower]) /
            (source_track_station_meters[upper] -
             source_track_station_meters[lower]);
        result.push_back(
            source_values[lower] +
            fraction * (source_values[upper] - source_values[lower]));
    }
    return result;
}

}  // namespace orvd::dynamics_qualification
