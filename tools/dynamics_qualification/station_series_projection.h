#pragma once

#include <span>
#include <vector>

namespace orvd::dynamics_qualification {

// Projects one forward-moving sampled scalar series onto a requested station
// grid using the bracketing source samples and linear interpolation.
//
// Source stations and target stations must each be finite and strictly
// increasing. Values must be finite. Every target must lie in the closed source
// interval; this helper neither extrapolates nor shifts either sequence.
[[nodiscard]] std::vector<double> ProjectMonotoneSeriesToStationGrid(
    std::span<const double> source_track_station_meters,
    std::span<const double> source_values,
    std::span<const double> target_track_station_meters);

}  // namespace orvd::dynamics_qualification
