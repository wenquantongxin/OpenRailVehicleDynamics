#pragma once

#include <variant>
#include <vector>

#include "orvd/track_irregularity/aar_track_irregularity_generator.h"
#include "orvd/track_irregularity/erri_b176_track_irregularity_generator.h"

namespace orvd::track_irregularity {

using TrackIrregularitySpectrum =
    std::variant<AarTrackClass, ErriB176IrregularityLevel>;
using SingleSpectrumTrackIrregularityMetadata =
    std::variant<AarTrackIrregularityGenerationMetadata,
                 ErriB176TrackIrregularityGenerationMetadata>;

struct CompositeTrackIrregularityComponent {
    TrackIrregularitySpectrum spectrum;
    SpatialFrequencyGridSpec frequency_grid;
};

/// Transition from component i to i+1. Endpoints need not be grid knots.
struct TrackIrregularitySpatialTransition {
    double start_meters{0.0};
    double end_meters{0.0};
};

/// Components share a seed, phase origin, station grid and placement envelope.
/// The already gated component displacements are mixed once with Smoothstep5.
struct CompositeTrackIrregularityGenerationSpec {
    std::vector<CompositeTrackIrregularityComponent> components;
    std::vector<TrackIrregularitySpatialTransition> transitions;
    TrackStationGridSpec station_grid;
    TrackIrregularityPlacementSpec placement;
    std::uint64_t realization_seed{0};
};

struct TrackIrregularityIntervalStatistics {
    double start_meters{0.0};
    double end_meters{0.0};
    TrackIrregularitySampleStatistics lateral;
    TrackIrregularitySampleStatistics vertical;
};

struct CompositeTrackIrregularityGenerationMetadata {
    CompositeTrackIrregularityGenerationSpec specification;
    std::vector<SingleSpectrumTrackIrregularityMetadata> components;
    std::size_t station_sample_count{0};
    std::string_view realization_algorithm{"orvd-composite-random-phase"};
    std::string_view transition_weight_algorithm{"smoothstep5-displacement"};
    TrackIrregularitySampleStatistics lateral;
    TrackIrregularitySampleStatistics vertical;
    /// Pure-component and transition intervals; half-open except final end.
    std::vector<TrackIrregularityIntervalStatistics> intervals;
};

struct GeneratedCompositeTrackIrregularity {
    std::vector<double> track_station_meters;
    std::vector<double> lateral_displacement_meters;
    std::vector<double> vertical_displacement_meters;
    CompositeTrackIrregularityGenerationMetadata metadata;
};

[[nodiscard]] GeneratedCompositeTrackIrregularity
GenerateCompositeTrackIrregularity(
    const CompositeTrackIrregularityGenerationSpec& specification);

}  // namespace orvd::track_irregularity
