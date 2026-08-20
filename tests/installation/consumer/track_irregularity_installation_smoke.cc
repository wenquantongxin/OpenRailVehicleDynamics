#include <cmath>
#include <cstdio>
#include <exception>

#include "orvd/track_irregularity/aar_track_irregularity_generator.h"

int main() {
    try {
        using orvd::track_irregularity::AarTrackClass;
        using orvd::track_irregularity::AarTrackIrregularityGenerationSpec;
        using orvd::track_irregularity::GenerateAarTrackIrregularity;
        using orvd::track_irregularity::SpatialFrequencyGridSpec;
        using orvd::track_irregularity::TrackIrregularityPlacementSpec;
        using orvd::track_irregularity::TrackStationGridSpec;

        const auto generated = GenerateAarTrackIrregularity(
            AarTrackIrregularityGenerationSpec{
                AarTrackClass::kAar6,
                SpatialFrequencyGridSpec{0.05, 0.13, 9},
                TrackStationGridSpec{0.0, 12.0, 0.25},
                TrackIrregularityPlacementSpec{2.0, 10.0, 2.0, 2.0},
                1234});
        const bool valid =
            generated.track_station_meters.size() == 49 &&
            generated.lateral_displacement_meters.size() == 49 &&
            generated.vertical_displacement_meters.size() == 49 &&
            generated.lateral_displacement_meters.front() == 0.0 &&
            generated.vertical_displacement_meters.back() == 0.0 &&
            generated.metadata.station_sample_count == 49 &&
            generated.metadata.lateral.seed !=
                generated.metadata.vertical.seed &&
            generated.metadata.lateral.discrete_harmonic_variance_meters_squared >
                0.0 &&
            generated.metadata.vertical
                    .continuous_band_variance_meters_squared >
                0.0 &&
            std::isfinite(generated.lateral_displacement_meters[24]) &&
            std::isfinite(generated.vertical_displacement_meters[24]);
        if (!valid) {
            std::fprintf(stderr,
                         "installed track-irregularity generator returned an "
                         "invalid realization\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr,
                     "installed track-irregularity smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
