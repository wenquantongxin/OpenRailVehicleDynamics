#include "orvd/configuration/resolve_track_irregularity_field.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>

#include "orvd/configuration/load_track_irregularity_field.h"

namespace orvd::configuration {
namespace {

void ValidateFrozenTrackIrregularityIdentifier(
    std::string_view track_irregularity_identifier) {
    constexpr std::array<std::string_view, 3> kAuthorizedIdentifiers{
        "aar5_irregularity", "aar6_irregularity", "erri_low_irregularity"};
    for (const std::string_view authorized : kAuthorizedIdentifiers) {
        if (track_irregularity_identifier == authorized) {
            return;
        }
    }
    throw std::invalid_argument(
        "track-irregularity source resolution: frozen field identifier must "
        "select aar5_irregularity, aar6_irregularity, or "
        "erri_low_irregularity");
}

std::size_t StationGridIndex(
    const track_irregularity::TrackStationGridSpec& station_grid,
    double track_station_meters) {
    const long double coordinate =
        (static_cast<long double>(track_station_meters) -
         static_cast<long double>(station_grid.start_meters)) /
        static_cast<long double>(station_grid.spacing_meters);
    return static_cast<std::size_t>(std::round(coordinate));
}

}  // namespace

ResolvedTrackIrregularityField ResolveTrackIrregularityField(
    const std::filesystem::path& data_root,
    const TrackIrregularityFieldSource& source) {
    return std::visit(
        [&data_root](const auto& selected) -> ResolvedTrackIrregularityField {
            using Source = std::decay_t<decltype(selected)>;
            if constexpr (std::is_same_v<
                              Source, FrozenTrackIrregularityFieldSource>) {
                ValidateFrozenTrackIrregularityIdentifier(
                    selected.track_irregularity_identifier);
                return ResolvedTrackIrregularityField{
                    std::make_unique<
                        wheel_rail_contact::TrackIrregularityField>(
                        LoadTrackIrregularityFieldFromDataRoot(
                            data_root,
                            selected.track_irregularity_identifier)),
                    std::nullopt};
            } else {
                auto generated =
                    track_irregularity::GenerateAarTrackIrregularity(
                        selected.specification);
                const std::size_t active_start_index = StationGridIndex(
                    selected.specification.station_grid,
                    selected.specification.placement.start_meters);
                const std::size_t active_end_index = StationGridIndex(
                    selected.specification.station_grid,
                    selected.specification.placement.end_meters);
                const std::size_t active_sample_count =
                    active_end_index - active_start_index + 1;
                const std::span<const double> active_station_meters(
                    generated.track_station_meters.begin() +
                        static_cast<std::ptrdiff_t>(active_start_index),
                    active_sample_count);
                const std::span<const double> active_lateral_meters(
                    generated.lateral_displacement_meters.begin() +
                        static_cast<std::ptrdiff_t>(active_start_index),
                    active_sample_count);
                const std::span<const double> active_vertical_meters(
                    generated.vertical_displacement_meters.begin() +
                        static_cast<std::ptrdiff_t>(active_start_index),
                    active_sample_count);
                auto field = std::make_unique<
                    wheel_rail_contact::TrackIrregularityField>(
                    active_station_meters, active_lateral_meters,
                    active_station_meters, active_vertical_meters);
                return ResolvedTrackIrregularityField{
                    std::move(field), std::move(generated.metadata)};
            }
        },
        source);
}

}  // namespace orvd::configuration
