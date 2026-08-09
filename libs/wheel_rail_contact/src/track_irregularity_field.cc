#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace orvd::wheel_rail_contact {

TrackIrregularityField::TrackIrregularityField(
    std::span<const double> lateral_track_station_meters,
    std::span<const double> lateral_displacement_meters,
    std::span<const double> vertical_track_station_meters,
    std::span<const double> vertical_displacement_meters)
    : lateral_displacement_(lateral_track_station_meters,
                            lateral_displacement_meters),
      vertical_displacement_(vertical_track_station_meters,
                             vertical_displacement_meters),
      lateral_start_track_station_meters_(
          lateral_track_station_meters.front()),
      lateral_end_track_station_meters_(lateral_track_station_meters.back()),
      vertical_start_track_station_meters_(
          vertical_track_station_meters.front()),
      vertical_end_track_station_meters_(vertical_track_station_meters.back()) {
}

double TrackIrregularityField::LateralDisplacementMeters(
    double track_station_meters) const {
    if (track_station_meters < lateral_start_track_station_meters_ ||
        track_station_meters > lateral_end_track_station_meters_) {
        return 0.0;
    }
    return lateral_displacement_.Evaluate(track_station_meters);
}

double TrackIrregularityField::VerticalDisplacementMeters(
    double track_station_meters) const {
    if (track_station_meters < vertical_start_track_station_meters_ ||
        track_station_meters > vertical_end_track_station_meters_) {
        return 0.0;
    }
    return vertical_displacement_.Evaluate(track_station_meters);
}

double TrackIrregularityField::LateralSlopeMetersPerMeter(
    double track_station_meters) const {
    if (track_station_meters < lateral_start_track_station_meters_ ||
        track_station_meters > lateral_end_track_station_meters_) {
        return 0.0;
    }
    return lateral_displacement_.EvaluateFirstDerivative(track_station_meters);
}

double TrackIrregularityField::VerticalSlopeMetersPerMeter(
    double track_station_meters) const {
    if (track_station_meters < vertical_start_track_station_meters_ ||
        track_station_meters > vertical_end_track_station_meters_) {
        return 0.0;
    }
    return vertical_displacement_.EvaluateFirstDerivative(track_station_meters);
}

}  // namespace orvd::wheel_rail_contact
