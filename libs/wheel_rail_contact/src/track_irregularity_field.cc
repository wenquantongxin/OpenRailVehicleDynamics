#include "orvd/wheel_rail_contact/track_irregularity_field.h"

#include <stdexcept>

namespace orvd::wheel_rail_contact {

TrackIrregularityField::TrackIrregularityField(
    std::span<const double> lateral_track_station_meters,
    std::span<const double> lateral_displacement_meters,
    std::span<const double> vertical_track_station_meters,
    std::span<const double> vertical_displacement_meters,
    std::span<const double> gauge_track_station_meters,
    std::span<const double> gauge_narrowing_meters,
    std::span<const double> roll_track_station_meters,
    std::span<const double> roll_radians)
    : TrackIrregularityField(lateral_track_station_meters,
                            lateral_displacement_meters,
                            vertical_track_station_meters,
                            vertical_displacement_meters) {
    if (gauge_track_station_meters.empty() != gauge_narrowing_meters.empty() ||
        roll_track_station_meters.empty() != roll_radians.empty()) {
        throw std::invalid_argument(
            "an absent gauge or roll channel needs both spans empty");
    }
    if (!gauge_track_station_meters.empty()) {
        gauge_narrowing_.emplace(gauge_track_station_meters,
                                gauge_narrowing_meters);
        gauge_start_track_station_meters_ = gauge_track_station_meters.front();
        gauge_end_track_station_meters_ = gauge_track_station_meters.back();
    }
    if (!roll_track_station_meters.empty()) {
        roll_.emplace(roll_track_station_meters, roll_radians);
        roll_start_track_station_meters_ = roll_track_station_meters.front();
        roll_end_track_station_meters_ = roll_track_station_meters.back();
    }
}

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

double TrackIrregularityField::GaugeNarrowingMeters(double station) const {
    if (!gauge_narrowing_ || station < gauge_start_track_station_meters_ ||
        station > gauge_end_track_station_meters_) {
        return 0.0;
    }
    return gauge_narrowing_->Evaluate(station);
}

double TrackIrregularityField::GaugeSlopeMetersPerMeter(double station) const {
    if (!gauge_narrowing_ || station < gauge_start_track_station_meters_ ||
        station > gauge_end_track_station_meters_) {
        return 0.0;
    }
    return gauge_narrowing_->EvaluateFirstDerivative(station);
}

double TrackIrregularityField::RollRadians(double station) const {
    if (!roll_ || station < roll_start_track_station_meters_ ||
        station > roll_end_track_station_meters_) {
        return 0.0;
    }
    return roll_->Evaluate(station);
}

double TrackIrregularityField::RollSlopeRadiansPerMeter(double station) const {
    if (!roll_ || station < roll_start_track_station_meters_ ||
        station > roll_end_track_station_meters_) {
        return 0.0;
    }
    return roll_->EvaluateFirstDerivative(station);
}

}  // namespace orvd::wheel_rail_contact
