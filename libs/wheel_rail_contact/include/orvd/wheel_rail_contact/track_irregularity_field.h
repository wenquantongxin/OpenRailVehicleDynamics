#pragma once

#include <optional>
#include <span>

#include "orvd/wheel_rail_contact/natural_cubic_spline.h"

// A resolved lateral/vertical/gauge/roll field over track station.
//
// Each channel owns a natural cubic spline with an independent station grid
// and domain. Constructor arguments assign channel roles.
//
// Displacements are expressed in the track frame, whose lateral axis points to
// the right and whose vertical axis points downward. The independent variable
// is the same planar-projected track station used by TrackGeometry. Each
// series' first and last knots are its explicit activation boundaries; strictly
// outside them both the channel value and slope are zero.

namespace orvd::wheel_rail_contact {

class TrackIrregularityField {
   public:
    // Builds and owns both station series.
    //
    // Each station/displacement pair is validated independently by
    // NaturalCubicSpline.  In particular, each pair must have equal lengths,
    // at least two finite samples and strictly increasing finite stations.
    TrackIrregularityField(
        std::span<const double> lateral_track_station_meters,
        std::span<const double> lateral_displacement_meters,
        std::span<const double> vertical_track_station_meters,
        std::span<const double> vertical_displacement_meters);

    // Gauge is the total narrowing of the two rails (positive narrows), in m.
    // Roll is about track +x, in rad (positive lowers the right rail). An empty
    // station/value pair denotes an absent gauge or roll channel, identically
    // zero. The four-argument constructor leaves both channels absent.
    TrackIrregularityField(
        std::span<const double> lateral_track_station_meters,
        std::span<const double> lateral_displacement_meters,
        std::span<const double> vertical_track_station_meters,
        std::span<const double> vertical_displacement_meters,
        std::span<const double> gauge_track_station_meters,
        std::span<const double> gauge_narrowing_meters,
        std::span<const double> roll_track_station_meters,
        std::span<const double> roll_radians);

    [[nodiscard]] double LateralDisplacementMeters(
        double track_station_meters) const;
    [[nodiscard]] double VerticalDisplacementMeters(
        double track_station_meters) const;
    [[nodiscard]] double LateralSlopeMetersPerMeter(
        double track_station_meters) const;
    [[nodiscard]] double VerticalSlopeMetersPerMeter(
        double track_station_meters) const;
    [[nodiscard]] double GaugeNarrowingMeters(double track_station_meters) const;
    [[nodiscard]] double GaugeSlopeMetersPerMeter(
        double track_station_meters) const;
    [[nodiscard]] double RollRadians(double track_station_meters) const;
    [[nodiscard]] double RollSlopeRadiansPerMeter(
        double track_station_meters) const;

   private:
    NaturalCubicSpline lateral_displacement_;
    NaturalCubicSpline vertical_displacement_;
    double lateral_start_track_station_meters_{0.0};
    double lateral_end_track_station_meters_{0.0};
    double vertical_start_track_station_meters_{0.0};
    double vertical_end_track_station_meters_{0.0};
    std::optional<NaturalCubicSpline> gauge_narrowing_;
    std::optional<NaturalCubicSpline> roll_;
    double gauge_start_track_station_meters_{0.0};
    double gauge_end_track_station_meters_{0.0};
    double roll_start_track_station_meters_{0.0};
    double roll_end_track_station_meters_{0.0};
};

}  // namespace orvd::wheel_rail_contact
