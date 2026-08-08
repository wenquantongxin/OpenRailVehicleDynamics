#pragma once

#include <span>

#include "orvd/wheel_rail_contact/natural_cubic_spline.h"

// A resolved lateral/vertical track-irregularity field over track station.
//
// This value object owns two independent natural cubic splines.  The lateral
// and vertical series need not share knots, a domain or a sample count: each is
// an independently measured station field.  Their roles are fixed by the
// constructor arguments rather than inferred from their numerical content.
//
// Displacements are expressed in the track frame, whose lateral axis points to
// the right and whose vertical axis points downward.  The independent variable
// is the same planar-projected track station used by TrackGeometry.  Outside a
// series' own knot range its displacement is held at the nearest endpoint and
// its slope is zero, exactly as specified by NaturalCubicSpline.

namespace orvd::wheel_rail_contact {

class TrackIrregularityField {
   public:
    // Builds and owns both station series.
    //
    // Each station/displacement pair is validated independently by
    // NaturalCubicSpline.  In particular, each pair must have equal lengths,
    // at least two finite samples and strictly increasing finite stations.
    // There is deliberately no cross-channel grid or domain requirement.
    TrackIrregularityField(
        std::span<const double> lateral_track_station_meters,
        std::span<const double> lateral_displacement_meters,
        std::span<const double> vertical_track_station_meters,
        std::span<const double> vertical_displacement_meters);

    [[nodiscard]] double LateralDisplacementMeters(
        double track_station_meters) const;
    [[nodiscard]] double VerticalDisplacementMeters(
        double track_station_meters) const;
    [[nodiscard]] double LateralSlopeMetersPerMeter(
        double track_station_meters) const;
    [[nodiscard]] double VerticalSlopeMetersPerMeter(
        double track_station_meters) const;

   private:
    NaturalCubicSpline lateral_displacement_;
    NaturalCubicSpline vertical_displacement_;
};

}  // namespace orvd::wheel_rail_contact
