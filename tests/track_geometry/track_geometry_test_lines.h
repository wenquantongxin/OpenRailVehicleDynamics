#pragma once

#include <vector>

#include "orvd/track_geometry/track_geometry.h"

// Lines the gates are evaluated on.
//
// Every gate that talks about an orientation or a derivative needs curvature,
// superelevation, grade and heading to be non-zero at the same station. With
// any one of them zero the corresponding assertion collapses into an identity:
// at zero heading the two possible construction orders of the track frame
// commute, at zero grade the tangent is already horizontal so normalising it
// changes nothing, and at zero superelevation the roll is the identity. The
// canonical line therefore carries all four at once.

namespace orvd::track_geometry::test_lines {

inline constexpr double kCanonicalRadiusMeters = 300.0;
inline constexpr double kCanonicalSuperelevationMeters = 0.12;
inline constexpr double kCanonicalGrade = 0.02;
inline constexpr double kRailReferenceLateralSpanMeters = 1.5;
inline constexpr double kNodeSpacingMeters = 0.5;

inline constexpr double kCanonicalStraightLengthMeters = 50.0;
inline constexpr double kCanonicalTransitionLengthMeters = 50.0;
inline constexpr double kCanonicalCircularLengthMeters = 200.0;
// Stations of the canonical curvature layout.
inline constexpr double kCanonicalTransitionStartMeters = 50.0;
inline constexpr double kCanonicalCircularStartMeters = 100.0;
inline constexpr double kCanonicalCircularEndMeters = 300.0;

inline TrackScalarSegment Constant(double length_meters, double value) {
    TrackScalarSegment segment;
    segment.length_meters = length_meters;
    segment.shape = TrackScalarSegmentShape::kConstant;
    segment.start_value = value;
    segment.end_value = value;
    return segment;
}

inline TrackScalarSegment Blend(double length_meters, double start_value,
                                double end_value) {
    TrackScalarSegment segment;
    segment.length_meters = length_meters;
    segment.shape = TrackScalarSegmentShape::kHermiteCubicBlend;
    segment.start_value = start_value;
    segment.end_value = end_value;
    return segment;
}

// Straight, transition, circular, transition, straight; superelevation and
// grade ramp in over the first transition and stay on. Right-hand curve, so the
// curvature is positive.
inline TrackGeometry MakeCanonicalLine() {
    const double curvature = 1.0 / kCanonicalRadiusMeters;
    TrackScalarProfile curvature_profile(
        0.0,
        {Constant(kCanonicalStraightLengthMeters, 0.0),
         Blend(kCanonicalTransitionLengthMeters, 0.0, curvature),
         Constant(kCanonicalCircularLengthMeters, curvature),
         Blend(kCanonicalTransitionLengthMeters, curvature, 0.0),
         Constant(kCanonicalStraightLengthMeters, 0.0)},
        {});
    TrackScalarProfile superelevation_profile(
        0.0,
        {Constant(kCanonicalStraightLengthMeters, 0.0),
         Blend(kCanonicalTransitionLengthMeters, 0.0,
               kCanonicalSuperelevationMeters),
         Constant(kCanonicalCircularLengthMeters,
                  kCanonicalSuperelevationMeters),
         Blend(kCanonicalTransitionLengthMeters, kCanonicalSuperelevationMeters,
               0.0),
         Constant(kCanonicalStraightLengthMeters, 0.0)},
        {});
    TrackScalarProfile grade_profile(
        0.0,
        {Constant(kCanonicalStraightLengthMeters, 0.0),
         Blend(kCanonicalTransitionLengthMeters, 0.0, kCanonicalGrade),
         Constant(300.0, kCanonicalGrade)},
        {});
    return TrackGeometry(std::move(curvature_profile),
                         std::move(superelevation_profile),
                         std::move(grade_profile),
                         kRailReferenceLateralSpanMeters, kNodeSpacingMeters);
}

// Everything constant and everything large: the grade is exaggerated so that a
// wrong pitch sign or a missing normalisation shows up as a first-order error
// rather than as something of order grade squared over two.
inline TrackGeometry MakeSteepConstantLine(double grade,
                                           double superelevation_meters) {
    const double curvature = 1.0 / kCanonicalRadiusMeters;
    TrackScalarProfile curvature_profile(0.0, {Constant(300.0, curvature)}, {});
    TrackScalarProfile superelevation_profile(
        0.0, {Constant(300.0, superelevation_meters)}, {});
    TrackScalarProfile grade_profile(0.0, {Constant(300.0, grade)}, {});
    return TrackGeometry(std::move(curvature_profile),
                         std::move(superelevation_profile),
                         std::move(grade_profile),
                         kRailReferenceLateralSpanMeters, kNodeSpacingMeters);
}

// A level straight stretch of stated length and node spacing. Used to put a
// projection root where a scan over interior nodes cannot see it: in the first
// or last half interval, exactly between two nodes, or on a line short enough
// that the grid has no interior node at all.
inline TrackGeometry MakeLevelStraightLine(double length_meters,
                                           double node_spacing_meters) {
    return TrackGeometry(
        TrackScalarProfile(0.0, {Constant(length_meters, 0.0)}, {}),
        TrackScalarProfile(0.0, {Constant(length_meters, 0.0)}, {}),
        TrackScalarProfile(0.0, {Constant(length_meters, 0.0)}, {}),
        kRailReferenceLateralSpanMeters, node_spacing_meters);
}

// A straight stretch meeting a circular one with a declared seam window across
// the boundary. Outside the window the circular curvature is exactly the
// reciprocal of the radius; inside it the quintic runs.
inline constexpr double kSeamBoundaryMeters = 100.0;
inline constexpr double kSeamWindowLengthMeters = 6.0;

inline TrackGeometry MakeSeamLine() {
    const double curvature = 1.0 / kCanonicalRadiusMeters;
    TrackSeamTransition seam;
    seam.boundary_track_station_meters = kSeamBoundaryMeters;
    seam.window_length_meters = kSeamWindowLengthMeters;
    TrackScalarProfile curvature_profile(
        0.0,
        {Constant(kSeamBoundaryMeters, 0.0), Constant(200.0, curvature)},
        {seam});
    TrackScalarProfile superelevation_profile(
        0.0, {Constant(300.0, 0.0)}, {});
    TrackScalarProfile grade_profile(0.0, {Constant(300.0, 0.0)}, {});
    return TrackGeometry(std::move(curvature_profile),
                         std::move(superelevation_profile),
                         std::move(grade_profile),
                         kRailReferenceLateralSpanMeters, kNodeSpacingMeters);
}

inline constexpr double kTieRadiusMeters = 10.0;
inline constexpr double kTieStraightLengthMeters = 60.0;

// A tight circular turn, used to exercise the node-spacing resolution bound:
// at ten metres of radius a half-metre spacing turns the heading by a twentieth
// of a radian per interval, while fifty metres would sweep several radians.
inline TrackGeometry MakeTightTurnLine(double node_spacing_meters) {
    const double curvature = 1.0 / kTieRadiusMeters;
    return TrackGeometry(TrackScalarProfile(0.0, {Constant(100.0, curvature)},
                                            {}),
                         TrackScalarProfile(0.0, {Constant(100.0, 0.0)}, {}),
                         TrackScalarProfile(0.0, {Constant(100.0, 0.0)}, {}),
                         kRailReferenceLateralSpanMeters, node_spacing_meters);
}

// Straight, half turn, straight back: the two straight stretches are parallel
// and a lateral span apart, so a point halfway between them is exactly as close
// to one as to the other and the nearest station stops being a function of the
// point.
inline TrackGeometry MakeEquidistantTieLine() {
    const double curvature = 1.0 / kTieRadiusMeters;
    const double half_turn_length = 3.14159265358979323846 * kTieRadiusMeters;
    TrackScalarProfile curvature_profile(
        0.0,
        {Constant(kTieStraightLengthMeters, 0.0),
         Constant(half_turn_length, curvature),
         Constant(kTieStraightLengthMeters, 0.0)},
        {});
    const double total =
        2.0 * kTieStraightLengthMeters + half_turn_length;
    TrackScalarProfile superelevation_profile(0.0, {Constant(total, 0.0)}, {});
    TrackScalarProfile grade_profile(0.0, {Constant(total, 0.0)}, {});
    return TrackGeometry(std::move(curvature_profile),
                         std::move(superelevation_profile),
                         std::move(grade_profile),
                         kRailReferenceLateralSpanMeters, 0.05);
}

}  // namespace orvd::track_geometry::test_lines
