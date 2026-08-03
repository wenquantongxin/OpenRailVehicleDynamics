#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

// One scalar profile along the line: curvature, superelevation or grade.
//
// A profile is a sequence of segments laid end to end from a start station.
// Each segment either holds its value constant or blends from one value to
// another through the Hermite cubic 3x^2 - 2x^3 of its normalised local
// coordinate. That blend is the transition curve of this library: its
// derivative vanishes at both ends and its second derivative jumps there. It is
// deliberately not a clothoid, whose curvature is linear in station and whose
// derivative is therefore non-zero at the ends.
//
// Adjacent segments may additionally declare a seam transition: a window of
// stated length, centred on their shared boundary, inside which the profile
// follows the quintic that matches value, first and second derivative at both
// window ends. A seam transition is a declared interval of the geometry, not a
// silent rewrite applied after construction, and any statement about a
// segment's own value applies to the part of that segment outside the declared
// windows.

namespace orvd::track_geometry {

class TrackGeometry;

enum class TrackScalarSegmentShape {
    // The value holds at start_value for the whole segment.
    kConstant,
    // The value blends from start_value to end_value through 3x^2 - 2x^3.
    kHermiteCubicBlend,
};

struct TrackScalarSegment {
    double length_meters{0.0};
    TrackScalarSegmentShape shape{TrackScalarSegmentShape::kConstant};
    double start_value{0.0};
    // Ignored for kConstant, where the segment holds start_value throughout.
    double end_value{0.0};
};

struct TrackSeamTransition {
    // The shared boundary of two adjacent segments, in track station.
    double boundary_track_station_meters{0.0};
    // The full width of the window, centred on that boundary.
    double window_length_meters{0.0};
};

class TrackScalarProfile {
   public:
    // Throws std::invalid_argument when the start station or any segment or
    // window parameter is not finite, when a segment has non-positive length,
    // when a seam boundary is not an interior segment boundary, when a window
    // reaches past either adjacent segment, or when two windows overlap.
    TrackScalarProfile(double start_track_station_meters,
                       std::vector<TrackScalarSegment> segments,
                       std::vector<TrackSeamTransition> seam_transitions);

    [[nodiscard]] double start_track_station_meters() const {
        return start_track_station_meters_;
    }
    [[nodiscard]] double end_track_station_meters() const {
        return end_track_station_meters_;
    }

    // Throws std::invalid_argument when the station is not finite or lies
    // outside [start, end]. The domain is a contract, not a hint: this profile
    // neither extrapolates nor clamps.
    [[nodiscard]] double Value(double track_station_meters) const;
    [[nodiscard]] double FirstDerivativePerMeter(
        double track_station_meters) const;
    [[nodiscard]] double SecondDerivativePerMeterSquared(
        double track_station_meters) const;

    // The exact integral from the start station to the given station. Every
    // admitted shape is a polynomial, so this is analytic rather than quadrature.
    [[nodiscard]] double IntegralFromStart(double track_station_meters) const;

    // The first station at or after which the profile is not identically zero,
    // or no value when it is identically zero everywhere. A start-up domain
    // contract has to be able to ask where a quantity begins to act, so this is
    // a declared property rather than a search over samples.
    [[nodiscard]] std::optional<double> support_start_track_station_meters()
        const {
        return support_start_track_station_meters_;
    }

    // The stations at which the piecewise definition changes: segment
    // boundaries and seam window edges, with the domain end appended.
    [[nodiscard]] const std::vector<double>& breakpoint_track_stations_meters()
        const & {
        return breakpoints_;
    }
    [[nodiscard]] std::vector<double> breakpoint_track_stations_meters() && {
        return std::move(breakpoints_);
    }
    [[nodiscard]] std::vector<double> breakpoint_track_stations_meters()
        const && {
        return breakpoints_;
    }

    // Whether the piece containing the station is a declared seam window.
    [[nodiscard]] bool IsInsideSeamWindow(double track_station_meters) const;

    // The degree of the local polynomial, with trailing zero coefficients
    // trimmed. A degree of zero says the profile is locally constant, which is
    // what lets a centerline integrate in closed form over that stretch instead
    // of falling back on quadrature.
    [[nodiscard]] std::size_t LocalPolynomialDegree(
        double track_station_meters) const;

   private:
    friend class TrackGeometry;

    struct ProfileExtremum {
        double value{0.0};
        double track_station_meters{0.0};
    };

    struct Piece {
        double start_track_station_meters{0.0};
        double end_track_station_meters{0.0};
        // The station at which the local polynomial coordinate is zero. Cutting
        // a segment at a window edge keeps the segment's own origin instead of
        // re-expanding the polynomial about the cut, which would trade exact
        // coefficients for rounding.
        double polynomial_origin_track_station_meters{0.0};
        // Ascending coefficients of the local coordinate. A constant piece uses
        // one term, a Hermite blend four, a seam window six.
        double coefficients[6]{};
        std::size_t coefficient_count{1};
        bool is_seam_window{false};
    };

    [[nodiscard]] std::size_t PieceIndexAt(double track_station_meters) const;
    [[nodiscard]] ProfileExtremum MaximumAbsoluteValue() const;
    void ThrowIfOutsideDomain(double track_station_meters) const;

    double start_track_station_meters_{0.0};
    double end_track_station_meters_{0.0};
    std::vector<Piece> pieces_;
    std::vector<double> breakpoints_;
    // Integral of the profile from the start station to each piece start.
    std::vector<double> integral_at_piece_start_;
    std::optional<double> support_start_track_station_meters_;
};

}  // namespace orvd::track_geometry
