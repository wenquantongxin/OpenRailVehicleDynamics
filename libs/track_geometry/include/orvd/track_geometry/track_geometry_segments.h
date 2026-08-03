#pragma once

#include <cstddef>
#include <optional>
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
    // The segment whose end the window straddles; the window is centred on the
    // boundary between it and the segment that follows. The seam is identified
    // by topology rather than by a written-out station, so nothing has to match
    // an accumulated sum of segment lengths under exact floating-point
    // equality.
    std::size_t preceding_segment_index{0};
    // The full width of the window, centred on that boundary.
    double window_length_meters{0.0};
};

class TrackScalarProfile {
   public:
    // Adjacent segments must agree at the boundary between them unless a seam
    // window is declared across it: a value that jumps leaves the first station
    // derivative of everything built on this profile undefined there, and an
    // evaluation on the right-hand-side path would have to return a number for
    // a quantity that does not exist.
    //
    // Throws std::invalid_argument on every declaration this profile cannot
    // stand behind, in four groups:
    //
    //   * the profile has no segments, so no domain to evaluate on;
    //   * a stated number is not finite, a segment length is not positive, a
    //     window length is not positive, or a segment shape is not one this
    //     library knows;
    //   * a seam names a segment with no successor, two seams name the same
    //     segment, a window reaches past either adjacent segment, two windows
    //     overlap, or an undeclared boundary is discontinuous;
    //   * the declaration is representable one number at a time but not as a
    //     whole in binary64 — a segment whose end station does not land
    //     strictly after its start, a window too narrow to have two distinct
    //     endpoints, or a coefficient or accumulated integral that comes out
    //     non-finite. These last are refused rather than rounded into
    //     something plausible.
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
    // Borrow only from a named, live profile. Moving the breakpoints out would
    // leave station bounds that no longer resolve into pieces; borrowing from
    // an expiring profile would return a dangling reference.
    [[nodiscard]] const std::vector<double>& breakpoint_track_stations_meters()
        const & {
        return breakpoints_;
    }
    [[nodiscard]] const std::vector<double>& breakpoint_track_stations_meters()
        && = delete;
    [[nodiscard]] const std::vector<double>& breakpoint_track_stations_meters()
        const && = delete;

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
    void ShortenDomainEndTo(double end_track_station_meters);
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
