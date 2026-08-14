#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

#include "orvd/track_geometry/track_geometry_segments.h"

// A self-contained vertical profile along planar-projected track station.
//
// The named segment types are physical definitions, not interpolation choices:
// constant grade, grade-linear parabolic vertical curve, and a strict projected
// circular vertical curve.  Every admitted segment supplies analytic grade,
// first and second station derivatives, and grade integral.  Declared seam
// windows replace the raw grade with a quintic matching value and the first two
// derivatives at both window ends.

namespace orvd::track_geometry {

class TrackGeometry;

struct ConstantGradeSegment {
    double length_meters{0.0};
    double centerline_upward_grade{0.0};
};

struct ParabolicVerticalCurveSegment {
    double length_meters{0.0};
    double start_centerline_upward_grade{0.0};
    double end_centerline_upward_grade{0.0};
};

struct CircularVerticalCurveSegment {
    double length_meters{0.0};
    double start_centerline_upward_grade{0.0};
    double end_centerline_upward_grade{0.0};
};

using TrackVerticalSegment =
    std::variant<ConstantGradeSegment, ParabolicVerticalCurveSegment,
                 CircularVerticalCurveSegment>;

class TrackVerticalProfile {
   public:
    // Segment lengths are accumulated from start_track_station_meters.  An
    // undeclared boundary must have exactly matching grades; a declared seam
    // may bridge different raw grades.  PL2 and CIR segments with equal endpoint
    // grades are refused in favour of the unique constant-grade expression.
    //
    // Throws std::invalid_argument for empty, non-finite, non-positive,
    // discontinuous, degenerate, unrepresentable or invalid-seam input.
    TrackVerticalProfile(
        double start_track_station_meters,
        std::vector<TrackVerticalSegment> segments,
        std::vector<TrackSeamTransition> seam_transitions);

    [[nodiscard]] double start_track_station_meters() const {
        return start_track_station_meters_;
    }
    [[nodiscard]] double end_track_station_meters() const {
        return end_track_station_meters_;
    }

    // The domain is strict: these functions neither clamp nor extrapolate.
    // At an unseamed internal boundary, derivative accessors return the
    // right-hand value; at the complete-profile endpoint, they return the
    // left-hand value.  If the two sides differ, no two-sided derivative
    // exists and the returned derivative is explicitly one-sided.
    [[nodiscard]] double Value(double track_station_meters) const;
    [[nodiscard]] double FirstDerivativePerMeter(
        double track_station_meters) const;
    [[nodiscard]] double SecondDerivativePerMeterSquared(
        double track_station_meters) const;
    [[nodiscard]] double IntegralFromStart(double track_station_meters) const;

    // The first raw or seam piece that is not identically zero.  No value means
    // that the complete profile is level.
    [[nodiscard]] std::optional<double> support_start_track_station_meters()
        const {
        return support_start_track_station_meters_;
    }

   private:
    friend class TrackGeometry;

    enum class RawSegmentShape {
        kConstantGrade,
        kParabolicVerticalCurve,
        kCircularVerticalCurve,
    };

    struct RawSegment {
        RawSegmentShape shape{RawSegmentShape::kConstantGrade};
        double start_track_station_meters{0.0};
        double end_track_station_meters{0.0};
        double length_meters{0.0};
        double start_grade{0.0};
        double end_grade{0.0};
        double grade_change_per_meter{0.0};
        double start_sine{0.0};
        double end_sine{0.0};
        double start_cosine{1.0};
        double end_cosine{1.0};
        double inverse_radius_per_meter{0.0};
    };

    enum class PieceShape {
        kRawSegment,
        kQuinticSeam,
    };

    struct Piece {
        PieceShape shape{PieceShape::kRawSegment};
        double start_track_station_meters{0.0};
        double end_track_station_meters{0.0};
        std::size_t raw_segment_index{0};
        double polynomial_origin_track_station_meters{0.0};
        std::array<double, 6> polynomial_coefficients{};
    };

    struct Sample {
        double value{0.0};
        double first_derivative_per_meter{0.0};
        double second_derivative_per_meter_squared{0.0};
    };

    [[nodiscard]] const std::vector<double>& breakpoint_track_stations_meters()
        const {
        return breakpoints_;
    }
    void ShortenDomainEndTo(double end_track_station_meters);
    void ThrowIfOutsideDomain(double track_station_meters) const;
    [[nodiscard]] std::size_t PieceIndexAt(double track_station_meters) const;
    [[nodiscard]] Sample EvaluateRawSegment(const RawSegment& segment,
                                            double track_station_meters) const;
    [[nodiscard]] double RawIntegralFromSegmentStart(
        const RawSegment& segment, double track_station_meters) const;
    [[nodiscard]] Sample EvaluatePiece(const Piece& piece,
                                       double track_station_meters) const;
    [[nodiscard]] double PieceIntegralFromOrigin(
        const Piece& piece, double track_station_meters) const;

    double start_track_station_meters_{0.0};
    double end_track_station_meters_{0.0};
    std::vector<RawSegment> raw_segments_;
    std::vector<Piece> pieces_;
    std::vector<double> breakpoints_;
    std::vector<double> integral_at_piece_start_;
    std::optional<double> support_start_track_station_meters_;
};

}  // namespace orvd::track_geometry
