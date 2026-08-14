#include "orvd/track_geometry/track_vertical_profile.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "track_profile_quintic.h"

namespace orvd::track_geometry {
namespace {

std::string Describe(double value) { return std::to_string(value); }

void RequireFinite(double value, const std::string& what) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument("TrackVerticalProfile: " + what +
                                    " must be finite, got " + Describe(value));
    }
}

double EvaluatePolynomial(const std::array<double, 6>& coefficients,
                          double local) {
    double result = 0.0;
    for (std::size_t index = coefficients.size(); index > 0; --index) {
        result = result * local + coefficients[index - 1];
    }
    return result;
}

double EvaluateFirstDerivative(const std::array<double, 6>& coefficients,
                               double local) {
    double result = 0.0;
    for (std::size_t index = coefficients.size(); index > 1; --index) {
        result = result * local +
                 coefficients[index - 1] * static_cast<double>(index - 1);
    }
    return result;
}

double EvaluateSecondDerivative(const std::array<double, 6>& coefficients,
                                double local) {
    double result = 0.0;
    for (std::size_t index = coefficients.size(); index > 2; --index) {
        const double order = static_cast<double>(index - 1);
        result = result * local +
                 coefficients[index - 1] * order * (order - 1.0);
    }
    return result;
}

double EvaluateAntiderivative(const std::array<double, 6>& coefficients,
                              double local) {
    double result = 0.0;
    for (std::size_t index = coefficients.size(); index > 0; --index) {
        result = result * local +
                 coefficients[index - 1] / static_cast<double>(index);
    }
    return result * local;
}

bool IsPolynomialIdenticallyZero(
    const std::array<double, 6>& coefficients) {
    return std::all_of(coefficients.begin(), coefficients.end(),
                       [](double coefficient) { return coefficient == 0.0; });
}

struct SeamWindow {
    std::size_t left_segment_index{0};
    double start_track_station_meters{0.0};
    double end_track_station_meters{0.0};
};

}  // namespace

TrackVerticalProfile::TrackVerticalProfile(
    double start_track_station_meters,
    std::vector<TrackVerticalSegment> segments,
    std::vector<TrackSeamTransition> seam_transitions)
    : start_track_station_meters_(start_track_station_meters) {
    RequireFinite(start_track_station_meters, "the start track station");
    if (segments.empty()) {
        throw std::invalid_argument(
            "TrackVerticalProfile: at least one segment is required; an empty "
            "profile has no station domain");
    }

    raw_segments_.reserve(segments.size());
    double station = start_track_station_meters;
    for (std::size_t index = 0; index < segments.size(); ++index) {
        const TrackVerticalSegment& declaration = segments[index];
        if (declaration.valueless_by_exception()) {
            throw std::invalid_argument(
                "TrackVerticalProfile: segment " + std::to_string(index) +
                " has no active vertical segment type");
        }

        RawSegment raw;
        raw.start_track_station_meters = station;
        const std::string where = "segment " + std::to_string(index);
        std::visit(
            [&raw, &where](const auto& segment) {
                using Segment = std::decay_t<decltype(segment)>;
                raw.length_meters = segment.length_meters;
                RequireFinite(raw.length_meters, "the length of " + where);
                if constexpr (std::is_same_v<Segment, ConstantGradeSegment>) {
                    raw.shape = RawSegmentShape::kConstantGrade;
                    raw.start_grade = segment.centerline_upward_grade;
                    raw.end_grade = segment.centerline_upward_grade;
                } else if constexpr (
                    std::is_same_v<Segment,
                                   ParabolicVerticalCurveSegment>) {
                    raw.shape = RawSegmentShape::kParabolicVerticalCurve;
                    raw.start_grade =
                        segment.start_centerline_upward_grade;
                    raw.end_grade = segment.end_centerline_upward_grade;
                } else {
                    static_assert(std::is_same_v<
                                  Segment, CircularVerticalCurveSegment>);
                    raw.shape = RawSegmentShape::kCircularVerticalCurve;
                    raw.start_grade =
                        segment.start_centerline_upward_grade;
                    raw.end_grade = segment.end_centerline_upward_grade;
                }
            },
            declaration);

        RequireFinite(raw.start_grade, "the start grade of " + where);
        RequireFinite(raw.end_grade, "the end grade of " + where);
        if (!(raw.length_meters > 0.0)) {
            throw std::invalid_argument(
                "TrackVerticalProfile: the length of " + where +
                " must be positive, got " + Describe(raw.length_meters));
        }
        raw.end_track_station_meters = station + raw.length_meters;
        if (!std::isfinite(raw.end_track_station_meters) ||
            !(raw.end_track_station_meters > station)) {
            throw std::invalid_argument(
                "TrackVerticalProfile: the endpoint of " + where +
                " is not a finite representable station strictly after " +
                Describe(station) + " m");
        }

        if (raw.shape != RawSegmentShape::kConstantGrade) {
            if (raw.start_grade == raw.end_grade) {
                throw std::invalid_argument(
                    "TrackVerticalProfile: " + where +
                    " has equal endpoint grades; use ConstantGradeSegment "
                    "for the unique non-degenerate representation");
            }
        }
        if (raw.shape == RawSegmentShape::kParabolicVerticalCurve) {
            const double change = raw.end_grade - raw.start_grade;
            raw.grade_change_per_meter = change / raw.length_meters;
            if (!std::isfinite(raw.grade_change_per_meter) ||
                raw.grade_change_per_meter == 0.0) {
                throw std::invalid_argument(
                    "TrackVerticalProfile: the grade rate of " + where +
                    " is not a finite non-zero binary64 value for the "
                    "declared endpoints and length");
            }
        }
        if (raw.shape == RawSegmentShape::kCircularVerticalCurve) {
            const double start_norm = std::hypot(1.0, raw.start_grade);
            const double end_norm = std::hypot(1.0, raw.end_grade);
            raw.start_sine = raw.start_grade / start_norm;
            raw.end_sine = raw.end_grade / end_norm;
            raw.start_cosine = 1.0 / start_norm;
            raw.end_cosine = 1.0 / end_norm;
            if (!(std::abs(raw.start_sine) < 1.0) ||
                !(std::abs(raw.end_sine) < 1.0) ||
                !(raw.start_cosine > 0.0) || !(raw.end_cosine > 0.0)) {
                throw std::invalid_argument(
                    "TrackVerticalProfile: the endpoint inclination of " +
                    where +
                    " cannot be represented with finite circular derivatives");
            }
            raw.inverse_radius_per_meter =
                (raw.end_sine - raw.start_sine) / raw.length_meters;
            if (!std::isfinite(raw.inverse_radius_per_meter) ||
                raw.inverse_radius_per_meter == 0.0) {
                throw std::invalid_argument(
                    "TrackVerticalProfile: the inverse radius of " + where +
                    " is not a finite non-zero binary64 value for the "
                    "declared endpoints and length");
            }
        }

        for (const double sample_station :
             {raw.start_track_station_meters,
              std::midpoint(raw.start_track_station_meters,
                            raw.end_track_station_meters),
              raw.end_track_station_meters}) {
            const Sample sample = EvaluateRawSegment(raw, sample_station);
            RequireFinite(sample.value, "a derived grade of " + where);
            RequireFinite(sample.first_derivative_per_meter,
                          "a derived first grade derivative of " + where);
            RequireFinite(sample.second_derivative_per_meter_squared,
                          "a derived second grade derivative of " + where);
        }
        RequireFinite(RawIntegralFromSegmentStart(
                          raw, raw.end_track_station_meters),
                      "the complete grade integral of " + where);

        raw_segments_.push_back(raw);
        station = raw.end_track_station_meters;
    }
    end_track_station_meters_ = station;

    std::vector<SeamWindow> windows;
    windows.reserve(seam_transitions.size());
    std::vector<bool> has_seam(raw_segments_.size(), false);
    for (std::size_t index = 0; index < seam_transitions.size(); ++index) {
        const TrackSeamTransition& seam = seam_transitions[index];
        const std::string where = "seam " + std::to_string(index);
        RequireFinite(seam.window_length_meters,
                      "the window length of " + where);
        if (!(seam.window_length_meters > 0.0)) {
            throw std::invalid_argument(
                "TrackVerticalProfile: " + where +
                " must have positive window length, got " +
                Describe(seam.window_length_meters));
        }
        const std::size_t left = seam.preceding_segment_index;
        if (left >= raw_segments_.size() - 1) {
            throw std::invalid_argument(
                "TrackVerticalProfile: " + where + " names segment " +
                std::to_string(left) +
                ", which has no following segment to form a boundary with");
        }
        if (has_seam[left]) {
            throw std::invalid_argument(
                "TrackVerticalProfile: " + where + " names segment " +
                std::to_string(left) +
                ", whose boundary already has a seam window");
        }
        has_seam[left] = true;

        const RawSegment& left_segment = raw_segments_[left];
        const RawSegment& right_segment = raw_segments_[left + 1];
        const double boundary = left_segment.end_track_station_meters;
        const double half_width = 0.5 * seam.window_length_meters;
        if (half_width > left_segment.length_meters ||
            half_width > right_segment.length_meters) {
            throw std::invalid_argument(
                "TrackVerticalProfile: half the window of " + where +
                " reaches past an adjacent segment");
        }
        SeamWindow window;
        window.left_segment_index = left;
        window.start_track_station_meters = boundary - half_width;
        window.end_track_station_meters = boundary + half_width;
        if (!std::isfinite(window.start_track_station_meters) ||
            !std::isfinite(window.end_track_station_meters) ||
            !(window.start_track_station_meters < boundary) ||
            !(boundary < window.end_track_station_meters)) {
            throw std::invalid_argument(
                "TrackVerticalProfile: " + where +
                " is too narrow to form representable endpoints around " +
                Describe(boundary) + " m");
        }
        windows.push_back(window);
    }

    for (std::size_t index = 0; index + 1 < raw_segments_.size(); ++index) {
        if (!has_seam[index] &&
            raw_segments_[index].end_grade !=
                raw_segments_[index + 1].start_grade) {
            throw std::invalid_argument(
                "TrackVerticalProfile: segment " + std::to_string(index) +
                " ends at grade " +
                Describe(raw_segments_[index].end_grade) + " while segment " +
                std::to_string(index + 1) + " starts at grade " +
                Describe(raw_segments_[index + 1].start_grade) +
                " with no seam declared across their boundary");
        }
    }

    std::sort(windows.begin(), windows.end(),
              [](const SeamWindow& first, const SeamWindow& second) {
                  return first.start_track_station_meters <
                         second.start_track_station_meters;
              });
    for (std::size_t index = 1; index < windows.size(); ++index) {
        if (windows[index].start_track_station_meters <
            windows[index - 1].end_track_station_meters) {
            throw std::invalid_argument(
                "TrackVerticalProfile: seam windows overlap between stations " +
                Describe(windows[index - 1].start_track_station_meters) +
                " m and " +
                Describe(windows[index].end_track_station_meters) + " m");
        }
    }

    const auto emit_raw_piece = [this](std::size_t raw_index, double from,
                                       double to) {
        Piece piece;
        piece.shape = PieceShape::kRawSegment;
        piece.start_track_station_meters = from;
        piece.end_track_station_meters = to;
        piece.raw_segment_index = raw_index;
        pieces_.push_back(piece);
    };

    std::size_t next_window = 0;
    double cursor = start_track_station_meters_;
    for (std::size_t index = 0; index < raw_segments_.size(); ++index) {
        while (next_window < windows.size() &&
               windows[next_window].left_segment_index == index) {
            const SeamWindow& window = windows[next_window];
            if (window.start_track_station_meters > cursor) {
                emit_raw_piece(index, cursor,
                               window.start_track_station_meters);
            }

            const Sample left = EvaluateRawSegment(
                raw_segments_[index], window.start_track_station_meters);
            const Sample right = EvaluateRawSegment(
                raw_segments_[index + 1], window.end_track_station_meters);
            Piece seam_piece;
            seam_piece.shape = PieceShape::kQuinticSeam;
            seam_piece.start_track_station_meters =
                window.start_track_station_meters;
            seam_piece.end_track_station_meters =
                window.end_track_station_meters;
            seam_piece.polynomial_origin_track_station_meters =
                window.start_track_station_meters;
            seam_piece.polynomial_coefficients =
                internal::BuildQuinticHermiteCoefficients(
                    window.end_track_station_meters -
                        window.start_track_station_meters,
                    {left.value, left.first_derivative_per_meter,
                     left.second_derivative_per_meter_squared},
                    {right.value, right.first_derivative_per_meter,
                     right.second_derivative_per_meter_squared});
            for (const double coefficient :
                 seam_piece.polynomial_coefficients) {
                if (!std::isfinite(coefficient)) {
                    throw std::invalid_argument(
                        "TrackVerticalProfile: a seam polynomial coefficient "
                        "is not finite for the declared window");
                }
            }
            pieces_.push_back(seam_piece);
            cursor = window.end_track_station_meters;
            ++next_window;
        }
        if (cursor < raw_segments_[index].end_track_station_meters) {
            emit_raw_piece(index, cursor,
                           raw_segments_[index].end_track_station_meters);
            cursor = raw_segments_[index].end_track_station_meters;
        }
    }

    integral_at_piece_start_.reserve(pieces_.size());
    double running_integral = 0.0;
    for (const Piece& piece : pieces_) {
        integral_at_piece_start_.push_back(running_integral);
        running_integral +=
            PieceIntegralFromOrigin(piece, piece.end_track_station_meters) -
            PieceIntegralFromOrigin(piece, piece.start_track_station_meters);
        if (!std::isfinite(running_integral)) {
            throw std::invalid_argument(
                "TrackVerticalProfile: the accumulated grade integral is not "
            "finite for the declared segments");
        }
    }

    // TrackGeometry will use every raw boundary and every seam edge as a node.
    // A raw boundary covered by a seam is not a formula switch, but retaining it
    // keeps the product's declared segment topology observable without changing
    // any analytic evaluation.
    breakpoints_.reserve(pieces_.size() + raw_segments_.size() + 1);
    for (const Piece& piece : pieces_) {
        breakpoints_.push_back(piece.start_track_station_meters);
    }
    for (const RawSegment& raw : raw_segments_) {
        breakpoints_.push_back(raw.end_track_station_meters);
    }
    std::sort(breakpoints_.begin(), breakpoints_.end());
    breakpoints_.erase(
        std::unique(breakpoints_.begin(), breakpoints_.end()),
        breakpoints_.end());

    for (const Piece& piece : pieces_) {
        bool identically_zero = false;
        if (piece.shape == PieceShape::kRawSegment) {
            const RawSegment& raw = raw_segments_[piece.raw_segment_index];
            identically_zero =
                raw.shape == RawSegmentShape::kConstantGrade &&
                raw.start_grade == 0.0;
        } else {
            identically_zero =
                IsPolynomialIdenticallyZero(piece.polynomial_coefficients);
        }
        if (!identically_zero) {
            support_start_track_station_meters_ =
                piece.start_track_station_meters;
            break;
        }
    }
}

TrackVerticalProfile::Sample TrackVerticalProfile::EvaluateRawSegment(
    const RawSegment& segment, double track_station_meters) const {
    const double local =
        track_station_meters - segment.start_track_station_meters;
    switch (segment.shape) {
        case RawSegmentShape::kConstantGrade:
            return {segment.start_grade, 0.0, 0.0};
        case RawSegmentShape::kParabolicVerticalCurve: {
            const double fraction = local / segment.length_meters;
            return {std::lerp(segment.start_grade, segment.end_grade, fraction),
                    segment.grade_change_per_meter, 0.0};
        }
        case RawSegmentShape::kCircularVerticalCurve: {
            double sine = 0.0;
            double cosine = 0.0;
            double grade = 0.0;
            if (track_station_meters ==
                segment.start_track_station_meters) {
                sine = segment.start_sine;
                cosine = segment.start_cosine;
                grade = segment.start_grade;
            } else if (track_station_meters ==
                       segment.end_track_station_meters) {
                sine = segment.end_sine;
                cosine = segment.end_cosine;
                grade = segment.end_grade;
            } else {
                const double fraction = local / segment.length_meters;
                sine = std::lerp(segment.start_sine, segment.end_sine,
                                 fraction);
                cosine = std::sqrt((1.0 - sine) * (1.0 + sine));
                grade = sine / cosine;
            }
            const double cosine_squared = cosine * cosine;
            const double cosine_cubed = cosine_squared * cosine;
            const double cosine_fifth = cosine_cubed * cosine_squared;
            const double inverse_radius =
                segment.inverse_radius_per_meter;
            return {grade, inverse_radius / cosine_cubed,
                    3.0 * inverse_radius * inverse_radius * sine /
                        cosine_fifth};
        }
    }
    throw std::logic_error(
        "TrackVerticalProfile: unsupported internal raw segment shape");
}

double TrackVerticalProfile::RawIntegralFromSegmentStart(
    const RawSegment& segment, double track_station_meters) const {
    const double local =
        track_station_meters - segment.start_track_station_meters;
    if (local == 0.0) {
        return 0.0;
    }
    switch (segment.shape) {
        case RawSegmentShape::kConstantGrade:
            return local * segment.start_grade;
        case RawSegmentShape::kParabolicVerticalCurve: {
            const double fraction = local / segment.length_meters;
            return local *
                   (segment.start_grade +
                    0.5 * (segment.end_grade - segment.start_grade) *
                        fraction);
        }
        case RawSegmentShape::kCircularVerticalCurve: {
            double sine = 0.0;
            double cosine = 0.0;
            if (track_station_meters ==
                segment.end_track_station_meters) {
                sine = segment.end_sine;
                cosine = segment.end_cosine;
            } else {
                const double fraction = local / segment.length_meters;
                sine = std::lerp(segment.start_sine, segment.end_sine,
                                 fraction);
                cosine = std::sqrt((1.0 - sine) * (1.0 + sine));
            }
            return local * (sine + segment.start_sine) /
                   (cosine + segment.start_cosine);
        }
    }
    throw std::logic_error(
        "TrackVerticalProfile: unsupported internal raw segment shape");
}

TrackVerticalProfile::Sample TrackVerticalProfile::EvaluatePiece(
    const Piece& piece, double track_station_meters) const {
    if (piece.shape == PieceShape::kRawSegment) {
        return EvaluateRawSegment(raw_segments_[piece.raw_segment_index],
                                  track_station_meters);
    }
    const double local = track_station_meters -
                         piece.polynomial_origin_track_station_meters;
    return {EvaluatePolynomial(piece.polynomial_coefficients, local),
            EvaluateFirstDerivative(piece.polynomial_coefficients, local),
            EvaluateSecondDerivative(piece.polynomial_coefficients, local)};
}

double TrackVerticalProfile::PieceIntegralFromOrigin(
    const Piece& piece, double track_station_meters) const {
    if (piece.shape == PieceShape::kRawSegment) {
        return RawIntegralFromSegmentStart(
            raw_segments_[piece.raw_segment_index], track_station_meters);
    }
    return EvaluateAntiderivative(
        piece.polynomial_coefficients,
        track_station_meters -
            piece.polynomial_origin_track_station_meters);
}

void TrackVerticalProfile::ShortenDomainEndTo(
    double end_track_station_meters) {
    if (!(end_track_station_meters >
              pieces_.back().start_track_station_meters &&
          end_track_station_meters <= end_track_station_meters_)) {
        throw std::invalid_argument(
            "TrackVerticalProfile: an equivalent common domain end must "
            "remain strictly inside the final piece");
    }
    end_track_station_meters_ = end_track_station_meters;
    pieces_.back().end_track_station_meters = end_track_station_meters;
    breakpoints_.back() = end_track_station_meters;
}

void TrackVerticalProfile::ThrowIfOutsideDomain(
    double track_station_meters) const {
    if (!std::isfinite(track_station_meters)) {
        throw std::invalid_argument(
            "TrackVerticalProfile: the track station must be finite, got " +
            Describe(track_station_meters));
    }
    if (track_station_meters < start_track_station_meters_ ||
        track_station_meters > end_track_station_meters_) {
        throw std::invalid_argument(
            "TrackVerticalProfile: the track station " +
            Describe(track_station_meters) + " m is outside the domain [" +
            Describe(start_track_station_meters_) + ", " +
            Describe(end_track_station_meters_) +
            "] m; this profile neither extrapolates nor clamps");
    }
}

std::size_t TrackVerticalProfile::PieceIndexAt(
    double track_station_meters) const {
    std::size_t low = 0;
    std::size_t high = pieces_.size() - 1;
    while (low < high) {
        const std::size_t middle = low + (high - low + 1) / 2;
        if (pieces_[middle].start_track_station_meters <=
            track_station_meters) {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    return low;
}

double TrackVerticalProfile::Value(double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    return EvaluatePiece(piece, track_station_meters).value;
}

double TrackVerticalProfile::FirstDerivativePerMeter(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    return EvaluatePiece(piece, track_station_meters)
        .first_derivative_per_meter;
}

double TrackVerticalProfile::SecondDerivativePerMeterSquared(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    return EvaluatePiece(piece, track_station_meters)
        .second_derivative_per_meter_squared;
}

double TrackVerticalProfile::IntegralFromStart(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const std::size_t index = PieceIndexAt(track_station_meters);
    const Piece& piece = pieces_[index];
    return integral_at_piece_start_[index] +
           PieceIntegralFromOrigin(piece, track_station_meters) -
           PieceIntegralFromOrigin(piece,
                                   piece.start_track_station_meters);
}

}  // namespace orvd::track_geometry
