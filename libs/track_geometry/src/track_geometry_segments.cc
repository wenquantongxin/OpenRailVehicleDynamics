#include "orvd/track_geometry/track_geometry_segments.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace orvd::track_geometry {
namespace {

std::string Describe(double value) { return std::to_string(value); }

void RequireFinite(double value, const std::string& what) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument("TrackScalarProfile: " + what +
                                    " must be finite, got " + Describe(value));
    }
}

double EvaluatePolynomial(const double* coefficients, std::size_t count,
                          double local) {
    double result = 0.0;
    for (std::size_t index = count; index > 0; --index) {
        result = result * local + coefficients[index - 1];
    }
    return result;
}

double EvaluateFirstDerivative(const double* coefficients, std::size_t count,
                               double local) {
    double result = 0.0;
    for (std::size_t index = count; index > 1; --index) {
        result = result * local +
                 coefficients[index - 1] * static_cast<double>(index - 1);
    }
    return result;
}

double EvaluateSecondDerivative(const double* coefficients, std::size_t count,
                                double local) {
    double result = 0.0;
    for (std::size_t index = count; index > 2; --index) {
        const double order = static_cast<double>(index - 1);
        result = result * local + coefficients[index - 1] * order * (order - 1.0);
    }
    return result;
}

// The antiderivative that vanishes at a zero local coordinate.
double EvaluateAntiderivative(const double* coefficients, std::size_t count,
                              double local) {
    double result = 0.0;
    for (std::size_t index = count; index > 0; --index) {
        result = result * local +
                 coefficients[index - 1] / static_cast<double>(index);
    }
    return result * local;
}

bool IsIdenticallyZero(const double* coefficients, std::size_t count) {
    for (std::size_t index = 0; index < count; ++index) {
        if (coefficients[index] != 0.0) {
            return false;
        }
    }
    return true;
}

// The quintic Hermite basis as coefficients of the normalised window
// coordinate. The rows are value, first and second derivative at the window
// start, then the same three at the window end.
constexpr double kQuinticBasis[6][6] = {
    {1.0, 0.0, 0.0, -10.0, 15.0, -6.0},
    {0.0, 1.0, 0.0, -6.0, 8.0, -3.0},
    {0.0, 0.0, 0.5, -1.5, 1.5, -0.5},
    {0.0, 0.0, 0.0, 10.0, -15.0, 6.0},
    {0.0, 0.0, 0.0, -4.0, 7.0, -3.0},
    {0.0, 0.0, 0.0, 0.5, -1.0, 0.5},
};

struct RawSegmentPiece {
    double start_track_station_meters{0.0};
    double end_track_station_meters{0.0};
    double coefficients[6]{};
    std::size_t coefficient_count{1};
};

struct SeamWindow {
    std::size_t left_segment_index{0};
    double start_track_station_meters{0.0};
    double end_track_station_meters{0.0};
};

}  // namespace

TrackScalarProfile::TrackScalarProfile(
    double start_track_station_meters, std::vector<TrackScalarSegment> segments,
    std::vector<TrackSeamTransition> seam_transitions)
    : start_track_station_meters_(start_track_station_meters) {
    RequireFinite(start_track_station_meters, "the start track station");
    if (segments.empty()) {
        throw std::invalid_argument(
            "TrackScalarProfile: at least one segment is required; a profile "
            "with no segments has no domain to evaluate on");
    }

    std::vector<RawSegmentPiece> raw;
    raw.reserve(segments.size());
    double station = start_track_station_meters;
    for (std::size_t index = 0; index < segments.size(); ++index) {
        const TrackScalarSegment& segment = segments[index];
        const std::string where = "segment " + std::to_string(index);
        RequireFinite(segment.length_meters, "the length of " + where);
        RequireFinite(segment.start_value, "the start value of " + where);
        if (!(segment.length_meters > 0.0)) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where +
                " must have positive length, got " +
                Describe(segment.length_meters));
        }
        RawSegmentPiece piece;
        piece.start_track_station_meters = station;
        piece.end_track_station_meters = station + segment.length_meters;
        if (segment.shape == TrackScalarSegmentShape::kConstant) {
            piece.coefficients[0] = segment.start_value;
            piece.coefficient_count = 1;
        } else {
            RequireFinite(segment.end_value, "the end value of " + where);
            const double length = segment.length_meters;
            const double change = segment.end_value - segment.start_value;
            piece.coefficients[0] = segment.start_value;
            piece.coefficients[2] = 3.0 * change / (length * length);
            piece.coefficients[3] = -2.0 * change / (length * length * length);
            piece.coefficient_count = 4;
        }
        raw.push_back(piece);
        station = piece.end_track_station_meters;
    }
    end_track_station_meters_ = station;

    // Every window is validated against the raw boundaries before any of them
    // is applied, so a rejected profile never half-exists.
    std::vector<SeamWindow> windows;
    windows.reserve(seam_transitions.size());
    for (std::size_t index = 0; index < seam_transitions.size(); ++index) {
        const TrackSeamTransition& seam = seam_transitions[index];
        const std::string where = "seam " + std::to_string(index);
        RequireFinite(seam.boundary_track_station_meters,
                      "the boundary station of " + where);
        RequireFinite(seam.window_length_meters, "the window length of " + where);
        if (!(seam.window_length_meters > 0.0)) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where +
                " must have positive window length, got " +
                Describe(seam.window_length_meters));
        }
        std::size_t left = raw.size();
        for (std::size_t candidate = 0; candidate + 1 < raw.size(); ++candidate) {
            if (raw[candidate].end_track_station_meters ==
                seam.boundary_track_station_meters) {
                left = candidate;
                break;
            }
        }
        if (left == raw.size()) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where + " sits at station " +
                Describe(seam.boundary_track_station_meters) +
                " m, which is not an interior boundary between two segments");
        }
        const double half = 0.5 * seam.window_length_meters;
        const double left_length = raw[left].end_track_station_meters -
                                   raw[left].start_track_station_meters;
        const double right_length = raw[left + 1].end_track_station_meters -
                                    raw[left + 1].start_track_station_meters;
        if (half > left_length || half > right_length) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where + " has half window " +
                Describe(half) +
                " m, which reaches past an adjacent segment (left " +
                Describe(left_length) + " m, right " + Describe(right_length) +
                " m)");
        }
        SeamWindow window;
        window.left_segment_index = left;
        window.start_track_station_meters =
            seam.boundary_track_station_meters - half;
        window.end_track_station_meters =
            seam.boundary_track_station_meters + half;
        windows.push_back(window);
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
                "TrackScalarProfile: seam windows overlap between stations " +
                Describe(windows[index - 1].start_track_station_meters) +
                " m and " +
                Describe(windows[index].end_track_station_meters) + " m");
        }
    }

    const auto raw_derivative = [&raw](std::size_t index, double at_station,
                                       int order) {
        const RawSegmentPiece& piece = raw[index];
        const double local = at_station - piece.start_track_station_meters;
        if (order == 0) {
            return EvaluatePolynomial(piece.coefficients,
                                      piece.coefficient_count, local);
        }
        if (order == 1) {
            return EvaluateFirstDerivative(piece.coefficients,
                                           piece.coefficient_count, local);
        }
        return EvaluateSecondDerivative(piece.coefficients,
                                        piece.coefficient_count, local);
    };

    const auto emit_segment_piece = [this, &raw](std::size_t index, double from,
                                                 double to) {
        Piece piece;
        piece.start_track_station_meters = from;
        piece.end_track_station_meters = to;
        piece.polynomial_origin_track_station_meters =
            raw[index].start_track_station_meters;
        std::copy(std::begin(raw[index].coefficients),
                  std::end(raw[index].coefficients),
                  std::begin(piece.coefficients));
        piece.coefficient_count = raw[index].coefficient_count;
        pieces_.push_back(piece);
    };

    std::size_t next_window = 0;
    double cursor = start_track_station_meters;
    for (std::size_t index = 0; index < raw.size(); ++index) {
        while (next_window < windows.size() &&
               windows[next_window].left_segment_index == index) {
            const SeamWindow& window = windows[next_window];
            if (window.start_track_station_meters > cursor) {
                emit_segment_piece(index, cursor,
                                   window.start_track_station_meters);
            }
            Piece seam_piece;
            seam_piece.start_track_station_meters =
                window.start_track_station_meters;
            seam_piece.end_track_station_meters =
                window.end_track_station_meters;
            seam_piece.polynomial_origin_track_station_meters =
                window.start_track_station_meters;
            seam_piece.coefficient_count = 6;
            seam_piece.is_seam_window = true;
            const double width = window.end_track_station_meters -
                                 window.start_track_station_meters;
            const double boundary[6] = {
                raw_derivative(index, window.start_track_station_meters, 0),
                width * raw_derivative(index, window.start_track_station_meters, 1),
                width * width *
                    raw_derivative(index, window.start_track_station_meters, 2),
                raw_derivative(index + 1, window.end_track_station_meters, 0),
                width *
                    raw_derivative(index + 1, window.end_track_station_meters, 1),
                width * width *
                    raw_derivative(index + 1, window.end_track_station_meters, 2),
            };
            double normalised[6]{};
            for (std::size_t row = 0; row < 6; ++row) {
                for (std::size_t order = 0; order < 6; ++order) {
                    normalised[order] += boundary[row] * kQuinticBasis[row][order];
                }
            }
            double scale = 1.0;
            for (std::size_t order = 0; order < 6; ++order) {
                seam_piece.coefficients[order] = normalised[order] / scale;
                scale *= width;
            }
            pieces_.push_back(seam_piece);
            cursor = window.end_track_station_meters;
            ++next_window;
        }
        if (cursor < raw[index].end_track_station_meters) {
            emit_segment_piece(index, cursor,
                               raw[index].end_track_station_meters);
            cursor = raw[index].end_track_station_meters;
        }
    }

    breakpoints_.reserve(pieces_.size() + 1);
    integral_at_piece_start_.reserve(pieces_.size());
    double running_integral = 0.0;
    for (const Piece& piece : pieces_) {
        breakpoints_.push_back(piece.start_track_station_meters);
        integral_at_piece_start_.push_back(running_integral);
        const double origin = piece.polynomial_origin_track_station_meters;
        running_integral +=
            EvaluateAntiderivative(piece.coefficients, piece.coefficient_count,
                                   piece.end_track_station_meters - origin) -
            EvaluateAntiderivative(piece.coefficients, piece.coefficient_count,
                                   piece.start_track_station_meters - origin);
    }
    breakpoints_.push_back(end_track_station_meters_);

    for (const Piece& piece : pieces_) {
        if (!IsIdenticallyZero(piece.coefficients, piece.coefficient_count)) {
            support_start_track_station_meters_ =
                piece.start_track_station_meters;
            break;
        }
    }
}

void TrackScalarProfile::ThrowIfOutsideDomain(
    double track_station_meters) const {
    if (!std::isfinite(track_station_meters)) {
        throw std::invalid_argument(
            "TrackScalarProfile: the track station must be finite, got " +
            Describe(track_station_meters));
    }
    if (track_station_meters < start_track_station_meters_ ||
        track_station_meters > end_track_station_meters_) {
        throw std::invalid_argument(
            "TrackScalarProfile: the track station " +
            Describe(track_station_meters) + " m is outside the domain [" +
            Describe(start_track_station_meters_) + ", " +
            Describe(end_track_station_meters_) +
            "] m; this profile neither extrapolates nor clamps");
    }
}

std::size_t TrackScalarProfile::PieceIndexAt(
    double track_station_meters) const {
    std::size_t low = 0;
    std::size_t high = pieces_.size() - 1;
    while (low < high) {
        const std::size_t middle = low + (high - low + 1) / 2;
        if (pieces_[middle].start_track_station_meters <= track_station_meters) {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    return low;
}

double TrackScalarProfile::Value(double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    return EvaluatePolynomial(
        piece.coefficients, piece.coefficient_count,
        track_station_meters - piece.polynomial_origin_track_station_meters);
}

double TrackScalarProfile::FirstDerivativePerMeter(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    return EvaluateFirstDerivative(
        piece.coefficients, piece.coefficient_count,
        track_station_meters - piece.polynomial_origin_track_station_meters);
}

double TrackScalarProfile::SecondDerivativePerMeterSquared(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    return EvaluateSecondDerivative(
        piece.coefficients, piece.coefficient_count,
        track_station_meters - piece.polynomial_origin_track_station_meters);
}

double TrackScalarProfile::IntegralFromStart(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const std::size_t index = PieceIndexAt(track_station_meters);
    const Piece& piece = pieces_[index];
    const double origin = piece.polynomial_origin_track_station_meters;
    return integral_at_piece_start_[index] +
           EvaluateAntiderivative(piece.coefficients, piece.coefficient_count,
                                  track_station_meters - origin) -
           EvaluateAntiderivative(piece.coefficients, piece.coefficient_count,
                                  piece.start_track_station_meters - origin);
}

bool TrackScalarProfile::IsInsideSeamWindow(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    return pieces_[PieceIndexAt(track_station_meters)].is_seam_window;
}

std::size_t TrackScalarProfile::LocalPolynomialDegree(
    double track_station_meters) const {
    ThrowIfOutsideDomain(track_station_meters);
    const Piece& piece = pieces_[PieceIndexAt(track_station_meters)];
    std::size_t degree = piece.coefficient_count;
    while (degree > 1 && piece.coefficients[degree - 1] == 0.0) {
        --degree;
    }
    return degree - 1;
}

}  // namespace orvd::track_geometry
