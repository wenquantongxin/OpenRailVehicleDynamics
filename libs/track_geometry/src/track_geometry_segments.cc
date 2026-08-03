#include "orvd/track_geometry/track_geometry_segments.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
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

double PolynomialRoundoffScale(const double* coefficients, std::size_t count,
                               double local) {
    double scale = 0.0;
    const double magnitude = std::abs(local);
    for (std::size_t index = count; index > 0; --index) {
        scale = scale * magnitude + std::abs(coefficients[index - 1]);
    }
    return scale;
}

bool IsPolynomialZeroWithinRoundoff(const double* coefficients,
                                    std::size_t count, double local) {
    constexpr double kRoundoffFactor = 128.0;
    const double scale =
        PolynomialRoundoffScale(coefficients, count, local);
    if (scale == 0.0) {
        return true;
    }
    return std::abs(EvaluatePolynomial(coefficients, count, local)) / scale <=
           kRoundoffFactor * std::numeric_limits<double>::epsilon();
}

void AddUniqueRoot(double root, double lower, double upper,
                   std::vector<double>& roots) {
    if (root < lower || root > upper || !std::isfinite(root)) {
        return;
    }
    const double clamped = std::clamp(root, lower, upper);
    constexpr double kRootMergeFactor = 256.0;
    for (const double existing : roots) {
        const double scale =
            std::max({1.0, std::abs(existing), std::abs(clamped)});
        if (std::abs(existing - clamped) <=
            kRootMergeFactor * std::numeric_limits<double>::epsilon() * scale) {
            return;
        }
    }
    roots.push_back(clamped);
}

// Finds every real root of a degree-at-most-four polynomial on a closed
// interval. Recursively finding derivative roots splits the interval into
// monotone stretches; a sign-changing stretch then has exactly one root. The
// derivative-root evaluations also retain repeated roots, which do not change
// sign and would otherwise be missed. This is construction-time work only.
void FindPolynomialRootsInClosedInterval(const double* coefficients,
                                         std::size_t count, double lower,
                                         double upper,
                                         std::vector<double>& roots) {
    while (count > 1 && coefficients[count - 1] == 0.0) {
        --count;
    }
    if (count <= 1) {
        return;
    }
    if (count == 2) {
        AddUniqueRoot(-coefficients[0] / coefficients[1], lower, upper, roots);
        return;
    }

    std::array<double, 5> derivative{};
    for (std::size_t order = 1; order < count; ++order) {
        derivative[order - 1] =
            static_cast<double>(order) * coefficients[order];
    }
    std::vector<double> critical_points;
    critical_points.reserve(count);
    FindPolynomialRootsInClosedInterval(derivative.data(), count - 1, lower,
                                        upper, critical_points);
    std::sort(critical_points.begin(), critical_points.end());

    std::vector<double> partitions;
    partitions.reserve(critical_points.size() + 2);
    partitions.push_back(lower);
    for (const double critical : critical_points) {
        AddUniqueRoot(critical, lower, upper, partitions);
    }
    std::sort(partitions.begin(), partitions.end());
    if (partitions.empty() || partitions.back() != upper) {
        partitions.push_back(upper);
    }

    for (const double point : partitions) {
        if (IsPolynomialZeroWithinRoundoff(coefficients, count, point)) {
            AddUniqueRoot(point, lower, upper, roots);
        }
    }
    for (std::size_t index = 0; index + 1 < partitions.size(); ++index) {
        double left = partitions[index];
        double right = partitions[index + 1];
        double left_value = EvaluatePolynomial(coefficients, count, left);
        const double right_value =
            EvaluatePolynomial(coefficients, count, right);
        if (std::signbit(left_value) == std::signbit(right_value) ||
            IsPolynomialZeroWithinRoundoff(coefficients, count, left) ||
            IsPolynomialZeroWithinRoundoff(coefficients, count, right)) {
            continue;
        }
        for (int iteration = 0; iteration < 80; ++iteration) {
            const double middle = std::midpoint(left, right);
            if (middle == left || middle == right) {
                break;
            }
            const double middle_value =
                EvaluatePolynomial(coefficients, count, middle);
            if (std::signbit(middle_value) == std::signbit(left_value)) {
                left = middle;
                left_value = middle_value;
            } else {
                right = middle;
            }
        }
        AddUniqueRoot(std::midpoint(left, right), lower, upper, roots);
    }
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
        if (!std::isfinite(piece.end_track_station_meters) ||
            !(piece.end_track_station_meters > station)) {
            throw std::invalid_argument(
                "TrackScalarProfile: the endpoint of " + where +
                " is not a finite representable station strictly after " +
                Describe(station) + " m");
        }
        switch (segment.shape) {
            case TrackScalarSegmentShape::kConstant:
                piece.coefficients[0] = segment.start_value;
                piece.coefficient_count = 1;
                break;
            case TrackScalarSegmentShape::kHermiteCubicBlend: {
                RequireFinite(segment.end_value, "the end value of " + where);
                const double length = segment.length_meters;
                const double change = segment.end_value - segment.start_value;
                piece.coefficients[0] = segment.start_value;
                piece.coefficients[2] = 3.0 * change / (length * length);
                piece.coefficients[3] =
                    -2.0 * change / (length * length * length);
                piece.coefficient_count = 4;
                break;
            }
            default:
                throw std::invalid_argument(
                    "TrackScalarProfile: " + where +
                    " has an unsupported segment shape");
        }
        for (std::size_t order = 0; order < piece.coefficient_count; ++order) {
            if (!std::isfinite(piece.coefficients[order])) {
                throw std::invalid_argument(
                    "TrackScalarProfile: the polynomial coefficients of " +
                    where + " are not finite for the declared values and "
                            "length");
            }
        }
        raw.push_back(piece);
        station = piece.end_track_station_meters;
    }
    end_track_station_meters_ = station;

    // Every window is validated against the raw boundaries before any of them
    // is applied, so a rejected profile never half-exists.
    // The value a segment declares at each of its ends, used both to validate
    // continuity and to build the seam windows. Reading the declared numbers
    // rather than evaluating the polynomial keeps the comparison exact: a
    // Hermite blend reaches its stated end value in arithmetic that need not
    // reproduce it bit for bit.
    const auto declared_start_value = [&segments](std::size_t index) {
        return segments[index].start_value;
    };
    const auto declared_end_value = [&segments](std::size_t index) {
        return segments[index].shape == TrackScalarSegmentShape::kConstant
                   ? segments[index].start_value
                   : segments[index].end_value;
    };

    std::vector<SeamWindow> windows;
    windows.reserve(seam_transitions.size());
    std::vector<bool> has_seam(raw.size(), false);
    for (std::size_t index = 0; index < seam_transitions.size(); ++index) {
        const TrackSeamTransition& seam = seam_transitions[index];
        const std::string where = "seam " + std::to_string(index);
        RequireFinite(seam.window_length_meters, "the window length of " + where);
        if (!(seam.window_length_meters > 0.0)) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where +
                " must have positive window length, got " +
                Describe(seam.window_length_meters));
        }
        const std::size_t left = seam.preceding_segment_index;
        // Compare before adding one: SIZE_MAX is a valid value of the public
        // unsigned field, and `left + 1` would wrap to zero before this guard
        // could reject it. `raw` is known non-empty above.
        if (left >= raw.size() - 1) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where + " names segment " +
                std::to_string(left) +
                ", which has no following segment to form a boundary with; the "
                "profile has " +
                std::to_string(raw.size()) + " segments");
        }
        if (has_seam[left]) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where +
                " names segment " + std::to_string(left) +
                ", which already carries a seam window; one boundary takes one "
                "window");
        }
        has_seam[left] = true;
        const double canonical_boundary = raw[left].end_track_station_meters;
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
        window.start_track_station_meters = canonical_boundary - half;
        window.end_track_station_meters = canonical_boundary + half;
        if (!std::isfinite(window.start_track_station_meters) ||
            !std::isfinite(window.end_track_station_meters) ||
            !(window.start_track_station_meters < canonical_boundary) ||
            !(canonical_boundary < window.end_track_station_meters)) {
            throw std::invalid_argument(
                "TrackScalarProfile: " + where +
                " is too narrow to form two finite representable window "
                "endpoints around station " +
                Describe(canonical_boundary) + " m");
        }
        windows.push_back(window);
    }

    // Every boundary without a declared window has to be continuous. A jump
    // there would make the first station derivative of the track frame
    // one-sided, and an evaluation would have to answer with one side of it as
    // though the quantity existed.
    for (std::size_t index = 0; index + 1 < raw.size(); ++index) {
        if (has_seam[index]) {
            continue;
        }
        const double leaving = declared_end_value(index);
        const double arriving = declared_start_value(index + 1);
        if (leaving != arriving) {
            throw std::invalid_argument(
                "TrackScalarProfile: segment " + std::to_string(index) +
                " ends at " + Describe(leaving) + " while segment " +
                std::to_string(index + 1) + " starts at " + Describe(arriving) +
                " with no seam window declared across the boundary at station " +
                Describe(raw[index].end_track_station_meters) +
                " m; declare a seam window or make the two values agree, "
                "because a jump leaves the first derivative undefined there");
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
                if (!std::isfinite(seam_piece.coefficients[order])) {
                    throw std::invalid_argument(
                        "TrackScalarProfile: a seam polynomial coefficient is "
                        "not finite for the declared window");
                }
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
        if (!std::isfinite(running_integral)) {
            throw std::invalid_argument(
                "TrackScalarProfile: the accumulated profile integral is not "
                "finite for the declared segments");
        }
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

TrackScalarProfile::ProfileExtremum
TrackScalarProfile::MaximumAbsoluteValue() const {
    ProfileExtremum extremum;
    bool have_value = false;
    const auto consider = [&extremum, &have_value](const Piece& piece,
                                                   double local) {
        const double value = EvaluatePolynomial(
            piece.coefficients, piece.coefficient_count, local);
        const double station =
            piece.polynomial_origin_track_station_meters + local;
        if (!have_value || std::abs(value) > std::abs(extremum.value)) {
            extremum.value = value;
            extremum.track_station_meters = station;
            have_value = true;
        }
    };

    for (const Piece& piece : pieces_) {
        const double lower = piece.start_track_station_meters -
                             piece.polynomial_origin_track_station_meters;
        const double upper = piece.end_track_station_meters -
                             piece.polynomial_origin_track_station_meters;
        consider(piece, lower);
        consider(piece, upper);

        std::array<double, 5> derivative{};
        for (std::size_t order = 1; order < piece.coefficient_count; ++order) {
            derivative[order - 1] =
                static_cast<double>(order) * piece.coefficients[order];
        }
        std::vector<double> roots;
        roots.reserve(piece.coefficient_count);
        FindPolynomialRootsInClosedInterval(
            derivative.data(),
            piece.coefficient_count > 1 ? piece.coefficient_count - 1 : 1,
            lower, upper, roots);
        for (const double root : roots) {
            consider(piece, root);
        }
    }
    return extremum;
}

void TrackScalarProfile::ShortenDomainEndTo(
    double end_track_station_meters) {
    if (!(end_track_station_meters >
              pieces_.back().start_track_station_meters &&
          end_track_station_meters <= end_track_station_meters_)) {
        throw std::invalid_argument(
            "TrackScalarProfile: an equivalent common domain end must remain "
            "strictly inside the final piece");
    }
    end_track_station_meters_ = end_track_station_meters;
    pieces_.back().end_track_station_meters = end_track_station_meters;
    breakpoints_.back() = end_track_station_meters;
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
