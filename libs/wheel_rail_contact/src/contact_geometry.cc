#include "orvd/wheel_rail_contact/contact_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace orvd::wheel_rail_contact {
namespace {

// The vertical height below which a bin is considered never to have been
// written. A finite sentinel rather than negative infinity, so that a
// comparison against it is an ordinary comparison and a bin that somehow
// received a non-finite sample does not silently win.
constexpr double kUnwrittenBinHeight = -1.0e300;

// The step the longitudinal resolution starts its outward search at, and how
// far around the wheel it is willing to look before giving up. A quarter radian
// is around 100 mm of arc at a nominal wheel radius — an order of magnitude
// beyond any real contact patch.
constexpr double kLongitudinalSearchStep = 5.0e-4;
constexpr double kLongitudinalSearchLimit = 0.25;
constexpr int kLongitudinalBisections = 36;

// Divisor floors. They exist so that a degenerate island returns a finite
// number instead of an infinity that would propagate silently; both are far
// below any area or width a real patch has.
constexpr double kCentroidDivisorFloor = 1.0e-20;
constexpr double kCurvatureDenominatorFloor = 1.0e-30;

double Clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
}

// Advances a segment cursor to the segment containing `abscissa`.
//
// Forward only: it never steps back. Every batch in this module queries an
// ascending grid, which is what makes that valid — and what makes feeding it a
// descending grid produce plausible wrong numbers rather than an error. The
// grids are ascending by construction, and each is built that way in one place.
std::size_t AdvanceSegment(const double* knots, std::size_t count, double abscissa,
                           std::size_t cursor) {
    const std::size_t last = count - 2;
    while (cursor < last && knots[cursor + 1] <= abscissa) {
        ++cursor;
    }
    return cursor;
}

std::size_t LocateSegment(const double* knots, std::size_t count, double abscissa) {
    if (!(abscissa > knots[0])) {
        return 0;
    }
    if (abscissa >= knots[count - 1]) {
        return count - 2;
    }
    const std::size_t upper = static_cast<std::size_t>(
        std::upper_bound(knots, knots + count, abscissa) - knots);
    return upper - 1;
}

// A cubic on one segment, from its two end values and two end slopes.
//
// Written out per query rather than from stored coefficients. The arithmetic is
// the same; what is avoided is a second array that has to be kept in step with
// the knots it belongs to, on a curve that is rebuilt every solve.
struct CubicSegment {
    double spacing{0.0};
    double constant{0.0};
    double linear{0.0};
    double quadratic{0.0};
    double cubic{0.0};

    static CubicSegment Between(double left_knot, double right_knot,
                                double left_value, double right_value,
                                double left_slope, double right_slope) {
        CubicSegment segment;
        segment.spacing = right_knot - left_knot;
        segment.constant = left_value;
        segment.linear = segment.spacing * left_slope;
        segment.quadratic = -3.0 * left_value + 3.0 * right_value -
                            2.0 * segment.spacing * left_slope -
                            segment.spacing * right_slope;
        segment.cubic = 2.0 * left_value - 2.0 * right_value +
                        segment.spacing * left_slope + segment.spacing * right_slope;
        return segment;
    }

    [[nodiscard]] double Value(double local) const {
        return ((cubic * local + quadratic) * local + linear) * local + constant;
    }

    [[nodiscard]] double Slope(double local) const {
        return ((3.0 * cubic * local + 2.0 * quadratic) * local + linear) / spacing;
    }
};

double LocalParameter(double abscissa, double left_knot, double spacing) {
    return Clamp((abscissa - left_knot) / spacing, 0.0, 1.0);
}

// Linear interpolation over a table, held flat outside it.
//
// This is the map from a projected lateral position back to the wheel station
// it came from, and it must stay piecewise linear. The envelope's abscissae can
// be one representable step apart, so any rule that forms a divided difference
// across a segment would produce a slope of order 1e17 and ring. And the
// stations themselves come from a per-bin selection, not a function evaluation,
// so where the projection folds they jump — a smooth interpolant would return a
// station no point of the wheel occupies.
double InterpolateLinear(const double* abscissae, const double* values,
                         std::size_t count, double at) {
    if (count == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (count == 1 || at <= abscissae[0]) {
        return values[0];
    }
    if (at >= abscissae[count - 1]) {
        return values[count - 1];
    }
    const std::size_t segment = LocateSegment(abscissae, count, at);
    const double left = abscissae[segment];
    const double right = abscissae[segment + 1];
    if (right == left) {
        return values[segment];
    }
    return values[segment] +
           (values[segment + 1] - values[segment]) * (at - left) / (right - left);
}

// The composite trapezoid rule, accumulated one segment at a time.
//
// Every integral over an island uses this rule on the same uniform grid. It is
// written as a running sum over segments rather than as the closed form for a
// uniform grid, because the several integrands below are accumulated together
// in one sweep and each must see the same summation order.
struct TrapezoidAccumulator {
    double total{0.0};

    void Add(double spacing, double left, double right) {
        total += 0.5 * spacing * (right + left);
    }
};

// The single secant step that places an island's edge between two grid points.
//
// One step, not an iteration: the gap is being interpolated linearly between
// two adjacent samples, and that linear model is the same one the segmentation
// used to decide the sign changed there. Iterating against the true gap would
// place the edge somewhere the segmentation does not agree is an edge.
double RefineEdge(double left_at, double left_gap, double right_at,
                  double right_gap) {
    if (right_gap == left_gap) {
        return 0.5 * (left_at + right_at);
    }
    return left_at + (0.0 - left_gap) * (right_at - left_at) / (right_gap - left_gap);
}

double SurfaceCurvature(const MonotoneCubicInterpolant& surface, double at) {
    const double slope = surface.EvaluateFirstDerivative(at);
    const double bend = surface.EvaluateSecondDerivative(at);
    if (!std::isfinite(slope) || !std::isfinite(bend)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double denominator = std::pow(1.0 + slope * slope, 1.5);
    if (denominator < kCurvatureDenominatorFloor) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return bend / denominator;
}

WheelProfileControlNodes ResolveWheelNodes(
    const ProfilePoints& wheel_profile, WheelSide side,
    const WheelProfilePreprocessingConfiguration& preparation) {
    const WheelProfilePreprocessing preprocessing{preparation};
    WheelProfileControlNodes nodes =
        preprocessing.LayControlNodes(wheel_profile, side);
    if (!nodes.lateral_meters.empty()) {
        return nodes;
    }
    // No preparation was selected, so the authored nodes are the node set. They
    // still have to be resolved for the side: the contact geometry works in a
    // frame where lateral is signed, and the asset is authored for one side.
    const SideResolvedProfile resolved = wheel_profile.ResolveForSide(side);
    nodes.lateral_meters.assign(resolved.lateral_meters().begin(),
                                resolved.lateral_meters().end());
    nodes.vertical_meters.assign(resolved.vertical_meters().begin(),
                                 resolved.vertical_meters().end());
    return nodes;
}

MonotoneCubicInterpolant SurfaceThroughNodes(std::span<const double> lateral,
                                             std::span<const double> vertical) {
    // A cubic segment is fixed by its end values and end slopes, so handing the
    // interpolant a natural spline's nodal slopes reproduces that spline exactly
    // between the nodes. What changes is the treatment outside them: the
    // interpolant holds the end value flat and reports no slope, where the
    // spline would keep extrapolating a cubic. A profile has no material
    // outside its own width, and a cubic continued past the last node climbs
    // away fast enough to invent contact where there is none.
    const NaturalCubicSpline spline{lateral, vertical};
    return MonotoneCubicInterpolant::FromNodalSlopes(
        lateral, vertical, spline.nodal_slopes(),
        OutsideDerivativeRule::kZeroOutsideKnots);
}

void RequireUsable(double value, const char* what) {
    if (!std::isfinite(value) || !(value > 0.0)) {
        throw std::invalid_argument(
            std::string("ContactGeometrySolver: ") + what +
            " must be a positive finite number, and " + std::to_string(value) +
            " is not");
    }
}

}  // namespace

void ContactGeometryWorkspace::EnsureCapacity(std::size_t sample_capacity,
                                              std::size_t envelope_capacity,
                                              std::size_t union_capacity,
                                              std::size_t quadrature_capacity) {
    const auto grow = [](std::vector<double>& buffer, std::size_t size) {
        if (buffer.size() < size) {
            buffer.resize(size);
        }
    };
    grow(projected_lateral_, sample_capacity);
    grow(projected_vertical_, sample_capacity);
    grow(projected_station_, sample_capacity);

    GrowEnvelope(envelope_capacity);
    GrowUnion(union_capacity);

    grow(quadrature_lateral_, quadrature_capacity);
    grow(quadrature_rail_, quadrature_capacity);
    grow(quadrature_overlap_, quadrature_capacity);
    grow(quadrature_wheel_slope_, quadrature_capacity);
    grow(quadrature_station_, quadrature_capacity);
}

void ContactGeometryWorkspace::GrowEnvelope(std::size_t bin_count) {
    if (bin_highest_.size() >= bin_count) {
        return;
    }
    bin_highest_.resize(bin_count);
    bin_argument_.resize(bin_count);
    envelope_lateral_.resize(bin_count);
    envelope_vertical_.resize(bin_count);
    envelope_station_.resize(bin_count);
    envelope_vertical_slopes_.resize(bin_count);
    envelope_spacing_scratch_.resize(bin_count);
    envelope_secant_scratch_.resize(bin_count);
}

void ContactGeometryWorkspace::GrowUnion(std::size_t union_capacity) {
    if (merged_lateral_.size() >= union_capacity) {
        return;
    }
    merged_lateral_.resize(union_capacity);
    union_lateral_.resize(union_capacity);
    union_gap_.resize(union_capacity);
}

ContactGeometrySolver::ContactGeometrySolver(
    const ProfilePoints& wheel_profile, const ProfilePoints& rail_profile,
    WheelSide side, const WheelProfilePreprocessingConfiguration& preparation,
    const ContactGeometryConfiguration& configuration)
    : configuration_(configuration),
      side_(side),
      side_sign_(side == WheelSide::kLeft ? 1.0 : -1.0),
      wheel_nodes_(ResolveWheelNodes(wheel_profile, side, preparation)),
      rail_side_(rail_profile.ResolveForSide(side)),
      wheel_spline_(wheel_nodes_.lateral_meters, wheel_nodes_.vertical_meters),
      wheel_surface_(SurfaceThroughNodes(wheel_nodes_.lateral_meters,
                                         wheel_nodes_.vertical_meters)),
      rail_surface_(SurfaceThroughNodes(rail_side_.lateral_meters(),
                                        rail_side_.vertical_meters())) {
    if (rail_profile.role() != ProfileRole::kRail) {
        throw std::invalid_argument(
            "ContactGeometrySolver: the rail profile '" + rail_profile.identifier() +
            "' is a wheel profile; the two roles are not interchangeable and the "
            "gauge side of a rail has no meaning on a wheel");
    }
    if (wheel_nodes_.lateral_meters.size() < 3 || rail_side_.size() < 3) {
        throw std::invalid_argument(
            "ContactGeometrySolver: both profiles need at least three points; "
            "the wheel has " + std::to_string(wheel_nodes_.lateral_meters.size()) +
            " and the rail has " + std::to_string(rail_side_.size()));
    }
    RequireUsable(configuration_.nominal_rolling_radius_meters,
                  "the nominal rolling radius");
    RequireUsable(configuration_.envelope_bin_width_meters,
                  "the envelope bin width");
    if (configuration_.outline_sample_count < 2) {
        throw std::invalid_argument(
            "ContactGeometrySolver: the visible outline needs at least two "
            "stations to span the profile");
    }
    if (configuration_.island_quadrature_stations < 3) {
        throw std::invalid_argument(
            "ContactGeometrySolver: an island needs at least three quadrature "
            "stations for the trapezoid rule to see its interior");
    }
    if (!std::isfinite(configuration_.contact_gap_epsilon_meters) ||
        !std::isfinite(configuration_.island_merge_gap_tolerance_meters) ||
        !std::isfinite(configuration_.rail_cant_radians)) {
        throw std::invalid_argument(
            "ContactGeometrySolver: the contact epsilon, the merge tolerance "
            "and the rail cant must all be finite");
    }

    // The visible outline: the wheel's height and slope at a fixed number of
    // stations spanning its node set. The slope comes from the spline rather
    // than from the shape-preserving interpolant, and that is load-bearing —
    // the shape-preserving rule flattens the slope to zero wherever the profile
    // turns, and the outline's longitudinal placement is proportional to that
    // slope. Flattening it would move the silhouette bodily along the track.
    const std::size_t samples = configuration_.outline_sample_count;
    const double first = wheel_nodes_.lateral_meters.front();
    const double last = wheel_nodes_.lateral_meters.back();
    outline_station_.resize(samples);
    outline_height_.resize(samples);
    outline_slope_.resize(samples);
    for (std::size_t index = 0; index < samples; ++index) {
        const double fraction =
            static_cast<double>(index) / static_cast<double>(samples - 1);
        const double station = first + (last - first) * fraction;
        outline_station_[index] = station;
        outline_height_[index] = wheel_spline_.Evaluate(station);
        outline_slope_[index] = wheel_spline_.EvaluateFirstDerivative(station);
    }

    envelope_capacity_ =
        static_cast<std::size_t>(
            std::ceil((last - first) / configuration_.envelope_bin_width_meters)) +
        1;
    union_capacity_ = envelope_capacity_ + rail_side_.size();
    quadrature_capacity_ = configuration_.island_quadrature_stations;
}

void ContactGeometrySolver::PrepareWorkspace(
    ContactGeometryWorkspace& workspace) const {
    workspace.EnsureCapacity(configuration_.outline_sample_count, envelope_capacity_,
                             union_capacity_, quadrature_capacity_);
}

double ContactGeometrySolver::ResolveLongitudinalLength(
    const ContactPoseScalars& pose, double lateral_offset, double vertical_offset,
    std::span<const double> stations) const {
    const double radius = configuration_.nominal_rolling_radius_meters;
    const double yaw_tangent = std::tan(pose.yaw_radians);
    const double yaw_cosine = std::cos(pose.yaw_radians);
    const double yaw_sine = std::sin(pose.yaw_radians);
    const double roll_cosine = std::cos(pose.roll_radians);
    const double roll_sine = std::sin(pose.roll_radians);

    // The same vertical measure the envelope produced, but with the wheel's
    // circumferential angle restored as a free variable instead of fixed at the
    // silhouette. Positive means the wheel's surface is inside the rail.
    const auto overlap_at = [&](double station, double angle) {
        const double local_radius = radius + wheel_spline_.Evaluate(station);
        const double wheel_x = local_radius * std::sin(angle);
        const double wheel_z = local_radius * std::cos(angle) - radius;
        const double yawed_lateral = yaw_sine * wheel_x + yaw_cosine * station;
        const double lateral =
            roll_cosine * yawed_lateral - roll_sine * wheel_z + lateral_offset;
        const double vertical =
            roll_sine * yawed_lateral + roll_cosine * wheel_z + vertical_offset;
        return vertical - rail_surface_.Evaluate(lateral);
    };

    // Bisection against the sign at the outside end of the bracket. Fixed
    // iteration count, no early exit: the caller has already established that
    // the two ends straddle the crossing, so there is nothing to test for and
    // the count is chosen to reduce the bracket far below the precision the
    // chord is used at.
    const auto refine = [&](double station, double outside, double inside) {
        double low = outside;
        double high = inside;
        double low_value = overlap_at(station, low);
        for (int step = 0; step < kLongitudinalBisections; ++step) {
            const double middle = 0.5 * (low + high);
            const double middle_value = overlap_at(station, middle);
            if ((middle_value > 0.0) == (low_value > 0.0)) {
                low = middle;
                low_value = middle_value;
            } else {
                high = middle;
            }
        }
        return 0.5 * (low + high);
    };

    double longest = 0.0;
    for (const double station : stations) {
        if (!std::isfinite(station)) {
            continue;
        }
        const double local_radius = radius + wheel_spline_.Evaluate(station);
        if (!std::isfinite(local_radius) || !(local_radius > 0.0)) {
            continue;
        }
        const double silhouette_sine = Clamp(
            -yaw_tangent * wheel_spline_.EvaluateFirstDerivative(station), -1.0, 1.0);
        const double silhouette_angle = std::asin(silhouette_sine);
        const double silhouette_overlap = overlap_at(station, silhouette_angle);
        // The envelope said this station penetrates; the exact circle can
        // disagree, because the envelope reads a shape-preserving fit of a
        // projection while this reads the surface itself. When it disagrees the
        // station simply contributes nothing.
        if (!std::isfinite(silhouette_overlap) || !(silhouette_overlap > 0.0)) {
            continue;
        }

        // Start from the circular estimate of the half-angle a chord of this
        // depth subtends, floored so that a vanishing depth still steps.
        double offset =
            std::max(kLongitudinalSearchStep,
                     std::sqrt(2.0 * silhouette_overlap / local_radius));
        bool bracketed_behind = false;
        bool bracketed_ahead = false;
        double behind = silhouette_angle;
        double ahead = silhouette_angle;
        while (offset <= kLongitudinalSearchLimit * (1.0 + 1.0e-12)) {
            if (!bracketed_behind) {
                behind = silhouette_angle - offset;
                bracketed_behind = overlap_at(station, behind) <= 0.0;
            }
            if (!bracketed_ahead) {
                ahead = silhouette_angle + offset;
                bracketed_ahead = overlap_at(station, ahead) <= 0.0;
            }
            if (bracketed_behind && bracketed_ahead) {
                break;
            }
            if (offset >= kLongitudinalSearchLimit) {
                break;
            }
            offset = std::min(2.0 * offset, kLongitudinalSearchLimit);
        }
        if (!bracketed_behind || !bracketed_ahead) {
            continue;
        }

        const double behind_crossing = refine(station, behind, silhouette_angle);
        const double ahead_crossing = refine(station, ahead, silhouette_angle);
        // The chord is measured along the track, so the wheel-frame chord is
        // shortened by the yaw.
        const double chord =
            std::abs(yaw_cosine * local_radius *
                     (std::sin(ahead_crossing) - std::sin(behind_crossing)));
        longest = std::max(longest, chord);
    }
    return longest;
}

ContactPatchSet ContactGeometrySolver::Solve(
    const ContactPoseScalars& pose, ContactGeometryWorkspace& workspace) const {
    ContactPatchSet result;
    PrepareWorkspace(workspace);

    const double radius = configuration_.nominal_rolling_radius_meters;
    const double yaw_tangent = std::tan(pose.yaw_radians);
    const double yaw_cosine = std::cos(pose.yaw_radians);
    const double yaw_sine = std::sin(pose.yaw_radians);
    const double roll_cosine = std::cos(pose.roll_radians);
    const double roll_sine = std::sin(pose.roll_radians);

    // The pose's two offsets, turned back into a plain lateral and vertical
    // displacement of the wheel's profile datum from the rail's.
    //
    // The map is its own inverse: the pose reduction produced the offsets by
    // applying exactly this expression to the plain displacement, so applying
    // it again recovers the displacement unchanged. It looks like a rotation
    // and is not one — its determinant is negative — and rewriting it into a
    // rotation would turn the round trip into a rotation by twice the roll,
    // which at a laying cant and a metre of lever is tens of millimetres.
    const double lateral_offset = pose.lateral_offset_meters * roll_cosine +
                                  pose.vertical_raise_meters * roll_sine;
    const double vertical_offset = pose.lateral_offset_meters * roll_sine -
                                   pose.vertical_raise_meters * roll_cosine;

    // --- the visible outline, projected into the rail's cross-section ---

    const std::size_t samples = configuration_.outline_sample_count;
    double* const projected_lateral = workspace.projected_lateral_.data();
    double* const projected_vertical = workspace.projected_vertical_.data();
    double* const projected_station = workspace.projected_station_.data();

    double lateral_low = std::numeric_limits<double>::max();
    double lateral_high = std::numeric_limits<double>::lowest();
    std::size_t valid_samples = 0;
    for (std::size_t index = 0; index < samples; ++index) {
        const double station = outline_station_[index];
        const double local_radius = radius + outline_height_[index];

        // For each station across the profile, the circumferential angle at
        // which the wheel surface is furthest forward along the yawed running
        // direction. That is where the rail sees it. The clamp is reached only
        // at yaws no wheelset runs at, and without it the cosine below would be
        // the square root of a negative number.
        const double sine_tau =
            Clamp(-yaw_tangent * outline_slope_[index], -1.0, 1.0);
        const double cosine_tau = std::sqrt(std::max(0.0, 1.0 - sine_tau * sine_tau));

        const double wheel_x = local_radius * sine_tau;
        const double wheel_z = local_radius * cosine_tau - radius;

        const double yawed_lateral = yaw_sine * wheel_x + yaw_cosine * station;

        // The roll turns the lateral and vertical and leaves the longitudinal
        // alone, which is exact: the roll is about the longitudinal axis.
        const double lateral =
            roll_cosine * yawed_lateral - roll_sine * wheel_z + lateral_offset;
        const double vertical =
            roll_sine * yawed_lateral + roll_cosine * wheel_z + vertical_offset;

        projected_lateral[index] = lateral;
        projected_vertical[index] = vertical;
        projected_station[index] = station;

        if (std::isfinite(lateral) && std::isfinite(vertical) &&
            std::isfinite(wheel_x) && std::isfinite(station)) {
            lateral_low = std::min(lateral_low, lateral);
            lateral_high = std::max(lateral_high, lateral);
            ++valid_samples;
        }
    }
    if (valid_samples == 0) {
        return result;
    }

    // --- the upper envelope ---

    const double bin_width = configuration_.envelope_bin_width_meters;
    const std::size_t bin_count =
        static_cast<std::size_t>(std::ceil((lateral_high - lateral_low) / bin_width)) +
        1;
    workspace.GrowEnvelope(bin_count);
    double* const bin_highest = workspace.bin_highest_.data();
    int* const bin_argument = workspace.bin_argument_.data();
    for (std::size_t bin = 0; bin < bin_count; ++bin) {
        bin_highest[bin] = kUnwrittenBinHeight;
        bin_argument[bin] = -1;
    }

    for (std::size_t index = 0; index < samples; ++index) {
        if (!std::isfinite(projected_lateral[index]) ||
            !std::isfinite(projected_vertical[index]) ||
            !std::isfinite(projected_station[index])) {
            continue;
        }
        const double offset = (projected_lateral[index] - lateral_low) / bin_width;
        std::size_t bin = 0;
        if (offset > 0.0) {
            const double floored = std::floor(offset);
            bin = floored >= static_cast<double>(bin_count - 1)
                      ? bin_count - 1
                      : static_cast<std::size_t>(floored);
        }
        // Strictly higher material wins the bin; on an exact tie the earlier
        // station keeps it. The sweep runs forward, so "earlier" is whichever
        // reached the bin first, and the rule is only ever exercised where the
        // projection folds one station onto another.
        if (projected_vertical[index] > bin_highest[bin] || bin_argument[bin] < 0) {
            bin_highest[bin] = projected_vertical[index];
            bin_argument[bin] = static_cast<int>(index);
        }
    }

    double* const envelope_lateral = workspace.envelope_lateral_.data();
    double* const envelope_vertical = workspace.envelope_vertical_.data();
    double* const envelope_station = workspace.envelope_station_.data();
    std::size_t envelope_size = 0;
    for (std::size_t bin = 0; bin < bin_count; ++bin) {
        if (bin_argument[bin] < 0) {
            continue;
        }
        const std::size_t source = static_cast<std::size_t>(bin_argument[bin]);
        // The sample's own lateral position, not the bin's centre. The bins are
        // coarser than the sample spacing, so quantising here would put every
        // contact point on a lattice.
        envelope_lateral[envelope_size] = projected_lateral[source];
        envelope_vertical[envelope_size] = bin_highest[bin];
        envelope_station[envelope_size] = projected_station[source];
        ++envelope_size;
    }

    // The abscissae are now strictly increasing, and nothing has to enforce it.
    // Each emitted point's lateral position lies in its own bin's half-open
    // interval, the intervals are disjoint and laid out in ascending order, and
    // the map from a position to its bin index is monotone. Two points from
    // different bins therefore cannot coincide or cross.
    //
    // This matters because everything downstream divides by the spacing between
    // adjacent knots. Were the invariant ever broken by a change to how the
    // envelope is emitted, the slope construction below refuses non-increasing
    // knots outright — so the failure would be an exception naming the problem,
    // not a silent infinity propagating into the gap function.
    workspace.envelope_size_ = envelope_size;

    const std::span<const double> rail_lateral = rail_side_.lateral_meters();
    if (envelope_size < 3 || rail_lateral.size() < 3) {
        return result;
    }
    const double overlap_low =
        std::max(envelope_lateral[0], rail_lateral.front());
    const double overlap_high =
        std::min(envelope_lateral[envelope_size - 1], rail_lateral.back());
    if (overlap_low >= overlap_high) {
        return result;
    }

    // --- the shared grid ---

    workspace.GrowUnion(envelope_size + rail_lateral.size());
    double* const merged = workspace.merged_lateral_.data();
    std::size_t merged_size = 0;
    {
        std::size_t from_envelope = 0;
        std::size_t from_rail = 0;
        bool has_previous = false;
        double previous = 0.0;
        while (from_envelope < envelope_size || from_rail < rail_lateral.size()) {
            double value;
            if (from_rail >= rail_lateral.size() ||
                (from_envelope < envelope_size &&
                 envelope_lateral[from_envelope] <= rail_lateral[from_rail])) {
                value = envelope_lateral[from_envelope++];
            } else {
                value = rail_lateral[from_rail++];
            }
            if (!has_previous || value != previous) {
                merged[merged_size++] = value;
                previous = value;
                has_previous = true;
            }
        }
    }

    double* const union_lateral = workspace.union_lateral_.data();
    std::size_t union_size = 0;
    for (std::size_t index = 0; index < merged_size; ++index) {
        // Both bounds inclusive. That inclusivity is what guarantees the grid's
        // ends coincide with a knot of at least one of the two surfaces, so
        // neither is ever asked for a value outside its own width.
        if (merged[index] >= overlap_low && merged[index] <= overlap_high) {
            union_lateral[union_size++] = merged[index];
        }
    }
    workspace.union_size_ = union_size;
    if (union_size < 3) {
        return result;
    }

    // --- the two surfaces, differenced ---

    double* const envelope_vertical_slopes = workspace.envelope_vertical_slopes_.data();
    const std::span<double> spacing_scratch(
        workspace.envelope_spacing_scratch_.data(), envelope_size);
    const std::span<double> secant_scratch(
        workspace.envelope_secant_scratch_.data(), envelope_size);
    ComputeShapePreservingNodalSlopes(
        std::span<const double>(envelope_lateral, envelope_size),
        std::span<const double>(envelope_vertical, envelope_size),
        std::span<double>(envelope_vertical_slopes, envelope_size), spacing_scratch,
        secant_scratch);

    double* const union_gap = workspace.union_gap_.data();
    bool any_contact = false;
    {
        std::size_t cursor = 0;
        for (std::size_t index = 0; index < union_size; ++index) {
            const double at = union_lateral[index];
            cursor = AdvanceSegment(envelope_lateral, envelope_size, at, cursor);
            const CubicSegment segment = CubicSegment::Between(
                envelope_lateral[cursor], envelope_lateral[cursor + 1],
                envelope_vertical[cursor], envelope_vertical[cursor + 1],
                envelope_vertical_slopes[cursor], envelope_vertical_slopes[cursor + 1]);
            const double wheel_height = segment.Value(
                LocalParameter(at, envelope_lateral[cursor], segment.spacing));
            const double rail_height = rail_surface_.Evaluate(at);
            const double gap = wheel_height - rail_height;
            union_gap[index] = gap;
            any_contact =
                any_contact || gap > configuration_.contact_gap_epsilon_meters;
        }
    }
    if (!any_contact) {
        return result;
    }

    // --- islands ---

    const double epsilon = configuration_.contact_gap_epsilon_meters;
    std::size_t island_count = 0;
    {
        bool inside = false;
        std::size_t start = 0;
        for (std::size_t index = 0; index < union_size; ++index) {
            const bool in_contact = union_gap[index] > epsilon;
            if (in_contact && !inside) {
                inside = true;
                start = index;
            } else if (!in_contact && inside) {
                inside = false;
                if (island_count < kMaxContactIslands) {
                    workspace.island_start_[island_count] = start;
                    workspace.island_end_[island_count] = index - 1;
                    ++island_count;
                }
            }
        }
        if (inside && island_count < kMaxContactIslands) {
            workspace.island_start_[island_count] = start;
            workspace.island_end_[island_count] = union_size - 1;
            ++island_count;
        }
    }
    result.island_count = island_count;
    if (island_count == 0) {
        return result;
    }

    // Two islands separated by a valley shallower than the merge tolerance are
    // one island. The scan spans both bracketing contact points, so the
    // minimum it finds is the valley floor. A vertical test, not a lateral one:
    // two patches a millimetre apart with a deep valley between them stay two.
    const double merge_tolerance = configuration_.island_merge_gap_tolerance_meters;
    std::size_t merged_count = 0;
    {
        std::size_t current_start = workspace.island_start_[0];
        std::size_t current_end = workspace.island_end_[0];
        for (std::size_t island = 1; island < island_count; ++island) {
            const std::size_t next_start = workspace.island_start_[island];
            const std::size_t next_end = workspace.island_end_[island];
            double valley = std::numeric_limits<double>::max();
            for (std::size_t index = current_end; index <= next_start; ++index) {
                valley = std::min(valley, union_gap[index]);
            }
            if (valley > -merge_tolerance) {
                current_end = next_end;
                continue;
            }
            if (merged_count < kMaxContactIslands) {
                workspace.merged_start_[merged_count] = current_start;
                workspace.merged_end_[merged_count] = current_end;
                ++merged_count;
            }
            current_start = next_start;
            current_end = next_end;
        }
        if (merged_count < kMaxContactIslands) {
            workspace.merged_start_[merged_count] = current_start;
            workspace.merged_end_[merged_count] = current_end;
            ++merged_count;
        }
    }
    result.merged_island_count = merged_count;

    // --- one patch per merged island ---

    const std::size_t stations = configuration_.island_quadrature_stations;
    double* const quadrature_lateral = workspace.quadrature_lateral_.data();
    double* const quadrature_rail = workspace.quadrature_rail_.data();
    double* const quadrature_overlap = workspace.quadrature_overlap_.data();
    double* const quadrature_wheel_slope = workspace.quadrature_wheel_slope_.data();
    double* const quadrature_station = workspace.quadrature_station_.data();

    for (std::size_t island = 0;
         island < merged_count && result.count < kMaxContactPatches; ++island) {
        const std::size_t start = workspace.merged_start_[island];
        const std::size_t end = workspace.merged_end_[island];

        double deepest = std::numeric_limits<double>::lowest();
        for (std::size_t index = start; index <= end; ++index) {
            deepest = std::max(deepest, union_gap[index]);
        }
        if (deepest <= epsilon) {
            continue;
        }

        const double left_edge =
            start == 0 ? union_lateral[0]
                       : RefineEdge(union_lateral[start - 1],
                                    union_gap[start - 1] - epsilon,
                                    union_lateral[start], union_gap[start] - epsilon);
        const double right_edge =
            end >= union_size - 1
                ? union_lateral[union_size - 1]
                : RefineEdge(union_lateral[end], union_gap[end] - epsilon,
                             union_lateral[end + 1], union_gap[end + 1] - epsilon);
        const double chord_width = right_edge - left_edge;
        if (!(chord_width > 0.0)) {
            continue;
        }

        // The island's own grid. It is not a refinement of the grid the island
        // was found on — usually it is coarser — so the deepest overlap it sees
        // may legitimately differ from the deepest the segmentation saw. Both
        // are honest answers to slightly different questions, and it is this
        // one the patch's shape is built from.
        std::size_t envelope_cursor = 0;
        for (std::size_t index = 0; index < stations; ++index) {
            const double fraction =
                static_cast<double>(index) / static_cast<double>(stations - 1);
            const double at = left_edge + (right_edge - left_edge) * fraction;
            quadrature_lateral[index] = at;

            envelope_cursor =
                AdvanceSegment(envelope_lateral, envelope_size, at, envelope_cursor);
            const double left_knot = envelope_lateral[envelope_cursor];
            const CubicSegment height_segment = CubicSegment::Between(
                left_knot, envelope_lateral[envelope_cursor + 1],
                envelope_vertical[envelope_cursor],
                envelope_vertical[envelope_cursor + 1],
                envelope_vertical_slopes[envelope_cursor],
                envelope_vertical_slopes[envelope_cursor + 1]);
            const double local = LocalParameter(at, left_knot, height_segment.spacing);

            const double wheel_height = height_segment.Value(local);
            const double rail_height = rail_surface_.Evaluate(at);
            quadrature_rail[index] = rail_height;
            quadrature_overlap[index] = std::max(0.0, wheel_height - rail_height);
            quadrature_wheel_slope[index] = height_segment.Slope(local);
        }

        double vertical_penetration = quadrature_overlap[0];
        std::size_t deepest_station = 0;
        for (std::size_t index = 0; index < stations; ++index) {
            if (quadrature_overlap[index] > vertical_penetration) {
                vertical_penetration = quadrature_overlap[index];
                deepest_station = index;
            }
        }
        if (vertical_penetration <= epsilon) {
            continue;
        }

        // Every integral over the island, accumulated together so that each
        // sees the same segment order.
        TrapezoidAccumulator area;
        TrapezoidAccumulator arc_width;
        TrapezoidAccumulator arc_weighted_area;
        TrapezoidAccumulator lateral_moment;
        TrapezoidAccumulator vertical_moment;
        // The wheel's arc element across the patch: how much surface each unit
        // of lateral extent covers once the profile's own slope is accounted
        // for. It is what turns a lateral width into the arc width the contact
        // ellipse is drawn from.
        const auto wheel_jacobian = [&](std::size_t index) {
            const double slope = quadrature_wheel_slope[index];
            return std::sqrt(1.0 + slope * slope);
        };
        const auto vertical_moment_term = [&](std::size_t index) {
            const double rail_height = quadrature_rail[index];
            const double top = rail_height + quadrature_overlap[index];
            return 0.5 * (top * top - rail_height * rail_height);
        };
        for (std::size_t index = 0; index + 1 < stations; ++index) {
            const double spacing =
                quadrature_lateral[index + 1] - quadrature_lateral[index];
            area.Add(spacing, quadrature_overlap[index], quadrature_overlap[index + 1]);
            arc_width.Add(spacing, wheel_jacobian(index), wheel_jacobian(index + 1));
            arc_weighted_area.Add(spacing,
                                  quadrature_overlap[index] * wheel_jacobian(index),
                                  quadrature_overlap[index + 1] *
                                      wheel_jacobian(index + 1));
            lateral_moment.Add(spacing,
                               quadrature_lateral[index] * quadrature_overlap[index],
                               quadrature_lateral[index + 1] *
                                   quadrature_overlap[index + 1]);
            vertical_moment.Add(spacing, vertical_moment_term(index),
                                vertical_moment_term(index + 1));
        }

        const double cross_section_area = area.total;
        if (!(cross_section_area > 0.0)) {
            continue;
        }

        // A width or an area that came out unusable falls back to the plainest
        // thing that means the same, in this order: an arc length that could
        // not be integrated becomes the straight chord, and an arc-weighted
        // area becomes the plain area.
        double wheel_arc_width = arc_width.total;
        if (!(std::isfinite(wheel_arc_width) && wheel_arc_width > 0.0)) {
            wheel_arc_width = chord_width;
        }
        double weighted_area = arc_weighted_area.total;
        if (!(std::isfinite(weighted_area) && weighted_area > 0.0)) {
            weighted_area = cross_section_area;
        }

        const double centroid_lateral =
            lateral_moment.total / std::max(cross_section_area, kCentroidDivisorFloor);
        const double centroid_vertical =
            vertical_moment.total / std::max(cross_section_area, kCentroidDivisorFloor);

        // The deepest overlap, turned onto the rail's surface normal there. The
        // cosine is the one at the deepest station, not an average over the
        // island and not the one at the centroid.
        const double deepest_lateral = quadrature_lateral[deepest_station];
        const double deepest_rail_slope =
            rail_surface_.EvaluateFirstDerivative(deepest_lateral);
        const double deepest_cosine =
            1.0 / std::sqrt(1.0 + deepest_rail_slope * deepest_rail_slope);

        // Back from the rail frame to the wheel's own profile, through the map
        // the envelope carries. The rolling radius is then the wheel's own
        // undeformed profile height there — not the envelope's height, which
        // carries the yaw and the roll.
        const double wheel_station = InterpolateLinear(
            envelope_lateral, envelope_station, envelope_size, centroid_lateral);
        const double rolling_radius = radius + wheel_surface_.Evaluate(wheel_station);

        const double rail_slope_at_centroid =
            rail_surface_.EvaluateFirstDerivative(centroid_lateral);
        const double rail_surface_angle = std::atan(rail_slope_at_centroid);

        // The common normal: the rail's surface normal at the centroid, carried
        // into the wheelset's frame by undoing the pair roll and then the yaw.
        // The laying cant does not appear — it is already inside the pose's
        // roll, and adding it here would count it twice.
        const double normal_angle = std::atan2(
            yaw_cosine * std::sin(rail_surface_angle - pose.roll_radians),
            std::cos(rail_surface_angle - pose.roll_radians));
        const double contact_sine_tau =
            Clamp(-yaw_tangent * std::tan(normal_angle), -1.0, 1.0);
        const double wheel_longitudinal = rolling_radius * contact_sine_tau;

        // The angle the contact and force frames are built on. This one does
        // carry the laying cant, explicitly, and it is not the common normal.
        const double rail_slope_angle =
            rail_surface_angle + side_sign_ * configuration_.rail_cant_radians;

        for (std::size_t index = 0; index < stations; ++index) {
            quadrature_station[index] =
                InterpolateLinear(envelope_lateral, envelope_station, envelope_size,
                                  quadrature_lateral[index]);
        }
        const double longitudinal_length = ResolveLongitudinalLength(
            pose, lateral_offset, vertical_offset,
            std::span<const double>(quadrature_station, stations));
        if (!std::isfinite(longitudinal_length) ||
            !(longitudinal_length > 0.0)) {
            throw std::runtime_error(
                "ContactGeometrySolver::Solve: could not resolve the "
                "three-dimensional longitudinal extent of a positive contact "
                "patch; continuing with a substitute length is not permitted");
        }

        ContactPatch& patch = result.patches[result.count];
        patch.common_normal_angle_radians = normal_angle;
        patch.rail_slope_angle_radians = rail_slope_angle;
        patch.centroid_lateral_meters = centroid_lateral;
        patch.centroid_vertical_meters = centroid_vertical;
        patch.wheel_station_meters = wheel_station;
        patch.wheel_longitudinal_meters = wheel_longitudinal;
        patch.rolling_radius_meters = rolling_radius;
        patch.vertical_penetration_meters = vertical_penetration;
        patch.normal_penetration_meters = vertical_penetration * deepest_cosine;
        patch.chord_width_meters = chord_width;
        patch.arc_width_meters = wheel_arc_width;
        patch.cross_section_area_square_meters = cross_section_area;
        patch.arc_weighted_area_square_meters = weighted_area;
        patch.longitudinal_length_meters = longitudinal_length;
        patch.wheel_curvature_per_meter = SurfaceCurvature(wheel_surface_, wheel_station);
        patch.rail_curvature_per_meter = SurfaceCurvature(rail_surface_, centroid_lateral);
        ++result.count;
    }

    // Ascending across the track. The order is a property of where the patches
    // are, not of how deep they are: a consumer that wants the deepest, or the
    // one nearest the tread, asks for that.
    std::sort(result.patches.begin(), result.patches.begin() + result.count,
              [](const ContactPatch& left, const ContactPatch& right) {
                  return left.centroid_lateral_meters < right.centroid_lateral_meters;
              });

    // The rail material point each patch reduces about. It is the accepted
    // wheel triple pushed through the pose's yaw and roll and then dropped
    // vertically onto the undeformed rail surface — so its height comes from
    // the rail, not from the transformed wheel point, and the pose's vertical
    // offset never enters.
    for (std::size_t slot = 0; slot < result.count; ++slot) {
        ContactPatch& patch = result.patches[slot];
        const double wheel_x = patch.wheel_longitudinal_meters;
        const double wheel_y = patch.wheel_station_meters;
        // The profile height at the station, with the circumferential tilt left
        // out of the vertical while the longitudinal keeps it. The asymmetry is
        // the definition: the reference point is where the rail is under the
        // contact, not where the wheel's surface point is.
        const double wheel_z = patch.rolling_radius_meters - radius;
        patch.rail_reference_longitudinal_meters =
            yaw_cosine * wheel_x - yaw_sine * wheel_y;
        const double yawed_lateral = yaw_sine * wheel_x + yaw_cosine * wheel_y;
        patch.rail_reference_lateral_meters =
            roll_cosine * yawed_lateral - roll_sine * wheel_z + lateral_offset;
        patch.rail_reference_vertical_meters =
            rail_surface_.Evaluate(patch.rail_reference_lateral_meters);
    }

    return result;
}

}  // namespace orvd::wheel_rail_contact
