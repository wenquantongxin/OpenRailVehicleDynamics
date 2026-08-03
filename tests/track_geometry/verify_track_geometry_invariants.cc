// G47 gates 1 and 3: the geometric invariants of a line, and the sign and
// magnitude relation between the track frame's pitch and the grade.
//
// Three tolerance budgets appear below and none of them stands in for another.
// A closed-form comparison gets a small multiple of the machine epsilon. A
// central difference gets a budget built from its own step and truncation
// order. A stretch whose position comes from quadrature is compared against the
// same geometry built at a quarter of the node spacing, which is a convergence
// statement about the rule rather than an accuracy claim pulled from the air.

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orvd/track_geometry/track_geometry.h"
#include "orvd/track_geometry/track_inertial_frame.h"
#include "track_geometry/track_geometry_test_lines.h"

namespace {

using orvd::track_geometry::GravitationalAccelerationInInertial;
using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackScalarProfile;
using orvd::track_geometry::TrackScalarSegment;
using orvd::track_geometry::TrackSeamTransition;
namespace lines = orvd::track_geometry::test_lines;

// An accessor that handed a profile out of an expiring geometry would leave
// behind an object whose station bounds still admit a query while its pieces
// are gone, and the piece lookup would then index an empty vector. These say
// that even an rvalue geometry lends rather than surrenders; reintroducing a
// move-out overload fails the build here.
static_assert(std::is_same_v<
              decltype(std::declval<const TrackGeometry&>().curvature_profile()),
              const TrackScalarProfile&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackGeometry&&>().curvature_profile()),
              const TrackScalarProfile&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackGeometry&&>().superelevation_profile()),
              const TrackScalarProfile&>);
static_assert(
    std::is_same_v<decltype(std::declval<TrackGeometry&&>().grade_profile()),
                   const TrackScalarProfile&>);
static_assert(std::is_same_v<
              decltype(std::declval<const TrackScalarProfile&>()
                           .breakpoint_track_stations_meters()),
              const std::vector<double>&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackScalarProfile&&>()
                           .breakpoint_track_stations_meters()),
              const std::vector<double>&>);

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

constexpr double kMachine = std::numeric_limits<double>::epsilon();

// A closed-form comparison: a small multiple of the machine epsilon, scaled by
// the magnitude being compared.
bool NearExact(double measured, double expected, double units_in_last_place) {
    const double bound =
        units_in_last_place * kMachine * std::max(1.0, std::abs(expected));
    return std::abs(measured - expected) <= bound;
}

void CheckCircularCurvatureOutsideSeamWindows() {
    const TrackGeometry line = lines::MakeSeamLine();
    const double curvature = 1.0 / lines::kCanonicalRadiusMeters;
    const double window_half = 0.5 * lines::kSeamWindowLengthMeters;
    const double interior_start = lines::kSeamBoundaryMeters + window_half;

    bool all_exact = true;
    for (int step = 0; step <= 40; ++step) {
        const double station =
            interior_start +
            (line.end_track_station_meters() - interior_start) *
                static_cast<double>(step) / 40.0;
        if (!NearExact(line.CurvatureRadiansPerMeter(station), curvature, 4.0)) {
            all_exact = false;
        }
    }
    Expect(all_exact,
           "the circular curvature is the reciprocal of the radius at every "
           "station of the interior interval left after the declared seam "
           "window is removed");

    // Inside the window the curvature is the quintic, not the circular value,
    // and that is a declared property rather than an accident.
    const double inside = lines::kSeamBoundaryMeters;
    Expect(line.curvature_profile().IsInsideSeamWindow(inside),
           "the seam boundary station reports as inside a declared window");
    Expect(std::abs(line.CurvatureRadiansPerMeter(inside) - curvature) > 1.0e-6,
           "the curvature inside the seam window differs from the circular "
           "value, so the interior-interval wording is doing work");
}

void CheckCircularPositionAgainstClosedForm() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const double radius = lines::kCanonicalRadiusMeters;
    const double start = lines::kCanonicalCircularStartMeters;
    const Eigen::Vector3d origin =
        line.CenterlinePositionInInertialMeters(start);
    const double heading_at_start = line.HeadingRadians(start);

    bool all_exact = true;
    for (int step = 1; step <= 20; ++step) {
        const double arc = (lines::kCanonicalCircularLengthMeters *
                            static_cast<double>(step)) /
                           20.0;
        const double station = start + arc;
        const double turn = arc / radius;
        // The exact planar displacement of a circular arc, written here rather
        // than read back out of the library.
        const double expected_x =
            radius * (std::sin(heading_at_start + turn) -
                      std::sin(heading_at_start));
        const double expected_y =
            radius * (std::cos(heading_at_start) -
                      std::cos(heading_at_start + turn));
        const Eigen::Vector3d moved =
            line.CenterlinePositionInInertialMeters(station) - origin;
        if (!NearExact(moved.x(), expected_x, 64.0) ||
            !NearExact(moved.y(), expected_y, 64.0)) {
            all_exact = false;
        }
    }
    Expect(all_exact,
           "the planar displacement along the circular stretch matches the "
           "closed-form arc to a small multiple of the machine epsilon");
}

// Every line above curves to the right. A left-hand curve is the same line
// reflected in the plane of the start, and nothing in the integration should
// prefer one sense over the other, so the mirrored line has to come out as the
// exact mirror rather than as something close to it.
void CheckLeftHandCurveMirrorsTheRightHandOne() {
    const double curvature = 1.0 / lines::kCanonicalRadiusMeters;
    const auto build = [](double signed_curvature, double superelevation) {
        return TrackGeometry(
            TrackScalarProfile(
                0.0,
                {lines::Constant(50.0, 0.0),
                 lines::Blend(50.0, 0.0, signed_curvature),
                 lines::Constant(200.0, signed_curvature)},
                {}),
            TrackScalarProfile(0.0,
                               {lines::Constant(50.0, 0.0),
                                lines::Blend(50.0, 0.0, superelevation),
                                lines::Constant(200.0, superelevation)},
                               {}),
            TrackScalarProfile(0.0, {lines::Constant(300.0, 0.02)}, {}),
            lines::kRailReferenceLateralSpanMeters, lines::kNodeSpacingMeters);
    };
    const TrackGeometry right =
        build(curvature, lines::kCanonicalSuperelevationMeters);
    const TrackGeometry left =
        build(-curvature, -lines::kCanonicalSuperelevationMeters);

    bool mirrors = true;
    bool separated = false;
    for (int step = 0; step <= 30; ++step) {
        const double station = 10.0 * static_cast<double>(step);
        const Eigen::Vector3d a =
            right.CenterlinePositionInInertialMeters(station);
        const Eigen::Vector3d b =
            left.CenterlinePositionInInertialMeters(station);
        if (std::abs(a.x() - b.x()) != 0.0 || std::abs(a.y() + b.y()) != 0.0 ||
            std::abs(a.z() - b.z()) != 0.0) {
            mirrors = false;
        }
        if (std::abs(a.y()) > 1.0) {
            separated = true;
        }
        const double roll_right = right.TrackRollRadians(station);
        const double roll_left = left.TrackRollRadians(station);
        if (roll_right + roll_left != 0.0) {
            mirrors = false;
        }
        if (std::abs(right.SuperelevationMeters(station) +
                     left.SuperelevationMeters(station)) != 0.0) {
            mirrors = false;
        }
    }
    Expect(mirrors,
           "a left-hand curve is the exact reflection of the right-hand one in "
           "the lateral coordinate, in position, in superelevation and in the "
           "roll angle it implies");
    Expect(separated,
           "the two lines are metres apart where they are compared, so the "
           "reflection is a statement about curved track and not about two "
           "copies of a straight one");
}

void CheckPlanarUnitSpeedAndGradeDerivative() {
    const TrackGeometry line = lines::MakeSteepConstantLine(0.30, 0.10);
    bool planar_unit = true;
    bool vertical_matches = true;
    for (int step = 0; step <= 20; ++step) {
        const double station = 10.0 + 12.0 * static_cast<double>(step);
        const Eigen::Vector3d derivative =
            line.CenterlineDerivativeInInertialMetersPerMeter(station);
        if (!NearExact(std::hypot(derivative.x(), derivative.y()), 1.0, 4.0)) {
            planar_unit = false;
        }
        if (!NearExact(derivative.z(), -line.CenterlineUpwardGrade(station),
                       4.0)) {
            vertical_matches = false;
        }
    }
    Expect(planar_unit,
           "the horizontal projection of the centerline derivative is a unit "
           "vector, because the station is planar projected mileage");
    // The competing answer is the unit tangent, whose horizontal part is
    // shorter by the tangent norm. Stating the full norm separately is what
    // makes the line above a claim about which of the two is returned rather
    // than a restatement of the Pythagorean identity.
    const Eigen::Vector3d at_midline =
        line.CenterlineDerivativeInInertialMetersPerMeter(150.0);
    const double tangent_norm = std::sqrt(1.0 + 0.30 * 0.30);
    Expect(NearExact(at_midline.norm(), tangent_norm, 8.0),
           "the full three-dimensional derivative has norm sqrt(1 + grade^2), "
           "so it is dr/ds and not the normalised tangent");
    Expect(std::abs(tangent_norm - 1.0) > 1.0e-2,
           "on this fixture those two candidates are far apart, so the pair of "
           "checks above discriminates between them");
    Expect(vertical_matches,
           "the vertical component of the centerline derivative is the "
           "negated grade, because the inertial frame has its z axis downward");

    // The same statement recovered from positions instead of from the
    // derivative entry, on a finite-difference budget.
    const double step_meters = 1.0e-4;
    const double station = 150.0;
    const double forward =
        line.CenterlinePositionInInertialMeters(station + step_meters).z();
    const double backward =
        line.CenterlinePositionInInertialMeters(station - step_meters).z();
    const double measured = (forward - backward) / (2.0 * step_meters);
    // Central difference: truncation of order step squared times the third
    // derivative, plus rounding of order epsilon over step.
    const double budget = step_meters * step_meters + kMachine / step_meters;
    Expect(std::abs(measured + line.CenterlineUpwardGrade(station)) <= budget,
           "the vertical rate recovered from positions by central difference "
           "is the negated grade, within the finite-difference budget");
}

void CheckTrackFrameLongitudinalAxis() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    bool matches = true;
    for (int step = 0; step <= 40; ++step) {
        const double station = 10.0 * static_cast<double>(step);
        const auto kinematics = line.EvaluateTrackFrame(station);
        const Eigen::Vector3d expected =
            kinematics.centerline_derivative_in_inertial_meters_per_meter()
                .normalized();
        const Eigen::Vector3d measured =
            kinematics.pose().rotation_inertial_from_track().col(0);
        if ((measured - expected).cwiseAbs().maxCoeff() > 8.0 * kMachine) {
            matches = false;
        }
    }
    Expect(matches,
           "the track frame's longitudinal axis is the normalised full "
           "three-dimensional centerline derivative, not its horizontal part");
}

void CheckTransitionIsHermiteCubic() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const double curvature = 1.0 / lines::kCanonicalRadiusMeters;
    const double start = lines::kCanonicalTransitionStartMeters;
    const double length = lines::kCanonicalTransitionLengthMeters;

    // The end-point first derivatives both vanish for a Hermite cubic, so on
    // their own they cannot tell this blend apart from a raised cosine or a
    // quintic. The interior point is what pins the identity.
    Expect(NearExact(line.CurvatureRadiansPerMeter(start + 0.25 * length),
                     0.0 + (5.0 / 32.0) * curvature, 8.0),
           "the transition curvature at a quarter of its length is the Hermite "
           "cubic value, which distinguishes this blend from other shapes that "
           "share its end conditions");
    Expect(NearExact(line.CurvatureRadiansPerMeter(start + 0.5 * length),
                     0.5 * curvature, 8.0),
           "the transition curvature at its midpoint is half the change");

    Expect(NearExact(
               line.curvature_profile().FirstDerivativePerMeter(start + 1.0e-9),
               0.0, 1.0e9),
           "the transition curvature derivative vanishes at its start, so the "
           "blend is not a clothoid");
    Expect(NearExact(line.curvature_profile().FirstDerivativePerMeter(
                         start + length - 1.0e-9),
                     0.0, 1.0e9),
           "the transition curvature derivative vanishes at its end");
}

// The seam quintic has to reach the centerline, not merely exist inside the
// profile. A continuity probe across the window edges of this fixture would say
// nothing: both sides of both edges are constant there, so every one-sided pair
// agrees whether or not the window was ever built. What separates a working
// seam from a missing one is the heading it integrates to, and the position and
// pose that heading produces.
void CheckSeamQuinticEntersTheCenterline() {
    const TrackGeometry line = lines::MakeSeamLine();
    const double curvature = 1.0 / lines::kCanonicalRadiusMeters;
    const double window = lines::kSeamWindowLengthMeters;
    const double window_start = lines::kSeamBoundaryMeters - 0.5 * window;
    const double window_end = lines::kSeamBoundaryMeters + 0.5 * window;

    // The quintic that matches (0, 0, 0) on the left and (curvature, 0, 0) on
    // the right, written out here instead of read back out of the library.
    const auto expected_curvature = [curvature, window_start, window](double s) {
        const double x = (s - window_start) / window;
        return curvature * x * x * x * (10.0 - 15.0 * x + 6.0 * x * x);
    };
    const auto expected_heading = [curvature, window_start, window](double s) {
        const double x = (s - window_start) / window;
        return curvature * window * x * x * x * x *
               (2.5 - 3.0 * x + x * x);
    };

    const double quarter = window_start + 0.25 * window;
    Expect(NearExact(line.CurvatureRadiansPerMeter(quarter),
                     expected_curvature(quarter), 16.0),
           "the seam curvature at a quarter of the window is the quintic value");
    // A Hermite cubic satisfies the same value and first-derivative data at both
    // ends, so only an interior point separates the two families.
    const double x = 0.25;
    const double cubic_here = curvature * (3.0 * x * x - 2.0 * x * x * x);
    Expect(std::abs(cubic_here - expected_curvature(quarter)) >
               1.0e-3 * curvature,
           "and the cubic that shares the window's value and slope data differs "
           "there, so this point tells the two families apart");

    const double measured_heading = line.HeadingRadians(window_end);
    Expect(NearExact(measured_heading, expected_heading(window_end), 64.0),
           "the heading at the window end is the exact integral of the quintic");
    const double boundary_heading =
        line.HeadingRadians(lines::kSeamBoundaryMeters);
    Expect(NearExact(boundary_heading,
                     expected_heading(lines::kSeamBoundaryMeters), 64.0),
           "and the heading half way through the window is too, which a line "
           "whose curvature only switched on at the boundary would not give");
    Expect(boundary_heading > 1.0e-4,
           "that half-way heading is far from the zero a missing window would "
           "leave, so the check above has something to discriminate");

    // The position the same heading produces, integrated independently by
    // composite Simpson. Everything before the window is straight and level, so
    // the centerline enters the window at (window_start, 0, 0).
    constexpr int kIntervals = 6000;
    const double step = window / static_cast<double>(kIntervals);
    double horizontal_x = 0.0;
    double horizontal_y = 0.0;
    for (int index = 0; index <= kIntervals; ++index) {
        const double station = window_start + step * static_cast<double>(index);
        const double weight = (index == 0 || index == kIntervals) ? 1.0
                              : (index % 2 == 1)                  ? 4.0
                                                                  : 2.0;
        horizontal_x += weight * std::cos(expected_heading(station));
        horizontal_y += weight * std::sin(expected_heading(station));
    }
    horizontal_x *= step / 3.0;
    horizontal_y *= step / 3.0;
    const Eigen::Vector3d measured =
        line.CenterlinePositionInInertialMeters(window_end);
    Expect(std::abs(measured.x() - (window_start + horizontal_x)) <= 1.0e-11 &&
               std::abs(measured.y() - horizontal_y) <= 1.0e-11,
           "the centerline through the seam window matches an independent "
           "quadrature of the quintic heading, so the window is integrated "
           "rather than stepped over");
    Expect(horizontal_y > 1.0e-4,
           "the lateral offset the window builds is far above that budget, so "
           "the comparison is not two ways of writing zero");

    const Eigen::Vector3d longitudinal =
        line.EvaluateTrackFrame(lines::kSeamBoundaryMeters)
            .pose()
            .rotation_inertial_from_track()
            .col(0);
    Expect(NearExact(longitudinal.x(), std::cos(boundary_heading), 16.0) &&
               NearExact(longitudinal.y(), std::sin(boundary_heading), 16.0),
           "the track frame inside the window carries the seam heading, so the "
           "window reaches the pose and not only the profile");

    Expect(!line.curvature_profile().IsInsideSeamWindow(
               std::nextafter(window_start, 0.0)),
           "the station just before the window is outside it");
    Expect(line.curvature_profile().IsInsideSeamWindow(window_start),
           "and the window start itself is inside");
}

struct HermiteCubicValues {
    double value;
    double first_derivative;
    double second_derivative;
};

HermiteCubicValues EvaluateIndependentHermiteCubic(
    double local_station_meters, double length_meters, double start_value,
    double end_value) {
    const double normalised = local_station_meters / length_meters;
    const double change = end_value - start_value;
    return HermiteCubicValues{
        start_value +
            change * (3.0 * normalised * normalised -
                      2.0 * normalised * normalised * normalised),
        change * (6.0 * normalised - 6.0 * normalised * normalised) /
            length_meters,
        change * (6.0 - 12.0 * normalised) /
            (length_meters * length_meters)};
}

void CheckSeamMatchesSixNonzeroBoundaryData() {
    constexpr double kFirstLength = 4.0;
    constexpr double kSecondLength = 5.0;
    constexpr double kBoundary = kFirstLength;
    constexpr double kWindowLength = 2.0;
    constexpr double kWindowStart = kBoundary - 0.5 * kWindowLength;
    constexpr double kWindowEnd = kBoundary + 0.5 * kWindowLength;
    constexpr double kFirstStart = 0.2;
    constexpr double kBoundaryValue = 1.4;
    constexpr double kSecondEnd = -0.7;

    TrackSeamTransition seam;
    seam.preceding_segment_index = 0;
    seam.window_length_meters = kWindowLength;
    const TrackScalarProfile profile(
        0.0,
        {lines::Blend(kFirstLength, kFirstStart, kBoundaryValue),
         lines::Blend(kSecondLength, kBoundaryValue, kSecondEnd)},
        {seam});

    const HermiteCubicValues left = EvaluateIndependentHermiteCubic(
        kWindowStart, kFirstLength, kFirstStart, kBoundaryValue);
    const HermiteCubicValues right = EvaluateIndependentHermiteCubic(
        kWindowEnd - kBoundary, kSecondLength, kBoundaryValue, kSecondEnd);
    Expect(std::abs(left.first_derivative) > 1.0e-3 &&
               std::abs(left.second_derivative) > 1.0e-3 &&
               std::abs(right.first_derivative) > 1.0e-3 &&
               std::abs(right.second_derivative) > 1.0e-3,
           "the seam fixture activates all four derivative boundary data");

    const auto matches = [&profile](double station,
                                    const HermiteCubicValues& expected,
                                    const char* side) {
        Expect(NearExact(profile.Value(station), expected.value, 256.0),
               std::string("the quintic seam matches the independent value at ") +
                   side);
        Expect(NearExact(profile.FirstDerivativePerMeter(station),
                         expected.first_derivative, 512.0),
               std::string("the quintic seam matches the independent first "
                           "derivative at ") +
                   side);
        Expect(NearExact(profile.SecondDerivativePerMeterSquared(station),
                         expected.second_derivative, 2048.0),
               std::string("the quintic seam matches the independent second "
                           "derivative at ") +
                   side);
    };
    // Piece lookup selects the seam at its left endpoint and the raw right
    // segment at its right endpoint. The neighbouring nextafter probes verify
    // the other one-sided values without using a finite-difference tolerance.
    matches(kWindowStart, left, "the window start");
    matches(std::nextafter(kWindowStart, -std::numeric_limits<double>::infinity()),
            left, "the raw side immediately before the window");
    matches(kWindowEnd, right, "the raw side at the window end");
    matches(std::nextafter(kWindowEnd, -std::numeric_limits<double>::infinity()),
            right, "the seam side immediately before the window end");
}

void CheckQuadratureConvergence() {
    // The same line at three node spacings. Two things have to hold, and only
    // the pair of them is a convergence statement: refining must move the
    // centerline by very little, and refining again must not move it by more.
    // The second half is the one with teeth. The horizontal position is a
    // running sum over the nodes, so a formulation that loses digits per step
    // gets worse as the spacing shrinks even while the quadrature itself
    // improves, and only the second refinement exposes that.
    const double curvature = 1.0 / lines::kCanonicalRadiusMeters;
    const auto build = [curvature](double spacing) {
        TrackScalarProfile curvature_profile(
            0.0,
            {lines::Constant(50.0, 0.0), lines::Blend(50.0, 0.0, curvature),
             lines::Constant(200.0, curvature),
             lines::Blend(50.0, curvature, 0.0), lines::Constant(50.0, 0.0)},
            {});
        TrackScalarProfile superelevation_profile(
            0.0, {lines::Constant(400.0, 0.0)}, {});
        TrackScalarProfile grade_profile(0.0, {lines::Constant(400.0, 0.0)},
                                         {});
        return TrackGeometry(std::move(curvature_profile),
                             std::move(superelevation_profile),
                             std::move(grade_profile),
                             lines::kRailReferenceLateralSpanMeters, spacing);
    };
    const TrackGeometry coarse = build(lines::kNodeSpacingMeters);
    const TrackGeometry fine = build(lines::kNodeSpacingMeters / 4.0);
    const TrackGeometry finer = build(lines::kNodeSpacingMeters / 16.0);

    const auto worst_planar_gap = [](const TrackGeometry& first,
                                     const TrackGeometry& second) {
        double worst = 0.0;
        for (int step = 0; step <= 80; ++step) {
            const double station = 5.0 * static_cast<double>(step);
            const Eigen::Vector3d a =
                first.CenterlinePositionInInertialMeters(station);
            const Eigen::Vector3d b =
                second.CenterlinePositionInInertialMeters(station);
            worst = std::max(worst, std::hypot(a.x() - b.x(), a.y() - b.y()));
        }
        return worst;
    };
    const double first_refinement = worst_planar_gap(coarse, fine);
    const double second_refinement = worst_planar_gap(fine, finer);

    Expect(first_refinement <= 1.0e-12,
           "refining the node spacing fourfold moves the planar centerline by "
           "less than a picometre over a four hundred metre line");
    Expect(second_refinement <= 1.0e-12,
           "refining it fourfold again moves it no more");
    Expect(second_refinement <= 2.0 * first_refinement + 1.0e-15,
           "the movement does not grow as the spacing shrinks, so what is left "
           "is the rounding floor of the accumulation rather than a per-step "
           "loss of significance that a finer grid would multiply");
}

void CheckDomainAndSuperelevationRefusals() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    bool refused_outside = false;
    try {
        (void)line.CurvatureRadiansPerMeter(
            line.end_track_station_meters() + 1.0);
    } catch (const std::invalid_argument&) {
        refused_outside = true;
    }
    Expect(refused_outside,
           "a station beyond the end of the line is refused rather than "
           "extrapolated or clamped");

    bool refused_non_finite = false;
    try {
        (void)line.CurvatureRadiansPerMeter(
            std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        refused_non_finite = true;
    }
    Expect(refused_non_finite, "a non-finite station is refused");

    // The counter-example fixture the gate calls for: an illegal input rather
    // than a temporarily broken product source.
    bool refused_superelevation = false;
    try {
        TrackScalarProfile curvature_profile(0.0, {lines::Constant(100.0, 0.0)},
                                             {});
        TrackScalarProfile superelevation_profile(
            0.0, {lines::Constant(100.0, 1.6)}, {});
        TrackScalarProfile grade_profile(0.0, {lines::Constant(100.0, 0.0)}, {});
        const TrackGeometry impossible(
            std::move(curvature_profile), std::move(superelevation_profile),
            std::move(grade_profile), lines::kRailReferenceLateralSpanMeters,
            lines::kNodeSpacingMeters);
        (void)impossible;
    } catch (const std::invalid_argument&) {
        refused_superelevation = true;
    }
    Expect(refused_superelevation,
           "a superelevation that reaches past the rail reference lateral span "
           "is refused at construction rather than clamped into a plausible "
           "line");

    const auto build_constant_superelevation = [](double superelevation,
                                                  double span) {
        return TrackGeometry(
            TrackScalarProfile(0.0, {lines::Constant(10.0, 0.0)}, {}),
            TrackScalarProfile(
                0.0, {lines::Constant(10.0, superelevation)}, {}),
            TrackScalarProfile(0.0, {lines::Constant(10.0, 0.0)}, {}), span,
            1.0);
    };
    bool refused_equal_superelevation = false;
    try {
        (void)build_constant_superelevation(1.5, 1.5);
    } catch (const std::invalid_argument&) {
        refused_equal_superelevation = true;
    }
    Expect(refused_equal_superelevation,
           "a superelevation equal to the reference span is refused because "
           "the first-order roll kinematics are singular there");
    bool accepted_below_superelevation = true;
    try {
        (void)build_constant_superelevation(std::nextafter(1.5, 0.0), 1.5);
    } catch (const std::exception&) {
        accepted_below_superelevation = false;
    }
    Expect(accepted_below_superelevation,
           "a representable superelevation immediately below the strict bound "
           "remains admissible");

    const auto build_seam_superelevation = [](double span) {
        TrackSeamTransition seam;
        seam.preceding_segment_index = 0;
        seam.window_length_meters = 1.0;
        return TrackGeometry(
            TrackScalarProfile(0.0, {lines::Constant(2.0, 0.0)}, {}),
            TrackScalarProfile(
                0.0,
                {lines::Blend(1.0, 0.0, 0.9),
                 lines::Blend(1.0, 0.9, 0.0)},
                {seam}),
            TrackScalarProfile(0.0, {lines::Constant(2.0, 0.0)}, {}), span,
            2.0);
    };
    bool refused_internal_overshoot = false;
    try {
        (void)build_seam_superelevation(0.8);
    } catch (const std::invalid_argument&) {
        refused_internal_overshoot = true;
    }
    Expect(refused_internal_overshoot,
           "the superelevation bound is checked at polynomial extrema, so a "
           "quintic seam cannot cross it between all sampled breakpoints");
    bool accepted_bounded_seam = true;
    try {
        (void)build_seam_superelevation(0.9);
    } catch (const std::exception&) {
        accepted_bounded_seam = false;
    }
    Expect(accepted_bounded_seam,
           "the same quintic seam is accepted when its true interior maximum "
           "lies below the reference span");

    // This profile differs from its unit upper bound only in the last few
    // binary64 digits. An absolute root tolerance would classify its small
    // derivative polynomial as identically zero, miss the seam's interior
    // maximum, and admit a frame whose roll-rate mapping is singular.
    bool refused_small_scale_internal_maximum = false;
    try {
        constexpr double kFirstLength = 4.0;
        constexpr double kSecondLength = 5.0;
        constexpr double kTotalLength = kFirstLength + kSecondLength;
        const double one_step_below_one = std::nextafter(1.0, 0.0);
        const double two_steps_below_one =
            std::nextafter(one_step_below_one, 0.0);
        double eighty_three_steps_below_one = 1.0;
        for (int step = 0; step < 83; ++step) {
            eighty_three_steps_below_one =
                std::nextafter(eighty_three_steps_below_one, 0.0);
        }
        TrackSeamTransition seam;
        seam.preceding_segment_index = 0;
        seam.window_length_meters = 2.0;
        (void)TrackGeometry(
            TrackScalarProfile(
                0.0, {lines::Constant(kTotalLength, 0.0)}, {}),
            TrackScalarProfile(
                0.0,
                {lines::Blend(kFirstLength, two_steps_below_one,
                              one_step_below_one),
                 lines::Blend(kSecondLength, one_step_below_one,
                              eighty_three_steps_below_one)},
                {seam}),
            TrackScalarProfile(
                0.0, {lines::Constant(kTotalLength, 0.0)}, {}),
            1.0, kTotalLength);
    } catch (const std::invalid_argument&) {
        refused_small_scale_internal_maximum = true;
    }
    Expect(refused_small_scale_internal_maximum,
           "the extremum search is scale-relative, so a seam that reaches the "
           "strict superelevation bound by only a few binary64 ulps is still "
           "refused before singular frame kinematics can be evaluated");

    bool refused_domain_mismatch = false;
    try {
        TrackScalarProfile curvature_profile(0.0, {lines::Constant(100.0, 0.0)},
                                             {});
        TrackScalarProfile superelevation_profile(
            0.0, {lines::Constant(90.0, 0.0)}, {});
        TrackScalarProfile grade_profile(0.0, {lines::Constant(100.0, 0.0)}, {});
        const TrackGeometry mismatched(
            std::move(curvature_profile), std::move(superelevation_profile),
            std::move(grade_profile), lines::kRailReferenceLateralSpanMeters,
            lines::kNodeSpacingMeters);
        (void)mismatched;
    } catch (const std::invalid_argument&) {
        refused_domain_mismatch = true;
    }
    Expect(refused_domain_mismatch,
           "three profiles that disagree on their station domain are refused");

    // The other side of that rule. Each family reaches the end of the line
    // through its own segment lengths, and the same total reached through a
    // different partition need not round to the same binary64 number. These two
    // partitions of 765.3 m differ in their last bits; refusing them would turn
    // a rule about covering the same line into a rule about how the line was
    // written down.
    const std::vector<double> first_partition{299.3, 140.7, 81.7, 243.6};
    const std::vector<double> second_partition{252.1, 410.2, 103.0};
    double first_end = 0.0;
    for (const double length : first_partition) {
        first_end += length;
    }
    double second_end = 0.0;
    for (const double length : second_partition) {
        second_end += length;
    }
    Expect(first_end != second_end &&
               std::abs(first_end - second_end) < 1.0e-9,
           "the two partitions accumulate to stations that differ in their last "
           "bits, so this fixture separates the two rules");

    const auto level_profile = [](const std::vector<double>& lengths) {
        std::vector<TrackScalarSegment> segments;
        segments.reserve(lengths.size());
        for (const double length : lengths) {
            segments.push_back(lines::Constant(length, 0.0));
        }
        return TrackScalarProfile(0.0, std::move(segments), {});
    };
    bool accepted_equivalent_partitions = true;
    try {
        const TrackGeometry line(level_profile(first_partition),
                                 level_profile(second_partition),
                                 level_profile(first_partition),
                                 lines::kRailReferenceLateralSpanMeters,
                                 lines::kNodeSpacingMeters);
        (void)line;
    } catch (const std::invalid_argument&) {
        accepted_equivalent_partitions = false;
    }
    Expect(accepted_equivalent_partitions,
           "three families that cover the same line through different segment "
           "partitions are accepted, because the domain rule asks them to cover "
           "the same stretch and not to have been added up the same way");

    // A spacing may be finite and positive and still ask for a node table
    // nothing can hold; the count is also converted to an unsigned integer,
    // which is undefined once the double leaves its range.
    bool refused_absurd_spacing = false;
    try {
        const TrackGeometry line(
            level_profile({100.0}), level_profile({100.0}),
            level_profile({100.0}), lines::kRailReferenceLateralSpanMeters,
            1.0e-30);
        (void)line;
    } catch (const std::invalid_argument&) {
        refused_absurd_spacing = true;
    }
    Expect(refused_absurd_spacing,
           "a node spacing that is finite and positive but would need more "
           "nodes than the table may hold is refused rather than converted out "
           "of range");
    bool accepted_fine_spacing = true;
    try {
        const TrackGeometry line(
            level_profile({100.0}), level_profile({100.0}),
            level_profile({100.0}), lines::kRailReferenceLateralSpanMeters,
            0.01);
        (void)line;
    } catch (const std::invalid_argument&) {
        accepted_fine_spacing = false;
    }
    Expect(accepted_fine_spacing,
           "a centimetre spacing over a hundred metres is still accepted, so "
           "the bound refuses the absurd rather than the merely fine");
}

void CheckSupportStarts() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    Expect(line.first_curved_track_station_meters().has_value() &&
               NearExact(*line.first_curved_track_station_meters(), 50.0, 4.0),
           "the curvature begins to act where its first non-constant segment "
           "begins, which is what a start-up domain contract compares against");
    Expect(line.first_graded_track_station_meters().has_value() &&
               NearExact(*line.first_graded_track_station_meters(), 50.0, 4.0),
           "the grade reports the same kind of support start");

    const TrackGeometry flat = lines::MakeSeamLine();
    Expect(!flat.first_graded_track_station_meters().has_value(),
           "a line with no grade anywhere reports no grade support start at "
           "all, rather than a station that would compare as if it acted");
}

void CheckProfileInputRefusals() {
    bool refused_unknown_shape = false;
    try {
        TrackScalarSegment bad = lines::Constant(1.0, 0.0);
        bad.shape = static_cast<orvd::track_geometry::TrackScalarSegmentShape>(99);
        (void)TrackScalarProfile(0.0, {bad}, {});
    } catch (const std::invalid_argument&) {
        refused_unknown_shape = true;
    }
    Expect(refused_unknown_shape,
           "an unknown segment shape is refused rather than interpreted as a "
           "different polynomial family");

    bool refused_unrepresentable_endpoint = false;
    try {
        (void)TrackScalarProfile(1.0e20, {lines::Constant(1.0, 0.0)}, {});
    } catch (const std::invalid_argument&) {
        refused_unrepresentable_endpoint = true;
    }
    Expect(refused_unrepresentable_endpoint,
           "a positive segment length too small to advance a large station is "
           "refused before a zero-width piece is formed");

    bool refused_unrepresentable_window = false;
    try {
        constexpr double kLargeStation = 1.0e20;
        constexpr double kRepresentableSegmentLength = 32768.0;
        TrackSeamTransition seam;
        seam.preceding_segment_index = 0;
        seam.window_length_meters = 1.0;
        (void)TrackScalarProfile(
            kLargeStation,
            {lines::Constant(kRepresentableSegmentLength, 0.0),
             lines::Constant(kRepresentableSegmentLength, 1.0)},
            {seam});
    } catch (const std::invalid_argument&) {
        refused_unrepresentable_window = true;
    }
    Expect(refused_unrepresentable_window,
           "a seam too narrow to form distinct floating-point endpoints is "
           "refused before its width is used as a divisor");
}

void CheckGravityDirection() {
    const Eigen::Vector3d gravity = GravitationalAccelerationInInertial(9.81);
    Expect(NearExact(gravity.x(), 0.0, 4.0) && NearExact(gravity.y(), 0.0, 4.0) &&
               NearExact(gravity.z(), 9.81, 4.0),
           "the inertial frame states gravity along its downward axis, so an "
           "assembler installs a positive vertical component");
    bool refused = false;
    try {
        (void)GravitationalAccelerationInInertial(-9.81);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused,
           "a negative magnitude is refused rather than quietly reversing the "
           "direction the frame has already fixed");
}

// Gate 3: the pitch of the track frame and the vertical rate of the centerline
// have to agree in magnitude, not merely in sign. A sign-only assertion passes
// for an implementation that is wrong by a factor of the tangent norm, which at
// a realistic grade is a relative error of order grade squared over two and
// therefore invisible.
void CheckGradeSignAndMagnitude() {
    for (const double grade : {0.30, -0.30, 0.02}) {
        const TrackGeometry line = lines::MakeSteepConstantLine(grade, 0.10);
        const double station = 150.0;
        const auto kinematics = line.EvaluateTrackFrame(station);
        const double longitudinal_vertical =
            kinematics.pose().rotation_inertial_from_track()(2, 0);
        const double expected = -grade / std::sqrt(1.0 + grade * grade);
        Expect(NearExact(longitudinal_vertical, expected, 8.0),
               "the vertical component of the track frame's longitudinal axis "
               "at grade " +
                   std::to_string(grade) +
                   " equals the negated grade over the tangent norm");
        // The quantity a sign-only gate would have missed: the difference
        // between the two candidate answers.
        const double naive = -grade;
        Expect(std::abs(naive - expected) >
                   1.0e-6 * std::max(1.0, std::abs(expected)),
               "at grade " + std::to_string(grade) +
                   " the normalised and unnormalised answers differ, so this "
                   "check has something to discriminate");
    }
}

void CheckSeamIdentityAndContinuityRefusals() {
    using orvd::track_geometry::TrackScalarSegmentShape;
    const auto refuses = [](auto&& build, std::string* message) {
        try {
            build();
        } catch (const std::invalid_argument& error) {
            if (message != nullptr) {
                *message = error.what();
            }
            return true;
        } catch (...) {
            return false;
        }
        return false;
    };

    // A boundary where the declared values disagree and no window is declared
    // across it. The first station derivative of everything built on the
    // profile does not exist there, so the profile is refused rather than
    // silently answering with one side of it.
    std::string discontinuity_message;
    Expect(refuses(
               [] {
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0), lines::Constant(10.0, 0.5)},
                       {});
                   (void)profile;
               },
               &discontinuity_message),
           "a boundary whose declared values disagree, with no seam window "
           "across it, is refused at construction");
    Expect(discontinuity_message.find("no seam window declared") !=
               std::string::npos,
           "and the refusal names the missing declaration rather than the "
           "numbers");

    // The same two segments become legal once the transition is declared.
    bool accepted = true;
    try {
        TrackSeamTransition seam;
        seam.preceding_segment_index = 0;
        seam.window_length_meters = 4.0;
        TrackScalarProfile profile(
            0.0, {lines::Constant(10.0, 0.0), lines::Constant(10.0, 0.5)},
            {seam});
        (void)profile;
    } catch (const std::exception&) {
        accepted = false;
    }
    Expect(accepted,
           "declaring a seam window across that same boundary makes it legal, "
           "so the rule asks for a declaration rather than forbidding the shape");

    // A seam naming a segment with nothing after it has no boundary to sit on.
    Expect(refuses(
               [] {
                   TrackSeamTransition seam;
                   seam.preceding_segment_index = 1;
                   seam.window_length_meters = 2.0;
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0), lines::Constant(10.0, 0.0)},
                       {seam});
                   (void)profile;
               },
               nullptr),
           "a seam naming the last segment is refused, because there is no "
           "following segment to form a boundary with");

    // The public index is unsigned. Checking `index + 1` would wrap SIZE_MAX
    // to zero and turn an invalid declaration into an out-of-bounds access.
    Expect(refuses(
               [] {
                   TrackSeamTransition seam;
                   seam.preceding_segment_index =
                       std::numeric_limits<std::size_t>::max();
                   seam.window_length_meters = 2.0;
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0),
                        lines::Constant(10.0, 0.0)},
                       {seam});
                   (void)profile;
               },
               nullptr),
           "the maximum representable seam index is refused without unsigned "
           "wraparound or an out-of-bounds access");

    // Two seams on one boundary. Their windows would also overlap, so a
    // refusal on its own says nothing about which rule fired; the message is
    // what distinguishes "you named this boundary twice" from "these two
    // windows collide", and only the first is true here.
    std::string duplicate_message;
    Expect(refuses(
               [] {
                   TrackSeamTransition first;
                   first.preceding_segment_index = 0;
                   first.window_length_meters = 2.0;
                   TrackSeamTransition second;
                   second.preceding_segment_index = 0;
                   second.window_length_meters = 4.0;
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0), lines::Constant(10.0, 0.0)},
                       {first, second});
                   (void)profile;
               },
               &duplicate_message),
           "two seams naming the same segment are refused");
    Expect(duplicate_message.find("already carries a seam window") !=
               std::string::npos,
           "and the refusal names the duplicated boundary rather than reporting "
           "the collision of the two windows it would have produced");

    // A window wider than the segment beside it would need values from a
    // segment further along, which is not what a boundary transition means.
    Expect(refuses(
               [] {
                   TrackSeamTransition seam;
                   seam.preceding_segment_index = 0;
                   seam.window_length_meters = 30.0;
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0), lines::Constant(10.0, 0.0)},
                       {seam});
                   (void)profile;
               },
               nullptr),
           "a half window reaching past an adjacent segment is refused");

    // A non-positive window has no interior to put a quintic on, and its width
    // is a divisor of the local coordinate.
    Expect(refuses(
               [] {
                   TrackSeamTransition seam;
                   seam.preceding_segment_index = 0;
                   seam.window_length_meters = 0.0;
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0), lines::Constant(10.0, 0.5)},
                       {seam});
                   (void)profile;
               },
               nullptr),
           "a window of zero length is refused rather than used as a divisor");

    // Two windows on different boundaries, each legal on its own, whose spans
    // collide. The piecewise definition would then have no single answer on the
    // overlap.
    std::string overlap_message;
    Expect(refuses(
               [] {
                   TrackSeamTransition first;
                   first.preceding_segment_index = 0;
                   first.window_length_meters = 8.0;
                   TrackSeamTransition second;
                   second.preceding_segment_index = 1;
                   second.window_length_meters = 8.0;
                   TrackScalarProfile profile(
                       0.0,
                       {lines::Constant(10.0, 0.0), lines::Constant(5.0, 0.5),
                        lines::Constant(10.0, 0.9)},
                       {first, second});
                   (void)profile;
               },
               &overlap_message),
           "two windows on different boundaries whose spans collide are "
           "refused");
    Expect(overlap_message.find("overlap") != std::string::npos,
           "and the refusal names the collision rather than one of the two "
           "declarations that are each legal on their own");

    // A blend that ends where the next segment starts is continuous by
    // declaration even though the polynomial arrives there by arithmetic, so
    // the rule compares what was declared rather than what was computed. The
    // fixture is chosen so the two really differ: evaluating this blend at its
    // own end station does not reproduce the declared end value bit for bit,
    // which is what a rule comparing computed values would refuse.
    constexpr double kBlendLength = 1.0;
    constexpr double kBlendStart = 0.1;
    constexpr double kBlendEnd = 0.3;
    const TrackScalarProfile blend_alone(
        0.0, {lines::Blend(kBlendLength, kBlendStart, kBlendEnd)}, {});
    Expect(blend_alone.Value(kBlendLength) != kBlendEnd &&
               std::abs(blend_alone.Value(kBlendLength) - kBlendEnd) <=
                   4.0 * kMachine,
           "this blend arrives within a few units in the last place of its "
           "declared end value without landing on it, so the two rules are "
           "distinguishable here");

    bool blend_accepted = true;
    try {
        TrackScalarProfile profile(
            0.0,
            {lines::Blend(kBlendLength, kBlendStart, kBlendEnd),
             lines::Constant(3.0, kBlendEnd)},
            {});
        (void)profile;
    } catch (const std::exception&) {
        blend_accepted = false;
    }
    Expect(blend_accepted,
           "a blend whose declared end value equals the next segment's declared "
           "start value is continuous, even where the arithmetic does not land "
           "on it bit for bit");
}

}  // namespace

int main() {
    CheckSeamIdentityAndContinuityRefusals();
    CheckCircularCurvatureOutsideSeamWindows();
    CheckCircularPositionAgainstClosedForm();
    CheckLeftHandCurveMirrorsTheRightHandOne();
    CheckPlanarUnitSpeedAndGradeDerivative();
    CheckTrackFrameLongitudinalAxis();
    CheckTransitionIsHermiteCubic();
    CheckSeamQuinticEntersTheCenterline();
    CheckSeamMatchesSixNonzeroBoundaryData();
    CheckQuadratureConvergence();
    CheckDomainAndSuperelevationRefusals();
    CheckSupportStarts();
    CheckProfileInputRefusals();
    CheckGravityDirection();
    CheckGradeSignAndMagnitude();
    if (failure_count != 0) {
        std::printf("%d track geometry invariant checks failed\n",
                    failure_count);
        return 1;
    }
    std::printf("track geometry invariants verified\n");
    return 0;
}
