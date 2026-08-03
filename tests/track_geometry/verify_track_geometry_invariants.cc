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

static_assert(std::is_same_v<
              decltype(std::declval<const TrackGeometry&>().curvature_profile()),
              const TrackScalarProfile&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackGeometry&&>().curvature_profile()),
              TrackScalarProfile>);
static_assert(std::is_same_v<
              decltype(std::declval<const TrackScalarProfile&>()
                           .breakpoint_track_stations_meters()),
              const std::vector<double>&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackScalarProfile&&>()
                           .breakpoint_track_stations_meters()),
              std::vector<double>>);

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

void CheckSeamWindowContinuity() {
    const TrackGeometry line = lines::MakeSeamLine();
    const TrackScalarProfile& profile = line.curvature_profile();
    const double half = 0.5 * lines::kSeamWindowLengthMeters;
    const double window_start = lines::kSeamBoundaryMeters - half;
    const double window_end = lines::kSeamBoundaryMeters + half;
    const double probe = 1.0e-7;

    const auto continuous = [&profile, probe](double at, const char* what) {
        const double left_value = profile.Value(at - probe);
        const double right_value = profile.Value(at + probe);
        const double left_first = profile.FirstDerivativePerMeter(at - probe);
        const double right_first = profile.FirstDerivativePerMeter(at + probe);
        const double left_second =
            profile.SecondDerivativePerMeterSquared(at - probe);
        const double right_second =
            profile.SecondDerivativePerMeterSquared(at + probe);
        Expect(std::abs(left_value - right_value) <= 1.0e-12,
               std::string("the seam window's value is continuous at its ") +
                   what);
        Expect(std::abs(left_first - right_first) <= 1.0e-8,
               std::string("the seam window's first derivative is continuous "
                           "at its ") +
                   what);
        Expect(std::abs(left_second - right_second) <= 1.0e-4,
               std::string("the seam window's second derivative is continuous "
                           "at its ") +
                   what);
    };
    continuous(window_start, "start");
    continuous(window_end, "end");
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
    seam.boundary_track_station_meters = kBoundary;
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
        seam.boundary_track_station_meters = 1.0;
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
        seam.boundary_track_station_meters = kFirstLength;
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
        seam.boundary_track_station_meters =
            kLargeStation + kRepresentableSegmentLength;
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

}  // namespace

int main() {
    CheckCircularCurvatureOutsideSeamWindows();
    CheckCircularPositionAgainstClosedForm();
    CheckPlanarUnitSpeedAndGradeDerivative();
    CheckTrackFrameLongitudinalAxis();
    CheckTransitionIsHermiteCubic();
    CheckSeamWindowContinuity();
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
