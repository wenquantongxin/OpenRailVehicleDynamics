#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orvd/track_geometry/track_vertical_profile.h"

namespace {

using orvd::track_geometry::CircularVerticalCurveSegment;
using orvd::track_geometry::ConstantGradeSegment;
using orvd::track_geometry::ParabolicVerticalCurveSegment;
using orvd::track_geometry::TrackSeamTransition;
using orvd::track_geometry::TrackVerticalProfile;
using orvd::track_geometry::TrackVerticalSegment;

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

bool Near(double measured, double expected, double absolute_tolerance = 1.0e-14,
          double relative_tolerance = 2.0e-13) {
    return std::abs(measured - expected) <=
           absolute_tolerance + relative_tolerance * std::abs(expected);
}

template <typename Function>
bool Refuses(Function&& function) {
    try {
        std::forward<Function>(function)();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

struct ExpectedSample {
    double grade{0.0};
    double first_derivative{0.0};
    double second_derivative{0.0};
    double integral{0.0};
};

ExpectedSample EvaluateIndependentCircular(double length, double start_grade,
                                           double end_grade, double local) {
    const long double long_length = static_cast<long double>(length);
    const long double long_local = static_cast<long double>(local);
    const long double start_angle =
        std::atan(static_cast<long double>(start_grade));
    const long double end_angle =
        std::atan(static_cast<long double>(end_grade));
    const long double radius =
        long_length / (std::sin(end_angle) - std::sin(start_angle));
    const long double sine = std::sin(start_angle) + long_local / radius;
    const long double angle = std::asin(sine);
    const long double cosine = std::cos(angle);
    return {static_cast<double>(std::tan(angle)),
            static_cast<double>(1.0 /
                                (radius * cosine * cosine * cosine)),
            static_cast<double>(
                3.0L * sine /
                (radius * radius * cosine * cosine * cosine * cosine * cosine)),
            static_cast<double>(radius * (std::cos(start_angle) - cosine))};
}

void CheckNamedSegmentFormulas() {
    const TrackVerticalProfile constant(
        5.0, {ConstantGradeSegment{8.0, 0.025}}, {});
    Expect(constant.Value(9.0) == 0.025,
           "a constant segment holds its declared grade");
    Expect(constant.FirstDerivativePerMeter(9.0) == 0.0 &&
               constant.SecondDerivativePerMeterSquared(9.0) == 0.0,
           "a constant segment has zero first and second grade derivatives");
    Expect(Near(constant.IntegralFromStart(13.0), 0.20),
           "a constant segment integrates to grade times station length");

    constexpr double kParabolicLength = 10.0;
    constexpr double kParabolicStartGrade = -0.02;
    constexpr double kParabolicEndGrade = 0.04;
    const TrackVerticalProfile parabolic(
        -2.0,
        {ParabolicVerticalCurveSegment{kParabolicLength,
                                       kParabolicStartGrade,
                                       kParabolicEndGrade}},
        {});
    const double parabolic_local = 4.0;
    const double parabolic_fraction =
        parabolic_local / kParabolicLength;
    const double expected_parabolic_grade =
        std::lerp(kParabolicStartGrade, kParabolicEndGrade,
                  parabolic_fraction);
    const double expected_parabolic_rate =
        (kParabolicEndGrade - kParabolicStartGrade) / kParabolicLength;
    const double expected_parabolic_integral =
        parabolic_local *
        (kParabolicStartGrade +
         0.5 * (kParabolicEndGrade - kParabolicStartGrade) *
             parabolic_fraction);
    Expect(Near(parabolic.Value(-2.0 + parabolic_local),
                expected_parabolic_grade) &&
               Near(parabolic.FirstDerivativePerMeter(
                        -2.0 + parabolic_local),
                    expected_parabolic_rate) &&
               parabolic.SecondDerivativePerMeterSquared(
                   -2.0 + parabolic_local) == 0.0,
           "a PL2 segment has grade linear in projected station");
    Expect(Near(parabolic.IntegralFromStart(-2.0 + parabolic_local),
                expected_parabolic_integral),
           "a PL2 segment has the analytic quadratic grade integral");

    struct CircularCase {
        double length;
        double start_grade;
        double end_grade;
        double local;
    };
    for (const CircularCase test :
         {CircularCase{20.0, -0.01, 0.03, 7.3},
          CircularCase{30.0, 0.04, -0.02, 11.0}}) {
        const TrackVerticalProfile circular(
            3.0,
            {CircularVerticalCurveSegment{test.length, test.start_grade,
                                          test.end_grade}},
            {});
        const ExpectedSample expected = EvaluateIndependentCircular(
            test.length, test.start_grade, test.end_grade, test.local);
        const double station = 3.0 + test.local;
        Expect(Near(circular.Value(station), expected.grade) &&
                   Near(circular.FirstDerivativePerMeter(station),
                        expected.first_derivative) &&
                   Near(circular.SecondDerivativePerMeterSquared(station),
                        expected.second_derivative),
               "positive and negative CIR segments follow the strict "
               "projected-circle derivatives");
        Expect(Near(circular.IntegralFromStart(station), expected.integral),
               "positive and negative CIR segments use the analytic grade "
               "integral");
        Expect(circular.Value(3.0) == test.start_grade &&
                   circular.Value(3.0 + test.length) == test.end_grade,
               "a CIR reaches both declared endpoint grades exactly");
    }
}

void CheckSmallRepresentableCircularChangeAndSupport() {
    constexpr double kStartGrade = 0.01;
    constexpr double kEndGrade = 0.010000000001;
    const TrackVerticalProfile small_circular(
        0.0,
        {CircularVerticalCurveSegment{100.0, kStartGrade, kEndGrade}}, {});
    const long double start_norm =
        std::hypot(1.0L, static_cast<long double>(kStartGrade));
    const long double end_norm =
        std::hypot(1.0L, static_cast<long double>(kEndGrade));
    const long double start_sine =
        static_cast<long double>(kStartGrade) / start_norm;
    const long double end_sine =
        static_cast<long double>(kEndGrade) / end_norm;
    const long double start_cosine = 1.0L / start_norm;
    const long double end_cosine = 1.0L / end_norm;
    const double expected_integral = static_cast<double>(
        100.0L * (start_sine + end_sine) /
        (start_cosine + end_cosine));
    Expect(small_circular.Value(100.0) == kEndGrade &&
               small_circular.FirstDerivativePerMeter(50.0) > 0.0 &&
               Near(small_circular.IntegralFromStart(100.0),
                    expected_integral),
           "a small but representable CIR is not replaced by a magic "
           "near-equality threshold or an unstable large-radius integral");

    const TrackVerticalProfile delayed_support(
        7.0,
        {ConstantGradeSegment{3.0, 0.0},
         ParabolicVerticalCurveSegment{4.0, 0.0, 0.01}},
        {});
    Expect(delayed_support.support_start_track_station_meters().has_value() &&
               *delayed_support.support_start_track_station_meters() == 10.0,
           "profile support begins at the first non-zero raw piece");
}

ExpectedSample EvaluateIndependentParabolic(double length, double start_grade,
                                             double end_grade, double local) {
    const double rate = (end_grade - start_grade) / length;
    return {start_grade + rate * local, rate, 0.0,
            local * (start_grade + 0.5 * rate * local)};
}

double EvaluateIndependentQuinticAtHalfWindow(
    double width, const ExpectedSample& start, const ExpectedSample& end) {
    constexpr double x = 0.5;
    const double x2 = x * x;
    const double x3 = x2 * x;
    const double x4 = x3 * x;
    const double x5 = x4 * x;
    return start.grade * (1.0 - 10.0 * x3 + 15.0 * x4 - 6.0 * x5) +
           width * start.first_derivative *
               (x - 6.0 * x3 + 8.0 * x4 - 3.0 * x5) +
           width * width * start.second_derivative *
               (0.5 * x2 - 1.5 * x3 + 1.5 * x4 - 0.5 * x5) +
           end.grade * (10.0 * x3 - 15.0 * x4 + 6.0 * x5) +
           width * end.first_derivative *
               (-4.0 * x3 + 7.0 * x4 - 3.0 * x5) +
           width * width * end.second_derivative *
               (0.5 * x3 - x4 + 0.5 * x5);
}

void CheckCircularToParabolicSeam() {
    constexpr double kBoundary = 10.0;
    constexpr double kWindow = 4.0;
    constexpr double kWindowStart = kBoundary - 0.5 * kWindow;
    constexpr double kWindowEnd = kBoundary + 0.5 * kWindow;
    TrackSeamTransition seam;
    seam.preceding_segment_index = 0;
    seam.window_length_meters = kWindow;
    const TrackVerticalProfile profile(
        0.0,
        {CircularVerticalCurveSegment{10.0, -0.01, 0.02},
         ParabolicVerticalCurveSegment{10.0, 0.03, 0.0}},
        {seam});

    const ExpectedSample left =
        EvaluateIndependentCircular(10.0, -0.01, 0.02, kWindowStart);
    const ExpectedSample right = EvaluateIndependentParabolic(
        10.0, 0.03, 0.0, kWindowEnd - kBoundary);
    const auto matches = [&profile](double station,
                                    const ExpectedSample& expected) {
        return Near(profile.Value(station), expected.grade, 2.0e-14) &&
               Near(profile.FirstDerivativePerMeter(station),
                    expected.first_derivative, 2.0e-14) &&
               Near(profile.SecondDerivativePerMeterSquared(station),
                    expected.second_derivative, 2.0e-14);
    };
    Expect(matches(kWindowStart, left) && matches(kWindowEnd, right),
           "a CIR-to-PL2 seam matches raw grade and two derivatives at both "
           "window ends");
    Expect(Near(profile.Value(kBoundary),
                EvaluateIndependentQuinticAtHalfWindow(kWindow, left, right),
                2.0e-14),
           "the interior of a CIR-to-PL2 seam is the declared quintic grade "
           "bridge");

    constexpr double kGaussNode = 0.7745966692414834;
    const double midpoint = 0.5 * (kWindowStart + kWindowEnd);
    const double half_width = 0.5 * kWindow;
    const double independently_integrated =
        half_width *
        ((5.0 / 9.0) *
             profile.Value(midpoint - half_width * kGaussNode) +
         (8.0 / 9.0) * profile.Value(midpoint) +
         (5.0 / 9.0) *
             profile.Value(midpoint + half_width * kGaussNode));
    const double reported_integral = profile.IntegralFromStart(kWindowEnd) -
                                     profile.IntegralFromStart(kWindowStart);
    Expect(Near(reported_integral, independently_integrated, 5.0e-14),
           "the profile integral includes the quintic replacement area");
}

void CheckRepresentativeRefusals() {
    Expect(Refuses([] { TrackVerticalProfile profile(0.0, {}, {}); }),
           "an empty vertical profile is refused");
    Expect(Refuses([] {
               TrackVerticalProfile profile(
                   0.0,
                   {ParabolicVerticalCurveSegment{10.0, 0.02, 0.02}}, {});
           }),
           "an equal-grade PL2 is refused in favour of a constant segment");
    Expect(Refuses([] {
               TrackVerticalProfile profile(
                   0.0,
                   {CircularVerticalCurveSegment{10.0, -0.01, -0.01}}, {});
           }),
           "an equal-grade CIR is refused in favour of a constant segment");
    Expect(Refuses([] {
               TrackVerticalProfile profile(
                   0.0,
                   {ConstantGradeSegment{5.0, 0.0},
                    ConstantGradeSegment{5.0, 0.01}},
                   {});
           }),
           "a grade jump without a declared seam is refused");
    Expect(Refuses([] {
               TrackSeamTransition seam;
               seam.preceding_segment_index = 0;
               seam.window_length_meters = 30.0;
               TrackVerticalProfile profile(
                   0.0,
                   {ConstantGradeSegment{5.0, 0.0},
                    ConstantGradeSegment{5.0, 0.01}},
                   {seam});
           }),
           "a seam extending past an adjacent segment is refused");
    Expect(Refuses([] {
               TrackVerticalProfile profile(
                   0.0,
                   {ConstantGradeSegment{
                       5.0, std::numeric_limits<double>::infinity()}},
                   {});
           }),
           "a non-finite grade is refused");

    const TrackVerticalProfile finite(
        0.0, {ConstantGradeSegment{5.0, 0.0}}, {});
    Expect(Refuses([&finite] { (void)finite.Value(-1.0); }) &&
               Refuses([&finite] {
                   (void)finite.Value(
                       std::numeric_limits<double>::quiet_NaN());
               }),
           "evaluation refuses out-of-domain and non-finite stations");
}

}  // namespace

int main() {
    CheckNamedSegmentFormulas();
    CheckSmallRepresentableCircularChangeAndSupport();
    CheckCircularToParabolicSeam();
    CheckRepresentativeRefusals();
    if (failure_count != 0) {
        std::printf("%d track vertical profile checks failed\n", failure_count);
        return 1;
    }
    std::printf("track vertical profile verified\n");
    return 0;
}
