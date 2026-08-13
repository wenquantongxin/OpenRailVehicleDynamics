// The equal-arc wheel-profile rescan, its authored-node limit, and the
// arc-length primitive it is built on.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/wheel_rail_contact/natural_cubic_spline.h"
#include "orvd/wheel_rail_contact/profile_arc_length.h"
#include "orvd/wheel_rail_contact/profile_points.h"
#include "orvd/wheel_rail_contact/wheel_profile_preprocessing.h"

namespace {

using orvd::wheel_rail_contact::AccumulateProfileArcLength;
using orvd::wheel_rail_contact::FindLateralCoordinateAtArcLength;
using orvd::wheel_rail_contact::IntegrateProfileArcLength;
using orvd::wheel_rail_contact::NaturalCubicSpline;
using orvd::wheel_rail_contact::ProfilePoints;
using orvd::wheel_rail_contact::ProfileRole;
using orvd::wheel_rail_contact::WheelProfileControlNodes;
using orvd::wheel_rail_contact::WheelProfileOutline;
using orvd::wheel_rail_contact::WheelProfilePreprocessing;
using orvd::wheel_rail_contact::WheelProfilePreprocessingConfiguration;
using orvd::wheel_rail_contact::WheelSide;

constexpr std::size_t kOutlineSamples = 401;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "wheel profile preprocessing: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireRefusal(const std::function<void()>& action, std::string_view fragment,
                    std::string_view what) {
    try {
        action();
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "wheel profile preprocessing: %.*s was refused for "
                         "another reason: %s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

// A synthetic wheel-shaped profile: a coned tread that turns sharply into a
// flange. The turn makes an equal-arc rescan place more nodes where the curve
// bends, so a fixture without it cannot prove that the grid follows curve
// length rather than lateral distance.
ProfilePoints MakeSyntheticWheelProfile(std::string identifier,
                                        bool authored_descending = false) {
    std::vector<double> lateral;
    std::vector<double> vertical;
    constexpr int kPoints = 241;
    for (int index = 0; index < kPoints; ++index) {
        const double lateral_at =
            -0.060 + 0.100 * (static_cast<double>(index) /
                              static_cast<double>(kPoints - 1));
        // A shallow cone over most of the width, plus a flange that rises
        // steeply toward the inner end.
        const double cone = -0.05 * lateral_at;
        const double flange =
            0.028 * std::exp(-std::pow((lateral_at + 0.050) / 0.009, 2.0));
        lateral.push_back(lateral_at);
        vertical.push_back(cone - flange);
    }
    if (authored_descending) {
        std::reverse(lateral.begin(), lateral.end());
        std::reverse(vertical.begin(), vertical.end());
    }
    return ProfilePoints::FromAuthoredOrder(ProfileRole::kWheel,
                                            std::move(identifier),
                                            std::move(lateral), std::move(vertical));
}

// Arc length between two arbitrary coordinates, integrated one source-knot
// interval at a time. The primitive under test integrates a single panel and
// says so; a panel spanning several knots would lose most of its accuracy,
// which would show up here as a false failure rather than as a real one.
double ArcLengthBetween(const NaturalCubicSpline& profile,
                        std::span<const double> knots, double from, double to) {
    double total = 0.0;
    double cursor = from;
    for (std::size_t node = 0; node + 1 < knots.size(); ++node) {
        const double segment_start = std::max(cursor, knots[node]);
        const double segment_end = std::min(to, knots[node + 1]);
        if (segment_end > segment_start) {
            total += IntegrateProfileArcLength(profile, segment_start, segment_end);
            cursor = segment_end;
        }
    }
    return total;
}

double WorstOutlineDifference(const WheelProfileOutline& left,
                              const WheelProfileOutline& right) {
    double worst = 0.0;
    for (std::size_t index = 0; index < left.height_meters.size(); ++index) {
        worst = std::max(worst, std::abs(left.height_meters[index] -
                                         right.height_meters[index]));
    }
    return worst;
}

}  // namespace

int main() {
    const ProfilePoints wheel = MakeSyntheticWheelProfile("synthetic_wheel");

    RequireRefusal(
        [] {
            (void)ProfilePoints::FromAuthoredOrder(
                ProfileRole::kWheel, "zigzag_wheel",
                std::vector<double>{0.0, 0.02, 0.01},
                std::vector<double>{0.0, -0.001, -0.002});
        },
        "one lateral direction",
        "a profile whose authored lateral coordinate doubles back");

    const WheelProfilePreprocessing plain{WheelProfilePreprocessingConfiguration{}};
    const WheelProfilePreprocessing rescanning{
        WheelProfilePreprocessingConfiguration{0.0015}};

    Require(plain.step_meters() == 0.0 &&
                rescanning.step_meters() == 0.0015,
            "a configuration did not retain its equal-arc rescan step");
    const WheelProfileControlNodes authored_nodes =
        plain.LayControlNodes(wheel, WheelSide::kRight);
    Require(authored_nodes.lateral_meters.empty() &&
                authored_nodes.vertical_meters.empty() &&
                authored_nodes.total_arc_length_meters == 0.0,
            "a zero rescan step replaced the authored nodes");
    RequireRefusal(
        [] {
            WheelProfilePreprocessing negative{
                WheelProfilePreprocessingConfiguration{-0.001}};
            (void)negative;
        },
        "not negative", "a negative rescan step");
    RequireRefusal(
        [] {
            WheelProfilePreprocessing infinite{
                WheelProfilePreprocessingConfiguration{
                    std::numeric_limits<double>::infinity()}};
            (void)infinite;
        },
        "finite", "an infinite rescan step");

    const WheelProfileOutline plain_outline =
        plain.SampleVisibleOutline(wheel, WheelSide::kRight, kOutlineSamples);
    const WheelProfileOutline rescan_outline =
        rescanning.SampleVisibleOutline(wheel, WheelSide::kRight, kOutlineSamples);

    Require(WorstOutlineDifference(plain_outline, rescan_outline) > 1.0e-8,
            "the equal-arc rescan left the authored spline unchanged");

    Require(rescan_outline.station_meters.front() ==
                    plain_outline.station_meters.front() &&
                rescan_outline.station_meters.back() ==
                    plain_outline.station_meters.back(),
            "a preparation moved the span the outline is sampled over");

    {
        const WheelProfileControlNodes nodes =
            rescanning.LayControlNodes(wheel, WheelSide::kRight);
        Require(nodes.lateral_meters.front() ==
                        wheel.authored_lateral_meters().front() &&
                    nodes.lateral_meters.back() ==
                        wheel.authored_lateral_meters().back(),
                "the rescan moved a physical endpoint");
        Require(nodes.total_arc_length_meters > 0.100,
                "the rescan reported an implausible curve length");

        // Equal arc length is the claim, so the intervals must be equal when
        // measured along the curve rather than across it. The last one is the
        // remainder and is allowed to be shorter than the step, never longer.
        const auto physical = wheel.ResolveForSide(WheelSide::kRight);
        const NaturalCubicSpline first_pass(physical.lateral_meters(),
                                            physical.vertical_meters());
        double worst_interval_error = 0.0;
        for (std::size_t node = 0; node + 2 < nodes.lateral_meters.size(); ++node) {
            const double arc = ArcLengthBetween(
                first_pass, physical.lateral_meters(), nodes.lateral_meters[node],
                nodes.lateral_meters[node + 1]);
            worst_interval_error =
                std::max(worst_interval_error, std::abs(arc - 0.0015));
        }
        Require(worst_interval_error < 1.0e-12,
                "the rescan's interior intervals are not equal in arc length");
        const std::size_t last = nodes.lateral_meters.size() - 1;
        const double remainder =
            ArcLengthBetween(first_pass, physical.lateral_meters(),
                             nodes.lateral_meters[last - 1],
                             nodes.lateral_meters[last]);
        Require(remainder > 0.0 && remainder <= 0.0015 + 1.0e-12,
                "the rescan's last interval is longer than one step");

        // Where the curve bends, equal steps along it must crowd together in
        // the lateral coordinate. If they did not, the preparation would have
        // no purpose.
        double narrowest = 1.0;
        double widest = 0.0;
        for (std::size_t node = 0; node + 2 < nodes.lateral_meters.size(); ++node) {
            const double width =
                nodes.lateral_meters[node + 1] - nodes.lateral_meters[node];
            narrowest = std::min(narrowest, width);
            widest = std::max(widest, width);
        }
        Require(widest > 2.0 * narrowest,
                "the rescan spaced its nodes evenly across the profile, so it "
                "did not follow the curve");
    }

    {
        // The left wheel must be the mirror of the right one. It is only so
        // because the rescan runs on the physical ordering and mirrors its
        // result; anchoring the march on an already-mirrored list moves every
        // interior node and leaves this assertion the only thing that notices.
        const WheelProfileControlNodes right =
            rescanning.LayControlNodes(wheel, WheelSide::kRight);
        const WheelProfileControlNodes left =
            rescanning.LayControlNodes(wheel, WheelSide::kLeft);
        Require(left.lateral_meters.size() == right.lateral_meters.size(),
                "the two wheels received different numbers of nodes");
        double worst = 0.0;
        for (std::size_t node = 0; node < left.lateral_meters.size(); ++node) {
            const std::size_t opposite = right.lateral_meters.size() - 1 - node;
            worst = std::max(worst, std::abs(left.lateral_meters[node] +
                                             right.lateral_meters[opposite]));
            worst = std::max(worst, std::abs(left.vertical_meters[node] -
                                             right.vertical_meters[opposite]));
        }
        Require(worst == 0.0,
                "the left wheel's rescan is not the exact mirror of the right "
                "wheel's");
        Require(left.total_arc_length_meters == right.total_arc_length_meters,
                "the two wheels measured different curve lengths");
    }

    {
        // Authored direction is metadata, not the phase authority. Resolving
        // both inputs to the physical right side before rescanning must make
        // the complete replacement grid independent of input order.
        const ProfilePoints descending =
            MakeSyntheticWheelProfile("synthetic_wheel_reversed", true);
        const WheelProfileControlNodes forward =
            rescanning.LayControlNodes(wheel, WheelSide::kRight);
        const WheelProfileControlNodes backward =
            rescanning.LayControlNodes(descending, WheelSide::kRight);
        Require(forward.lateral_meters == backward.lateral_meters &&
                    forward.vertical_meters == backward.vertical_meters &&
                    forward.total_arc_length_meters ==
                        backward.total_arc_length_meters,
                "authored point order changed the physical-right rescan phase");
    }

    {
        // The replacement nodes define a second natural cubic spline, not a
        // control polygon. Rebuild that spline independently and require every
        // published outline value and slope to come from it; then show that the
        // curved fixture is observably different from straight interpolation
        // between the same replacement nodes.
        const WheelProfileControlNodes nodes =
            rescanning.LayControlNodes(wheel, WheelSide::kRight);
        const NaturalCubicSpline second_pass(nodes.lateral_meters,
                                             nodes.vertical_meters);
        double worst_linear_difference = 0.0;
        for (std::size_t index = 0; index < rescan_outline.station_meters.size();
             ++index) {
            const double station = rescan_outline.station_meters[index];
            Require(rescan_outline.height_meters[index] ==
                            second_pass.Evaluate(station) &&
                        rescan_outline.height_slope[index] ==
                            second_pass.EvaluateFirstDerivative(station),
                    "the rescan outline did not use its second natural spline");
            if (station <= nodes.lateral_meters.front() ||
                station >= nodes.lateral_meters.back()) {
                continue;
            }
            const auto upper = std::upper_bound(nodes.lateral_meters.begin(),
                                                nodes.lateral_meters.end(),
                                                station);
            const std::size_t above =
                static_cast<std::size_t>(upper - nodes.lateral_meters.begin());
            const std::size_t below = above - 1;
            const double parameter =
                (station - nodes.lateral_meters[below]) /
                (nodes.lateral_meters[above] - nodes.lateral_meters[below]);
            const double linear =
                nodes.vertical_meters[below] +
                parameter * (nodes.vertical_meters[above] -
                             nodes.vertical_meters[below]);
            worst_linear_difference =
                std::max(worst_linear_difference,
                         std::abs(rescan_outline.height_meters[index] - linear));
        }
        Require(worst_linear_difference > 1.0e-8,
                "the second natural spline collapsed to a control polygon");
    }

    {
        // The arc-length primitive on its own. A straight segment's length is
        // its chord, which is the one case with an answer in closed form.
        const std::vector<double> knots{0.0, 1.0, 2.0, 3.0};
        const std::vector<double> values{0.0, 0.5, 1.0, 1.5};
        const NaturalCubicSpline straight(knots, values);
        const double expected = 3.0 * std::hypot(1.0, 0.5);
        const std::vector<double> cumulative =
            AccumulateProfileArcLength(straight, knots);
        Require(std::abs(cumulative.back() - expected) < 1.0e-12,
                "the arc length of a straight profile is not its chord");
        Require(cumulative.front() == 0.0,
                "the cumulative arc length does not start at zero");

        const double halfway = FindLateralCoordinateAtArcLength(
            straight, knots, cumulative, 0.5 * expected);
        Require(std::abs(halfway - 1.5) < 1.0e-12,
                "the halfway point of a straight profile is not its midpoint");
        RequireRefusal(
            [&] {
                (void)FindLateralCoordinateAtArcLength(straight, knots, cumulative,
                                                       2.0 * expected);
            },
            "outside the curve's range", "an arc length beyond the profile");
    }

    RequireRefusal(
        [&] {
            const ProfilePoints rail = ProfilePoints::FromAuthoredOrder(
                ProfileRole::kRail, "synthetic_rail",
                std::vector<double>{-0.03, 0.0, 0.03},
                std::vector<double>{0.01, 0.0, 0.01});
            (void)rescanning.SampleVisibleOutline(rail, WheelSide::kRight,
                                                  kOutlineSamples);
        },
        "is a rail profile", "a rail profile handed to a wheel preparation");

    if (failures != 0) {
        return 1;
    }
    std::puts("wheel profile preprocessing verified");
    return 0;
}
