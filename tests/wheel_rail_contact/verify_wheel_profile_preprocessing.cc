// The two admitted wheel-profile preparations, their mutual exclusion, and the
// arc-length primitive one of them is built on.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
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
using orvd::wheel_rail_contact::WheelProfilePreparation;
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
// flange. The turn is what makes the two preparations disagree — one puts more
// points where the curve bends and the other spaces them evenly across it — so
// a fixture without it cannot tell them apart.
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
        WheelProfilePreprocessingConfiguration{0.0015, 0.0}};
    const WheelProfilePreprocessing rediscretising{
        WheelProfilePreprocessingConfiguration{0.0, 0.0015}};

    Require(plain.preparation() == WheelProfilePreparation::kAuthoredNodes &&
                rescanning.preparation() ==
                    WheelProfilePreparation::kEqualArcLengthRescan &&
                rediscretising.preparation() ==
                    WheelProfilePreparation::kSourceLateralRediscretisation,
            "a configuration did not resolve to the preparation it selects");

    // The rule that has to be enforceable rather than merely true.
    RequireRefusal(
        [] {
            WheelProfilePreprocessing both{
                WheelProfilePreprocessingConfiguration{0.001, 0.001}};
            (void)both;
        },
        "at most one may be active",
        "a configuration selecting both preparations");
    RequireRefusal(
        [] {
            WheelProfilePreprocessing negative{
                WheelProfilePreprocessingConfiguration{-0.001, 0.0}};
            (void)negative;
        },
        "not negative", "a negative rescan step");

    const WheelProfileOutline plain_outline =
        plain.SampleVisibleOutline(wheel, WheelSide::kRight, kOutlineSamples);
    const WheelProfileOutline rescan_outline =
        rescanning.SampleVisibleOutline(wheel, WheelSide::kRight, kOutlineSamples);
    const WheelProfileOutline rediscretised_outline =
        rediscretising.SampleVisibleOutline(wheel, WheelSide::kRight,
                                            kOutlineSamples);

    // The discriminating comparison: the two preparations produce different
    // kinds of artefact, and the sampled outline is where they can be held
    // against each other point by point.
    const double between_preparations =
        WorstOutlineDifference(rescan_outline, rediscretised_outline);
    Require(between_preparations > 1.0e-6,
            "the two preparations produce the same outline, so nothing in this "
            "test can tell them apart");
    Require(WorstOutlineDifference(plain_outline, rediscretised_outline) > 1.0e-6,
            "the rediscretisation left the outline as the authored nodes gave it");

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
        // The rediscretisation marches from the authored first point, so which
        // end carries the short remainder is a property of how the asset was
        // written. Reversing the authored order must therefore move it.
        const ProfilePoints descending =
            MakeSyntheticWheelProfile("synthetic_wheel_reversed", true);
        const WheelProfileControlNodes forward =
            rediscretising.LayControlNodes(wheel, WheelSide::kRight);
        const WheelProfileControlNodes backward =
            rediscretising.LayControlNodes(descending, WheelSide::kRight);
        Require(forward.lateral_meters.front() == backward.lateral_meters.front() &&
                    forward.lateral_meters.back() == backward.lateral_meters.back(),
                "reversing the authored order moved an endpoint");

        const std::size_t forward_last = forward.lateral_meters.size() - 1;
        const double forward_remainder =
            forward.lateral_meters[forward_last] -
            forward.lateral_meters[forward_last - 1];
        const double backward_remainder =
            backward.lateral_meters[1] - backward.lateral_meters[0];
        Require(forward_remainder < 0.0015 - 1.0e-9 &&
                    backward_remainder < 0.0015 - 1.0e-9,
                "the fixture has no short remainder interval to place");
        Require(std::abs(forward_remainder - backward_remainder) < 1.0e-12,
                "the remainder changed size when the authored order was "
                "reversed");
        double worst = 0.0;
        for (std::size_t node = 1; node + 1 < forward.lateral_meters.size();
             ++node) {
            worst = std::max(worst, std::abs(forward.lateral_meters[node] -
                                             backward.lateral_meters[node]));
        }
        Require(worst > 1.0e-6,
                "reversing the authored order left every interior node where it "
                "was, so the march is not anchored on the authored first point");
    }

    {
        // The control polygon is a polygon: between its nodes the outline is a
        // straight line and its slope is constant. That is the visible
        // difference from the other two preparations, which are curves.
        const WheelProfileControlNodes nodes =
            rediscretising.LayControlNodes(wheel, WheelSide::kRight);
        const double first = nodes.lateral_meters[10];
        const double second = nodes.lateral_meters[11];
        const WheelProfileOutline dense = rediscretising.SampleVisibleOutline(
            wheel, WheelSide::kRight, kOutlineSamples);
        double worst_slope_spread = 0.0;
        double reference = 0.0;
        bool have_reference = false;
        for (std::size_t index = 0; index < dense.station_meters.size(); ++index) {
            const double station = dense.station_meters[index];
            if (station <= first || station >= second) {
                continue;
            }
            if (!have_reference) {
                reference = dense.height_slope[index];
                have_reference = true;
                continue;
            }
            worst_slope_spread =
                std::max(worst_slope_spread,
                         std::abs(dense.height_slope[index] - reference));
        }
        Require(have_reference,
                "no outline station fell inside a control interval");
        Require(worst_slope_spread == 0.0,
                "the rediscretised outline's slope varies inside a control "
                "interval, so it is not a polygon");
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
