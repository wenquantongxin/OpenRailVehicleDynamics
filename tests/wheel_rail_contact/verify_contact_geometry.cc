// The contact geometry solve: where a wheel and a rail overlap and what shape
// the overlap has.
//
// Every fixture here is a shape whose overlap can be written down. A flat wheel
// on a parabolic rail gives a closed form for the patch's width, area, depth and
// centroid height; a flat wheel on a flat rail gives one for the longitudinal
// extent, because the overlap of a cylinder and a plane is a circular chord.
// Nothing is compared against a remembered number: each assertion is the
// geometry, evaluated independently of the code under test.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "allocation_probe.h"
#include "orvd/wheel_rail_contact/contact_geometry.h"

namespace {

using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::ContactGeometryConfiguration;
using orvd::wheel_rail_contact::ContactGeometrySolver;
using orvd::wheel_rail_contact::ContactGeometryWorkspace;
using orvd::wheel_rail_contact::ContactPatchSet;
using orvd::wheel_rail_contact::ContactPoseScalars;
using orvd::wheel_rail_contact::ProfilePoints;
using orvd::wheel_rail_contact::ProfileRole;
using orvd::wheel_rail_contact::WheelProfilePreprocessingConfiguration;
using orvd::wheel_rail_contact::WheelSide;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "contact geometry: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double relative_tolerance,
                  std::string_view what) {
    const double scale = std::max(std::abs(expected), 1.0e-30);
    if (!(std::abs(actual - expected) <= relative_tolerance * scale)) {
        std::fprintf(stderr,
                     "contact geometry: %.*s (expected %.17g, got %.17g, "
                     "relative %.3g)\n",
                     static_cast<int>(what.size()), what.data(), expected, actual,
                     std::abs(actual - expected) / scale);
        ++failures;
    }
}

void RequireAbsoluteClose(double actual, double expected,
                          double absolute_tolerance, std::string_view what) {
    if (!(std::abs(actual - expected) <= absolute_tolerance)) {
        std::fprintf(stderr,
                     "contact geometry: %.*s (expected %.17g, got %.17g, "
                     "absolute %.3g)\n",
                     static_cast<int>(what.size()), what.data(), expected, actual,
                     std::abs(actual - expected));
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
                         "contact geometry: %.*s was refused for another reason: "
                         "%s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

// A profile sampled from a function of the lateral coordinate.
ProfilePoints Sample(ProfileRole role, std::string identifier, double half_width,
                     int points, const std::function<double(double)>& height) {
    std::vector<double> lateral;
    std::vector<double> vertical;
    lateral.reserve(static_cast<std::size_t>(points));
    vertical.reserve(static_cast<std::size_t>(points));
    for (int index = 0; index < points; ++index) {
        const double at = -half_width +
                          2.0 * half_width * (static_cast<double>(index) /
                                              static_cast<double>(points - 1));
        lateral.push_back(at);
        vertical.push_back(height(at));
    }
    return ProfilePoints::FromAuthoredOrder(role, std::move(identifier),
                                            std::move(lateral), std::move(vertical));
}

constexpr double kNominalRadius = 0.42;

ContactGeometryConfiguration DefaultConfiguration() {
    ContactGeometryConfiguration configuration;
    configuration.nominal_rolling_radius_meters = kNominalRadius;
    configuration.envelope_bin_width_meters = 2.0e-5;
    configuration.outline_sample_count = 1000;
    configuration.contact_gap_epsilon_meters = 0.0;
    configuration.island_merge_gap_tolerance_meters = 1.0e-6;
    configuration.island_quadrature_stations = 120;
    configuration.rail_cant_radians = 0.024994792;
    return configuration;
}

// A pose with the wheel pushed straight down into the rail by `penetration`,
// its profile datum `lateral` across from the rail's.
ContactPoseScalars StraightDown(double lateral, double penetration) {
    ContactPoseScalars pose;
    pose.roll_radians = 0.0;
    pose.yaw_radians = 0.0;
    pose.lateral_offset_meters = lateral;
    // A raise is positive up, so a penetration is a negative raise.
    pose.vertical_raise_meters = -penetration;
    return pose;
}

}  // namespace

int main() {
    const WheelProfilePreprocessingConfiguration no_preparation;
    // A wheel with no profile at all: a plain cylinder. Its silhouette is
    // exactly its own profile, so every projection step can be checked against
    // arithmetic instead of against another interpolant.
    const ProfilePoints flat_wheel =
        Sample(ProfileRole::kWheel, "flat_wheel", 0.05, 1001,
               [](double) { return 0.0; });

    {
        // A flat wheel on a parabolic rail. The overlap is the region between a
        // horizontal line at the penetration depth and the parabola, so its
        // width, area, depth and centroid height are all elementary.
        const double crown_curvature = 1.667;
        const double penetration = 1.0e-4;
        const double half_contact = std::sqrt(penetration / crown_curvature);
        const ProfilePoints parabolic_rail =
            Sample(ProfileRole::kRail, "parabolic_rail", 0.06, 1201,
                   [&](double at) { return crown_curvature * at * at; });

        const ContactGeometrySolver solver(flat_wheel, parabolic_rail,
                                           WheelSide::kRight, no_preparation,
                                           DefaultConfiguration());
        ContactGeometryWorkspace workspace;
        const ContactPatchSet patches =
            solver.Solve(StraightDown(0.0, penetration), workspace);

        Require(patches.count == 1, "a single crown produced other than one patch");
        Require(patches.island_count == 1 && patches.merged_island_count == 1,
                "a single crown produced other than one island");
        if (patches.count == 1) {
            const auto& patch = patches.patches[0];
            RequireClose(patch.chord_width_meters, 2.0 * half_contact, 1.0e-4,
                         "the chord width is not where the parabola crosses the "
                         "penetration depth");
            RequireClose(patch.cross_section_area_square_meters,
                         (4.0 / 3.0) * penetration * half_contact, 1.0e-3,
                         "the overlap area is not the area between the line and "
                         "the parabola");
            RequireClose(patch.centroid_vertical_meters, 0.6 * penetration, 1.0e-3,
                         "the centroid height of a parabolic overlap is not three "
                         "fifths of its depth");
            Require(std::abs(patch.centroid_lateral_meters) < 1.0e-6,
                    "a symmetric overlap has an off-centre centroid");

            // The quadrature grid is coarser than the grid the island was found
            // on and does not contain the crown, so the depth it reports is
            // slightly short of the true depth. That is the design, not an
            // error, and it is asserted rather than tolerated.
            Require(patch.vertical_penetration_meters <= penetration,
                    "the quadrature reported a deeper overlap than the geometry "
                    "has");
            RequireClose(patch.vertical_penetration_meters, penetration, 1.0e-3,
                         "the quadrature depth is far from the true depth");

            // A flat wheel's envelope has no slope, so its arc across the patch
            // is the straight chord.
            RequireClose(patch.arc_width_meters, patch.chord_width_meters, 1.0e-12,
                         "a flat wheel's arc width differs from its chord width");
            RequireClose(patch.rolling_radius_meters, kNominalRadius, 1.0e-12,
                         "a flat wheel does not roll on its nominal radius");
            Require(std::abs(patch.wheel_station_meters) < 1.0e-6,
                    "a centred wheel contacts away from its own profile centre");

            // The rail's curvature at the crown of z = c*y^2 is 2c; the flat
            // wheel has none.
            RequireClose(patch.rail_curvature_per_meter, 2.0 * crown_curvature,
                         1.0e-6, "the rail curvature at the crown is not twice "
                                 "the parabola's coefficient");
            Require(std::abs(patch.wheel_curvature_per_meter) < 1.0e-9,
                    "a flat wheel was given a curvature");
        }

        {
            // Moving the wheel across moves the patch with it, and the patch
            // stays where the rail's crown is: the overlap is centred on the
            // crown, not on the wheel.
            const ContactPatchSet moved =
                solver.Solve(StraightDown(0.004, penetration), workspace);
            Require(moved.count == 1, "an offset wheel lost its patch");
            if (moved.count == 1) {
                Require(std::abs(moved.patches[0].centroid_lateral_meters) < 1.0e-6,
                        "the patch moved off the crown when the wheel moved");
                RequireClose(moved.patches[0].wheel_station_meters, -0.004, 1.0e-4,
                             "the contact's station on the wheel did not move "
                             "with the wheel");
            }
        }

        {
            // Lifted clear, there is nothing.
            const ContactPatchSet clear =
                solver.Solve(StraightDown(0.0, -1.0e-4), workspace);
            Require(clear.count == 0 && clear.island_count == 0,
                    "a wheel held clear of the rail was given a patch");
        }
    }

    {
        // A flat wheel on a flat rail. Now the overlap spans the whole width and
        // the only interesting number is how far it reaches along the track,
        // which is the chord a cylinder cuts through a plane.
        const double penetration = 2.0e-4;
        const ProfilePoints flat_rail = Sample(ProfileRole::kRail, "flat_rail", 0.06,
                                               1201, [](double) { return 0.0; });
        const ContactGeometrySolver solver(flat_wheel, flat_rail, WheelSide::kRight,
                                           no_preparation, DefaultConfiguration());
        ContactGeometryWorkspace workspace;
        const ContactPatchSet patches =
            solver.Solve(StraightDown(0.0, penetration), workspace);

        Require(patches.count == 1, "a flat pair produced other than one patch");
        if (patches.count == 1) {
            const auto& patch = patches.patches[0];
            const double expected_chord =
                2.0 * std::sqrt(2.0 * kNominalRadius * penetration -
                                penetration * penetration);
            // The longitudinal root solve owns a 20 nm absolute chord budget.
            // This fixture checks that physical contract against the closed
            // form without retaining a picometre-scale historical requirement
            // or exposing the private bisection count.
            RequireAbsoluteClose(
                patch.longitudinal_length_meters, expected_chord, 2.0e-8,
                "the longitudinal extent is not the chord a cylinder cuts "
                "through a plane");
            RequireClose(patch.vertical_penetration_meters, penetration, 1.0e-12,
                         "a flat pair does not overlap by the depth it was "
                         "pushed in");

            // The common normal of a flat rail under an unrolled wheelset is
            // straight down. The frame angle is not: it carries the laying
            // cant, and on the right rail the cant is negative.
            Require(std::abs(patch.common_normal_angle_radians) < 1.0e-15,
                    "a flat rail under an upright wheelset produced a tilted "
                    "common normal");
            RequireClose(patch.rail_slope_angle_radians, -0.024994792, 1.0e-12,
                         "the frame angle does not carry the laying cant on the "
                         "right rail");
        }

        {
            // The left rail takes the cant the other way. Everything else about
            // a symmetric fixture is unchanged.
            const ContactGeometrySolver mirrored(flat_wheel, flat_rail,
                                                 WheelSide::kLeft, no_preparation,
                                                 DefaultConfiguration());
            ContactGeometryWorkspace mirrored_workspace;
            const ContactPatchSet left =
                mirrored.Solve(StraightDown(0.0, penetration), mirrored_workspace);
            Require(left.count == 1, "the left side lost its patch");
            if (left.count == 1 && patches.count == 1) {
                RequireClose(left.patches[0].rail_slope_angle_radians, 0.024994792,
                             1.0e-12,
                             "the frame angle does not reverse with the side");
                RequireClose(left.patches[0].longitudinal_length_meters,
                             patches.patches[0].longitudinal_length_meters, 1.0e-12,
                             "a symmetric fixture gave its two sides different "
                             "longitudinal extents");
            }
        }

        {
            // Rolling the wheelset tilts the common normal with it. A flat rail
            // has no slope of its own, so the whole of the normal angle is the
            // roll — that is what makes this fixture able to tell the two
            // angles apart.
            ContactPoseScalars rolled = StraightDown(0.0, penetration);
            rolled.roll_radians = 0.003;
            const ContactPatchSet tilted = solver.Solve(rolled, workspace);
            Require(tilted.count >= 1, "a rolled wheelset lost contact");
            if (tilted.count >= 1) {
                RequireClose(tilted.patches[0].common_normal_angle_radians, -0.003,
                             1.0e-9,
                             "the common normal did not take up the wheelset's "
                             "roll");
            }
        }

        {
            // Yaw shortens the chord the overlap is measured along.
            //
            // It does not move the contact along the track here, and that is
            // the geometry rather than an omission: a cylinder against a plane
            // touches at the bottom of the cylinder whichever way it is turned,
            // so the contact stays on the wheelset's own cross section. The
            // sloped-rail fixture below is where the yaw moves it.
            ContactPoseScalars yawed = StraightDown(0.0, penetration);
            yawed.yaw_radians = 0.01;
            const ContactPatchSet turned = solver.Solve(yawed, workspace);
            Require(turned.count >= 1, "a yawed wheelset lost contact");
            if (turned.count >= 1 && patches.count == 1) {
                Require(turned.patches[0].longitudinal_length_meters > 0.0 &&
                            turned.patches[0].longitudinal_length_meters <
                                patches.patches[0].longitudinal_length_meters,
                        "yaw did not shorten the longitudinal extent");
                Require(std::abs(turned.patches[0].common_normal_angle_radians) <
                            1.0e-15,
                        "a flat rail under a yawed wheelset tilted the common "
                        "normal");
                Require(std::abs(
                            turned.patches[0].rail_reference_longitudinal_meters) <
                            1.0e-12,
                        "a cylinder on a plane left its own cross section under "
                        "yaw");
            }
        }

        {
            // A deterministic deep overlap extends beyond the
            // three-dimensional search bracket. It is a branch fixture, not a
            // qualified vehicle state: the geometry must preserve the positive
            // patch and publish an unavailable length for the force law to
            // handle in the same evaluation.
            constexpr double unresolved_penetration_meters = 0.014;
            const ContactPatchSet unresolved = solver.Solve(
                StraightDown(0.0, unresolved_penetration_meters), workspace);
            Require(unresolved.count == 1,
                    "an unresolved longitudinal length swallowed its positive "
                    "geometry patch");
            if (unresolved.count == 1) {
                Require(unresolved.patches[0].longitudinal_length_meters == 0.0,
                        "the geometry layer substituted a longitudinal length");
                Require(unresolved.patches[0].vertical_penetration_meters > 0.0 &&
                            unresolved.patches[0]
                                    .cross_section_area_square_meters > 0.0,
                        "the unresolved fixture did not preserve its measured "
                        "cross-section");
            }
        }

        {
            // Once the workspace has been through a solve it must not allocate
            // again. This runs inside the integrator's right-hand side.
            const ContactPoseScalars pose = StraightDown(0.0, penetration);
            (void)solver.Solve(pose, workspace);
            const AllocationScope scope;
            for (int step = 0; step < 8; ++step) {
                const ContactPatchSet repeat = solver.Solve(pose, workspace);
                Require(repeat.count == 1, "a repeated solve changed its answer");
            }
            Require(scope.allocations() == 0,
                    "the contact geometry solve allocated after its workspace "
                    "was warm");
        }
    }

    {
        // PrepareWorkspace must cover a later pose whose projection is wider
        // than the authored lateral span. A modest 0.2 profile slope and
        // -0.1-radian roll make the old unrotated-span capacity allocate eight
        // envelope buffers. These values are a mechanical-scale pressure point,
        // not a vehicle pose limit.
        const ProfilePoints sloped_wheel =
            Sample(ProfileRole::kWheel, "sloped_capacity_wheel", 0.05, 1001,
                   [](double at) { return 0.2 * at; });
        const ProfilePoints flat_rail =
            Sample(ProfileRole::kRail, "flat_capacity_rail", 0.06, 1201,
                   [](double) { return 0.0; });
        const ContactGeometrySolver solver(sloped_wheel, flat_rail,
                                           WheelSide::kRight, no_preparation,
                                           DefaultConfiguration());
        ContactGeometryWorkspace workspace;
        solver.PrepareWorkspace(workspace);
        ContactPoseScalars pose = StraightDown(0.0, -0.01);
        pose.roll_radians = -0.1;
        const AllocationScope scope;
        const ContactPatchSet clear = solver.Solve(pose, workspace);
        Require(clear.count == 0,
                "the workspace-capacity fixture unexpectedly made contact");
        Require(scope.allocations() == 0,
                "a prepared geometry workspace allocated at a wider projected "
                "pose");
    }

    {
        // Two crowns. Whether they are one patch or two is decided by how deep
        // the valley between them is, compared against the merge tolerance —
        // and by nothing else, which is what these two fixtures separate.
        const double penetration = 1.0e-4;
        const double crown_offset = 0.01;
        const auto make_rail = [&](double valley_excess) {
            const double coefficient =
                (penetration + valley_excess) / (crown_offset * crown_offset);
            return Sample(ProfileRole::kRail, "double_crown_rail", 0.06, 1201,
                          [&, coefficient](double at) {
                              const double shifted =
                                  at * at - crown_offset * crown_offset;
                              return coefficient * shifted * shifted /
                                     (crown_offset * crown_offset);
                          });
        };

        {
            // The valley is half a micrometre deep: shallower than the
            // tolerance, so the two islands are one patch.
            const ProfilePoints rail = make_rail(5.0e-7);
            const ContactGeometrySolver solver(flat_wheel, rail, WheelSide::kRight,
                                               no_preparation, DefaultConfiguration());
            ContactGeometryWorkspace workspace;
            const ContactPatchSet patches =
                solver.Solve(StraightDown(0.0, penetration), workspace);
            Require(patches.island_count == 2,
                    "a double crown did not produce two islands");
            Require(patches.merged_island_count == 1 && patches.count == 1,
                    "two islands separated by less than the merge tolerance were "
                    "not merged");
        }

        {
            // Five micrometres: deeper than the tolerance, so they stay two.
            const ProfilePoints rail = make_rail(5.0e-6);
            const ContactGeometrySolver solver(flat_wheel, rail, WheelSide::kRight,
                                               no_preparation, DefaultConfiguration());
            ContactGeometryWorkspace workspace;
            const ContactPatchSet patches =
                solver.Solve(StraightDown(0.0, penetration), workspace);
            Require(patches.island_count == 2 && patches.merged_island_count == 2 &&
                        patches.count == 2,
                    "two islands separated by more than the merge tolerance were "
                    "merged anyway");
            if (patches.count == 2) {
                Require(patches.patches[0].centroid_lateral_meters <
                            patches.patches[1].centroid_lateral_meters,
                        "the patches are not ordered across the track");
                // Each patch straddles its own crown but its area centroid
                // sits inboard of it: a quartic rises faster on the outboard
                // side of a crown than on the valley side, so the overlap is
                // wider inboard. The centroid is therefore strictly between the
                // valley and the crown, and asserting it sits *on* the crown
                // would be asserting a symmetry the shape does not have.
                Require(patches.patches[1].centroid_lateral_meters > 0.0 &&
                            patches.patches[1].centroid_lateral_meters < crown_offset,
                        "the outboard patch's centroid is not between the valley "
                        "and its crown");
                Require(patches.patches[0].centroid_lateral_meters < 0.0 &&
                            patches.patches[0].centroid_lateral_meters > -crown_offset,
                        "the inboard patch's centroid is not between the valley "
                        "and its crown");
                Require(std::abs(patches.patches[0].centroid_lateral_meters +
                                 patches.patches[1].centroid_lateral_meters) < 1.0e-9,
                        "a symmetric double crown gave asymmetric patches");
                // Each patch does contain its crown, which is where the rail is
                // highest and so where the overlap is deepest.
                RequireClose(patches.patches[1].vertical_penetration_meters,
                             penetration, 1.0e-3,
                             "a patch's deepest overlap is not the depth at its "
                             "crown, so the patch does not contain the crown");
            }
        }
    }

    {
        // A coned wheel on the crowned rail, yawed. The contact then sits where
        // the cone's slope matches the crown's, so the rail has a real slope
        // there, and the common normal is the rail's normal carried into the
        // wheelset's frame — through the roll *and* through the yaw.
        //
        // The yaw's part of that is easy to leave out and impossible to notice
        // on a level contact, because it multiplies the sine of an angle that is
        // then zero. Here it is not zero, and the closed form below is written
        // out with it.
        const double cone_slope = 0.05;
        const double crown_curvature = 1.667;
        const ProfilePoints coned_wheel =
            Sample(ProfileRole::kWheel, "coned_wheel", 0.05, 1001,
                   [&](double at) { return cone_slope * at; });
        const ProfilePoints crowned_rail =
            Sample(ProfileRole::kRail, "crowned_rail", 0.06, 1201,
                   [&](double at) { return crown_curvature * at * at; });
        ContactGeometryConfiguration configuration = DefaultConfiguration();
        // With no laying cant the reported frame angle is the rail's own
        // surface slope, which is what the closed form below needs.
        configuration.rail_cant_radians = 0.0;
        const ContactGeometrySolver solver(coned_wheel, crowned_rail,
                                           WheelSide::kRight, no_preparation,
                                           configuration);
        ContactGeometryWorkspace workspace;

        for (const double yaw : {0.0, 0.08, -0.15}) {
            for (const double roll : {0.0, 0.01}) {
                ContactPoseScalars pose = StraightDown(0.0, 0.0);
                pose.vertical_raise_meters = 2.75e-4;
                pose.yaw_radians = yaw;
                pose.roll_radians = roll;
                const ContactPatchSet patches = solver.Solve(pose, workspace);
                Require(patches.count == 1,
                        "the yawed coned fixture produced other than one patch");
                if (patches.count != 1) {
                    continue;
                }
                const auto& patch = patches.patches[0];
                const double rail_angle = patch.rail_slope_angle_radians;
                Require(std::abs(rail_angle) > 0.04,
                        "the coned fixture's rail slope is too small to show the "
                        "yaw factor");
                const double expected =
                    std::atan2(std::cos(yaw) * std::sin(rail_angle - roll),
                               std::cos(rail_angle - roll));
                RequireClose(patch.common_normal_angle_radians, expected, 1.0e-12,
                             "the common normal is not the rail's normal carried "
                             "through the roll and the yaw");
                if (yaw != 0.0) {
                    // Without the yaw factor the answer would simply be the
                    // rail's slope less the roll. Confirm the fixture separates
                    // the two.
                    Require(std::abs(expected - (rail_angle - roll)) > 1.0e-5,
                            "the yawed fixture does not separate the common "
                            "normal from the plain angle difference");
                }
            }
        }
    }

    {
        // What "upper envelope" means. The projected outline is collapsed to one
        // point per lateral bin, and the point kept is the one with the most
        // material — the part of the wheel that can actually touch. Keeping the
        // least would keep the hidden side.
        //
        // At the working bin width every sample gets its own bin and the rule
        // never bites, so the fixture coarsens the bins until a bin holds
        // several samples, and puts a narrow ridge on an otherwise flat wheel.
        // Kept, the ridge touches; discarded, nothing does.
        // The ridge is a smooth bump, not a step. A step is not a profile: the
        // spline the outline is traced with would ring at its edges and the
        // overshoot, not the ridge, would be what touched.
        const double ridge_height = 2.0e-4;
        const double ridge_width = 5.0e-4;
        const ProfilePoints ridged_wheel =
            Sample(ProfileRole::kWheel, "ridged_wheel", 0.05, 1001,
                   [&](double at) {
                       const double reduced = at / ridge_width;
                       return ridge_height * std::exp(-0.5 * reduced * reduced);
                   });
        const ProfilePoints flat_rail = Sample(ProfileRole::kRail, "flat_rail", 0.06,
                                               1201, [](double) { return 0.0; });
        ContactGeometryConfiguration configuration = DefaultConfiguration();
        // Fifty sample spacings to a bin, so one bin holds both the ridge and
        // the flat tread around it and the rule has to choose between them.
        configuration.envelope_bin_width_meters = 5.0e-3;
        configuration.rail_cant_radians = 0.0;
        const ContactGeometrySolver solver(ridged_wheel, flat_rail,
                                           WheelSide::kRight, no_preparation,
                                           configuration);
        ContactGeometryWorkspace workspace;

        // The flat part is held one hundred micrometres clear; only the ridge,
        // two hundred tall, reaches the rail.
        const ContactPatchSet patches =
            solver.Solve(StraightDown(0.0, -1.0e-4), workspace);
        Require(patches.count >= 1,
                "the outermost material in a bin was discarded, so a ridge that "
                "reaches the rail was not seen");
        if (patches.count >= 1) {
            Require(patches.patches[0].vertical_penetration_meters > 0.0 &&
                        patches.patches[0].vertical_penetration_meters <=
                            ridge_height - 1.0e-4 + 1.0e-9,
                    "the ridge's overlap is not the height it stands above the "
                    "clearance");
        }
        // And the same wheel held clear of even the ridge touches nothing.
        Require(solver.Solve(StraightDown(0.0, -2.5e-4), workspace).count == 0,
                "a wheel held clear of its own ridge still found contact");
    }

    {
        // Refusals.
        const ProfilePoints rail = Sample(ProfileRole::kRail, "flat_rail", 0.06, 1201,
                                          [](double) { return 0.0; });
        RequireRefusal(
            [&] {
                (void)ContactGeometrySolver(flat_wheel, flat_wheel, WheelSide::kRight,
                                            no_preparation, DefaultConfiguration());
            },
            "is a wheel profile", "a wheel profile handed in as the rail");
        RequireRefusal(
            [&] {
                ContactGeometryConfiguration configuration = DefaultConfiguration();
                configuration.island_quadrature_stations = 2;
                (void)ContactGeometrySolver(flat_wheel, rail, WheelSide::kRight,
                                            no_preparation, configuration);
            },
            "at least three quadrature stations",
            "an island quadrature too coarse to have an interior");
        RequireRefusal(
            [&] {
                ContactGeometryConfiguration configuration = DefaultConfiguration();
                configuration.nominal_rolling_radius_meters = 0.0;
                (void)ContactGeometrySolver(flat_wheel, rail, WheelSide::kRight,
                                            no_preparation, configuration);
            },
            "the nominal rolling radius", "a wheel with no radius");
        RequireRefusal(
            [&] {
                const ProfilePoints stub = ProfilePoints::FromAuthoredOrder(
                    ProfileRole::kRail, "stub", std::vector<double>{-0.01, 0.01},
                    std::vector<double>{0.0, 0.0});
                (void)ContactGeometrySolver(flat_wheel, stub, WheelSide::kRight,
                                            no_preparation, DefaultConfiguration());
            },
            "at least three points", "a rail of two points");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("contact geometry verified");
    return 0;
}
