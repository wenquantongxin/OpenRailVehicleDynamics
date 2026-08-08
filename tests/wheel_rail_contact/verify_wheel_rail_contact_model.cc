// The assembled contact model: geometry, normal law, creepages and tangential
// solution, reduced to a paired wrench.
//
// The stages are checked individually elsewhere. What is checked here is what
// only the assembly can get wrong: that the frame the creepages are resolved in
// is the frame the force is written in, that the wrench is the force each stage
// actually produced, that the two halves survive being moved, and that a patch
// which is geometrically touching but carrying nothing does not become a force.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "allocation_probe.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"

namespace {

using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::ContactPoseScalars;
using orvd::wheel_rail_contact::ContactPatch;
using orvd::wheel_rail_contact::ProfilePoints;
using orvd::wheel_rail_contact::ProfileRole;
using orvd::wheel_rail_contact::ProfileTrackRollTransport;
using orvd::wheel_rail_contact::SpatialWrench;
using orvd::wheel_rail_contact::TransportWrench;
using orvd::wheel_rail_contact::WheelProfilePreprocessingConfiguration;
using orvd::wheel_rail_contact::WheelRailContactConfiguration;
using orvd::wheel_rail_contact::WheelRailContactInput;
using orvd::wheel_rail_contact::WheelRailContactModel;
using orvd::wheel_rail_contact::WheelRailContactResult;
using orvd::wheel_rail_contact::WheelRailContactWorkspace;
using orvd::wheel_rail_contact::WheelSide;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "wheel rail contact model: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double relative_tolerance,
                  std::string_view what) {
    const double scale = std::max(std::abs(expected), 1.0e-30);
    if (!(std::abs(actual - expected) <= relative_tolerance * scale)) {
        std::fprintf(stderr,
                     "wheel rail contact model: %.*s (expected %.17g, got %.17g, "
                     "relative %.3g)\n",
                     static_cast<int>(what.size()), what.data(), expected, actual,
                     std::abs(actual - expected) / scale);
        ++failures;
    }
}

ProfilePoints Sample(ProfileRole role, std::string identifier, double half_width,
                     int points, const std::function<double(double)>& height) {
    std::vector<double> lateral;
    std::vector<double> vertical;
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
constexpr double kConeSlope = 0.05;
constexpr double kCrownCurvature = 1.667;
constexpr double kSpeed = 16.666666666666668;

WheelRailContactConfiguration Configuration() {
    WheelRailContactConfiguration configuration;
    configuration.geometry.nominal_rolling_radius_meters = kNominalRadius;
    configuration.geometry.outline_sample_count = 1000;
    configuration.geometry.island_quadrature_stations = 120;
    // The laying cant is left out of this fixture so that the frame angle is
    // the rail's own surface slope and nothing else. The cant's arithmetic is
    // checked in the geometry's own tests; mixing it in here would only make
    // the angles below harder to write down.
    configuration.geometry.rail_cant_radians = 0.0;
    configuration.tangential.longitudinal_cells = 21;
    configuration.tangential.lateral_strips = 21;
    configuration.tangential.refinement_width = 0.01;
    return configuration;
}

Eigen::Vector3d RailMaterialPoint(const WheelRailContactInput& input,
                                  const ContactPatch& patch) {
    return input.rail_frame.origin_track_meters +
           input.rail_frame.rotation_track_from_profile *
               Eigen::Vector3d(patch.rail_reference_longitudinal_meters,
                               patch.rail_reference_lateral_meters,
                               patch.rail_reference_vertical_meters);
}

Eigen::Vector3d WheelSurfacePoint(
    const WheelRailContactInput& input,
    const orvd::wheel_rail_contact::WheelRailContactPatchResult& patch) {
    return input.wheel.origin_track_meters +
           input.wheel.rotation_track_from_profile *
               Eigen::Vector3d(
                   patch.geometry.wheel_longitudinal_meters,
                   patch.geometry.wheel_station_meters,
                   patch.geometry.rolling_radius_meters -
                       0.5 * patch.normal.equivalent_penetration_meters);
}

}  // namespace

int main() {
    const WheelProfilePreprocessingConfiguration no_preparation;
    // A coned wheel on a crowned rail: the contact then sits off the crown, at
    // a real angle, which is what makes the frame and the spin non-trivial.
    const ProfilePoints wheel =
        Sample(ProfileRole::kWheel, "coned_wheel", 0.05, 1001,
               [](double at) { return kConeSlope * at; });
    const ProfilePoints rail =
        Sample(ProfileRole::kRail, "crowned_rail", 0.06, 1201,
               [](double at) { return kCrownCurvature * at * at; });

    const WheelRailContactModel model(wheel, rail, WheelSide::kRight, no_preparation,
                                      Configuration());
    WheelRailContactWorkspace workspace;

    // The wheel's axle sits one nominal radius above the rail datum, and the
    // rail's own frame is the track's.
    const Eigen::Vector3d axle(0.0, 0.0, -kNominalRadius);

    WheelRailContactInput resting;
    resting.pose.roll_radians = 0.0;
    resting.pose.yaw_radians = 0.0;
    resting.pose.lateral_offset_meters = 0.0;
    resting.pose.vertical_raise_meters = 2.75e-4;
    resting.wheel.origin_track_meters = axle;

    {
        // A wheelset pressed into the rail and not moving. Nothing creeps, so
        // the whole force is normal — and being normal is checkable: it must be
        // exactly the negative of the normal force along the contact frame's
        // third axis.
        const WheelRailContactResult result = model.Evaluate(resting, workspace);
        Require(result.count == 1, "a resting wheel produced other than one patch");
        Require(result.geometric_patch_count == 1 &&
                    result.analytic_longitudinal_length_fallback_count == 0,
                "the ordinary contact did not report one successful "
                "three-dimensional length resolution");
        if (result.count == 1) {
            const auto& patch = result.patches[0];
            Require(patch.geometry.vertical_penetration_meters > 3.0e-5 &&
                        patch.geometry.vertical_penetration_meters < 5.0e-4,
                    "the resting fixture is not at a plausible penetration");
            Require(patch.normal.in_contact && patch.normal.normal_force_newtons > 0.0,
                    "a pressed wheel carried no normal force");
            RequireClose(patch.normal.damping_force_newtons, 0.0, 0.0,
                         "a stationary wheel acquired a damping force");
            Require(patch.creepages.longitudinal == 0.0 &&
                        patch.creepages.lateral == 0.0 &&
                        patch.creepages.spin_per_meter == 0.0,
                    "a stationary wheel acquired a creepage");
            Require(patch.tangential.longitudinal_force_newtons == 0.0 &&
                        patch.tangential.lateral_force_newtons == 0.0,
                    "a stationary wheel acquired a tangential force");

            // The force is purely normal, and the normal is the one built from
            // the rail's slope angle. This is the check that the frame the
            // force is written in is the frame the model says it is.
            const double angle = patch.geometry.rail_slope_angle_radians;
            const Eigen::Vector3d normal(0.0, -std::sin(angle), std::cos(angle));
            const Eigen::Vector3d expected =
                -patch.normal.normal_force_newtons * normal;
            Require((patch.wrench.rail_on_wheel.force_newtons - expected)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-9,
                    "the wrench is not the normal force along the contact "
                    "normal");
            // A wheel on a rail is pushed up, and up is the negative vertical.
            Require(patch.wrench.rail_on_wheel.force_newtons.z() < 0.0,
                    "the contact pushed the wheel down");
            // The contact sits where the cone's slope matches the crown's, off
            // the crown itself.
            RequireClose(patch.geometry.centroid_lateral_meters,
                         kConeSlope / (2.0 * kCrownCurvature), 5.0e-2,
                         "the coned wheel did not settle where its slope matches "
                         "the crown's");
            Require(std::abs(angle) > 0.04,
                    "the fixture's contact angle is too small to discriminate");
        }
    }

    // Rolling forward. The wheel's rotation rate is chosen so that the contact
    // point's own velocity vanishes — pure rolling — which leaves the spin as
    // the only creepage. Finding it needs the contact point, so the fixture
    // solves once to locate it and then rolls.
    WheelRailContactInput rolling = resting;
    rolling.wheel.origin_velocity_track_meters_per_second =
        Eigen::Vector3d(kSpeed, 0.0, 0.0);
    rolling.wheel.arc_rate_meters_per_second = kSpeed;
    {
        const WheelRailContactResult located = model.Evaluate(resting, workspace);
        Require(located.count == 1, "the locating solve lost its patch");
        if (located.count == 1) {
            // Creepages are evaluated at the rail material reference point R,
            // not at the wheel surface force point P. Pure rolling must
            // therefore be tuned with R's lever from the profile datum.
            const double lever =
                RailMaterialPoint(resting, located.patches[0].geometry).z() -
                axle.z();
            const double rate = -kSpeed / lever;
            rolling.wheel.angular_velocity_track_radians_per_second =
                Eigen::Vector3d(0.0, rate, 0.0);
            rolling.wheel.wheel_pitch_rate_radians_per_second = rate;
        }
    }

    {
        // Pure rolling on a tilted contact. The two translational creepages
        // vanish and the spin does not, because the wheel's own rotation
        // resolved onto a tilted normal is a rotation about that normal. A
        // model that produced no force here would be losing a real part of the
        // guidance force.
        const WheelRailContactResult result = model.Evaluate(rolling, workspace);
        Require(result.count == 1, "a rolling wheel produced other than one patch");
        if (result.count == 1) {
            const auto& patch = result.patches[0];
            Require(std::abs(patch.creepages.longitudinal) < 1.0e-6 &&
                        std::abs(patch.creepages.lateral) < 1.0e-6,
                    "the pure-rolling fixture has a translational creepage, so it "
                    "does not isolate the spin");
            Require(std::abs(patch.creepages.spin_per_meter) > 0.05,
                    "a tilted rolling contact produced no spin");
            Require(std::abs(patch.tangential.lateral_force_newtons) > 1.0,
                    "spin alone produced no lateral force");
            RequireClose(patch.friction_coefficient,
                         Configuration().friction.static_coefficient, 1.0e-6,
                         "a contact with no translational creepage did not keep "
                         "its static friction");
        }
    }

    {
        // A driven wheel. The longitudinal creepage now dominates, and the
        // whole chain must respect the bound the tangential stage reports.
        WheelRailContactInput driven = rolling;
        driven.wheel.angular_velocity_track_radians_per_second *= 1.01;
        driven.wheel.wheel_pitch_rate_radians_per_second *= 1.01;
        const WheelRailContactResult result = model.Evaluate(driven, workspace);
        Require(result.count == 1, "a driven wheel produced other than one patch");
        if (result.count == 1) {
            const auto& patch = result.patches[0];
            Require(patch.creepages.longitudinal != 0.0,
                    "over-rotating the wheel produced no longitudinal creepage");
            Require(patch.friction_coefficient <
                        Configuration().friction.static_coefficient,
                    "a slipping contact kept its static friction");

            // The wrench, rebuilt here from the two force stages and the frame
            // angle rather than taken from the model. Every sign in the
            // assembly is in this one comparison.
            const double angle = patch.geometry.rail_slope_angle_radians;
            Eigen::Matrix3d frame;
            frame << 1.0, 0.0, 0.0,
                     0.0, std::cos(angle), -std::sin(angle),
                     0.0, std::sin(angle), std::cos(angle);
            const Eigen::Vector3d expected =
                frame * Eigen::Vector3d(patch.tangential.longitudinal_force_newtons,
                                        patch.tangential.lateral_force_newtons,
                                        -patch.normal.normal_force_newtons);
            Require((patch.wrench.rail_on_wheel.force_newtons - expected)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-9,
                    "the emitted wrench is not the force the two stages "
                    "produced, rotated by the contact frame");
            Require(patch.wrench.rail_on_wheel.moment_newton_meters ==
                        Eigen::Vector3d::Zero(),
                    "a moment accompanied the force at the contact point, which "
                    "this formulation suppresses");
        }
    }

    {
        // The pair survives being moved. Transporting both halves to any other
        // point leaves them cancelling — and transporting then negating is the
        // same as negating then transporting, which is what lets a consumer
        // reduce the two sides about different points safely.
        const WheelRailContactResult result = model.Evaluate(rolling, workspace);
        Require(result.count == 1, "the transport fixture lost its patch");
        if (result.count == 1) {
            const auto& patch = result.patches[0];
            const Eigen::Vector3d elsewhere(1.7, -0.9, 2.3);
            const SpatialWrench moved_wheel = TransportWrench(
                patch.wrench.rail_on_wheel, patch.wrench.contact_point_meters,
                elsewhere);
            const SpatialWrench moved_rail = TransportWrench(
                patch.wrench.wheel_on_rail, patch.wrench.contact_point_meters,
                elsewhere);
            Require((moved_wheel.force_newtons + moved_rail.force_newtons)
                            .cwiseAbs()
                            .maxCoeff() == 0.0,
                    "the transported pair's forces do not cancel");
            Require((moved_wheel.moment_newton_meters +
                     moved_rail.moment_newton_meters)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-9,
                    "the transported pair's moments do not cancel");
            // Moving a force off its own line of action produces a moment. If
            // it did not, this check would pass for a broken transport too.
            Require(moved_wheel.moment_newton_meters.norm() > 1.0,
                    "moving the wrench a metre and a half produced no moment, so "
                    "this fixture cannot tell a working transport from a "
                    "no-op");
        }
    }

    {
        // Where the profile/track roll correction reaches, and where it does
        // not. Its roll offset tilts the frame the contact resolves its forces
        // in and nothing else inside this model: the patch's shape, the point
        // it reduces about and the relative velocity at that point are all
        // untouched. The lateral and vertical offsets never arrive here at all
        // — they enter the pose reduction upstream.
        const WheelRailContactResult plain = model.Evaluate(rolling, workspace);
        Require(plain.count == 1, "the transport fixture lost its patch");

        WheelRailContactInput suppressed = rolling;
        suppressed.roll_transport = ProfileTrackRollTransport{};
        const WheelRailContactResult unchanged = model.Evaluate(suppressed, workspace);
        Require(unchanged.count == plain.count &&
                    unchanged.patches[0].wrench.rail_on_wheel.force_newtons ==
                        plain.patches[0].wrench.rail_on_wheel.force_newtons &&
                    unchanged.patches[0].contact_frame_angle_radians ==
                        plain.patches[0].contact_frame_angle_radians,
                "a suppressed roll transport is not bit-for-bit no transport");

        WheelRailContactInput corrected = rolling;
        corrected.roll_transport.roll_offset_radians = 1.5e-3;
        corrected.roll_transport.lateral_offset_meters = 2.0e-6;
        corrected.roll_transport.vertical_offset_meters = -3.0e-6;
        const WheelRailContactResult tilted = model.Evaluate(corrected, workspace);
        Require(tilted.count == 1, "the corrected transport lost its patch");
        if (tilted.count == 1 && plain.count == 1) {
            RequireClose(tilted.patches[0].contact_frame_angle_radians -
                             plain.patches[0].contact_frame_angle_radians,
                         1.5e-3, 1.0e-12,
                         "the roll offset did not tilt the contact frame by "
                         "itself");
            RequireClose(tilted.patches[0].contact_frame_angle_radians,
                         tilted.patches[0].geometry.rail_slope_angle_radians +
                             1.5e-3,
                         1.0e-12,
                         "the contact frame is not the rail's slope angle plus "
                         "the roll offset");
            Require((tilted.patches[0].wrench.rail_on_wheel.force_newtons -
                     plain.patches[0].wrench.rail_on_wheel.force_newtons)
                            .cwiseAbs()
                            .maxCoeff() > 1.0,
                    "the roll offset did not reach the force");

            // And nowhere else. The patch's shape and the point it reduces
            // about come from the geometry, which never saw the correction.
            Require(tilted.patches[0].wrench.contact_point_meters ==
                        plain.patches[0].wrench.contact_point_meters,
                    "the roll transport moved the contact point, which it must "
                    "not — the contact-point reduction uses the raw pose");
            Require(tilted.patches[0].geometry.rail_slope_angle_radians ==
                            plain.patches[0].geometry.rail_slope_angle_radians &&
                        tilted.patches[0].geometry.vertical_penetration_meters ==
                            plain.patches[0].geometry.vertical_penetration_meters &&
                        tilted.patches[0].geometry.arc_width_meters ==
                            plain.patches[0].geometry.arc_width_meters,
                    "the roll transport reached the patch's own geometry");
            // The two translational offsets are not read here. Passing them and
            // getting the same answer as passing only the roll is the check.
            WheelRailContactInput roll_only = rolling;
            roll_only.roll_transport.roll_offset_radians = 1.5e-3;
            const WheelRailContactResult without_translation =
                model.Evaluate(roll_only, workspace);
            Require(without_translation.patches[0].wrench.rail_on_wheel.force_newtons ==
                        tilted.patches[0].wrench.rail_on_wheel.force_newtons,
                    "the transport's lateral or vertical offset reached this "
                    "model, which only the pose reduction upstream may read");
        }
    }

    {
        // The rail material reference point R and the wheel surface force point
        // P are separate. Moving the rail frame moves R and therefore the
        // relative velocity; it must not move P, whose placement comes from the
        // wheel profile's own rigid frame.
        const WheelRailContactResult plain = model.Evaluate(rolling, workspace);
        Require(plain.count == 1, "the two-point fixture lost its patch");

        const Eigen::Vector3d displacement(0.0, 0.05, 0.01);
        WheelRailContactInput displaced = rolling;
        displaced.rail_frame.origin_track_meters += displacement;
        const WheelRailContactResult moved = model.Evaluate(displaced, workspace);
        Require(moved.count == 1, "a displaced rail frame lost its patch");

        const Eigen::Matrix3d rail_turn =
            Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX()).toRotationMatrix();
        WheelRailContactInput turned = rolling;
        turned.rail_frame.rotation_track_from_profile = rail_turn;
        const WheelRailContactResult rotated = model.Evaluate(turned, workspace);
        Require(rotated.count == 1, "a rotated rail frame lost its patch");

        if (plain.count == 1 && moved.count == 1 && rotated.count == 1) {
            const Eigen::Vector3d plain_r =
                RailMaterialPoint(rolling, plain.patches[0].geometry);
            const Eigen::Vector3d moved_r =
                RailMaterialPoint(displaced, moved.patches[0].geometry);
            const Eigen::Vector3d rotated_r =
                RailMaterialPoint(turned, rotated.patches[0].geometry);
            Require((moved_r - (plain_r + displacement)).cwiseAbs().maxCoeff() <
                        1.0e-15,
                    "displacing the rail frame did not displace R exactly");
            Require((rotated_r - rail_turn * plain_r).cwiseAbs().maxCoeff() <
                        1.0e-15,
                    "rotating the rail frame did not rotate R exactly");
            Require(moved.patches[0].wrench.contact_point_meters ==
                            plain.patches[0].wrench.contact_point_meters &&
                        rotated.patches[0].wrench.contact_point_meters ==
                            plain.patches[0].wrench.contact_point_meters,
                    "the rail material frame moved the wheel surface force "
                    "point P");

            Require(std::abs(moved.patches[0].creepages.longitudinal -
                             plain.patches[0].creepages.longitudinal) > 1.0e-3,
                    "displacing R did not reach the relative velocity");
            Require(std::abs(rotated.patches[0].creepages.longitudinal -
                             plain.patches[0].creepages.longitudinal) > 1.0e-4,
                    "rotating R did not reach the relative velocity");
            Require(rotated.patches[0].creepages.spin_per_meter ==
                        plain.patches[0].creepages.spin_per_meter,
                    "rotating the rail frame turned the contact normal");

            const auto& reference_patch = plain.patches[0].geometry;
            const auto& displaced_patch = moved.patches[0].geometry;
            const auto& rotated_patch = rotated.patches[0].geometry;
            Require(displaced_patch.vertical_penetration_meters ==
                            reference_patch.vertical_penetration_meters &&
                        displaced_patch.arc_width_meters ==
                            reference_patch.arc_width_meters &&
                        displaced_patch.rail_slope_angle_radians ==
                            reference_patch.rail_slope_angle_radians &&
                        displaced_patch.centroid_lateral_meters ==
                            reference_patch.centroid_lateral_meters,
                    "displacing the rail frame changed the patch geometry");
            Require(rotated_patch.vertical_penetration_meters ==
                            reference_patch.vertical_penetration_meters &&
                        rotated_patch.rail_slope_angle_radians ==
                            reference_patch.rail_slope_angle_radians,
                    "rotating the rail frame changed the patch geometry");

            // A common translation of R and the wheel profile datum translates
            // P by the same amount while preserving every relative quantity.
            WheelRailContactInput together = displaced;
            together.wheel.origin_track_meters += displacement;
            const WheelRailContactResult carried =
                model.Evaluate(together, workspace);
            Require(carried.count == 1, "the covariance fixture lost its patch");
            if (carried.count == 1) {
                Require((carried.patches[0].wrench.contact_point_meters -
                         (plain.patches[0].wrench.contact_point_meters +
                          displacement))
                                .cwiseAbs()
                                .maxCoeff() < 1.0e-15,
                        "a common translation did not translate P exactly");
                RequireClose(carried.patches[0].creepages.longitudinal,
                             plain.patches[0].creepages.longitudinal, 1.0e-12,
                             "a common translation changed the creepage");
                RequireClose(carried.patches[0].normal.normal_force_newtons,
                             plain.patches[0].normal.normal_force_newtons, 1.0e-12,
                             "a common translation changed the normal force");
            }
        }
    }

    {
        // Exercise P's independent construction with a non-trivial wheel
        // profile frame. This deliberately perturbs only that input seam: R and
        // the relative motion stay fixed, while P must move according to the
        // complete profile-coordinate formula, including half the equivalent
        // penetration.
        const WheelRailContactResult unprofiled =
            model.Evaluate(rolling, workspace);
        Require(unprofiled.count == 1,
                "the wheel-profile datum's reference solve lost its patch");
        WheelRailContactInput profiled = rolling;
        profiled.wheel.rotation_track_from_profile =
            Eigen::AngleAxisd(0.021, Eigen::Vector3d::UnitX())
                .toRotationMatrix() *
            Eigen::AngleAxisd(-0.013, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        // Move from the wheelset origin to the authored wheel-profile datum on
        // the axle. Its velocity is moved to that same point, so the rigid-body
        // velocity at R remains the reference solve's velocity.
        const Eigen::Vector3d profile_datum_from_axle =
            profiled.wheel.rotation_track_from_profile *
            Eigen::Vector3d(0.0, 0.63, 0.0);
        profiled.wheel.origin_track_meters += profile_datum_from_axle;
        profiled.wheel.origin_velocity_track_meters_per_second +=
            profiled.wheel.angular_velocity_track_radians_per_second.cross(
                profile_datum_from_axle);
        const WheelRailContactResult result = model.Evaluate(profiled, workspace);
        Require(result.count == 1, "the wheel-profile-frame fixture lost its patch");
        if (result.count == 1) {
            const auto& patch = result.patches[0];
            const Eigen::Vector3d expected_p = WheelSurfacePoint(profiled, patch);
            const Eigen::Vector3d r = RailMaterialPoint(profiled, patch.geometry);
            Require((patch.wrench.contact_point_meters - expected_p)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-15,
                    "the wrench point is not the qualified wheel surface point P");
            Require((expected_p - r).norm() > 1.0e-3,
                    "the P/R fixture is degenerate");
            if (unprofiled.count == 1) {
                Require(
                    std::abs(patch.creepages.longitudinal -
                             unprofiled.patches[0].creepages.longitudinal) <
                        1.0e-12,
                    "changing the rigid reference point changed the R-based "
                    "relative velocity");
            }

            const SpatialWrench reduced_at_r = TransportWrench(
                patch.wrench.rail_on_wheel, patch.wrench.contact_point_meters, r);
            const Eigen::Vector3d expected_moment =
                (expected_p - r).cross(patch.wrench.rail_on_wheel.force_newtons);
            Require(expected_moment.norm() > 1.0,
                    "the P/R fixture's force has no discriminating lever arm");
            Require((reduced_at_r.moment_newton_meters - expected_moment)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-9,
                    "transporting the P-reduced wrench to R lost its lever-arm "
                    "moment");
        }
    }

    {
        // A wheel lifted clear carries nothing, and a wheel separating fast
        // enough carries nothing either — the geometry still sees the overlap,
        // and the model reports the difference rather than emitting a patch
        // with no load.
        WheelRailContactInput lifted = resting;
        lifted.pose.vertical_raise_meters = 0.01;
        const WheelRailContactResult clear = model.Evaluate(lifted, workspace);
        Require(clear.count == 0 && clear.geometric_patch_count == 0 &&
                    clear.analytic_longitudinal_length_fallback_count == 0,
                "a wheel held clear of the rail produced a patch");

        WheelRailContactInput flying = resting;
        flying.wheel.origin_velocity_track_meters_per_second =
            Eigen::Vector3d(0.0, 0.0, -100.0);
        const WheelRailContactResult separating = model.Evaluate(flying, workspace);
        Require(separating.geometric_patch_count == 1,
                "the separating fixture is not geometrically in contact");
        Require(separating.count == 0,
                "a patch whose damping cancelled its elastic force was emitted "
                "as a force");
        Require(separating.analytic_longitudinal_length_fallback_count == 0,
                "a separating geometric patch unexpectedly lost its resolved "
                "three-dimensional length before the force gate");
    }

    {
        // A deterministic deep overlap makes the three-dimensional search miss
        // its bracket. The assembled path must keep the patch, use the analytic
        // baseline once, and expose both facts from this same evaluation. This
        // is a branch fixture, not a qualified vehicle operating point.
        const ProfilePoints flat_wheel =
            Sample(ProfileRole::kWheel, "fallback_flat_wheel", 0.05, 1001,
                   [](double) { return 0.0; });
        const ProfilePoints flat_rail =
            Sample(ProfileRole::kRail, "fallback_flat_rail", 0.06, 1201,
                   [](double) { return 0.0; });
        const WheelRailContactModel fallback_model(
            flat_wheel, flat_rail, WheelSide::kRight, no_preparation,
            Configuration());
        WheelRailContactWorkspace fallback_workspace;
        WheelRailContactInput unresolved;
        unresolved.pose.vertical_raise_meters = -0.014;
        unresolved.wheel.origin_track_meters = axle;
        const WheelRailContactResult result =
            fallback_model.Evaluate(unresolved, fallback_workspace);
        Require(result.geometric_patch_count == 1 && result.count == 1,
                "the assembled model swallowed a patch whose longitudinal "
                "resolution was unavailable");
        Require(result.analytic_longitudinal_length_fallback_count == 1,
                "the assembled model did not report one analytic fallback");
        if (result.count == 1) {
            Require(result.patches[0]
                            .normal.used_analytic_longitudinal_length_fallback &&
                        result.patches[0].normal.normal_force_newtons > 0.0,
                    "the fallback patch did not carry a visible positive force");
        }
    }

    {
        // The evaluation runs twice per wheelset per derivative evaluation.
        (void)model.Evaluate(rolling, workspace);
        double sink = 0.0;
        const AllocationScope scope;
        for (int step = 0; step < 8; ++step) {
            const WheelRailContactResult result = model.Evaluate(rolling, workspace);
            sink += result.patches[0].normal.normal_force_newtons;
        }
        Require(scope.allocations() == 0,
                "the assembled contact evaluation allocated after its workspace "
                "was warm");
        Require(sink > 0.0, "the allocation sweep produced no force");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("wheel rail contact model verified");
    return 0;
}
