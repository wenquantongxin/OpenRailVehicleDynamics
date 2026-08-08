// The rail gauge datum derived from a rail profile, and the reduction of a
// wheelset placement to the four scalars the contact geometry solves against.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

namespace {

using orvd::wheel_rail_contact::ComputeRailGaugeDatum;
using orvd::wheel_rail_contact::ContactPoseScalars;
using orvd::wheel_rail_contact::PlaceRailProfileLongitudinalOrigin;
using orvd::wheel_rail_contact::RailProfileOriginMode;
using orvd::wheel_rail_contact::BuildContactPoseScalars;
using orvd::wheel_rail_contact::ProfilePoints;
using orvd::wheel_rail_contact::ProfileRole;
using orvd::wheel_rail_contact::ProfileTrackRollTransport;
using orvd::wheel_rail_contact::RailGaugeDatum;
using orvd::wheel_rail_contact::SafeAtan2Ratio;
using orvd::wheel_rail_contact::TrackIrregularity;
using orvd::wheel_rail_contact::WheelRailPoseConstants;
using orvd::wheel_rail_contact::WheelRailPoseInput;
using orvd::wheel_rail_contact::WheelSide;
using orvd::wheel_rail_contact::WheelsetPlacement;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "wheel rail pose: %.*s\n",
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
                         "wheel rail pose: %.*s was refused for another reason: "
                         "%s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

// A symmetric rail head: a flat-bottomed vee whose crown is at the origin and
// whose flanks rise linearly. Its gauge face at a stated depth is then known in
// closed form, which is what makes the datum checkable without a table.
ProfilePoints MakeSymmetricRailProfile(std::string identifier, double half_width,
                                       double flank_slope) {
    std::vector<double> lateral;
    std::vector<double> vertical;
    constexpr int kPoints = 401;
    for (int index = 0; index < kPoints; ++index) {
        const double at = -half_width + 2.0 * half_width *
                                            (static_cast<double>(index) /
                                             static_cast<double>(kPoints - 1));
        lateral.push_back(at);
        vertical.push_back(flank_slope * std::abs(at));
    }
    return ProfilePoints::FromAuthoredOrder(ProfileRole::kRail,
                                            std::move(identifier),
                                            std::move(lateral), std::move(vertical));
}

// A wheelset running forward over a rail that is exactly where the line says it
// is. Every test that is not about irregularity uses this, and it passes the
// zeros explicitly rather than letting a default hide them.
WheelRailPoseInput Running(const WheelsetPlacement& placement) {
    WheelRailPoseInput input;
    input.placement = placement;
    input.arc_rate_meters_per_second = 16.666666666666668;
    return input;
}

}  // namespace

int main() {
    {
        // Profile-coordinate placement is measured from the wheel BODY origin,
        // not from the authored wheel-profile datum.  At non-zero yaw the two
        // differ along x, so this fixture would expose the historical double
        // application of the axle-datum station offset.
        const Eigen::Vector3d rail_origin(0.42, 0.73, -0.01);
        const Eigen::Matrix3d rail_rotation =
            Eigen::AngleAxisd(0.07, Eigen::Vector3d::UnitX())
                .toRotationMatrix();
        const Eigen::Vector3d wheel_body_origin(-0.12, 0.03, -0.41);
        const double effective_station_offset = -0.031;
        const Eigen::Vector3d placed = PlaceRailProfileLongitudinalOrigin(
            RailProfileOriginMode::kProfileCoordinate, rail_origin,
            rail_rotation, wheel_body_origin, effective_station_offset);
        const double resulting_coordinate =
            (rail_rotation.transpose() * (placed - wheel_body_origin)).x();
        Require(std::abs(resulting_coordinate - effective_station_offset) <
                    1.0e-15,
                "profile-coordinate rail placement was not measured from the "
                "wheel body origin");
        Require(PlaceRailProfileLongitudinalOrigin(
                    RailProfileOriginMode::kTrackStation, rail_origin,
                    rail_rotation, wheel_body_origin,
                    effective_station_offset) == rail_origin,
                "track-station rail placement changed its origin");
    }

    const double half_width = 0.037;
    const double flank_slope = 1.0;
    const ProfilePoints rail =
        MakeSymmetricRailProfile("synthetic_rail", half_width, flank_slope);
    const double gauge = 1.435;
    const double depth = 0.016;

    {
        // With the rail laid flat, the vee's gauge face sits exactly one
        // measuring depth from the crown, divided by the flank slope.
        const RailGaugeDatum right =
            ComputeRailGaugeDatum(rail, gauge, depth, 0.0, WheelSide::kRight);
        const double expected_face = depth / flank_slope;
        Require(std::abs(right.gauge_face_offset_meters - expected_face) < 1.0e-12,
                "the gauge face of a flat-laid vee is not where its slope puts it");
        Require(std::abs(right.lateral_datum_meters -
                         (0.5 * gauge + expected_face)) < 1.0e-12,
                "the rail datum is not the half gauge plus the gauge face offset");
        Require(right.roll_radians == 0.0,
                "a rail laid without cant was given a roll");

        const RailGaugeDatum left =
            ComputeRailGaugeDatum(rail, gauge, depth, 0.0, WheelSide::kLeft);
        Require(left.lateral_datum_meters == -right.lateral_datum_meters &&
                    left.gauge_face_offset_meters == right.gauge_face_offset_meters,
                "the two rails of a symmetric pair are not mirror images");
    }

    {
        // Canting the rail moves the gauge face, and the two rails move in
        // opposite directions. A datum that ignored the cant would return the
        // same number for both.
        const double cant = std::atan(1.0 / 40.0);
        const RailGaugeDatum right =
            ComputeRailGaugeDatum(rail, gauge, depth, cant, WheelSide::kRight);
        const RailGaugeDatum left =
            ComputeRailGaugeDatum(rail, gauge, depth, cant, WheelSide::kLeft);
        Require(right.roll_radians == -cant && left.roll_radians == cant,
                "the two rails were not given opposite rolls");
        const RailGaugeDatum flat =
            ComputeRailGaugeDatum(rail, gauge, depth, 0.0, WheelSide::kRight);
        Require(std::abs(right.gauge_face_offset_meters -
                         flat.gauge_face_offset_meters) > 1.0e-5,
                "canting the rail did not move its gauge face");
        Require(std::abs(right.gauge_face_offset_meters -
                         left.gauge_face_offset_meters) < 1.0e-12,
                "a symmetric profile gave its two canted rails different faces");
        // The measuring depth is measured from the crown, and canting moves the
        // crown, so the two must be found together.
        Require(std::abs(right.lateral_datum_meters +
                         left.lateral_datum_meters) < 1.0e-12,
                "the canted pair is not symmetric about the track centre");
    }

    {
        // A deeper measuring line finds a wider face on a vee, linearly.
        const RailGaugeDatum shallow =
            ComputeRailGaugeDatum(rail, gauge, 0.008, 0.0, WheelSide::kRight);
        const RailGaugeDatum deep =
            ComputeRailGaugeDatum(rail, gauge, 0.016, 0.0, WheelSide::kRight);
        Require(std::abs(deep.gauge_face_offset_meters -
                         2.0 * shallow.gauge_face_offset_meters) < 1.0e-12,
                "doubling the measuring depth did not double the face offset on "
                "a straight flank");
    }

    RequireRefusal(
        [&] {
            (void)ComputeRailGaugeDatum(rail, gauge, 0.5, 0.0, WheelSide::kRight);
        },
        "no gauge face", "a measuring depth below the whole profile");
    RequireRefusal(
        [&] {
            const ProfilePoints wheel = ProfilePoints::FromAuthoredOrder(
                ProfileRole::kWheel, "synthetic_wheel",
                std::vector<double>{-0.03, 0.0, 0.03},
                std::vector<double>{0.0, -0.001, -0.002});
            (void)ComputeRailGaugeDatum(wheel, gauge, depth, 0.0, WheelSide::kRight);
        },
        "is a wheel profile", "a wheel profile handed to a rail construction");

    // --- the pose reduction ---
    const RailGaugeDatum datum = ComputeRailGaugeDatum(
        rail, gauge, depth, std::atan(1.0 / 40.0), WheelSide::kRight);
    WheelRailPoseConstants constants;
    constants.rail_lateral_datum_meters = datum.lateral_datum_meters;
    constants.rail_vertical_datum_meters = -0.0003;
    constants.rail_roll_radians = datum.roll_radians;
    constants.wheel_lateral_datum_meters = 0.7465;
    constants.nominal_rolling_radius_meters = 0.42;

    const ProfileTrackRollTransport no_transport;

    {
        // A centred, upright, unyawed wheelset whose centre sits one nominal
        // radius above the rail datum must produce no raise and a lateral
        // offset that is the reach beyond the rail datum.
        WheelsetPlacement centred;
        centred.lateral_meters = 0.0;
        centred.vertical_meters =
            constants.rail_vertical_datum_meters - constants.nominal_rolling_radius_meters;
        const ContactPoseScalars scalars =
            BuildContactPoseScalars(constants, Running(centred), no_transport);
        Require(std::abs(scalars.roll_radians + datum.roll_radians) < 1.0e-15,
                "an upright wheelset over a canted rail did not acquire the "
                "cant as its pair roll");
        Require(std::abs(scalars.yaw_radians) == 0.0,
                "an unyawed wheelset acquired a pair yaw");
        // The wheel datum sits further out than the rail datum by the
        // difference of the two lateral positions, resolved in the rail's axes.
        Require(std::abs(scalars.lateral_offset_meters) > 1.0e-3,
                "the fixture places the wheel datum on the rail datum, which "
                "tests nothing");
    }

    {
        // Lowering the wheelset raises nothing and lowers the raise by the same
        // amount, to first order in the cant. The sign is the thing under test:
        // a raise is positive up while the vertical axis points down.
        WheelsetPlacement high;
        high.vertical_meters = -0.42;
        WheelsetPlacement low = high;
        low.vertical_meters = -0.42 + 0.001;
        const ContactPoseScalars raised =
            BuildContactPoseScalars(constants, Running(high), no_transport);
        const ContactPoseScalars dropped =
            BuildContactPoseScalars(constants, Running(low), no_transport);
        Require(dropped.vertical_raise_meters < raised.vertical_raise_meters,
                "moving the wheelset down did not reduce its raise above the "
                "rail");
        Require(std::abs((raised.vertical_raise_meters -
                          dropped.vertical_raise_meters) - 0.001) < 1.0e-6,
                "a one millimetre drop did not move the raise by one millimetre");
    }

    {
        // The transport correction reaches the pose and nothing else here. A
        // suppressed correction must leave the answer bit-for-bit alone.
        WheelsetPlacement placement;
        placement.lateral_meters = 0.002;
        placement.vertical_meters = -0.4195;
        placement.roll_radians = 0.001;
        placement.yaw_radians = 0.0005;
        const ContactPoseScalars plain =
            BuildContactPoseScalars(constants, Running(placement), no_transport);

        ProfileTrackRollTransport correction;
        correction.roll_offset_radians = 1.0e-5;
        correction.lateral_offset_meters = 2.0e-6;
        correction.vertical_offset_meters = -3.0e-6;
        const ContactPoseScalars corrected =
            BuildContactPoseScalars(constants, Running(placement), correction);
        Require(std::abs(corrected.roll_radians - plain.roll_radians -
                         (-1.0e-5)) < 1.0e-15,
                "the transport roll is not subtracted from the pair roll");
        Require(corrected.yaw_radians == plain.yaw_radians,
                "the transport correction reached the pair yaw");
        Require(corrected.lateral_offset_meters != plain.lateral_offset_meters &&
                    corrected.vertical_raise_meters != plain.vertical_raise_meters,
                "the transport correction did not reach the two offsets");

        const ContactPoseScalars suppressed =
            BuildContactPoseScalars(constants, Running(placement), ProfileTrackRollTransport{});
        Require(suppressed.roll_radians == plain.roll_radians &&
                    suppressed.yaw_radians == plain.yaw_radians &&
                    suppressed.lateral_offset_meters == plain.lateral_offset_meters &&
                    suppressed.vertical_raise_meters == plain.vertical_raise_meters,
                "a suppressed transport is not bit-for-bit no transport");
    }

    {
        // The pair encoding is its own inverse: decoding with the same
        // expression recovers the rail-frame separation. The geometry solver
        // relies on that, so it is checked here rather than assumed.
        WheelsetPlacement placement;
        placement.lateral_meters = -0.003;
        placement.vertical_meters = -0.4188;
        placement.roll_radians = -0.0024;
        placement.yaw_radians = 0.0011;
        const ContactPoseScalars scalars =
            BuildContactPoseScalars(constants, Running(placement), no_transport);
        const double cosine = std::cos(scalars.roll_radians);
        const double sine = std::sin(scalars.roll_radians);
        const double lateral = scalars.lateral_offset_meters * cosine +
                               scalars.vertical_raise_meters * sine;
        const double vertical = scalars.lateral_offset_meters * sine -
                                scalars.vertical_raise_meters * cosine;
        const ContactPoseScalars again =
            BuildContactPoseScalars(constants, Running(placement), no_transport);
        const double re_lateral = again.lateral_offset_meters * cosine +
                                  again.vertical_raise_meters * sine;
        Require(std::abs(lateral - re_lateral) == 0.0,
                "the reduction is not a function of its inputs alone");
        Require(std::abs(vertical) > 1.0e-6 && std::abs(lateral) > 1.0e-4,
                "the decode fixture is degenerate");
    }

    // --- the irregularity inputs, which this project holds at zero ---

    WheelsetPlacement general;
    general.lateral_meters = 0.0017;
    general.vertical_meters = -0.41935;
    general.roll_radians = 0.0031;
    general.yaw_radians = 0.0024;

    {
        // The smooth-track limit. With every irregularity number zero and the
        // wheelset running forward, the pair roll is the wheelset roll less the
        // rail's laying cant and the pair yaw is the wheelset yaw.
        //
        // This is a limit in exact arithmetic, not an identity in binary64: the
        // general path forms a product of three rotations and extracts an angle
        // with atan2, and that is a different sequence of roundings from a
        // subtraction. The tolerance below is a few ulp of an angle of order
        // one, and it is deliberately not zero. A future reader who tightens it
        // to an exact comparison will be reverting a correction, not finding a
        // bug.
        const ContactPoseScalars scalars =
            BuildContactPoseScalars(constants, Running(general), no_transport);
        const double limit_roll = general.roll_radians - constants.rail_roll_radians;
        Require(std::abs(scalars.roll_radians - limit_roll) < 1.0e-15,
                "the smooth-track pair roll is not the wheelset roll less the "
                "laying cant");
        Require(scalars.yaw_radians == general.yaw_radians,
                "the smooth-track pair yaw is not the wheelset yaw");
    }

    {
        // A rail displaced sideways moves the wheel relative to it, by the same
        // amount and the other way. This is the input a smooth-track caller
        // holds at zero, and the check is that it is genuinely an input.
        WheelRailPoseInput displaced = Running(general);
        displaced.irregularity.lateral_meters = 0.004;
        const ContactPoseScalars plain =
            BuildContactPoseScalars(constants, Running(general), no_transport);
        const ContactPoseScalars moved =
            BuildContactPoseScalars(constants, displaced, no_transport);
        Require(std::abs((plain.lateral_offset_meters -
                          moved.lateral_offset_meters) - 0.004) < 1.0e-7,
                "a laterally displaced rail did not move the lateral offset by "
                "its displacement");

        WheelRailPoseInput lifted = Running(general);
        lifted.irregularity.vertical_meters = -0.002;
        const ContactPoseScalars raised =
            BuildContactPoseScalars(constants, lifted, no_transport);
        Require(std::abs((raised.vertical_raise_meters -
                          plain.vertical_raise_meters) - (-0.002)) < 1.0e-7,
                "a rail lifted by two millimetres did not reduce the raise by "
                "two millimetres");
    }

    {
        // An alignment slope is an angle of attack against the rail's actual
        // direction: it subtracts from the pair yaw and it also tilts the frame
        // the pair roll is measured against.
        WheelRailPoseInput sloped = Running(general);
        sloped.irregularity.lateral_rate_meters_per_second = 0.05;
        const ContactPoseScalars slanted =
            BuildContactPoseScalars(constants, sloped, no_transport);
        const double expected_correction =
            SafeAtan2Ratio(0.05, sloped.arc_rate_meters_per_second);
        Require(expected_correction > 1.0e-4,
                "the alignment-slope fixture is too small to discriminate");
        Require(std::abs(slanted.yaw_radians -
                         (general.yaw_radians - expected_correction)) < 1.0e-15,
                "the pair yaw did not lose the alignment slope angle");
        const ContactPoseScalars plain =
            BuildContactPoseScalars(constants, Running(general), no_transport);
        Require(slanted.roll_radians != plain.roll_radians,
                "the alignment slope did not reach the pair roll, so the pair "
                "roll is not being measured against the tangential rail frame");
    }

    {
        // A level slope shortens the rolling-radius lever by its cosine, and
        // that shortening is gated on its own switch rather than on whether the
        // rates are used anywhere else.
        WheelRailPoseInput graded = Running(general);
        graded.irregularity.vertical_rate_meters_per_second = 0.2;
        const ContactPoseScalars corrected =
            BuildContactPoseScalars(constants, graded, no_transport);

        WheelRailPoseConstants uncorrected_constants = constants;
        uncorrected_constants.apply_pitch_correction = false;
        const ContactPoseScalars uncorrected =
            BuildContactPoseScalars(uncorrected_constants, graded, no_transport);
        Require(corrected.vertical_raise_meters != uncorrected.vertical_raise_meters,
                "the pitch correction did not reach the raise");

        // The lever shortens, so the wheel datum sits closer to the wheelset
        // centre, which on a z-down axis means it is higher — a larger raise.
        const double grade = graded.irregularity.vertical_rate_meters_per_second /
                             graded.arc_rate_meters_per_second;
        const double shortening =
            constants.nominal_rolling_radius_meters *
            (1.0 - std::cos(std::atan(grade)));
        Require(std::abs((corrected.vertical_raise_meters -
                          uncorrected.vertical_raise_meters) - shortening) <
                    1.0e-6,
                "the pitch correction is not the cosine shortening of the "
                "nominal rolling radius");
        Require(shortening > 1.0e-5,
                "the grade fixture is too small to discriminate");
    }

    {
        // The two traps in the guarded ratio, recorded rather than defended
        // against. Standing still resolves to no correction; running backwards
        // does not, because atan2 of a positive zero over a negative number is
        // pi and not zero. A reversing caller must supply its irregularity
        // rates and its arc rate with consistent signs; holding the rates at
        // zero is not enough.
        WheelRailPoseInput standing = Running(general);
        standing.arc_rate_meters_per_second = 0.0;
        standing.irregularity.lateral_rate_meters_per_second = 0.05;
        const ContactPoseScalars still =
            BuildContactPoseScalars(constants, standing, no_transport);
        Require(still.yaw_radians == general.yaw_radians,
                "a stationary wheelset was given an angle-of-attack correction "
                "from a ratio with no denominator");

        WheelRailPoseInput reversing = Running(general);
        reversing.arc_rate_meters_per_second = -16.666666666666668;
        const ContactPoseScalars backwards =
            BuildContactPoseScalars(constants, reversing, no_transport);
        Require(std::abs(backwards.yaw_radians -
                         (general.yaw_radians - std::acos(-1.0))) < 1.0e-15,
                "the documented reverse-travel trap did not fire: either the "
                "guard changed or the ratio is no longer formed from the arc "
                "rate's sign");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("wheel rail pose verified");
    return 0;
}
