#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

#include <cmath>

namespace orvd::wheel_rail_contact {
namespace {

// The threshold below which an arc rate carries no direction. Chosen to sit far
// under any rate a running vehicle produces and far over the rates a settling
// static solve produces, so that standing still yields zero rather than the
// arbitrary angle a ratio of two near-zeros would give.
constexpr double kArcRateFloor = 1.0e-10;

// The roll of the wheelset frame relative to the local tangential rail frame.
//
// The local tangential frame is the rail's laying cant, further yawed by the
// alignment slope and pitched by the level slope. This forms the two entries of
// the relative orientation that the roll extraction reads and nothing else:
// only column 1 of the wheelset's orientation is needed (the axle direction,
// which is independent of the wheel's spin), and only columns 1 and 2 of the
// rail frame's.
double ComputePairRoll(double wheelset_roll, double wheelset_yaw,
                       double rail_roll, double lateral_rate,
                       double vertical_rate, double arc_rate) {
    const double yaw_excursion = SafeAtan2Ratio(lateral_rate, arc_rate);
    // Negative because the vertical axis points down: a rail climbing along the
    // line pitches the frame nose-up, which is a negative rotation about +y.
    const double pitch_excursion = -SafeAtan2Ratio(vertical_rate, arc_rate);

    const double cos_yaw = std::cos(yaw_excursion);
    const double sin_yaw = std::sin(yaw_excursion);
    const double cos_pitch = std::cos(pitch_excursion);
    const double sin_pitch = std::sin(pitch_excursion);
    const double cos_cant = std::cos(rail_roll);
    const double sin_cant = std::sin(rail_roll);

    // Columns 1 and 2 of Rz(yaw)·Ry(pitch)·Rx(cant).
    const double rail_01 = cos_yaw * sin_pitch * sin_cant - sin_yaw * cos_cant;
    const double rail_02 = cos_yaw * sin_pitch * cos_cant + sin_yaw * sin_cant;
    const double rail_11 = sin_yaw * sin_pitch * sin_cant + cos_yaw * cos_cant;
    const double rail_12 = sin_yaw * sin_pitch * cos_cant - cos_yaw * sin_cant;
    const double rail_21 = cos_pitch * sin_cant;
    const double rail_22 = cos_pitch * cos_cant;

    // Column 1 of Rx(roll)·Rz(yaw): the axle direction in track axes.
    const double axle_0 = -std::sin(wheelset_yaw);
    const double axle_1 = std::cos(wheelset_roll) * std::cos(wheelset_yaw);
    const double axle_2 = std::sin(wheelset_roll) * std::cos(wheelset_yaw);

    const double relative_11 =
        rail_01 * axle_0 + rail_11 * axle_1 + rail_21 * axle_2;
    const double relative_21 =
        rail_02 * axle_0 + rail_12 * axle_1 + rail_22 * axle_2;
    return std::atan2(relative_21, relative_11);
}

}  // namespace

Eigen::Vector3d PlaceRailProfileLongitudinalOrigin(
    RailProfileOriginMode mode,
    const Eigen::Vector3d& rail_origin_in_track_meters,
    const Eigen::Matrix3d& rotation_track_from_rail_profile,
    const Eigen::Vector3d& wheel_body_origin_in_track_meters,
    double effective_station_offset_meters) {
    if (mode == RailProfileOriginMode::kTrackStation) {
        return rail_origin_in_track_meters;
    }
    const Eigen::Vector3d origin_from_wheel_in_profile =
        rotation_track_from_rail_profile.transpose() *
        (rail_origin_in_track_meters - wheel_body_origin_in_track_meters);
    return rail_origin_in_track_meters +
           rotation_track_from_rail_profile *
               Eigen::Vector3d(effective_station_offset_meters -
                                   origin_from_wheel_in_profile.x(),
                               0.0, 0.0);
}

double SafeAtan2Ratio(double numerator, double denominator) {
    if (std::abs(denominator) < kArcRateFloor) {
        return 0.0;
    }
    return std::atan2(numerator, denominator);
}

ContactPoseScalars BuildContactPoseScalars(
    const WheelRailPoseConstants& constants, const WheelRailPoseInput& input,
    const ProfileTrackRollTransport& transport) {
    const WheelsetPlacement& placement = input.placement;
    const TrackIrregularity& irregularity = input.irregularity;

    // The correction enters the placement and nothing else. The comparisons are
    // exact so that a suppressed correction is bit-for-bit no correction rather
    // than an addition of zero, which is not the same thing for a negative zero.
    const double lateral = transport.lateral_offset_meters == 0.0
                               ? placement.lateral_meters
                               : placement.lateral_meters +
                                     transport.lateral_offset_meters;
    const double vertical = transport.vertical_offset_meters == 0.0
                                ? placement.vertical_meters
                                : placement.vertical_meters +
                                      transport.vertical_offset_meters;
    const double roll = transport.roll_offset_radians == 0.0
                            ? placement.roll_radians
                            : placement.roll_radians -
                                  transport.roll_offset_radians;

    // The two pair angles. The roll is a full three-dimensional extraction; the
    // yaw is not, and must not be unified with it — the yaw is the wheelset's
    // own yaw measured against the rail's actual lateral direction, which is an
    // angle of attack, not a component of the relative orientation.
    const double pair_roll = ComputePairRoll(
        roll, placement.yaw_radians, constants.rail_roll_radians,
        irregularity.lateral_rate_meters_per_second,
        irregularity.vertical_rate_meters_per_second,
        input.track_station_rate_meters_per_second);
    const double pair_yaw =
        placement.yaw_radians -
        SafeAtan2Ratio(irregularity.lateral_rate_meters_per_second,
                       input.track_station_rate_meters_per_second);

    // The rolling radius is a lever laid along the wheelset's own vertical, and
    // the plane these scalars live in is pitched away from that vertical by the
    // level grade. Projecting the lever into the plane shortens it. Cosine is
    // even, so only the magnitude of the grade angle matters.
    double rolling_radius = constants.nominal_rolling_radius_meters;
    if (constants.apply_pitch_correction) {
        const double pitch_excursion =
            -SafeAtan2Ratio(irregularity.vertical_rate_meters_per_second,
                            input.track_station_rate_meters_per_second);
        rolling_radius *= std::cos(pitch_excursion);
    }

    // The wheel profile datum, carried from the wheelset centre out along the
    // axle and down to the rolling circle. The yaw shortens the lateral reach;
    // its longitudinal component belongs to the station the section is taken
    // at, which is the caller's business, not this plane's.
    //
    // Both the reach and the roll here use the wheelset's own angles, not the
    // pair angles. The pair angles describe the wheel relative to the rail; the
    // lever is a piece of the wheelset and is placed by the wheelset's attitude.
    const double reach =
        std::cos(placement.yaw_radians) * constants.wheel_lateral_datum_meters;
    const double roll_cosine = std::cos(roll);
    const double roll_sine = std::sin(roll);
    const double wheel_lateral = lateral + roll_cosine * reach -
                                 roll_sine * rolling_radius;
    const double wheel_vertical = vertical + roll_sine * reach +
                                  roll_cosine * rolling_radius;

    // Where the rail actually is, as opposed to where the line says it is.
    const double rail_lateral =
        constants.rail_lateral_datum_meters + irregularity.lateral_meters;
    const double rail_vertical =
        constants.rail_vertical_datum_meters + irregularity.vertical_meters;

    // The separation, re-expressed in the rail's own axes. This uses the pure
    // cant frame, not the yawed and pitched tangential frame the pair roll was
    // measured against; the two are different frames on purpose.
    const double lateral_separation = wheel_lateral - rail_lateral;
    const double vertical_separation = wheel_vertical - rail_vertical;
    const double rail_cosine = std::cos(constants.rail_roll_radians);
    const double rail_sine = std::sin(constants.rail_roll_radians);
    const double lateral_in_rail =
        rail_cosine * lateral_separation + rail_sine * vertical_separation;
    const double vertical_in_rail =
        -rail_sine * lateral_separation + rail_cosine * vertical_separation;

    ContactPoseScalars scalars;
    scalars.roll_radians = pair_roll;
    scalars.yaw_radians = pair_yaw;
    // Flip the vertical to point up, then turn into the pair frame. The
    // resulting pair is its own inverse, which is what lets the geometry decode
    // it with the same expression.
    const double pair_cosine = std::cos(pair_roll);
    const double pair_sine = std::sin(pair_roll);
    scalars.lateral_offset_meters =
        pair_cosine * lateral_in_rail + pair_sine * vertical_in_rail;
    scalars.vertical_raise_meters =
        pair_sine * lateral_in_rail - pair_cosine * vertical_in_rail;
    return scalars;
}

}  // namespace orvd::wheel_rail_contact
