#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"

#include <cmath>

namespace orvd::wheel_rail_contact {

RollYawPitchAngles ResolveRollYawPitch(const Eigen::Matrix3d& rotation) {
    // Five entries of the nine carry the whole answer. The yaw's second
    // argument is a non-negative square root, which is what confines the result
    // to the quarter-turn range and makes the resolution single-valued.
    RollYawPitchAngles angles;
    angles.roll_radians = std::atan2(rotation(2, 1), rotation(1, 1));
    angles.yaw_radians = std::atan2(
        -rotation(0, 1), std::sqrt(rotation(0, 0) * rotation(0, 0) +
                                   rotation(0, 2) * rotation(0, 2)));
    angles.pitch_radians = std::atan2(rotation(0, 2), rotation(0, 0));
    return angles;
}

ProfileTrackRollTransport ComputeProfileTrackRollTransport(
    const Eigen::Vector3d& shared_track_origin_in_inertial_meters,
    const Eigen::Matrix3d& shared_rotation_inertial_from_track,
    const Eigen::Vector3d& shared_body_position_in_track_meters,
    const Eigen::Vector3d& profile_track_origin_in_inertial_meters,
    const Eigen::Matrix3d& profile_rotation_inertial_from_track) {
    const Eigen::Matrix3d shared_rotation_track_from_inertial =
        shared_rotation_inertial_from_track.transpose();
    const Eigen::Matrix3d profile_rotation_track_from_inertial =
        profile_rotation_inertial_from_track.transpose();

    // The attitude of the profile-station frame as seen from the shared one.
    const Eigen::Matrix3d shared_from_profile =
        shared_rotation_track_from_inertial * profile_rotation_inertial_from_track;

    // The body position, expressed in the profile-station frame. The full
    // three-vector is used, longitudinal component included: the two frames
    // differ along the track as well as across it, and dropping that component
    // would misplace the point.
    //
    // The two track origins are differenced against each other before anything
    // is added to them. Lifting the body into the inertial frame first and
    // subtracting the profile origin afterwards is the same expression on
    // paper, and on a line kilometres long it subtracts two numbers of order
    // the mileage to obtain one of order a micrometre. Differencing the origins
    // first keeps every quantity at the scale of the answer.
    const Eigen::Vector3d origin_separation =
        shared_track_origin_in_inertial_meters -
        profile_track_origin_in_inertial_meters;
    const Eigen::Vector3d body_in_profile_track =
        profile_rotation_track_from_inertial *
        (origin_separation +
         shared_rotation_inertial_from_track *
             shared_body_position_in_track_meters);

    ProfileTrackRollTransport transport;
    transport.roll_offset_radians =
        ResolveRollYawPitch(shared_from_profile).roll_radians;
    transport.lateral_offset_meters =
        body_in_profile_track.y() - shared_body_position_in_track_meters.y();
    transport.vertical_offset_meters =
        body_in_profile_track.z() - shared_body_position_in_track_meters.z();
    return transport;
}

}  // namespace orvd::wheel_rail_contact
