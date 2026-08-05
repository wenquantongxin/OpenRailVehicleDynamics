#include "orvd/wheel_rail_contact/contact_creepage.h"

#include <cmath>

namespace orvd::wheel_rail_contact {

ContactFrame MakeContactFrame(double rail_contact_angle_radians) {
    const double cosine = std::cos(rail_contact_angle_radians);
    const double sine = std::sin(rail_contact_angle_radians);
    ContactFrame frame;
    // A roll about +x. Written out rather than assembled from an axis-angle so
    // that the columns are visibly orthonormal by construction and the third
    // one is visibly the normal.
    frame.rotation_track_from_contact << 1.0, 0.0, 0.0,
                                          0.0, cosine, -sine,
                                          0.0, sine, cosine;
    frame.normal_track = frame.rotation_track_from_contact.col(2);
    return frame;
}

Creepages ComputeCreepages(const ContactRelativeMotion& motion,
                           const ContactFrame& frame, double rolling_radius_meters,
                           const CreepageConfiguration& configuration) {
    // Into the contact's own axes. The transpose is the rotation the other way;
    // for an orthonormal frame that is the inverse, exactly.
    const Eigen::Vector3d velocity =
        frame.rotation_track_from_contact.transpose() *
        motion.relative_velocity_track_meters_per_second;
    const Eigen::Vector3d spin = frame.rotation_track_from_contact.transpose() *
                                 motion.relative_spin_track_radians_per_second;

    // The mean of how fast the contact advances and how fast the rim turns
    // under it. The two agree exactly only in pure rolling; their difference is
    // the creep this whole module is about, and their mean is the speed that
    // creep is measured against.
    const double rim_speed =
        -motion.wheel_pitch_rate_radians_per_second * rolling_radius_meters;
    double reference_speed = 0.5 * (motion.arc_rate_meters_per_second + rim_speed);
    const double floor_speed = configuration.minimum_reference_speed_meters_per_second;
    if (std::abs(reference_speed) < floor_speed) {
        // Exactly zero takes the forward sign. There is no continuous choice
        // here — see the header note.
        reference_speed = reference_speed != 0.0
                              ? std::copysign(floor_speed, reference_speed)
                              : floor_speed;
    }

    Creepages creepages;
    creepages.longitudinal = velocity.x() / reference_speed;
    creepages.lateral = velocity.y() / reference_speed;
    creepages.spin_per_meter = spin.z() / reference_speed;
    creepages.reference_speed_meters_per_second = reference_speed;
    return creepages;
}

double ComputeNormalApproachSpeed(const ContactRelativeMotion& motion,
                                  const ContactFrame& frame) {
    // The vertical axis points down and the normal points into the rail, so a
    // wheel descending onto the rail projects positively. No sign flip is
    // needed and adding one would make the damping pull.
    return frame.normal_track.dot(motion.relative_velocity_track_meters_per_second);
}

}  // namespace orvd::wheel_rail_contact
