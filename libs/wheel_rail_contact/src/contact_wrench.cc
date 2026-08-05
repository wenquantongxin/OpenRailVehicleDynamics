#include "orvd/wheel_rail_contact/contact_wrench.h"

#include <Eigen/Geometry>

namespace orvd::wheel_rail_contact {

SpatialWrench TransportWrench(const SpatialWrench& wrench,
                              const Eigen::Vector3d& from_point_meters,
                              const Eigen::Vector3d& to_point_meters) {
    SpatialWrench moved;
    moved.force_newtons = wrench.force_newtons;
    moved.moment_newton_meters =
        wrench.moment_newton_meters +
        (from_point_meters - to_point_meters).cross(wrench.force_newtons);
    return moved;
}

SpatialWrench RotateWrench(const SpatialWrench& wrench,
                           const Eigen::Matrix3d& rotation) {
    SpatialWrench turned;
    turned.moment_newton_meters = rotation * wrench.moment_newton_meters;
    turned.force_newtons = rotation * wrench.force_newtons;
    return turned;
}

PairedContactWrench MakePairedContactWrench(
    const Eigen::Vector3d& contact_point_meters,
    const Eigen::Vector3d& force_on_wheel_newtons,
    const Eigen::Vector3d& moment_on_wheel_newton_meters) {
    PairedContactWrench pair;
    pair.contact_point_meters = contact_point_meters;
    pair.rail_on_wheel.force_newtons = force_on_wheel_newtons;
    pair.rail_on_wheel.moment_newton_meters = moment_on_wheel_newton_meters;
    pair.wheel_on_rail.force_newtons = -force_on_wheel_newtons;
    pair.wheel_on_rail.moment_newton_meters = -moment_on_wheel_newton_meters;
    return pair;
}

}  // namespace orvd::wheel_rail_contact
