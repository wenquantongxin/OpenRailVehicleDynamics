// The wrench algebra one contact patch is reported through.

#include <cmath>
#include <cstdio>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/wheel_rail_contact/contact_wrench.h"

namespace {

using orvd::wheel_rail_contact::MakePairedContactWrench;
using orvd::wheel_rail_contact::PairedContactWrench;
using orvd::wheel_rail_contact::RotateWrench;
using orvd::wheel_rail_contact::SpatialWrench;
using orvd::wheel_rail_contact::TransportWrench;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "contact wrench: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

}  // namespace

int main() {
    const Eigen::Vector3d contact_point(1.7, -0.75, 0.42);
    const Eigen::Vector3d force(2100.0, -830.0, -68000.0);
    const Eigen::Vector3d moment(0.0, 0.0, 0.0);
    const PairedContactWrench pair =
        MakePairedContactWrench(contact_point, force, moment);

    Require(pair.rail_on_wheel.force_newtons == force,
            "the wheel-side force is not the force it was given");
    Require(pair.wheel_on_rail.force_newtons == -force &&
                pair.wheel_on_rail.moment_newton_meters == -moment,
            "the rail-side wrench is not the exact negation of the wheel side");

    {
        // Transported to any common point, the pair cancels term by term. The
        // residual is arithmetic, not physics: this catches a consumer that
        // moved the two halves differently, and nothing else.
        const Eigen::Vector3d elsewhere(-120.0, 33.0, 7.5);
        const SpatialWrench on_wheel =
            TransportWrench(pair.rail_on_wheel, contact_point, elsewhere);
        const SpatialWrench on_rail =
            TransportWrench(pair.wheel_on_rail, contact_point, elsewhere);
        const double force_residual =
            (on_wheel.force_newtons + on_rail.force_newtons).norm();
        const double moment_residual =
            (on_wheel.moment_newton_meters + on_rail.moment_newton_meters).norm();
        Require(force_residual == 0.0 && moment_residual == 0.0,
                "the paired wrench does not cancel at a common point");
        Require(on_wheel.moment_newton_meters.norm() > 1.0e5,
                "the fixture transports over too short a lever to be a test of "
                "anything");
    }

    {
        // Transport is invertible and additive, which is what makes a chain of
        // reductions independent of the order it was applied in.
        const Eigen::Vector3d first(0.5, 2.0, -3.0);
        const Eigen::Vector3d second(-4.0, 1.25, 6.0);
        const SpatialWrench direct =
            TransportWrench(pair.rail_on_wheel, contact_point, second);
        const SpatialWrench staged = TransportWrench(
            TransportWrench(pair.rail_on_wheel, contact_point, first), first,
            second);
        Require((direct.moment_newton_meters - staged.moment_newton_meters)
                        .cwiseAbs()
                        .maxCoeff() < 1.0e-9,
                "transporting in two steps does not agree with one");
        const SpatialWrench returned =
            TransportWrench(direct, second, contact_point);
        Require((returned.moment_newton_meters - pair.rail_on_wheel.moment_newton_meters)
                        .cwiseAbs()
                        .maxCoeff() < 1.0e-9,
                "transporting and coming back is not the identity");
        const SpatialWrench unmoved =
            TransportWrench(pair.rail_on_wheel, contact_point, contact_point);
        Require(unmoved.moment_newton_meters == pair.rail_on_wheel.moment_newton_meters,
                "transporting to the same point changed the moment");
    }

    {
        // The cross-product order. Reversing it is invisible whenever the two
        // points coincide, so the fixture has to separate them.
        const Eigen::Vector3d elsewhere(0.0, 0.0, 0.0);
        const SpatialWrench moved =
            TransportWrench(pair.rail_on_wheel, contact_point, elsewhere);
        const Eigen::Vector3d expected =
            moment + contact_point.cross(force);
        Require((moved.moment_newton_meters - expected).cwiseAbs().maxCoeff() ==
                    0.0,
                "the transport moment is not the lever from the new point to "
                "the old one crossed into the force");
        const Eigen::Vector3d reversed = moment - contact_point.cross(force);
        Require((expected - reversed).norm() > 1.0e3,
                "the fixture cannot tell the two cross-product orders apart");
    }

    {
        // Rotation carries both halves and leaves the pairing intact.
        const Eigen::Matrix3d rotation =
            Eigen::Matrix3d(Eigen::AngleAxisd(0.37, Eigen::Vector3d(0.3, -0.6, 0.74)
                                                        .normalized()));
        const SpatialWrench turned = RotateWrench(pair.rail_on_wheel, rotation);
        Require(std::abs(turned.force_newtons.norm() -
                         pair.rail_on_wheel.force_newtons.norm()) < 1.0e-9,
                "rotating a wrench changed the magnitude of its force");
        const SpatialWrench turned_rail = RotateWrench(pair.wheel_on_rail, rotation);
        Require((turned.force_newtons + turned_rail.force_newtons).norm() < 1.0e-9,
                "rotating both halves broke the pairing");
    }

    {
        // A non-zero accompanying moment must survive both operations, so that a
        // model which does compute a spin torque is not silently truncated.
        const Eigen::Vector3d spin_moment(0.0, 0.0, -12.5);
        const PairedContactWrench spinning =
            MakePairedContactWrench(contact_point, force, spin_moment);
        Require(spinning.rail_on_wheel.moment_newton_meters == spin_moment,
                "an accompanying moment was dropped");
        const SpatialWrench moved = TransportWrench(
            spinning.rail_on_wheel, contact_point, Eigen::Vector3d::Zero());
        Require((moved.moment_newton_meters -
                 (spin_moment + contact_point.cross(force)))
                        .cwiseAbs()
                        .maxCoeff() == 0.0,
                "an accompanying moment did not survive transport");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("contact wrench verified");
    return 0;
}
