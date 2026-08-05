// The contact frame, the three creepages, and the reference speed they divide
// by.

#include <cmath>
#include <cstdio>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/LU>

#include "allocation_probe.h"
#include "orvd/wheel_rail_contact/contact_creepage.h"

namespace {

using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::ComputeCreepages;
using orvd::wheel_rail_contact::ComputeNormalApproachSpeed;
using orvd::wheel_rail_contact::ContactFrame;
using orvd::wheel_rail_contact::ContactRelativeMotion;
using orvd::wheel_rail_contact::CreepageConfiguration;
using orvd::wheel_rail_contact::Creepages;
using orvd::wheel_rail_contact::MakeContactFrame;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "contact creepage: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double relative_tolerance,
                  std::string_view what) {
    const double scale = std::max(std::abs(expected), 1.0e-30);
    if (!(std::abs(actual - expected) <= relative_tolerance * scale)) {
        std::fprintf(stderr,
                     "contact creepage: %.*s (expected %.17g, got %.17g)\n",
                     static_cast<int>(what.size()), what.data(), expected, actual);
        ++failures;
    }
}

constexpr double kRollingRadius = 0.42;
constexpr double kSpeed = 16.666666666666668;

// A wheel rolling forward at the given speed with the given relative motion at
// the contact. The pitch rate is the one that makes the rim speed match the
// travel speed, so the reference speed comes out as the travel speed itself.
ContactRelativeMotion Rolling(const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d& spin) {
    ContactRelativeMotion motion;
    motion.relative_velocity_track_meters_per_second = velocity;
    motion.relative_spin_track_radians_per_second = spin;
    motion.arc_rate_meters_per_second = kSpeed;
    motion.wheel_pitch_rate_radians_per_second = -kSpeed / kRollingRadius;
    return motion;
}

}  // namespace

int main() {
    const CreepageConfiguration configuration;

    {
        // The frame is a pure roll: orthonormal, right-handed, and its third
        // column is the normal.
        for (const double angle : {0.0, 0.025, -0.4, 1.2}) {
            const ContactFrame frame = MakeContactFrame(angle);
            const Eigen::Matrix3d product = frame.rotation_track_from_contact.transpose() *
                                            frame.rotation_track_from_contact;
            Require((product - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <
                        1.0e-15,
                    "the contact frame is not orthonormal");
            RequireClose(frame.rotation_track_from_contact.determinant(), 1.0, 1.0e-15,
                         "the contact frame is not right-handed");
            Require((frame.normal_track -
                     Eigen::Vector3d(0.0, -std::sin(angle), std::cos(angle)))
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-15,
                    "the contact normal is not the frame's third column");
            // The frame's first axis is exactly the track tangent. That is what
            // makes the longitudinal creepage's numerator the raw longitudinal
            // relative velocity, and it is the property that separates this
            // frame from one built through the wheel's yaw.
            Require(frame.rotation_track_from_contact.col(0) ==
                        Eigen::Vector3d::UnitX(),
                    "the contact frame's longitudinal axis is not the track "
                    "tangent");
        }
    }

    {
        // Pure rolling: no relative motion at the contact, no creepage.
        const Creepages creepages =
            ComputeCreepages(Rolling(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                             MakeContactFrame(0.02), kRollingRadius, configuration);
        Require(creepages.longitudinal == 0.0 && creepages.lateral == 0.0 &&
                    creepages.spin_per_meter == 0.0,
                "pure rolling produced a creepage");
        RequireClose(creepages.reference_speed_meters_per_second, kSpeed, 1.0e-15,
                     "the reference speed of pure rolling is not the travel "
                     "speed");
    }

    {
        // The longitudinal creepage is the raw longitudinal slip over the
        // reference speed, and the contact angle does not touch it.
        const Eigen::Vector3d slip(0.02, 0.0, 0.0);
        double first = 0.0;
        for (const double angle : {0.0, 0.3, -0.7}) {
            const Creepages creepages =
                ComputeCreepages(Rolling(slip, Eigen::Vector3d::Zero()),
                                 MakeContactFrame(angle), kRollingRadius,
                                 configuration);
            RequireClose(creepages.longitudinal, 0.02 / kSpeed, 1.0e-15,
                         "the longitudinal creepage is not the longitudinal slip "
                         "over the reference speed");
            if (angle == 0.0) {
                first = creepages.longitudinal;
            } else {
                Require(creepages.longitudinal == first,
                        "the contact angle reached the longitudinal creepage");
            }
        }
    }

    {
        // The lateral creepage and the spin are the ones the frame turns.
        const double angle = 0.35;
        const Eigen::Vector3d slip(0.0, 0.03, 0.01);
        const Eigen::Vector3d spin(0.0, -39.66, 0.5);
        const Creepages creepages =
            ComputeCreepages(Rolling(slip, spin), MakeContactFrame(angle),
                             kRollingRadius, configuration);
        RequireClose(creepages.lateral,
                     (std::cos(angle) * slip.y() + std::sin(angle) * slip.z()) /
                         kSpeed,
                     1.0e-15,
                     "the lateral creepage is not the rolled lateral slip");
        RequireClose(creepages.spin_per_meter,
                     (-std::sin(angle) * spin.y() + std::cos(angle) * spin.z()) /
                         kSpeed,
                     1.0e-15,
                     "the spin creepage is not the rolled spin");
        // The dominant spin at a rail contact comes from the wheel's own
        // rotation resolved onto the tilted normal, and it is not small.
        Require(std::abs(creepages.spin_per_meter) > 0.1,
                "a canted contact at speed produced a negligible spin, so this "
                "fixture cannot discriminate");
    }

    {
        // A wheel turning faster than it travels creeps forward.
        ContactRelativeMotion motion =
            Rolling(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        motion.wheel_pitch_rate_radians_per_second *= 1.01;
        const Creepages creepages = ComputeCreepages(
            motion, MakeContactFrame(0.0), kRollingRadius, configuration);
        RequireClose(creepages.reference_speed_meters_per_second,
                     0.5 * (kSpeed + 1.01 * kSpeed), 1.0e-15,
                     "the reference speed is not the mean of the travel and rim "
                     "speeds");
    }

    {
        // The floor on the reference speed, and its discontinuity through zero.
        // Both are recorded rather than smoothed: smoothing would change the
        // force at every speed, not only at rest.
        ContactRelativeMotion crawling =
            Rolling(Eigen::Vector3d(0.001, 0.0, 0.0), Eigen::Vector3d::Zero());
        crawling.arc_rate_meters_per_second = 0.001;
        crawling.wheel_pitch_rate_radians_per_second = -0.001 / kRollingRadius;
        const Creepages slow = ComputeCreepages(crawling, MakeContactFrame(0.0),
                                                kRollingRadius, configuration);
        RequireClose(slow.reference_speed_meters_per_second,
                     configuration.minimum_reference_speed_meters_per_second, 0.0,
                     "a crawling contact was not floored");

        ContactRelativeMotion stopped = crawling;
        stopped.arc_rate_meters_per_second = 0.0;
        stopped.wheel_pitch_rate_radians_per_second = 0.0;
        const Creepages still = ComputeCreepages(stopped, MakeContactFrame(0.0),
                                                 kRollingRadius, configuration);
        RequireClose(still.reference_speed_meters_per_second,
                     configuration.minimum_reference_speed_meters_per_second, 0.0,
                     "a stationary contact was not given the forward floor");

        ContactRelativeMotion reversing = crawling;
        reversing.arc_rate_meters_per_second = -0.001;
        reversing.wheel_pitch_rate_radians_per_second = 0.001 / kRollingRadius;
        const Creepages backwards = ComputeCreepages(
            reversing, MakeContactFrame(0.0), kRollingRadius, configuration);
        RequireClose(backwards.reference_speed_meters_per_second,
                     -configuration.minimum_reference_speed_meters_per_second, 0.0,
                     "a reversing contact was not floored the other way");
        Require(slow.longitudinal == -backwards.longitudinal,
                "the creepage did not reverse sign across standstill, so the "
                "documented discontinuity is gone");

        // Just above the floor nothing is done at all.
        ContactRelativeMotion above = crawling;
        above.arc_rate_meters_per_second = 0.02;
        above.wheel_pitch_rate_radians_per_second = -0.02 / kRollingRadius;
        RequireClose(ComputeCreepages(above, MakeContactFrame(0.0), kRollingRadius,
                                      configuration)
                         .reference_speed_meters_per_second,
                     0.02, 0.0, "a contact above the floor was floored anyway");
    }

    {
        // The closing speed is the relative velocity on the normal, positive
        // while approaching. With the vertical pointing down, a wheel settling
        // onto the rail has a positive vertical velocity and must give a
        // positive closing speed.
        const ContactFrame frame = MakeContactFrame(0.02);
        const ContactRelativeMotion settling =
            Rolling(Eigen::Vector3d(0.0, 0.0, 0.05), Eigen::Vector3d::Zero());
        Require(ComputeNormalApproachSpeed(settling, frame) > 0.0,
                "a wheel settling onto the rail gave a negative closing speed, "
                "so the damping would pull");
        RequireClose(ComputeNormalApproachSpeed(settling, frame),
                     frame.normal_track.z() * 0.05, 1.0e-15,
                     "the closing speed is not the relative velocity projected "
                     "on the normal");
        const ContactRelativeMotion lifting =
            Rolling(Eigen::Vector3d(0.0, 0.0, -0.05), Eigen::Vector3d::Zero());
        Require(ComputeNormalApproachSpeed(lifting, frame) < 0.0,
                "a lifting wheel gave a positive closing speed");
    }

    {
        // Both run once per patch per derivative evaluation.
        const ContactFrame frame = MakeContactFrame(0.025);
        const ContactRelativeMotion motion =
            Rolling(Eigen::Vector3d(0.01, 0.02, 0.003),
                    Eigen::Vector3d(0.1, -39.66, 0.4));
        double sink = 0.0;
        const AllocationScope scope;
        for (int step = 0; step < 200; ++step) {
            const Creepages creepages =
                ComputeCreepages(motion, frame, kRollingRadius, configuration);
            sink += creepages.longitudinal + creepages.lateral +
                    creepages.spin_per_meter + ComputeNormalApproachSpeed(motion, frame);
        }
        Require(scope.allocations() == 0,
                "the creepage computation allocated on the integrator's path");
        Require(std::isfinite(sink), "the allocation sweep produced no numbers");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("contact creepage verified");
    return 0;
}
