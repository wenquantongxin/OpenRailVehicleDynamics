#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <exception>
#include <numbers>
#include <string>
#include <vector>

#include "orvd/vehicle_project_editing/vehicle_project_rotation_from_and_to_roll_pitch_yaw_radians.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// The roll-pitch-yaw convention orientations are read and edited in, stated
// from what it means rather than from a run: a quarter turn of yaw takes x to
// y and y to -x, of pitch z to x, of roll y to z, and a composed case is the
// product Rz Ry Rx written out here. Over a grid of angles inside the ranges,
// the angles read from the rotation they give lie in those ranges and give
// that rotation again; where pitch is pi/2 or -pi/2, yaw reads 0 and the
// angles still give the rotation.

namespace {

using namespace orvd;
using editing_verification::Require;
using vehicle_project_editing::RollPitchYawRadiansOfRotation;
using vehicle_project_editing::RotationFromRollPitchYawRadians;

constexpr double kPi = std::numbers::pi;

// A few rounding steps of a product of three rotations.
constexpr double kRotationTolerance = 1.0e-14;

template <typename Expected, typename Actual>
bool Near(const Eigen::MatrixBase<Expected>& expected, const Eigen::MatrixBase<Actual>& actual) {
    return (expected - actual).cwiseAbs().maxCoeff() <= kRotationTolerance;
}

std::string Describe(const Eigen::Vector3d& roll_pitch_yaw_radians) {
    return "roll " + std::to_string(roll_pitch_yaw_radians.x()) + ", pitch " +
           std::to_string(roll_pitch_yaw_radians.y()) + ", yaw " +
           std::to_string(roll_pitch_yaw_radians.z());
}

void VerifyQuarterTurns() {
    const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
    const Eigen::Matrix3d yaw = RotationFromRollPitchYawRadians({0.0, 0.0, kPi / 2.0});
    Require(Near(y, yaw * x) && Near(-x, yaw * y) && Near(z, yaw * z),
            "a quarter turn of yaw does not take x to y and y to -x about z");
    const Eigen::Matrix3d pitch = RotationFromRollPitchYawRadians({0.0, kPi / 2.0, 0.0});
    Require(Near(x, pitch * z) && Near(-z, pitch * x) && Near(y, pitch * y),
            "a quarter turn of pitch does not take z to x and x to -z about y");
    const Eigen::Matrix3d roll = RotationFromRollPitchYawRadians({kPi / 2.0, 0.0, 0.0});
    Require(Near(z, roll * y) && Near(-y, roll * z) && Near(x, roll * x),
            "a quarter turn of roll does not take y to z and z to -y about x");
}

// Rz(yaw) Ry(pitch) Rx(roll), each factor written out.
Eigen::Matrix3d ProductWrittenOut(double roll, double pitch, double yaw) {
    Eigen::Matrix3d about_z;
    about_z << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d about_y;
    about_y << std::cos(pitch), 0.0, std::sin(pitch), 0.0, 1.0, 0.0, -std::sin(pitch), 0.0,
        std::cos(pitch);
    Eigen::Matrix3d about_x;
    about_x << 1.0, 0.0, 0.0, 0.0, std::cos(roll), -std::sin(roll), 0.0, std::sin(roll),
        std::cos(roll);
    return about_z * about_y * about_x;
}

void VerifyComposedCase() {
    const Eigen::Vector3d angles(0.3, -0.7, 2.1);
    const Eigen::Matrix3d rotation = RotationFromRollPitchYawRadians(angles);
    Require(Near(ProductWrittenOut(angles.x(), angles.y(), angles.z()), rotation),
            "the rotation of " + Describe(angles) + " is not Rz(yaw) Ry(pitch) Rx(roll)");
    Require(Near(angles, RollPitchYawRadiansOfRotation(rotation)),
            "the angles read from the rotation of " + Describe(angles) + " are not those angles");
}

// The angles read lie in the ranges: roll and yaw in (-pi, pi], pitch in
// [-pi/2, pi/2].
bool InRanges(const Eigen::Vector3d& angles) {
    return angles.x() > -kPi && angles.x() <= kPi && angles.y() >= -kPi / 2.0 &&
           angles.y() <= kPi / 2.0 && angles.z() > -kPi && angles.z() <= kPi;
}

// The angles read from the rotation `angles` give lie in the ranges and give
// that rotation again.
Eigen::Vector3d RequireReadBackGivesTheRotation(const Eigen::Vector3d& angles) {
    const Eigen::Matrix3d rotation = RotationFromRollPitchYawRadians(angles);
    const Eigen::Vector3d read = RollPitchYawRadiansOfRotation(rotation);
    Require(InRanges(read) && Near(rotation, RotationFromRollPitchYawRadians(read)),
            "the angles read from the rotation of " + Describe(angles) + ", " + Describe(read) +
                ", lie outside the ranges or give another rotation");
    return read;
}

// Over a grid of twelfths of a turn, and at pitches close to pi/2 and -pi/2.
void VerifyBothDirectionsAgree() {
    constexpr int kStepsPerHalfTurn = 6;
    constexpr double kStep = kPi / kStepsPerHalfTurn;
    const std::vector<double> pitches_close_to_the_poles = {
        1.2, -1.2, 1.5, -1.5, kPi / 2.0 - 1.0e-7, -kPi / 2.0 + 1.0e-7};
    for (int roll_step = -kStepsPerHalfTurn + 1; roll_step <= kStepsPerHalfTurn; ++roll_step) {
        for (int yaw_step = -kStepsPerHalfTurn + 1; yaw_step <= kStepsPerHalfTurn; ++yaw_step) {
            for (int pitch_step = -kStepsPerHalfTurn / 2; pitch_step <= kStepsPerHalfTurn / 2;
                 ++pitch_step) {
                const Eigen::Vector3d angles(roll_step * kStep, pitch_step * kStep,
                                             yaw_step * kStep);
                const Eigen::Vector3d read = RequireReadBackGivesTheRotation(angles);
                if (std::abs(pitch_step) == kStepsPerHalfTurn / 2) {
                    Require(
                        std::abs(read.y()) == kPi / 2.0 && read.z() == 0.0,
                        "at " + Describe(angles) + " pitch does not read pi/2 or -pi/2 with yaw 0");
                }
            }
            for (const double pitch : pitches_close_to_the_poles) {
                (void)RequireReadBackGivesTheRotation({roll_step * kStep, pitch, yaw_step * kStep});
            }
        }
    }
}

}  // namespace

int main() {
    try {
        VerifyQuarterTurns();
        VerifyComposedCase();
        VerifyBothDirectionsAgree();
        std::puts("rotation from and to roll, pitch and yaw in radians passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "rotation from and to roll, pitch and yaw in radians: %s\n",
                     error.what());
        return 1;
    }
}
