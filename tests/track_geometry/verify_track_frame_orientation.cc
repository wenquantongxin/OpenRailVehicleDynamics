// G47 gate 2: the orientation of the track frame, the order in which it is
// built, and its rotation rate.
//
// The main fixture carries non-zero curvature, superelevation, grade and
// heading, so the frame construction order is observable.

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Dense>

#include "orvd/track_geometry/track_geometry.h"
#include "track_geometry/track_geometry_test_lines.h"

namespace {

using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackFrameKinematics;
using orvd::track_geometry::TrackFramePose;
namespace lines = orvd::track_geometry::test_lines;

static_assert(std::is_same_v<
              decltype(std::declval<const TrackFramePose&>()
                           .rotation_inertial_from_track()),
              const Eigen::Matrix3d&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackFramePose&&>()
                           .rotation_inertial_from_track()),
              Eigen::Matrix3d>);
static_assert(std::is_same_v<
              decltype(std::declval<const TrackFrameKinematics&>().pose()),
              const TrackFramePose&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackFrameKinematics&&>().pose()),
              TrackFramePose>);
static_assert(std::is_same_v<decltype(std::declval<TrackFramePose&&>()
                                          .origin_in_inertial_meters()),
                             Eigen::Vector3d>);
static_assert(
    std::is_same_v<decltype(std::declval<TrackFrameKinematics&&>()
                                .centerline_derivative_in_inertial_meters_per_meter()),
                   Eigen::Vector3d>);
static_assert(
    std::is_same_v<
        decltype(std::declval<TrackFrameKinematics&&>()
                     .track_frame_rotation_rate_in_inertial_radians_per_meter()),
        Eigen::Vector3d>);

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

constexpr double kMachine = std::numeric_limits<double>::epsilon();

bool NearExact(double measured, double expected, double units_in_last_place) {
    return std::abs(measured - expected) <=
           units_in_last_place * kMachine * std::max(1.0, std::abs(expected));
}

Eigen::Matrix3d RollAboutFirstAxis(double angle_radians) {
    Eigen::Matrix3d roll = Eigen::Matrix3d::Identity();
    roll(1, 1) = std::cos(angle_radians);
    roll(1, 2) = -std::sin(angle_radians);
    roll(2, 1) = std::sin(angle_radians);
    roll(2, 2) = std::cos(angle_radians);
    return roll;
}

// The roll-free frame written out here rather than read back from the library:
// x along the three-dimensional unit tangent, y horizontal and to the track's
// right, z completing the right-handed triad.
Eigen::Matrix3d ExpectedRollFreeRotation(double heading_radians, double grade) {
    const double cosine = std::cos(heading_radians);
    const double sine = std::sin(heading_radians);
    const double tangent_norm = std::sqrt(1.0 + grade * grade);
    Eigen::Matrix3d rotation;
    rotation.col(0) = Eigen::Vector3d(cosine, sine, -grade) / tangent_norm;
    rotation.col(1) = Eigen::Vector3d(-sine, cosine, 0.0);
    rotation.col(2) =
        Eigen::Vector3d(grade * cosine, grade * sine, 1.0) / tangent_norm;
    return rotation;
}

constexpr double kSteepGrade = 0.30;
constexpr double kSteepSuperelevationMeters = 0.10;
constexpr double kStationMeters = 150.0;

void CheckClosedFormRotation() {
    const TrackGeometry line =
        lines::MakeSteepConstantLine(kSteepGrade, kSteepSuperelevationMeters);
    // The curvature is constant, so the heading is the product of curvature and
    // station exactly and does not have to be read out of the library.
    const double heading =
        kStationMeters / lines::kCanonicalRadiusMeters;
    const double roll =
        std::asin(kSteepSuperelevationMeters /
                  lines::kSuperelevationReferenceBaselengthMeters);
    const Eigen::Matrix3d expected =
        ExpectedRollFreeRotation(heading, kSteepGrade) * RollAboutFirstAxis(roll);
    const Eigen::Matrix3d measured = line.EvaluateTrackFrame(kStationMeters)
                                         .pose()
                                         .rotation_inertial_from_track();
    Expect((measured - expected).cwiseAbs().maxCoeff() <= 16.0 * kMachine,
           "all nine components of the track frame rotation match the closed "
           "form built from heading, grade and roll");

    const Eigen::Matrix3d residual =
        measured.transpose() * measured - Eigen::Matrix3d::Identity();
    Expect(residual.cwiseAbs().maxCoeff() <= 16.0 * kMachine,
           "the track frame rotation is orthonormal");
    Expect(NearExact(measured.determinant(), 1.0, 16.0),
           "the track frame rotation has unit positive determinant, so the "
           "triad is right-handed rather than mirrored");
}

void CheckSuperelevationSignByReferencePointHeights() {
    // Two abstract reference points sit half a superelevation reference
    // baselength either side of the centerline along the track frame's
    // transverse axis. Their height difference, measured along the roll-free
    // frame's vertical axis, is the declared superelevation. These points do
    // not assert the physical rail gauge or rail-profile reference locations.
    for (const double superelevation : {kSteepSuperelevationMeters,
                                        -kSteepSuperelevationMeters}) {
        const TrackGeometry line =
            lines::MakeSteepConstantLine(kSteepGrade, superelevation);
        const auto kinematics = line.EvaluateTrackFrame(kStationMeters);
        const Eigen::Matrix3d& rotation =
            kinematics.pose().rotation_inertial_from_track();
        const double half_baselength =
            0.5 * lines::kSuperelevationReferenceBaselengthMeters;
        const Eigen::Vector3d right_reference =
            half_baselength * rotation.col(1);
        const Eigen::Vector3d left_reference =
            -half_baselength * rotation.col(1);

        const double heading = kStationMeters / lines::kCanonicalRadiusMeters;
        const Eigen::Vector3d roll_free_vertical =
            ExpectedRollFreeRotation(heading, kSteepGrade).col(2);
        const double reference_height_difference =
            (right_reference - left_reference).dot(roll_free_vertical);
        Expect(NearExact(reference_height_difference, superelevation, 16.0),
               "the reference-point height difference along the roll-free "
               "vertical is the signed superelevation at superelevation " +
                   std::to_string(superelevation));

        const double tangent_norm =
            std::sqrt(1.0 + kSteepGrade * kSteepGrade);
        const double along_inertial_vertical =
            right_reference.z() - left_reference.z();
        Expect(NearExact(along_inertial_vertical,
                         superelevation / tangent_norm, 16.0),
               "the same difference measured along the inertial vertical is "
               "shortened by the tangent norm, which is the pitch showing up");
    }
}

void CheckRotationRateIdentity() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const double step_meters = 1.0e-5;
    double worst = 0.0;
    for (int index = 0; index <= 20; ++index) {
        const double station = 60.0 + 14.0 * static_cast<double>(index);
        const auto kinematics = line.EvaluateTrackFrame(station);
        const Eigen::Matrix3d& rotation =
            kinematics.pose().rotation_inertial_from_track();
        const Eigen::Vector3d rate =
            kinematics
                .track_frame_rotation_rate_in_inertial_radians_per_meter();
        const Eigen::Matrix3d forward =
            line.EvaluateTrackFrame(station + step_meters)
                .pose()
                .rotation_inertial_from_track();
        const Eigen::Matrix3d backward =
            line.EvaluateTrackFrame(station - step_meters)
                .pose()
                .rotation_inertial_from_track();
        const Eigen::Matrix3d measured_rate =
            (forward - backward) / (2.0 * step_meters);

        Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
        skew(0, 1) = -rate.z();
        skew(0, 2) = rate.y();
        skew(1, 0) = rate.z();
        skew(1, 2) = -rate.x();
        skew(2, 0) = -rate.y();
        skew(2, 1) = rate.x();
        worst = std::max(
            worst, (measured_rate - skew * rotation).cwiseAbs().maxCoeff());
    }
    // Central difference: truncation of order step squared, rounding of order
    // epsilon over step.
    const double budget = step_meters * step_meters + kMachine / step_meters;
    Expect(worst <= 100.0 * budget,
           "the reported rotation rate reproduces the station derivative of "
           "the rotation through the skew identity, within the "
           "finite-difference budget");
}

void CheckRotationRateIsNotMerelyTheHeadingRate() {
    // The station matters. Where the grade and the superelevation are constant
    // the whole triad simply turns about the inertial vertical, so the rate has
    // only a vertical component and an implementation that returned the heading
    // rate alone would pass. Inside a transition, where both are changing, the
    // rate has to pick up roll and pitch as well.
    const TrackGeometry line = lines::MakeCanonicalLine();
    const Eigen::Vector3d inside_transition =
        line.EvaluateTrackFrame(75.0)
            .track_frame_rotation_rate_in_inertial_radians_per_meter();
    Expect(std::abs(inside_transition.x()) > 1.0e-6 ||
               std::abs(inside_transition.y()) > 1.0e-6,
           "inside a transition the rotation rate has a component outside the "
           "inertial vertical, so a heading-rate-only implementation fails");
    Expect(std::abs(inside_transition.z()) > 1.0e-6,
           "and it still carries the heading turn");

    const Eigen::Vector3d in_the_constant_stretch =
        line.EvaluateTrackFrame(200.0)
            .track_frame_rotation_rate_in_inertial_radians_per_meter();
    Expect(std::abs(in_the_constant_stretch.x()) <= 1.0e-15 &&
               std::abs(in_the_constant_stretch.y()) <= 1.0e-15,
           "where the grade and superelevation hold steady the rate is purely "
           "vertical, which is why the check above is not evaluated there");
}

}  // namespace

int main() {
    CheckClosedFormRotation();
    CheckSuperelevationSignByReferencePointHeights();
    CheckRotationRateIdentity();
    CheckRotationRateIsNotMerelyTheHeadingRate();
    if (failure_count != 0) {
        std::printf("%d track frame orientation checks failed\n",
                    failure_count);
        return 1;
    }
    std::printf("track frame orientation verified\n");
    return 0;
}
