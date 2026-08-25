// G47 gate 4: tracking the branch of the centerline a caller is already on.
//
// The checks cover ordinary roots on each line shape, second-order rejection,
// definition-boundary continuation and invalid inputs.

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Dense>

#include "orvd/track_geometry/track_geometry.h"
#include "track_geometry/track_geometry_test_lines.h"

namespace {

using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackStationProjection;
namespace lines = orvd::track_geometry::test_lines;

static_assert(
    std::is_same_v<decltype(std::declval<const TrackStationProjection&>()
                                .closest_centerline_point_in_inertial_meters()),
                   const Eigen::Vector3d&>);
static_assert(
    std::is_same_v<decltype(std::declval<TrackStationProjection&&>()
                                .closest_centerline_point_in_inertial_meters()),
                   Eigen::Vector3d>);

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

// A point a stated distance to the track's right of a stated station.
Eigen::Vector3d PointBesideStation(const TrackGeometry& line, double station,
                                   double lateral_offset_meters) {
    const auto kinematics = line.EvaluateTrackFrame(station);
    return kinematics.pose().origin_in_inertial_meters() +
           lateral_offset_meters *
               kinematics.pose().rotation_inertial_from_track().col(1);
}

void CheckOrthogonalityAndAgreementOnEachShape() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    struct Probe {
        double station;
        const char* shape;
    };
    const Probe probes[] = {{25.0, "straight"},
                            {75.0, "transition"},
                            {200.0, "circular"}};
    for (const Probe& probe : probes) {
        const Eigen::Vector3d point =
            PointBesideStation(line, probe.station, 0.8);
        const auto projection = line.ProjectPointOntoSeededBranch(
            point, probe.station + 2.0);

        const Eigen::Vector3d offset =
            point - projection.closest_centerline_point_in_inertial_meters();
        const Eigen::Vector3d tangent =
            line.CenterlineDerivativeInInertialMetersPerMeter(
                projection.track_station_meters());
        Expect(std::abs(offset.dot(tangent)) <=
                   1.0e-9 * (offset.norm() * tangent.norm() + 1.0),
               std::string("the offset is orthogonal to the tangent on the ") +
                   probe.shape + " stretch");

        Expect(std::abs(projection.track_station_meters() - probe.station) <=
                   1.0e-9,
               std::string("a point placed beside a station projects back onto "
                           "that station on the ") +
                   probe.shape + " stretch");
    }
}

void CheckRootsTheNodeGridDoesNotSurround() {
    // Roots the grid brackets but that a scan over interior nodes looking for a
    // low value cannot see. Each was reported as "no projection exists" or as a
    // pair of equally near stations before the search moved to interval
    // brackets.
    struct Case {
        double length_meters;
        double node_spacing_meters;
        double station_meters;
        const char* description;
    };
    const Case cases[] = {
        {10.0, 20.0, 5.0,
         "a line short enough that the grid has no interior node at all"},
        {10.0, 1.0, 3.0, "a root sitting exactly on a node"},
    };
    for (const Case& one : cases) {
        const TrackGeometry line = lines::MakeLevelStraightLine(
            one.length_meters, one.node_spacing_meters);
        const Eigen::Vector3d point(one.station_meters, 1.0, 0.0);
        bool projected = false;
        double station = 0.0;
        try {
            station = line
                          .ProjectPointOntoSeededBranch(point,
                                                        one.station_meters)
                          .track_station_meters();
            projected = true;
        } catch (const std::exception&) {
        }
        Expect(projected, std::string("the search finds ") + one.description);
        Expect(projected && std::abs(station - one.station_meters) <= 1.0e-12,
               std::string("and returns the station the point was placed "
                           "beside, for ") +
                   one.description);
    }
}

void CheckSecondOrderConditionIsEnforced() {
    // Beyond the centre of curvature the stationary point of the distance is a
    // maximum, not a minimum. Keep the line planar so grade or superelevation
    // cannot hide the sign of the planar distance Hessian. These are ordinary
    // railway dimensions: a 150 m curve and a point 151 m toward its centre.
    constexpr double kRadiusMeters = 150.0;
    constexpr double kLateralOffsetMeters = 151.0;
    const TrackGeometry line =
        lines::MakeLevelCircularLine(kRadiusMeters, 300.0, 0.5);
    const double station = 200.0;
    const Eigen::Vector3d point =
        PointBesideStation(line, station, kLateralOffsetMeters);

    bool refused = false;
    try {
        (void)line.ProjectPointOntoSeededBranch(point, station);
    } catch (const std::runtime_error&) {
        refused = true;
    }
    Expect(refused,
           "a stationary point that is a maximum of the distance "
           "is refused rather than returned as a projection");

    // A point at the same lateral distance but on the outside of the curve has
    // a genuine minimum there, so the refusal above is about the second-order
    // condition and not about the distance being large.
    const Eigen::Vector3d outside =
        PointBesideStation(line, station, -kLateralOffsetMeters);
    bool accepted = true;
    try {
        (void)line.ProjectPointOntoSeededBranch(outside, station);
    } catch (const std::exception&) {
        accepted = false;
    }
    Expect(accepted,
           "the mirror-image point outside the curve, at the same distance, is "
           "accepted");
}

void CheckStraightContinuationsAndBoundaryCrossings() {
    const TrackGeometry line = lines::MakeLevelStraightLine(20.0, 1.0);
    struct Probe {
        double station;
        double seed;
        const char* description;
    };
    const Probe probes[] = {
        {-4.0, -3.5, "the left straight continuation"},
        {27.0, 26.5, "the right straight continuation"},
        {0.4, -0.5, "a projection crossing the left definition boundary"},
        {19.6, 20.5, "a projection crossing the right definition boundary"},
    };
    for (const Probe& probe : probes) {
        const Eigen::Vector3d point =
            PointBesideStation(line, probe.station, 0.7);
        bool accepted = true;
        double projected_station = 0.0;
        try {
            projected_station = line.ProjectPointOntoSeededBranch(
                                        point, probe.seed)
                                    .track_station_meters();
        } catch (const std::exception&) {
            accepted = false;
        }
        Expect(accepted &&
                   std::abs(projected_station - probe.station) <= 1.0e-12,
               std::string(probe.description) +
                   " is searched without clipping to the definition interval");
    }
}

void CheckArgumentRefusals() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const Eigen::Vector3d good = PointBesideStation(line, 200.0, 0.5);

    const auto refuses_invalid_argument = [](auto&& call) {
        try {
            call();
        } catch (const std::invalid_argument&) {
            return true;
        } catch (...) {
            return false;
        }
        return false;
    };

    Expect(refuses_invalid_argument([&] {
               Eigen::Vector3d bad = good;
               bad.y() = std::numeric_limits<double>::quiet_NaN();
               (void)line.ProjectPointOntoSeededBranch(bad, 200.0);
           }),
           "a non-finite point is refused");
    Expect(refuses_invalid_argument([&] {
               (void)line.ProjectPointOntoSeededBranch(
                   good, std::numeric_limits<double>::quiet_NaN());
           }),
           "a non-finite seed is refused");
}

}  // namespace

int main() {
    CheckOrthogonalityAndAgreementOnEachShape();
    CheckRootsTheNodeGridDoesNotSurround();
    CheckSecondOrderConditionIsEnforced();
    CheckStraightContinuationsAndBoundaryCrossings();
    CheckArgumentRefusals();
    if (failure_count != 0) {
        std::printf("%d track station projection checks failed\n",
                    failure_count);
        return 1;
    }
    std::printf("track station projection verified\n");
    return 0;
}
