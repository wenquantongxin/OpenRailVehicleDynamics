// G47 gate 4: projecting a point onto the centerline.
//
// Orthogonality on its own is not a gate. It holds at a maximum of the distance
// just as it holds at a minimum, so an implementation that walked uphill and
// stopped would satisfy it. The checks below therefore pair it with the second
// derivative of the objective, with agreement against a dense independent scan,
// and with the refusals the contract owes: a point with two equally near
// stations, a search interval that reaches outside the line, and a seed whose
// neighbourhood contains no minimum at all.

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

static_assert(std::is_same_v<
              decltype(std::declval<const TrackStationProjection&>()
                           .closest_centerline_point_in_inertial_meters()),
              const Eigen::Vector3d&>);
static_assert(std::is_same_v<
              decltype(std::declval<TrackStationProjection&&>()
                           .closest_centerline_point_in_inertial_meters()),
              Eigen::Vector3d>);

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

// A dense independent scan for the nearest station, with no shared code with
// the library's own search: it never differentiates, never brackets and never
// refines.
double NearestStationByDenseScan(const TrackGeometry& line,
                                 const Eigen::Vector3d& point, double from,
                                 double to, int samples) {
    double best_station = from;
    double best_squared = std::numeric_limits<double>::infinity();
    for (int index = 0; index <= samples; ++index) {
        const double station =
            from + (to - from) * static_cast<double>(index) /
                       static_cast<double>(samples);
        const double squared =
            (point - line.CenterlinePositionInInertialMeters(station))
                .squaredNorm();
        if (squared < best_squared) {
            best_squared = squared;
            best_station = station;
        }
    }
    return best_station;
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
        const auto projection = line.ProjectPointColdStart(point);

        const Eigen::Vector3d offset =
            point - projection.closest_centerline_point_in_inertial_meters();
        const Eigen::Vector3d tangent =
            line.CenterlineDerivativeInInertialMetersPerMeter(
                projection.track_station_meters());
        Expect(std::abs(offset.dot(tangent)) <=
                   1.0e-9 * (offset.norm() * tangent.norm() + 1.0),
               std::string("the offset is orthogonal to the tangent on the ") +
                   probe.shape + " stretch");

        // The independent statement orthogonality cannot make: this is the
        // nearest station, not merely a stationary one.
        const double scanned = NearestStationByDenseScan(
            line, point, probe.station - 5.0, probe.station + 5.0, 20000);
        Expect(std::abs(projection.track_station_meters() - scanned) <= 1.0e-3,
               std::string("the returned station agrees with a dense "
                           "independent scan on the ") +
                   probe.shape + " stretch");
        Expect(std::abs(projection.track_station_meters() - probe.station) <=
                   1.0e-9,
               std::string("a point placed beside a station projects back onto "
                           "that station on the ") +
                   probe.shape + " stretch");
    }
}

void CheckColdAndSeededAgreeOnAUniqueRoot() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const double station = 180.0;
    const Eigen::Vector3d point = PointBesideStation(line, station, -1.1);
    const auto cold = line.ProjectPointColdStart(point);
    const auto seeded = line.ProjectPointNearSeed(point, station + 3.0, 8.0);
    Expect(std::abs(cold.track_station_meters() -
                    seeded.track_station_meters()) <= 1.0e-9,
           "on a fixture with one minimum the cold start and the seeded query "
           "converge to the same station");
    Expect(std::abs(cold.track_station_meters() - station) <= 1.0e-9,
           "and that station is the one the point was placed beside");
}

void CheckProjectionKeepsNoHistory() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const Eigen::Vector3d first = PointBesideStation(line, 120.0, 0.6);
    const Eigen::Vector3d second = PointBesideStation(line, 260.0, -0.6);
    const double first_alone = line.ProjectPointColdStart(first)
                                   .track_station_meters();
    (void)line.ProjectPointColdStart(second);
    const double first_again = line.ProjectPointColdStart(first)
                                   .track_station_meters();
    Expect(first_alone == first_again,
           "projecting a different point in between does not change the result "
           "for the first, so the primitive holds no history");
}

void CheckTemporaryProjectionOwnsItsPoint() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const Eigen::Vector3d query = PointBesideStation(line, 120.0, 0.6);
    const auto& projected = line.ProjectPointColdStart(query)
                                .closest_centerline_point_in_inertial_meters();
    Expect(projected.allFinite(),
           "accessing the point through a temporary projection result returns an "
           "owned value rather than a dangling reference");
}

void CheckEquidistantMinimaAreRefused() {
    const TrackGeometry line = lines::MakeEquidistantTieLine();
    // The two straight stretches run parallel, one turn diameter apart. A point
    // halfway between them is exactly as near to one as to the other.
    const double half_way_lateral = lines::kTieRadiusMeters;
    const Eigen::Vector3d point =
        PointBesideStation(line, 30.0, half_way_lateral);

    bool refused = false;
    std::string message;
    try {
        (void)line.ProjectPointColdStart(point);
    } catch (const std::runtime_error& error) {
        refused = true;
        message = error.what();
    }
    Expect(refused,
           "a point equally near two distinct stations is refused, because the "
           "nearest station is not a function of that point");
    Expect(message.find("equally close") != std::string::npos,
           "the refusal says what went wrong rather than merely failing");

    // The same line still projects an ordinary point, so the fixture has not
    // simply broken the search.
    const Eigen::Vector3d ordinary = PointBesideStation(line, 30.0, 1.0);
    bool ordinary_works = true;
    try {
        (void)line.ProjectPointColdStart(ordinary);
    } catch (const std::exception&) {
        ordinary_works = false;
    }
    Expect(ordinary_works,
           "a point clearly nearer one stretch than the other still projects "
           "on the same line");
}

void CheckSecondOrderConditionIsEnforced() {
    // Beyond the centre of curvature the stationary point of the distance is a
    // maximum, not a minimum: the second derivative of the objective is
    // one minus the lateral offset over the radius.
    const TrackGeometry line = lines::MakeCanonicalLine();
    const double station = 200.0;
    const double beyond_centre = lines::kCanonicalRadiusMeters + 50.0;
    const Eigen::Vector3d point =
        PointBesideStation(line, station, beyond_centre);

    bool refused = false;
    try {
        (void)line.ProjectPointNearSeed(point, station, 20.0);
    } catch (const std::runtime_error&) {
        refused = true;
    }
    Expect(refused,
           "a seed at a station whose stationary point is a maximum of the "
           "distance is refused rather than returned as a projection");

    // A point at the same lateral distance but on the outside of the curve has
    // a genuine minimum there, so the refusal above is about the second-order
    // condition and not about the distance being large.
    const Eigen::Vector3d outside =
        PointBesideStation(line, station, -beyond_centre);
    bool accepted = true;
    try {
        (void)line.ProjectPointNearSeed(outside, station, 20.0);
    } catch (const std::exception&) {
        accepted = false;
    }
    Expect(accepted,
           "the mirror-image point outside the curve, at the same distance, is "
           "accepted");
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
               (void)line.ProjectPointColdStart(bad);
           }),
           "a non-finite point is refused by the cold start");
    Expect(refuses_invalid_argument([&] {
               (void)line.ProjectPointNearSeed(good, 5.0, 20.0);
           }),
           "a search interval that reaches past the start of the line is "
           "refused rather than clipped to the domain");
    Expect(refuses_invalid_argument([&] {
               (void)line.ProjectPointNearSeed(good, 200.0, 0.0);
           }),
           "a non-positive search half width is refused");
    Expect(refuses_invalid_argument([&] {
               (void)line.ProjectPointNearSeed(good, 1.0e9, 1.0);
           }),
           "a seed outside the domain is refused");
}

void CheckRootsTheNodeGridDoesNotSurround() {
    // Four configurations an adversarial review produced before any of them had
    // a test. Each is a root the grid brackets but that a scan over interior
    // nodes looking for a low value cannot see, so all four used to be reported
    // as "no projection exists" or as a pair of equally near stations.
    struct Case {
        double length_meters;
        double node_spacing_meters;
        double station_meters;
        const char* description;
    };
    const Case cases[] = {
        {10.0, 20.0, 5.0,
         "a line short enough that the grid has no interior node at all"},
        {10.0, 1.0, 0.2, "a root inside the first half interval"},
        {10.0, 1.0, 9.8, "a root inside the last half interval"},
        {10.0, 1.0, 2.5, "a root exactly halfway between two nodes"},
        {10.0, 1.0, 3.0, "a root sitting exactly on a node"},
    };
    for (const Case& one : cases) {
        const TrackGeometry line =
            lines::MakeLevelStraightLine(one.length_meters,
                                         one.node_spacing_meters);
        const Eigen::Vector3d point(one.station_meters, 1.0, 0.0);
        bool projected = false;
        double station = 0.0;
        try {
            station = line.ProjectPointColdStart(point).track_station_meters();
            projected = true;
        } catch (const std::exception&) {
        }
        Expect(projected,
               std::string("the cold start projects a point beside ") +
                   one.description);
        Expect(projected && std::abs(station - one.station_meters) <= 1.0e-12,
               std::string("and returns the station the point was placed "
                           "beside, for ") +
                   one.description);
    }
}

void CheckACoarseGridOnACurveIsRefused() {
    // The interval scan can only claim completeness while the heading is nearly
    // straight across one interval. A spacing that lets the heading swing
    // through a quarter of a radian per interval is refused at construction
    // with a request for a finer one, rather than searched with a grid too
    // coarse to resolve it.
    bool refused = false;
    std::string message;
    try {
        const TrackGeometry too_coarse = lines::MakeTightTurnLine(50.0);
        (void)too_coarse;
    } catch (const std::invalid_argument& error) {
        refused = true;
        message = error.what();
    }
    Expect(refused,
           "a node spacing that lets the heading turn far within one interval "
           "is refused at construction");
    Expect(message.find("node spacing") != std::string::npos,
           "and the refusal says which parameter to change");

    bool accepted = true;
    try {
        const TrackGeometry fine_enough = lines::MakeTightTurnLine(0.5);
        (void)fine_enough;
    } catch (const std::exception&) {
        accepted = false;
    }
    Expect(accepted, "the same line with a finer spacing is accepted");
}

}  // namespace

int main() {
    CheckOrthogonalityAndAgreementOnEachShape();
    CheckRootsTheNodeGridDoesNotSurround();
    CheckACoarseGridOnACurveIsRefused();
    CheckColdAndSeededAgreeOnAUniqueRoot();
    CheckProjectionKeepsNoHistory();
    CheckTemporaryProjectionOwnsItsPoint();
    CheckEquidistantMinimaAreRefused();
    CheckSecondOrderConditionIsEnforced();
    CheckArgumentRefusals();
    if (failure_count != 0) {
        std::printf("%d track station projection checks failed\n",
                    failure_count);
        return 1;
    }
    std::printf("track station projection verified\n");
    return 0;
}
