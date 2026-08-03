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

#include <Eigen/Dense>

#include "orvd/track_geometry/track_geometry.h"
#include "track_geometry/track_geometry_test_lines.h"

namespace {

using orvd::track_geometry::TrackGeometry;
namespace lines = orvd::track_geometry::test_lines;

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

}  // namespace

int main() {
    CheckOrthogonalityAndAgreementOnEachShape();
    CheckColdAndSeededAgreeOnAUniqueRoot();
    CheckProjectionKeepsNoHistory();
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
