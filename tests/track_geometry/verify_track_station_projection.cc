// G47 gate 4: tracking the branch of the centerline a caller is already on.
//
// Orthogonality on its own is not a gate. It holds at a maximum of the distance
// just as it holds at a minimum, so an implementation that walked uphill and
// stopped would satisfy it. The checks below pair it with the second derivative
// of the objective, with agreement against a dense independent scan, and with
// the refusals the contract owes: a window holding two minima the same distance
// from the seed, a window reaching outside the line, a stationary point sitting
// on the wall of the window, and a seed whose neighbourhood holds no minimum.
//
// There is no whole-line search to test, and the last check in this file is the
// reason there is not one.

#include <algorithm>
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
using orvd::track_geometry::TrackScalarProfile;
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
        const auto projection =
            line.ProjectPointNearSeed(point, probe.station + 2.0, 8.0);

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
        // nearest station in the window, not merely a stationary one.
        const double scanned = NearestStationByDenseScan(
            line, point, probe.station - 6.0, probe.station + 10.0, 40000);
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

void CheckTheSeedDoesNotDecideTheAnswer() {
    // Four seeds either side of the same root, each with a window containing
    // it. A search that merely iterated from the seed could drift; a search of
    // the declared window cannot.
    const TrackGeometry line = lines::MakeCanonicalLine();
    const double station = 180.0;
    const Eigen::Vector3d point = PointBesideStation(line, station, -1.1);
    for (const double seed :
         {station - 7.0, station - 0.1, station + 0.1, station + 7.0}) {
        const auto projection = line.ProjectPointNearSeed(point, seed, 9.0);
        Expect(std::abs(projection.track_station_meters() - station) <= 1.0e-9,
               "the branch found from a seed at " + std::to_string(seed) +
                   " m is the same as from any other seed in the window");
    }
}

void CheckProjectionKeepsNoHistory() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const Eigen::Vector3d first = PointBesideStation(line, 120.0, 0.6);
    const Eigen::Vector3d second = PointBesideStation(line, 260.0, -0.6);
    const double first_alone =
        line.ProjectPointNearSeed(first, 120.0, 5.0).track_station_meters();
    (void)line.ProjectPointNearSeed(second, 260.0, 5.0);
    const double first_again =
        line.ProjectPointNearSeed(first, 120.0, 5.0).track_station_meters();
    Expect(first_alone == first_again,
           "projecting a different point in between does not change the result "
           "for the first, so the primitive holds no history");
}

void CheckTemporaryProjectionOwnsItsPoint() {
    const TrackGeometry line = lines::MakeCanonicalLine();
    const Eigen::Vector3d point = PointBesideStation(line, 150.0, 0.4);
    const Eigen::Vector3d taken =
        line.ProjectPointNearSeed(point, 150.0, 5.0)
            .closest_centerline_point_in_inertial_meters();
    Expect(taken.allFinite(),
           "the point taken from a temporary projection survives the temporary, "
           "because the rvalue accessor returns a value rather than a reference "
           "into an object that has already gone");
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
        {10.0, 1.0, 0.4, "a root inside the first node interval"},
        {10.0, 1.0, 9.6, "a root inside the last node interval"},
        {10.0, 1.0, 2.5, "a root exactly halfway between two nodes"},
        {10.0, 1.0, 3.0, "a root sitting exactly on a node"},
    };
    for (const Case& one : cases) {
        const TrackGeometry line = lines::MakeLevelStraightLine(
            one.length_meters, one.node_spacing_meters);
        const Eigen::Vector3d point(one.station_meters, 1.0, 0.0);
        bool projected = false;
        double station = 0.0;
        try {
            station = line.ProjectPointNearSeed(point, one.station_meters, 0.3)
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

void CheckDistinctRootsAreNotMergedByAnAbsoluteStationOrigin() {
    // A one-millimetre version of the symmetric grade arch, translated to a
    // one-million-metre station origin. The one query has two minima about
    // 0.612 mm apart. The retired relative merge tolerance grew to 1 mm at
    // this origin and collapsed them; topology-based duplicate removal must
    // keep them distinct so the equal seed-distance refusal can see both.
    constexpr double kStartStationMeters = 1.0e6;
    constexpr double kLengthMeters = 1.0e-3;
    constexpr double kNodeSpacingMeters = 1.0e-4;
    const TrackGeometry line(
        TrackScalarProfile(
            kStartStationMeters, {lines::Constant(kLengthMeters, 0.0)}, {}),
        TrackScalarProfile(
            kStartStationMeters, {lines::Constant(kLengthMeters, 0.0)}, {}),
        TrackScalarProfile(
            kStartStationMeters,
            {lines::Blend(kLengthMeters, lines::kArchGrade,
                          -lines::kArchGrade)},
            {}),
        lines::kRailReferenceLateralSpanMeters, kNodeSpacingMeters);
    const double start = line.start_track_station_meters();
    const double end = line.end_track_station_meters();
    const double length = end - start;
    const double midpoint = 0.5 * (start + end);
    Eigen::Vector3d point =
        line.CenterlinePositionInInertialMeters(midpoint);
    point.z() += 0.5125 * length;

    const double left =
        line.ProjectPointNearSeed(point, start + 0.2 * length,
                                  0.15 * length)
            .track_station_meters();
    const double right =
        line.ProjectPointNearSeed(point, start + 0.8 * length,
                                  0.15 * length)
            .track_station_meters();
    Expect(right - left > 0.5 * length && right - left < 1.0e-3,
           "two local windows first establish distinct sub-millimetre roots "
           "at the translated station origin");

    bool refused = false;
    std::string message;
    try {
        (void)line.ProjectPointNearSeed(point, midpoint, 0.49 * length);
    } catch (const std::runtime_error& error) {
        refused = true;
        message = error.what();
    }
    Expect(refused,
           "one wide local window retains both roots at a million-metre "
           "station origin and refuses their equal seed distance");
    Expect(message.find("same distance from the seed") != std::string::npos,
           "the translated fixture is refused for branch ambiguity rather "
           "than for an unrelated search failure");
}

void CheckBoundaryCanonicalisationKeepsTheResidualGate() {
    // A very large positive Hessian can make gradient / Hessian lie within a
    // few station ULPs even while the gradient itself is nowhere near the
    // public stationary-point residual. Canonicalising a shared-node root is a
    // duplicate-removal device; it must not bypass candidate qualification.
    constexpr double kStartStationMeters = 99.999;
    constexpr double kLengthMeters = 0.002;
    constexpr double kNodeSpacingMeters = 0.001;
    constexpr double kCurvaturePerMeter = 1.0e10;
    const TrackGeometry line(
        TrackScalarProfile(
            kStartStationMeters,
            {lines::Constant(kLengthMeters, kCurvaturePerMeter)}, {}),
        TrackScalarProfile(
            kStartStationMeters, {lines::Constant(kLengthMeters, 0.0)}, {}),
        TrackScalarProfile(
            kStartStationMeters, {lines::Constant(kLengthMeters, 0.0)}, {}),
        lines::kRailReferenceLateralSpanMeters, kNodeSpacingMeters);
    const double node = kStartStationMeters + kNodeSpacingMeters;
    const auto frame = line.EvaluateTrackFrame(node);
    const Eigen::Vector3d tangent =
        line.CenterlineDerivativeInInertialMetersPerMeter(node);
    const Eigen::Vector3d point =
        frame.pose().origin_in_inertial_meters() + 0.005 * tangent -
        frame.pose().rotation_inertial_from_track().col(1);

    bool respected = false;
    try {
        const auto projection =
            line.ProjectPointNearSeed(point, node, 0.00075);
        const double station = projection.track_station_meters();
        const Eigen::Vector3d projected_tangent =
            line.CenterlineDerivativeInInertialMetersPerMeter(station);
        const Eigen::Vector3d offset =
            point - projection.closest_centerline_point_in_inertial_meters();
        const double gradient = std::abs(offset.dot(projected_tangent));
        const double bound =
            1.0e-10 * (offset.norm() * projected_tangent.norm() + 1.0);
        respected = gradient <= bound;
    } catch (const std::runtime_error&) {
        // This deliberately under-resolved extreme-curvature fixture need not
        // yield a branch, but it must never return a non-stationary point.
        respected = true;
    }
    Expect(respected,
           "shared-node canonicalisation cannot admit a candidate that fails "
           "the same first-derivative residual as every refined candidate");
}

void CheckMinimaEquallyFarFromTheSeedAreRefused() {
    // The arch: straight in plan, rising then falling. A point below its apex
    // is exactly as near to one flank as to the other, and a seed at the apex
    // is exactly as far from one root as from the other.
    // The node spacing is what separates the two roots into different brackets;
    // a grid too coarse to separate them would hand back one of them without
    // noticing the other, which is why the resolution is a stated property of
    // the line rather than something the contract can promise on its own.
    const TrackGeometry line = lines::MakeSymmetricGradeArchLine(1.0);
    const Eigen::Vector3d point(0.5 * lines::kArchLengthMeters, 0.0, 2.0);
    bool refused = false;
    std::string message;
    try {
        (void)line.ProjectPointNearSeed(point, 0.5 * lines::kArchLengthMeters,
                                        4.9);
    } catch (const std::runtime_error& error) {
        refused = true;
        message = error.what();
    }
    Expect(refused,
           "a window holding two minima the same distance from the seed is "
           "refused, because which branch the caller is on is then not a "
           "function of what the caller supplied");
    Expect(message.find("same distance from the seed") != std::string::npos,
           "and the refusal says what went wrong");

    // A seed nearer one flank than the other resolves it, so the fixture has
    // not simply broken the search.
    bool resolved = true;
    try {
        (void)line.ProjectPointNearSeed(point, 2.0, 1.5);
    } catch (const std::exception&) {
        resolved = false;
    }
    Expect(resolved,
           "moving the seed onto one flank resolves the same point on the same "
           "line");

    // Keep both minima in the declared window and move only the seed. This
    // directly exercises the branch-selection rule instead of shrinking the
    // window until only one candidate remains.
    const double left =
        line.ProjectPointNearSeed(point, 4.5, 4.4).track_station_meters();
    const double right =
        line.ProjectPointNearSeed(point, 5.5, 4.4).track_station_meters();
    Expect(left < 0.5 * lines::kArchLengthMeters &&
               right > 0.5 * lines::kArchLengthMeters,
           "when two non-equidistant branches remain in the window, the "
           "candidate nearest the supplied seed is selected");
}

void CheckSecondOrderConditionIsEnforced() {
    // Beyond the centre of curvature the stationary point of the distance is a
    // maximum, not a minimum: the second derivative of the objective is the
    // general expression, and it goes negative there.
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
           "a window whose only stationary point is a maximum of the distance "
           "is refused rather than returned as a projection");

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

void CheckAStationaryPointOnTheWindowWallIsRefused() {
    // The contract is strictly interior. A window whose wall lands on the root
    // is a window in the wrong place, and saying so is more use to the caller
    // than returning the wall.
    const TrackGeometry line = lines::MakeLevelStraightLine(20.0, 1.0);
    const Eigen::Vector3d point(10.0, 0.7, 0.0);
    bool refused = false;
    std::string message;
    try {
        (void)line.ProjectPointNearSeed(point, 12.0, 2.0);
    } catch (const std::runtime_error& error) {
        refused = true;
        message = error.what();
    }
    Expect(refused, "a root sitting on the wall of the window is refused");
    Expect(message.find("wall of the window") != std::string::npos,
           "and the refusal points at the window rather than at the line");

    bool accepted = true;
    try {
        (void)line.ProjectPointNearSeed(point, 12.0, 3.0);
    } catch (const std::exception&) {
        accepted = false;
    }
    Expect(accepted,
           "widening the window so the same root falls strictly inside it "
           "succeeds");
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
               (void)line.ProjectPointNearSeed(bad, 200.0, 5.0);
           }),
           "a non-finite point is refused");
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

void CheckWhyThereIsNoWholeLineSearch() {
    // This is the check that explains the shape of the interface. Both fixtures
    // below defeat any attempt to certify a whole-line nearest-station search
    // from a construction-time bound on the geometry, which is why the library
    // takes a seed instead of promising to find the branch by itself.
    //
    // The first is straight in plan: its heading is identically zero, so no
    // bound on how far the heading turns per node interval says anything about
    // it, yet the line holds two minima the same distance from the point.
    const TrackGeometry arch = lines::MakeSymmetricGradeArchLine(1.0);
    const Eigen::Vector3d below_apex(0.5 * lines::kArchLengthMeters, 0.0, 2.0);
    double first_root = 0.0;
    double second_root = 0.0;
    double first_distance = std::numeric_limits<double>::infinity();
    double second_distance = std::numeric_limits<double>::infinity();
    const int samples = 200000;
    for (int index = 0; index <= samples; ++index) {
        const double station = lines::kArchLengthMeters *
                               static_cast<double>(index) /
                               static_cast<double>(samples);
        const double distance =
            (below_apex - arch.CenterlinePositionInInertialMeters(station))
                .norm();
        if (station < 0.5 * lines::kArchLengthMeters) {
            if (distance < first_distance) {
                first_distance = distance;
                first_root = station;
            }
        } else if (distance < second_distance) {
            second_distance = distance;
            second_root = station;
        }
    }
    Expect(std::abs(first_root - second_root) > 1.0,
           "the arch really does have two separated minima");
    Expect(std::abs(first_distance - second_distance) <= 1.0e-9,
           "and they are the same distance from the point, so no single "
           "nearest station exists for it");
    Expect(arch.HeadingRadians(0.5 * lines::kArchLengthMeters) == 0.0,
           "while the heading is identically zero, so a bound on the heading "
           "turn per interval would have let this line through");

    // The second is a curvature that swings symmetrically: the heading returns
    // to where it started across the segment while sweeping several radians
    // inside it, so a net turn measured across an interval says nothing either.
    const TrackScalarProfile swing(0.0, {lines::Blend(1.0, 10.0, -10.0)}, {});
    Expect(std::abs(swing.IntegralFromStart(1.0)) <= 1.0e-12,
           "the swinging curvature leaves the heading unchanged across the "
           "whole segment");
    Expect(std::abs(swing.IntegralFromStart(0.5)) > 3.0,
           "while the heading reaches several radians inside it, so the net "
           "turn across an interval is not a resolution measure either");
}

}  // namespace

int main() {
    CheckOrthogonalityAndAgreementOnEachShape();
    CheckTheSeedDoesNotDecideTheAnswer();
    CheckProjectionKeepsNoHistory();
    CheckTemporaryProjectionOwnsItsPoint();
    CheckRootsTheNodeGridDoesNotSurround();
    CheckDistinctRootsAreNotMergedByAnAbsoluteStationOrigin();
    CheckBoundaryCanonicalisationKeepsTheResidualGate();
    CheckMinimaEquallyFarFromTheSeedAreRefused();
    CheckSecondOrderConditionIsEnforced();
    CheckAStationaryPointOnTheWindowWallIsRefused();
    CheckArgumentRefusals();
    CheckWhyThereIsNoWholeLineSearch();
    if (failure_count != 0) {
        std::printf("%d track station projection checks failed\n",
                    failure_count);
        return 1;
    }
    std::printf("track station projection verified\n");
    return 0;
}
