#include <cmath>

#include <Eigen/Core>

#include "orvd/track_geometry/track_geometry.h"

int main() {
    using orvd::track_geometry::TrackGeometry;
    using orvd::track_geometry::TrackScalarProfile;
    using orvd::track_geometry::TrackScalarSegment;
    using orvd::track_geometry::TrackScalarSegmentShape;

    TrackScalarSegment level;
    level.length_meters = 10.0;
    level.shape = TrackScalarSegmentShape::kConstant;
    const TrackGeometry line(TrackScalarProfile(0.0, {level}, {}),
                             TrackScalarProfile(0.0, {level}, {}),
                             TrackScalarProfile(0.0, {level}, {}), 1.5, 1.0);
    const Eigen::Vector3d point =
        line.CenterlinePositionInInertialMeters(4.0);
    return std::abs(point.x() - 4.0) <= 1.0e-12 && point.y() == 0.0 &&
                   point.z() == 0.0
               ? 0
               : 1;
}
