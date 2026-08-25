#include "orvd/track_geometry/track_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>

namespace orvd::track_geometry {
namespace {

std::string Describe(double value) { return std::to_string(value); }

// An eight-point Gauss-Legendre rule on [-1, 1]. Fixed order and fixed panel
// length make the quadrature error a stated property of the node spacing rather
// than something an adaptive rule decides per call, and keep the evaluation
// free of any allocation.
constexpr int kQuadraturePointCount = 8;
constexpr double kQuadratureAbscissae[kQuadraturePointCount] = {
    -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
    -0.1834346424956498, 0.1834346424956498,  0.5255324099163290,
    0.7966664774136267,  0.9602898564975363};
constexpr double kQuadratureWeights[kQuadraturePointCount] = {
    0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
    0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
    0.2223810344533745, 0.1012285362903763};

// Bound the whole immutable node table, not each profile stretch separately.
// 2^20 nodes still admit a 10 km line at approximately 1 cm spacing while
// refusing accidental allocations of hundreds of megabytes before conversion
// or allocation begins.
constexpr std::size_t kMaximumStationNodeCount = 1u << 20;

// The first derivative of the squared-distance objective has units of metres
// times metres per metre, so its residual bound scales with the distance.
constexpr double kObjectiveGradientRelativeTolerance = 1.0e-10;

void RequireFinitePositive(double value, const char* what) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string("TrackGeometry: ") + what +
                                    " must be finite and positive, got " +
                                    Describe(value));
    }
}

}  // namespace

TrackGeometry::TrackGeometry(TrackScalarProfile curvature_radians_per_meter,
                             TrackScalarProfile superelevation_meters,
                             TrackVerticalProfile vertical_profile,
                             double superelevation_reference_baselength_meters,
                             double station_node_spacing_meters)
    : curvature_(std::move(curvature_radians_per_meter)),
      superelevation_(std::move(superelevation_meters)),
      grade_(std::move(vertical_profile)),
      superelevation_reference_baselength_meters_(
          superelevation_reference_baselength_meters),
      station_node_spacing_meters_(station_node_spacing_meters) {
    RequireFinitePositive(superelevation_reference_baselength_meters,
                          "the superelevation reference baselength");
    RequireFinitePositive(station_node_spacing_meters,
                          "the station node spacing");

    // Different segment partitions can accumulate the same declared endpoint
    // with slightly different rounding. Bound that difference by the number of
    // additions at the start/end station scale.
    const double common_start = curvature_.start_track_station_meters();
    const auto agrees = [common_start](double left, double right,
                                       std::size_t additions) {
        const double scale =
            std::max({1.0, std::abs(common_start), std::abs(left),
                      std::abs(right)});
        // Compare normalised values so the bound itself cannot overflow.
        const double normalised_difference =
            std::abs(left / scale - right / scale);
        const double normalised_bound =
            4.0 * static_cast<double>(additions + 1) *
            std::numeric_limits<double>::epsilon();
        return normalised_difference <= normalised_bound;
    };
    const auto same_domain = [this, &agrees](const auto& other,
                                             const char* name) {
        const std::size_t additions =
            other.breakpoint_track_stations_meters().size() +
            curvature_.breakpoint_track_stations_meters().size();
        if (other.start_track_station_meters() !=
                curvature_.start_track_station_meters() ||
            !agrees(other.end_track_station_meters(),
                    curvature_.end_track_station_meters(), additions)) {
            throw std::invalid_argument(
                std::string("TrackGeometry: the ") + name +
                " profile covers [" +
                Describe(other.start_track_station_meters()) + ", " +
                Describe(other.end_track_station_meters()) +
                "] m while the curvature profile covers [" +
                Describe(curvature_.start_track_station_meters()) + ", " +
                Describe(curvature_.end_track_station_meters()) +
                "] m; one line has one station domain");
        }
    };
    same_domain(superelevation_, "superelevation");
    same_domain(grade_, "grade");

    // A tolerant comparison alone is not enough: evaluating the curvature
    // endpoint would otherwise be an out-of-domain query for whichever family
    // rounded a little shorter. Canonicalise to the shortest equivalent end so
    // no profile is ever extrapolated.
    const double common_end =
        std::min({curvature_.end_track_station_meters(),
                  superelevation_.end_track_station_meters(),
                  grade_.end_track_station_meters()});
    curvature_.ShortenDomainEndTo(common_end);
    superelevation_.ShortenDomainEndTo(common_end);
    grade_.ShortenDomainEndTo(common_end);

    // Nodes sit on the ordered union of curvature and vertical breakpoints.
    // Horizontal quadrature only depends on curvature, but projection also
    // consumes the vertical first derivative and must not search across a
    // switch of either centerline formula as if it were one smooth interval.
    std::vector<double> breakpoints =
        curvature_.breakpoint_track_stations_meters();
    const std::vector<double>& vertical_breakpoints =
        grade_.breakpoint_track_stations_meters();
    breakpoints.insert(breakpoints.end(), vertical_breakpoints.begin(),
                       vertical_breakpoints.end());
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()),
                      breakpoints.end());
    // First count the whole line. A per-stretch check is not a total bound: a
    // multi-piece profile could otherwise request the full limit repeatedly,
    // and even one stretch at the old limit acquired one extra terminal node.
    std::size_t total_node_count = 1;
    for (std::size_t index = 0; index + 1 < breakpoints.size(); ++index) {
        const double from = breakpoints[index];
        const double to = breakpoints[index + 1];
        const double length = to - from;
        const double requested_panels =
            std::max(1.0, std::ceil(length / station_node_spacing_meters_));
        const std::size_t remaining =
            kMaximumStationNodeCount - total_node_count;
        // A spacing may be finite and positive and still be absurd. Compare in
        // double before converting, and accumulate against the remaining whole
        // table budget so neither conversion nor size_t addition can overflow.
        if (requested_panels > static_cast<double>(remaining)) {
            throw std::invalid_argument(
                "TrackGeometry: a station node spacing of " +
                Describe(station_node_spacing_meters_) +
                " m would make the whole line exceed the limit of " +
                std::to_string(kMaximumStationNodeCount) +
                " station nodes while processing a stretch of " +
                Describe(length) + " m; choose a coarser spacing");
        }
        total_node_count += static_cast<std::size_t>(requested_panels);
    }
    nodes_.reserve(total_node_count);
    for (std::size_t index = 0; index + 1 < breakpoints.size(); ++index) {
        const double from = breakpoints[index];
        const double to = breakpoints[index + 1];
        const double length = to - from;
        const auto panels = static_cast<std::size_t>(
            std::max(1.0,
                     std::ceil(length / station_node_spacing_meters_)));
        for (std::size_t panel = 0; panel < panels; ++panel) {
            StationNode node;
            node.track_station_meters =
                from + length * static_cast<double>(panel) /
                           static_cast<double>(panels);
            nodes_.push_back(node);
        }
    }
    StationNode last_node;
    last_node.track_station_meters = curvature_.end_track_station_meters();
    nodes_.push_back(last_node);

    for (std::size_t index = 0; index + 1 < nodes_.size(); ++index) {
        const double midpoint =
            0.5 * (nodes_[index].track_station_meters +
                   nodes_[index + 1].track_station_meters);
        nodes_[index].interval_has_constant_curvature =
            curvature_.LocalPolynomialDegree(midpoint) == 0;
        nodes_[index].interval_curvature_radians_per_meter =
            curvature_.Value(midpoint);
    }

    // The superelevation bound is a construction-time refusal. A roll of a
    // quarter turn is not a line, and the roll rate is unbounded there, so the
    // admissible interval is open rather than closed.
    const TrackScalarProfile::ProfileExtremum superelevation_extremum =
        superelevation_.MaximumAbsoluteValue();
    if (std::abs(superelevation_extremum.value) >=
        superelevation_reference_baselength_meters_) {
        throw std::invalid_argument(
            "TrackGeometry: the superelevation " +
            Describe(superelevation_extremum.value) + " m at station " +
            Describe(superelevation_extremum.track_station_meters) +
            " m reaches or exceeds the superelevation reference baselength " +
            Describe(superelevation_reference_baselength_meters_) +
            " m; the roll-to-superelevation derivative is singular there, so "
            "finite first-order track-frame kinematics cannot be formed");
    }

    // Heading is the exact integral of the curvature; only the horizontal
    // position needs quadrature, and only where the curvature is not constant.
    //
    // The horizontal position is a running sum over the nodes, so its error
    // would otherwise grow with the node count and a finer spacing would make
    // the absolute position worse even as the quadrature got better. Neumaier
    // compensation removes that growth, which keeps "refine the spacing and the
    // position stops moving" a statement about the rule rather than a race
    // between two error sources.
    nodes_.front().heading_radians = 0.0;
    nodes_.front().centerline_position_in_inertial_meters =
        Eigen::Vector3d::Zero();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double compensation_x = 0.0;
    double compensation_y = 0.0;
    const auto accumulate = [](double& sum, double& compensation, double term) {
        const double total = sum + term;
        if (std::abs(sum) >= std::abs(term)) {
            compensation += (sum - total) + term;
        } else {
            compensation += (term - total) + sum;
        }
        sum = total;
    };
    for (std::size_t index = 0; index + 1 < nodes_.size(); ++index) {
        StationNode& next = nodes_[index + 1];
        next.heading_radians =
            curvature_.IntegralFromStart(next.track_station_meters);
        const Eigen::Vector2d step =
            HorizontalDisplacementFromNode(index, next.track_station_meters);
        accumulate(sum_x, compensation_x, step.x());
        accumulate(sum_y, compensation_y, step.y());
        next.centerline_position_in_inertial_meters.x() = sum_x + compensation_x;
        next.centerline_position_in_inertial_meters.y() = sum_y + compensation_y;
        next.centerline_position_in_inertial_meters.z() =
            -grade_.IntegralFromStart(next.track_station_meters);
    }

}

void TrackGeometry::RequireFiniteTrackStation(double track_station_meters,
                                              const char* argument_name) const {
    if (!std::isfinite(track_station_meters)) {
        throw std::invalid_argument(std::string("TrackGeometry: ") +
                                    argument_name + " must be finite, got " +
                                    Describe(track_station_meters));
    }
}

TrackStationRegion TrackGeometry::ClassifyTrackStation(
    double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    if (track_station_meters < start_track_station_meters()) {
        return TrackStationRegion::kBeforeDefinedInterval;
    }
    if (track_station_meters > end_track_station_meters()) {
        return TrackStationRegion::kAfterDefinedInterval;
    }
    return TrackStationRegion::kWithinDefinedInterval;
}

std::size_t TrackGeometry::NodeIndexAtOrBefore(
    double track_station_meters) const {
    std::size_t low = 0;
    std::size_t high = nodes_.size() - 1;
    while (low < high) {
        const std::size_t middle = low + (high - low + 1) / 2;
        if (nodes_[middle].track_station_meters <= track_station_meters) {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    // The final node starts no interval, so a station that lands on it belongs
    // to the interval that ends there.
    if (low + 1 == nodes_.size() && low > 0) {
        --low;
    }
    return low;
}

double TrackGeometry::HeadingRadiansUnchecked(
    double track_station_meters) const {
    const double definition_station = std::clamp(
        track_station_meters, start_track_station_meters(),
        end_track_station_meters());
    return curvature_.IntegralFromStart(definition_station);
}

Eigen::Vector2d TrackGeometry::HorizontalDisplacementFromNode(
    std::size_t node_index, double to_track_station_meters) const {
    const StationNode& node = nodes_[node_index];
    const double from = node.track_station_meters;
    const double length = to_track_station_meters - from;
    if (length == 0.0) {
        return Eigen::Vector2d::Zero();
    }
    const double heading_at_node = node.heading_radians;
    if (node.interval_has_constant_curvature) {
        // The half-angle form of the circular chord. Written as a difference of
        // sines divided by the curvature it would subtract two nearly equal
        // numbers and then multiply the remainder by the radius, which on a
        // gentle curve with short panels loses most of the significant digits.
        // This form has no cancellation and degrades to the straight case on
        // its own, so there is no separate branch to keep consistent.
        const double half_turn =
            0.5 * node.interval_curvature_radians_per_meter * length;
        const double heading_at_middle = heading_at_node + half_turn;
        const double chord_ratio =
            half_turn == 0.0 ? 1.0 : std::sin(half_turn) / half_turn;
        return Eigen::Vector2d(
            length * chord_ratio * std::cos(heading_at_middle),
            length * chord_ratio * std::sin(heading_at_middle));
    }
    const double half = 0.5 * length;
    const double middle = 0.5 * (from + to_track_station_meters);
    Eigen::Vector2d displacement = Eigen::Vector2d::Zero();
    for (int point = 0; point < kQuadraturePointCount; ++point) {
        const double station = middle + half * kQuadratureAbscissae[point];
        const double heading = curvature_.IntegralFromStart(station);
        displacement.x() += kQuadratureWeights[point] * std::cos(heading);
        displacement.y() += kQuadratureWeights[point] * std::sin(heading);
    }
    return half * displacement;
}

Eigen::Vector3d TrackGeometry::CenterlinePositionUnchecked(
    double track_station_meters) const {
    if (track_station_meters < start_track_station_meters() ||
        track_station_meters > end_track_station_meters()) {
        const double boundary =
            track_station_meters < start_track_station_meters()
                ? start_track_station_meters()
                : end_track_station_meters();
        return CenterlinePositionUnchecked(boundary) +
               (track_station_meters - boundary) *
                   CenterlineDerivativeUnchecked(boundary);
    }
    const std::size_t index = NodeIndexAtOrBefore(track_station_meters);
    const Eigen::Vector2d step =
        HorizontalDisplacementFromNode(index, track_station_meters);
    const Eigen::Vector3d& base =
        nodes_[index].centerline_position_in_inertial_meters;
    return Eigen::Vector3d(
        base.x() + step.x(), base.y() + step.y(),
        -grade_.IntegralFromStart(track_station_meters));
}

Eigen::Vector3d TrackGeometry::CenterlineDerivativeUnchecked(
    double track_station_meters) const {
    const double heading = HeadingRadiansUnchecked(track_station_meters);
    const double definition_station = std::clamp(
        track_station_meters, start_track_station_meters(),
        end_track_station_meters());
    return Eigen::Vector3d(std::cos(heading), std::sin(heading),
                           -grade_.Value(definition_station));
}

Eigen::Vector3d TrackGeometry::CenterlineSecondDerivativeUnchecked(
    double track_station_meters) const {
    if (track_station_meters < start_track_station_meters() ||
        track_station_meters > end_track_station_meters()) {
        return Eigen::Vector3d::Zero();
    }
    const double heading = HeadingRadiansUnchecked(track_station_meters);
    const double curvature = curvature_.Value(track_station_meters);
    return Eigen::Vector3d(
        -curvature * std::sin(heading), curvature * std::cos(heading),
        -grade_.FirstDerivativePerMeter(track_station_meters));
}

double TrackGeometry::CurvatureRadiansPerMeter(
    double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    if (track_station_meters < start_track_station_meters() ||
        track_station_meters > end_track_station_meters()) {
        return 0.0;
    }
    return curvature_.Value(track_station_meters);
}

double TrackGeometry::SuperelevationMeters(double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    return superelevation_.Value(std::clamp(
        track_station_meters, start_track_station_meters(),
        end_track_station_meters()));
}

double TrackGeometry::CenterlineUpwardGrade(
    double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    return grade_.Value(std::clamp(track_station_meters,
                                   start_track_station_meters(),
                                   end_track_station_meters()));
}

double TrackGeometry::HeadingRadians(double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    return HeadingRadiansUnchecked(track_station_meters);
}

double TrackGeometry::TrackRollRadians(double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    const double definition_station = std::clamp(
        track_station_meters, start_track_station_meters(),
        end_track_station_meters());
    return std::asin(superelevation_.Value(definition_station) /
                     superelevation_reference_baselength_meters_);
}

Eigen::Vector3d TrackGeometry::CenterlinePositionInInertialMeters(
    double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    return CenterlinePositionUnchecked(track_station_meters);
}

Eigen::Vector3d TrackGeometry::CenterlineDerivativeInInertialMetersPerMeter(
    double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");
    return CenterlineDerivativeUnchecked(track_station_meters);
}

TrackFrameKinematics TrackGeometry::EvaluateTrackFrame(
    double track_station_meters) const {
    RequireFiniteTrackStation(track_station_meters, "the track station");

    const double heading = HeadingRadiansUnchecked(track_station_meters);
    const bool is_within_definition =
        track_station_meters >= start_track_station_meters() &&
        track_station_meters <= end_track_station_meters();
    const double definition_station = std::clamp(
        track_station_meters, start_track_station_meters(),
        end_track_station_meters());
    const double curvature =
        is_within_definition ? curvature_.Value(definition_station) : 0.0;
    const double grade = grade_.Value(definition_station);
    const double grade_rate =
        is_within_definition
            ? grade_.FirstDerivativePerMeter(definition_station)
            : 0.0;
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    const double slope_norm = std::sqrt(1.0 + grade * grade);
    const double slope_norm_rate = grade * grade_rate / slope_norm;

    // The roll-free tangent frame: x along the three-dimensional unit tangent,
    // y horizontal and pointing to the track's right, z completing the triad.
    const Eigen::Vector3d tangent(cosine, sine, -grade);
    const Eigen::Vector3d roll_free_x = tangent / slope_norm;
    const Eigen::Vector3d roll_free_y(-sine, cosine, 0.0);
    const Eigen::Vector3d roll_free_z = roll_free_x.cross(roll_free_y);

    const Eigen::Vector3d tangent_rate(-curvature * sine, curvature * cosine,
                                       -grade_rate);
    const Eigen::Vector3d roll_free_x_rate =
        tangent_rate / slope_norm - roll_free_x * (slope_norm_rate / slope_norm);
    const Eigen::Vector3d roll_free_y_rate(-curvature * cosine,
                                           -curvature * sine, 0.0);
    const Eigen::Vector3d roll_free_z_rate =
        roll_free_x_rate.cross(roll_free_y) + roll_free_x.cross(roll_free_y_rate);

    Eigen::Matrix3d rotation_roll_free;
    rotation_roll_free.col(0) = roll_free_x;
    rotation_roll_free.col(1) = roll_free_y;
    rotation_roll_free.col(2) = roll_free_z;

    // A positive superelevation puts the right rail lower, which is a positive
    // roll about the roll-free x axis: it turns +y toward +z, and +z is down.
    const double superelevation = superelevation_.Value(definition_station);
    const double superelevation_rate =
        is_within_definition
            ? superelevation_.FirstDerivativePerMeter(definition_station)
            : 0.0;
    const double sine_roll =
        superelevation / superelevation_reference_baselength_meters_;
    const double roll = std::asin(sine_roll);
    const double roll_rate =
        (superelevation_rate / superelevation_reference_baselength_meters_) /
        std::sqrt(1.0 - sine_roll * sine_roll);
    const double cosine_roll = std::cos(roll);
    const double sine_roll_value = std::sin(roll);

    Eigen::Matrix3d roll_about_x = Eigen::Matrix3d::Identity();
    roll_about_x(1, 1) = cosine_roll;
    roll_about_x(1, 2) = -sine_roll_value;
    roll_about_x(2, 1) = sine_roll_value;
    roll_about_x(2, 2) = cosine_roll;
    const Eigen::Matrix3d rotation = rotation_roll_free * roll_about_x;

    // The rotation rate of the roll-free frame, then the roll about its own x
    // axis. Half the sum of column cross products is the angular velocity of an
    // orthonormal triad, which avoids writing three special cases by hand.
    const Eigen::Vector3d roll_free_rate =
        0.5 * (roll_free_x.cross(roll_free_x_rate) +
               roll_free_y.cross(roll_free_y_rate) +
               roll_free_z.cross(roll_free_z_rate));
    const Eigen::Vector3d rotation_rate =
        roll_free_rate + roll_rate * roll_free_x;

    const TrackFramePose pose(rotation,
                              CenterlinePositionUnchecked(track_station_meters));
    return TrackFrameKinematics(pose, tangent, rotation_rate);
}

TrackGeometry::ObjectiveDerivatives TrackGeometry::EvaluateObjectiveDerivatives(
    const Eigen::Vector3d& point_in_inertial_meters,
    double track_station_meters) const {
    const Eigen::Vector3d centerline =
        CenterlinePositionUnchecked(track_station_meters);
    const Eigen::Vector3d first =
        CenterlineDerivativeUnchecked(track_station_meters);
    const Eigen::Vector3d second =
        CenterlineSecondDerivativeUnchecked(track_station_meters);
    const Eigen::Vector3d offset = point_in_inertial_meters - centerline;
    ObjectiveDerivatives derivatives;
    derivatives.gradient = -offset.dot(first);
    derivatives.hessian = first.squaredNorm() - offset.dot(second);
    derivatives.gradient_bound = kObjectiveGradientRelativeTolerance *
                                 (offset.norm() * first.norm() + 1.0);
    return derivatives;
}

TrackStationProjection TrackGeometry::ProjectPointOntoSeededBranch(
    const Eigen::Vector3d& point_in_inertial_meters,
    double branch_seed_track_station_meters) const {
    if (!point_in_inertial_meters.allFinite()) {
        throw std::invalid_argument(
            "TrackGeometry::ProjectPointOntoSeededBranch: the point must be "
            "finite");
    }
    RequireFiniteTrackStation(branch_seed_track_station_meters,
                              "the branch seed track station");

    double station = branch_seed_track_station_meters;
    for (int correction_count = 0; correction_count <= 2;
         ++correction_count) {
        const ObjectiveDerivatives derivatives =
            EvaluateObjectiveDerivatives(point_in_inertial_meters, station);
        const bool finite = std::isfinite(derivatives.gradient) &&
                            std::isfinite(derivatives.hessian) &&
                            std::isfinite(derivatives.gradient_bound);
        if (finite && derivatives.hessian > 0.0 &&
            std::abs(derivatives.gradient) <= derivatives.gradient_bound) {
            return TrackStationProjection(
                station, CenterlinePositionUnchecked(station));
        }
        if (!finite || !(derivatives.hessian > 0.0) ||
            correction_count == 2) {
            throw TrackStationLocalProjectionFailure(
                "TrackGeometry::ProjectPointOntoSeededBranch: the supplied "
                "station does not identify a regular local projection within "
                "two Newton corrections");
        }
        const double next_station =
            station - derivatives.gradient / derivatives.hessian;
        if (!std::isfinite(next_station) || next_station == station) {
            throw TrackStationLocalProjectionFailure(
                "TrackGeometry::ProjectPointOntoSeededBranch: Newton "
                "correction made no finite representable progress");
        }
        station = next_station;
    }
    throw std::logic_error(
        "TrackGeometry::ProjectPointOntoSeededBranch: unreachable solver "
        "state");
}

}  // namespace orvd::track_geometry
