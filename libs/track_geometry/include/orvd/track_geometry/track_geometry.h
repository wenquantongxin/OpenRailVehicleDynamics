#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "orvd/track_geometry/track_frame_pose.h"
#include "orvd/track_geometry/track_geometry_segments.h"
#include "orvd/track_geometry/track_station_projection.h"

// A line: three scalar profiles plus the frames they induce.
//
// The independent variable is the track station, which is planar projected
// mileage and not three-dimensional arc length. Two consequences run through
// everything below:
//
//   * the horizontal projection of the centerline derivative is a unit vector,
//     while the full three-dimensional derivative has norm sqrt(1 + grade^2);
//   * the grade is dimensionless and positive uphill along increasing station,
//     so, because the inertial frame has +z downward,
//
//         d(centerline z)/d(track station) = -centerline_upward_grade.
//
// The heading turns from +x toward +y at the rate given by the curvature, so a
// positive curvature is a right-hand curve. The centerline starts at the origin
// of the inertial frame with zero heading; the inertial frame is defined by the
// line, so there is nothing left for a caller to place.
//
// Straight and circular stretches integrate in closed form. A Hermite blend or
// a seam window makes the heading a polynomial whose sine and cosine have no
// elementary antiderivative, so a fixed-order Gauss-Legendre rule over panels
// no longer than the node spacing precomputes the panel endpoints. A non-node
// evaluation integrates at most the remaining part of one panel. Accuracy is
// set by a stated spacing rather than by whatever the caller happens to ask for;
// arbitrary positive spacings are not claimed to share one error bound.
//
// Construction may allocate. An evaluation may not: it neither builds nor grows
// a dynamic container, looks up no name, formats no string, writes no log and
// touches no file. The node table and the quadrature coefficients belong to the
// immutable geometry object.

namespace orvd::track_geometry {

class TrackGeometry {
   public:
    // The three profiles must share one station domain. The rail reference
    // lateral span is the rail-to-rail distance across which superelevation is
    // measured along the roll-free frame's vertical axis. The track roll angle
    // is asin(superelevation / span), and every admissible superelevation
    // satisfies |superelevation| < span. The strict inequality keeps the
    // first-order roll kinematics finite.
    //
    // Throws std::invalid_argument when the profiles disagree on their domain,
    // when the span or node spacing is not finite and positive, or when any
    // superelevation in the profile reaches the span in magnitude. The
    // superelevation bound is refused at construction rather than clamped:
    // clamping would turn a modelling mistake into a plausible line.
    TrackGeometry(TrackScalarProfile curvature_radians_per_meter,
                  TrackScalarProfile superelevation_meters,
                  TrackScalarProfile centerline_upward_grade,
                  double rail_reference_lateral_span_meters,
                  double station_node_spacing_meters);

    [[nodiscard]] double start_track_station_meters() const {
        return curvature_.start_track_station_meters();
    }
    [[nodiscard]] double end_track_station_meters() const {
        return curvature_.end_track_station_meters();
    }
    [[nodiscard]] double rail_reference_lateral_span_meters() const {
        return rail_reference_lateral_span_meters_;
    }

    [[nodiscard]] const TrackScalarProfile& curvature_profile() const & {
        return curvature_;
    }
    [[nodiscard]] TrackScalarProfile curvature_profile() && {
        return std::move(curvature_);
    }
    [[nodiscard]] TrackScalarProfile curvature_profile() const && {
        return curvature_;
    }

    [[nodiscard]] const TrackScalarProfile& superelevation_profile() const & {
        return superelevation_;
    }
    [[nodiscard]] TrackScalarProfile superelevation_profile() && {
        return std::move(superelevation_);
    }
    [[nodiscard]] TrackScalarProfile superelevation_profile() const && {
        return superelevation_;
    }

    [[nodiscard]] const TrackScalarProfile& grade_profile() const & {
        return grade_;
    }
    [[nodiscard]] TrackScalarProfile grade_profile() && {
        return std::move(grade_);
    }
    [[nodiscard]] TrackScalarProfile grade_profile() const && {
        return grade_;
    }

    // Every station argument below is refused when it is not finite or lies
    // outside the domain.
    [[nodiscard]] double CurvatureRadiansPerMeter(
        double track_station_meters) const;
    [[nodiscard]] double SuperelevationMeters(double track_station_meters) const;
    [[nodiscard]] double CenterlineUpwardGrade(
        double track_station_meters) const;
    [[nodiscard]] double HeadingRadians(double track_station_meters) const;
    [[nodiscard]] double TrackRollRadians(double track_station_meters) const;

    [[nodiscard]] Eigen::Vector3d CenterlinePositionInInertialMeters(
        double track_station_meters) const;
    [[nodiscard]] Eigen::Vector3d
    CenterlineDerivativeInInertialMetersPerMeter(
        double track_station_meters) const;

    // The pose and its first station derivative in one evaluation. Splitting
    // them would make a caller pay for the centerline twice and would let the
    // two drift apart.
    [[nodiscard]] TrackFrameKinematics EvaluateTrackFrame(
        double track_station_meters) const;

    // Where each family starts to act, for a start-up domain contract to
    // compare against an axle station. No value means the family is identically
    // zero over the whole line.
    [[nodiscard]] std::optional<double> first_curved_track_station_meters()
        const {
        return curvature_.support_start_track_station_meters();
    }
    [[nodiscard]] std::optional<double>
    first_superelevated_track_station_meters() const {
        return superelevation_.support_start_track_station_meters();
    }
    [[nodiscard]] std::optional<double> first_graded_track_station_meters()
        const {
        return grade_.support_start_track_station_meters();
    }

    // Projection onto the centerline. Both queries minimise
    //
    //     objective = 0.5 * |point - centerline(station)|^2
    //
    // and admit a station only when the first derivative of that objective
    // meets the residual bound, its second derivative is strictly positive, and
    // the station lies inside the searched domain. The second-derivative test
    // is the general one,
    //
    //     objective'' = |centerline'|^2 - (point - centerline) . centerline'',
    //
    // not the planar special case that a circle would suggest.
    //
    // The cold start searches the whole domain and throws std::runtime_error
    // when two distinct minima are equally close, because then the nearest
    // station is not a function of the point. The seeded query searches only
    // [seed - half width, seed + half width] and throws std::runtime_error when
    // the refined station leaves that interval. Neither clamps.
    //
    // Both throw std::invalid_argument for a non-finite point, a non-finite or
    // out-of-domain seed, or a non-positive half width.
    [[nodiscard]] TrackStationProjection ProjectPointColdStart(
        const Eigen::Vector3d& point_in_inertial_meters) const;
    [[nodiscard]] TrackStationProjection ProjectPointNearSeed(
        const Eigen::Vector3d& point_in_inertial_meters,
        double seed_track_station_meters, double search_half_width_meters) const;

   private:
    struct StationNode {
        double track_station_meters{0.0};
        double heading_radians{0.0};
        Eigen::Vector3d centerline_position_in_inertial_meters{
            Eigen::Vector3d::Zero()};
        // Properties of the interval that starts at this node. Nodes are laid
        // on the curvature profile's own breakpoints, so an interval never
        // straddles a change of shape and either integrates in closed form or
        // needs the quadrature rule, never both.
        bool interval_has_constant_curvature{false};
        double interval_curvature_radians_per_meter{0.0};
    };

    struct ProjectionCandidate {
        double track_station_meters{0.0};
        double squared_distance_meters_squared{0.0};
        bool found{false};
    };

    void ThrowIfOutsideDomain(double track_station_meters,
                              const char* argument_name) const;
    [[nodiscard]] std::size_t NodeIndexAtOrBefore(
        double track_station_meters) const;
    // The exact heading at a station: the curvature profile integrates in
    // closed form because every admitted shape is a polynomial.
    [[nodiscard]] double HeadingRadiansUnchecked(
        double track_station_meters) const;
    [[nodiscard]] Eigen::Vector3d CenterlinePositionUnchecked(
        double track_station_meters) const;
    [[nodiscard]] Eigen::Vector3d CenterlineDerivativeUnchecked(
        double track_station_meters) const;
    // Needed by the projection's second-order test; kept private until an
    // outside consumer actually asks for it.
    [[nodiscard]] Eigen::Vector3d CenterlineSecondDerivativeUnchecked(
        double track_station_meters) const;
    // Integrates the horizontal centerline displacement from the given node to
    // a station inside that node's interval, in closed form where the curvature
    // is constant and by the fixed Gauss-Legendre rule otherwise.
    [[nodiscard]] Eigen::Vector2d HorizontalDisplacementFromNode(
        std::size_t node_index, double to_track_station_meters) const;
    [[nodiscard]] ProjectionCandidate RefineFromBracket(
        const Eigen::Vector3d& point_in_inertial_meters, double initial_station,
        double lower_bound_station, double upper_bound_station) const;

    TrackScalarProfile curvature_;
    TrackScalarProfile superelevation_;
    TrackScalarProfile grade_;
    double rail_reference_lateral_span_meters_{0.0};
    double station_node_spacing_meters_{0.0};
    std::vector<StationNode> nodes_;
};

}  // namespace orvd::track_geometry
