#pragma once

#include <cstddef>
#include <optional>
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
// Construction may allocate. A successful evaluation may not: it neither
// builds nor grows a dynamic container, looks up no name, formats no string,
// writes no log and touches no file. A rejected evaluation may format its
// diagnostic before throwing. The node table and the quadrature coefficients
// belong to the immutable geometry object.

namespace orvd::track_geometry {

// Where a station lies relative to the finite definition interval declared by
// the base-track asset. The interval remains observable even though
// TrackGeometry continues the centerline as a straight tangent beyond either
// definition boundary.
enum class TrackStationRegion {
    kBeforeDefinedInterval,
    kWithinDefinedInterval,
    kAfterDefinedInterval,
};

class TrackGeometry {
   public:
    // The three profiles must share one station domain. The superelevation
    // reference baselength is the transverse baseline b used to convert the
    // declared level difference u into track roll asin(u / b). It is not the
    // nominal track gauge or a wheel/rail profile-placement distance. Every
    // admissible superelevation satisfies |u| < b; the strict inequality keeps
    // the first-order roll kinematics finite.
    //
    // Throws std::invalid_argument when the profiles disagree on their domain,
    // when the baselength or node spacing is not finite and positive, when the
    // requested node table exceeds the construction resource bound, or when
    // any superelevation in the profile reaches the baselength in magnitude.
    // The superelevation bound is refused at construction rather than clamped:
    // clamping would turn a modelling mistake into a plausible line.
    TrackGeometry(TrackScalarProfile curvature_radians_per_meter,
                  TrackScalarProfile superelevation_meters,
                  TrackScalarProfile centerline_upward_grade,
                  double superelevation_reference_baselength_meters,
                  double station_node_spacing_meters);

    [[nodiscard]] double start_track_station_meters() const {
        return curvature_.start_track_station_meters();
    }
    [[nodiscard]] double end_track_station_meters() const {
        return curvature_.end_track_station_meters();
    }
    [[nodiscard]] double superelevation_reference_baselength_meters() const {
        return superelevation_reference_baselength_meters_;
    }

    // Every finite station is meaningful. Inside the base track's definition
    // interval these accessors evaluate the three profiles. Outside it, the
    // centerline follows the nearest definition boundary's three-dimensional
    // tangent, curvature and all station rates are zero, and the boundary
    // heading, grade and superelevation are held. Non-finite stations are
    // refused.
    [[nodiscard]] TrackStationRegion ClassifyTrackStation(
        double track_station_meters) const;
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

    // Tracks the branch of the centerline already isolated by the caller's seed
    // and local window; this is not a whole-line search. The window must contain
    // exactly one admissible minimum at the declared node resolution. A station
    // is admissible only when the objective's first derivative meets the
    // residual bound and its second derivative is strictly positive:
    //
    //     objective'' = |centerline'|^2 - (point - centerline) . centerline'',
    //
    // Multiple stationary points hidden inside one node interval remain outside
    // the completeness contract.
    //
    // The finite search window may lie inside the definition interval, outside
    // it on either straight continuation, or cross either definition boundary.
    // It is never clipped to the finite asset interval. Throws
    // std::invalid_argument for a
    // non-finite point or seed, a non-positive half width, or non-finite window
    // bounds. Throws TrackStationProjectionWindowMiss if no admissible minimum
    // lies strictly inside the window, and std::runtime_error if more than one
    // does.
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
        bool found{false};
    };

    void RequireFiniteTrackStation(double track_station_meters,
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
    struct ObjectiveDerivatives {
        double gradient{0.0};
        double hessian{0.0};
    };
    [[nodiscard]] ObjectiveDerivatives EvaluateObjectiveDerivatives(
        const Eigen::Vector3d& point_in_inertial_meters,
        double track_station_meters) const;
    // Accepts a qualified endpoint root or refines an interior root bracketed
    // by a non-positive-to-non-negative gradient change. Newton steps remain
    // inside a bracket that bisection always shrinks.
    [[nodiscard]] ProjectionCandidate RefineBracketedMinimum(
        const Eigen::Vector3d& point_in_inertial_meters,
        double lower_bound_station, double upper_bound_station) const;

    TrackScalarProfile curvature_;
    TrackScalarProfile superelevation_;
    TrackScalarProfile grade_;
    double superelevation_reference_baselength_meters_{0.0};
    double station_node_spacing_meters_{0.0};
    std::vector<StationNode> nodes_;
};

}  // namespace orvd::track_geometry
