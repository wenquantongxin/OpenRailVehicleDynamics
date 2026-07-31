// What a comparison requires and the physical kind used to judge it.
//
// This lives with the judge, not with the emitters. If each side declared its
// own required set and its own tolerance, two sides sharing one wrong belief
// would ratify each other, and the comparison would report agreement precisely
// when it is least deserved.
#pragma once

#include <string>
#include <vector>

#include "contract/observation_semantics.h"
#include "contract/scenario_definition.h"

namespace orvd_comparison {

// One quantity the comparison insists on. Only the judge declares its physical
// kind; emitters provide names and raw values without declaring their own rule.
struct RequiredObservation {
    std::string name;
    orvd_contract::ObservationKind kind;
};

// A rotation is required as a whole: all nine elements of `<group>[row,col]`.
struct RequiredRotation {
    std::string group_name;
};

struct ComparisonRequirements {
    std::vector<std::string> topology_fact_names;
    std::vector<RequiredObservation> scalars;
    std::vector<RequiredRotation> rotations;
};

// What a candidate that can do position kinematics must agree on: the coordinate
// counts and every element's coordinate range, each link's pose, and the
// generalized positions read back after evaluation.
//
// A named capability rather than a switch. Requirements are widened by a goal
// that landed the capability, and each wider set is built by calling the
// narrower one first — so a later set cannot quietly drop something an earlier
// one insisted on, and there is no runtime choice for a failing comparison to
// be made to pass by.
[[nodiscard]] ComparisonRequirements
MakePositionKinematicsComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario);

// The above, plus every link body frame's world spatial velocity (with its
// translational component at the body origin), the named relative spatial
// velocities, and the generalized velocities read back after evaluation.
// Structurally a superset of position kinematics.
[[nodiscard]] ComparisonRequirements
MakeSpatialKinematicsComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario);

// The spatial-kinematics set, plus qdot/v mappings, explicit-vdot body-frame
// accelerations and a non-state probe through every body-point kV Jacobian.
// This is G33's cross-process qualification surface.
[[nodiscard]] ComparisonRequirements
MakeDifferentialKinematicsComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario);

// The above, plus every mass-matrix column represented as a generalized-force
// response to that column's nonzero acceleration probe. Structurally a strict
// superset: it starts from the differential-kinematics set.
[[nodiscard]] ComparisonRequirements MakeMassMatrixComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario);

}  // namespace orvd_comparison
