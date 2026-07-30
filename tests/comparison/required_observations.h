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

// Builds the requirements for one scenario: the mass-matrix columns, the
// inverse-dynamics generalized forces, each link pose, and generalized positions
// read back after evaluation.
[[nodiscard]] ComparisonRequirements MakeComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario);

}  // namespace orvd_comparison
