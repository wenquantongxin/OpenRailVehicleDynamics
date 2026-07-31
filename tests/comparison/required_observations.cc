#include "comparison/required_observations.h"

#include <stdexcept>
#include <string>

namespace orvd_comparison {
namespace {

orvd_contract::ObservationKind ObservationKindForGeneralizedForceComponent(
    orvd_contract::GeneralizedForceComponentKind component_kind) {
    using orvd_contract::GeneralizedForceComponentKind;
    using orvd_contract::ObservationKind;
    switch (component_kind) {
        case GeneralizedForceComponentKind::kForceNewtons:
            return ObservationKind::kForceNewtons;
        case GeneralizedForceComponentKind::kTorqueNewtonMetres:
            return ObservationKind::kTorqueNewtonMetres;
    }
    throw std::logic_error(
        "Generalized-force component has no observation dimension");
}

}  // namespace

ComparisonRequirements MakePositionKinematicsComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario) {
    using orvd_contract::ObservationKind;

    ComparisonRequirements requirements;
    requirements.topology_fact_names = {"num_positions", "num_velocities",
                                        "num_rigid_bodies_excluding_world"};

    // Where each element's coordinates sit. Both sides answer from the model
    // they actually built, so if the two lay their coordinates out differently
    // the comparison says so — instead of the scenario's index-keyed state and
    // read-back observations quietly meaning two different things.
    for (const auto& joint : scenario.revolute_joints) {
        for (const char* fact :
             {"joint_position_range_start", "joint_position_range_size",
              "joint_velocity_range_start", "joint_velocity_range_size"}) {
            requirements.topology_fact_names.push_back(std::string(fact) + "[" +
                                                       joint.name + "]");
        }
    }
    for (const auto& free_body_name : scenario.free_body_names) {
        for (const char* fact : {"free_body_position_range_start",
                                 "free_body_position_range_size",
                                 "free_body_velocity_range_start",
                                 "free_body_velocity_range_size"}) {
            requirements.topology_fact_names.push_back(std::string(fact) + "[" +
                                                       free_body_name + "]");
        }
    }

    for (const auto& link : scenario.links) {
        requirements.rotations.push_back({"pose_" + link.name});
        for (int axis_index = 0; axis_index < 3; ++axis_index)
            requirements.scalars.push_back(
                {"pose_" + link.name + "_translation[" + std::to_string(axis_index) + "]",
                 ObservationKind::kTranslationMeters});
    }

    // Generalized positions read back after evaluation. Comparing poses alone
    // cannot show that an implementation normalized or rewrote those positions
    // in place.
    for (std::size_t position_index = 0;
         position_index < scenario.generalized_positions.size(); ++position_index) {
        requirements.scalars.push_back(
            {"state_readback_position[" + std::to_string(position_index) + "]",
             scenario.generalized_position_observation_kinds[position_index]});
    }
    return requirements;
}

ComparisonRequirements MakeComparisonRequirements(

    const orvd_contract::ScenarioDefinition& scenario) {
    using orvd_contract::ObservationKind;

    // Built on top of the narrower set, not beside it: two independently
    // maintained lists drift, and the drift shows up as a capability that used
    // to be compared and silently stopped being.
    ComparisonRequirements requirements =
        MakePositionKinematicsComparisonRequirements(scenario);

    const std::size_t velocity_count = scenario.generalized_velocities.size();
    // The mass matrix is required column by column. A single matrix-vector
    // product collapses the operator onto one vector and leaves the rest of it
    // unobserved, where a symmetric error can sit undetected.
    for (std::size_t column = 0; column < velocity_count; ++column) {
        for (std::size_t generalized_force_index = 0;
             generalized_force_index < velocity_count;
             ++generalized_force_index) {
            const ObservationKind kind = ObservationKindForGeneralizedForceComponent(
                scenario.generalized_force_component_kinds[generalized_force_index]);
            requirements.scalars.push_back(
                {"mass_matrix_column_generalized_force_response[" +
                     std::to_string(column) + "][" +
                     std::to_string(generalized_force_index) + "]",
                 kind});
        }
    }

    for (std::size_t velocity_index = 0; velocity_index < velocity_count;
         ++velocity_index) {
        const ObservationKind kind = ObservationKindForGeneralizedForceComponent(
            scenario.generalized_force_component_kinds[velocity_index]);
        requirements.scalars.push_back(
            {"inverse_dynamics[" + std::to_string(velocity_index) + "]", kind});
    }

    return requirements;
}

}  // namespace orvd_comparison
