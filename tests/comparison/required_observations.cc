#include "comparison/required_observations.h"

#include <string>

namespace orvd_comparison {
namespace {

orvd_contract::ObservationKind ObservationKindForGeneralizedForceComponent(
    orvd_contract::GeneralizedForceComponentKind component_kind) {
    using orvd_contract::GeneralizedForceComponentKind;
    using orvd_contract::ObservationKind;
    return component_kind == GeneralizedForceComponentKind::kTorqueNewtonMetres
               ? ObservationKind::kTorqueNewtonMetres
               : ObservationKind::kForceNewtons;
}

}  // namespace

ComparisonRequirements MakeComparisonRequirements(
    const orvd_contract::ScenarioDefinition& scenario) {
    using orvd_contract::ObservationKind;

    ComparisonRequirements requirements;
    requirements.topology_fact_names = {"num_positions", "num_velocities", "num_bodies"};

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

    for (const auto& link : scenario.links) {
        requirements.rotations.push_back({"pose_" + link.name});
        for (int axis_index = 0; axis_index < 3; ++axis_index)
            requirements.scalars.push_back(
                {"pose_" + link.name + "_translation[" + std::to_string(axis_index) + "]",
                 ObservationKind::kTranslationMeters});
    }

    // The state read back after evaluation. Comparing poses alone cannot show
    // that a context was left untouched; an implementation that normalizes or
    // rewrites state in place becomes visible here.
    for (std::size_t position_index = 0;
         position_index < scenario.generalized_positions.size(); ++position_index) {
        requirements.scalars.push_back(
            {"state_readback_position[" + std::to_string(position_index) + "]",
             scenario.generalized_position_observation_kinds[position_index]});
    }
    return requirements;
}

}  // namespace orvd_comparison
