// Checks the scenario and observation contract on its own terms. Drake is not
// involved: what is under test here is whether the contract states what it
// claims to state, which has to hold before any cross-process comparison built
// on it means anything.
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "contract/observation_stream.h"
#include "contract/scenario_definition.h"

namespace {

int failure_count = 0;

void Expect(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failure_count;
    }
}

void CheckScenarioStatesAreDimensionallyConsistent() {
    for (const std::string_view excitation :
         {"near_zero_cancellation", "dynamic_excitation"}) {
        const orvd_contract::ScenarioDefinition scenario =
            orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(excitation);
        const std::size_t velocity_count = scenario.generalized_velocities.size();

        Expect(scenario.generalized_accelerations.size() == velocity_count,
               "accelerations must be indexed like velocities");
        Expect(scenario.mass_matrix_column_generalized_accelerations.size() ==
                   velocity_count,
               "mass-matrix excitation must have one entry per velocity");
        Expect(scenario.generalized_force_component_kinds.size() == velocity_count,
               "generalized-force kinds must have one entry per velocity");
        Expect(scenario.generalized_position_observation_kinds.size() ==
                   scenario.generalized_positions.size(),
               "position kinds must have one entry per position");
        const std::vector<orvd_contract::ObservationKind> expected_position_kinds = {
            orvd_contract::ObservationKind::kAngleRadians,
            orvd_contract::ObservationKind::kAngleRadians,
            orvd_contract::ObservationKind::kUnitQuaternionComponent,
            orvd_contract::ObservationKind::kUnitQuaternionComponent,
            orvd_contract::ObservationKind::kUnitQuaternionComponent,
            orvd_contract::ObservationKind::kUnitQuaternionComponent,
            orvd_contract::ObservationKind::kTranslationMeters,
            orvd_contract::ObservationKind::kTranslationMeters,
            orvd_contract::ObservationKind::kTranslationMeters,
        };
        Expect(scenario.generalized_position_observation_kinds ==
                   expected_position_kinds,
               "position kinds must match the scenario topology");

        using orvd_contract::GeneralizedForceComponentKind;
        const std::vector<GeneralizedForceComponentKind> expected_force_kinds = {
            GeneralizedForceComponentKind::kTorqueNewtonMetres,
            GeneralizedForceComponentKind::kTorqueNewtonMetres,
            GeneralizedForceComponentKind::kTorqueNewtonMetres,
            GeneralizedForceComponentKind::kTorqueNewtonMetres,
            GeneralizedForceComponentKind::kTorqueNewtonMetres,
            GeneralizedForceComponentKind::kForceNewtons,
            GeneralizedForceComponentKind::kForceNewtons,
            GeneralizedForceComponentKind::kForceNewtons,
        };
        Expect(scenario.generalized_force_component_kinds == expected_force_kinds,
               "generalized-force kinds must match the scenario topology");
        if (scenario.mass_matrix_column_generalized_accelerations.size() !=
                velocity_count ||
            scenario.generalized_force_component_kinds.size() != velocity_count) {
            continue;
        }

        // Every column of the mass matrix must be excited, or that column is
        // simply never observed and an error there cannot be seen.
        for (const double generalized_acceleration :
             scenario.mass_matrix_column_generalized_accelerations)
            Expect(std::isfinite(generalized_acceleration) &&
                       generalized_acceleration > 0.0,
                   "every mass-matrix column acceleration must be finite and positive");

        for (const GeneralizedForceComponentKind component_kind :
             scenario.generalized_force_component_kinds)
            Expect(component_kind == GeneralizedForceComponentKind::kForceNewtons ||
                       component_kind ==
                           GeneralizedForceComponentKind::kTorqueNewtonMetres,
                   "each generalized-force component must be force or torque");
    }
}

void CheckQuaternionStateIsUnitLength() {
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario("dynamic_excitation");
    Expect(scenario.generalized_positions.size() >= 6,
           "the scenario must contain the floating quaternion");
    if (scenario.generalized_positions.size() < 6) return;
    // Positions [2..5] are the floating body's quaternion in this topology.
    double squared_norm = 0.0;
    for (std::size_t index = 2; index < 6; ++index)
        squared_norm += scenario.generalized_positions[index] *
                        scenario.generalized_positions[index];
    Expect(std::abs(squared_norm - 1.0) < 1e-12,
           "the prescribed floating quaternion must be unit length");
}

void CheckLooseObservationStreamParsing() {
    orvd_contract::ObservationStream written;
    written.topology_facts = {{"num_positions", 9}, {"num_velocities", 8}};
    written.observations = {
        {"shoulder_angle", 0.37},
        {"floating_translation[0]", 0.11},
    };
    const std::string text = orvd_contract::FormatObservationStream(written);

    const orvd_contract::ObservationStream round_tripped =
        orvd_contract::ParseObservationStream(text);
    Expect(round_tripped.observations.size() == 2, "round trip must keep both observations");
    const auto* angle = round_tripped.FindObservation("shoulder_angle");
    Expect(angle != nullptr && angle->value == 0.37,
           "round trip must preserve the exact value");

    // Loose on form: unknown directives, trailing fields and a repeated
    // singleton are all things a trusted producer may legitimately emit.
    orvd_contract::ObservationStream replacement;
    replacement.observations = {{"shoulder_angle", 0.55}};
    const std::string tolerated =
        "@unknown_directive whatever 1\n" + text +
        "@topology num_velocities 8 trailing fields ignored\n"
        "@observation unused_extension opaque_payload\n" +
        orvd_contract::FormatObservationStream(replacement);
    const orvd_contract::ObservationStream loose =
        orvd_contract::ParseObservationStream(tolerated);
    Expect(loose.observations.size() == 2,
           "a repeated observation must replace, not duplicate");
    const auto* replaced_angle = loose.FindObservation("shoulder_angle");
    Expect(replaced_angle != nullptr && replaced_angle->value == 0.55,
           "a repeated observation must keep its last value");
    Expect(loose.FindUnusableObservation("unused_extension") != nullptr,
           "an unused malformed observation must be retained without rejecting the stream");
}

}  // namespace

int main() {
    CheckScenarioStatesAreDimensionallyConsistent();
    CheckQuaternionStateIsUnitLength();
    CheckLooseObservationStreamParsing();

    if (failure_count > 0) {
        std::fprintf(stderr, "%d contract check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("scenario and observation contract verified\n");
    return 0;
}
