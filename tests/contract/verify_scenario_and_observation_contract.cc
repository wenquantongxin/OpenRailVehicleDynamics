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
        Expect(scenario.mass_matrix_excitation_scales.size() == velocity_count,
               "mass-matrix excitation must have one entry per velocity");
        Expect(scenario.comparison_scales
                       .generalized_force_component_newtons.size() == velocity_count,
               "force scales must have one entry per velocity");
        Expect(scenario.comparison_scales
                       .generalized_torque_component_newton_metres.size() == velocity_count,
               "torque scales must have one entry per velocity");

        // Every column of the mass matrix must be excited, or that column is
        // simply never observed and an error there cannot be seen.
        for (const double excitation_scale : scenario.mass_matrix_excitation_scales)
            Expect(std::isfinite(excitation_scale) && excitation_scale > 0.0,
                   "every mass-matrix excitation scale must be finite and positive");

        // Force and torque are separate dimensions: a velocity index carries one
        // or the other, never both and never neither. Sharing a single scale
        // would let an error in the smaller component hide behind the larger.
        for (std::size_t velocity_index = 0; velocity_index < velocity_count;
             ++velocity_index) {
            const double force_scale =
                scenario.comparison_scales
                    .generalized_force_component_newtons[velocity_index];
            const double torque_scale =
                scenario.comparison_scales
                    .generalized_torque_component_newton_metres[velocity_index];
            Expect((force_scale > 0.0) != (torque_scale > 0.0),
                   "each velocity index must carry exactly one of force or torque");
        }

        Expect(scenario.comparison_scales.translation_meters > 0.0,
               "translation scale must be positive");
        Expect(scenario.comparison_scales.angle_radians > 0.0,
               "angle scale must be positive");
    }
}

void CheckScenarioCarriesNoVehicleConcept() {
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario("dynamic_excitation");
    std::string every_name = scenario.name;
    for (const auto& link : scenario.links) every_name += " " + link.name;
    for (const auto& joint : scenario.revolute_joints) every_name += " " + joint.name;
    every_name += " " + scenario.free_floating_link_name;

    for (const std::string_view vehicle_word :
         {"wheel", "rail", "axle", "bogie", "gz18", "sh17", "carbody", "vehicle"})
        Expect(every_name.find(vehicle_word) == std::string::npos,
               "scenario names must stay model-neutral");
}

void CheckQuaternionStateIsUnitLength() {
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario("dynamic_excitation");
    // Positions [2..5] are the floating body's quaternion in this topology.
    double squared_norm = 0.0;
    for (std::size_t index = 2; index < 6; ++index)
        squared_norm += scenario.generalized_positions[index] *
                        scenario.generalized_positions[index];
    Expect(std::abs(squared_norm - 1.0) < 1e-12,
           "the prescribed floating quaternion must be unit length");
}

void CheckLooseParsingAcceptsFormAndRejectsSubstance() {
    orvd_contract::ObservationStream written;
    written.topology_facts = {{"num_positions", 9}, {"num_velocities", 8}};
    written.observations = {
        {"shoulder_angle", orvd_contract::ObservationKind::kAngleRadians, 0.37},
        {"floating_translation[0]",
         orvd_contract::ObservationKind::kTranslationMeters, 0.11},
    };
    const std::string text = orvd_contract::FormatObservationStream(written);

    orvd_contract::ObservationStream round_tripped;
    std::string parse_error;
    Expect(orvd_contract::ParseObservationStream(text, &round_tripped, &parse_error),
           "a stream this writer produced must parse");
    Expect(round_tripped.observations.size() == 2, "round trip must keep both observations");
    const auto* angle = round_tripped.FindObservation("shoulder_angle");
    Expect(angle != nullptr && angle->value == 0.37,
           "round trip must preserve the exact value");

    // Loose on form: unknown directives, trailing fields and a repeated
    // singleton are all things a trusted producer may legitimately emit.
    const std::string tolerated =
        "@unknown_directive whatever 1\n" + text +
        "@topology num_velocities 8 trailing fields ignored\n"
        "@observation shoulder_angle angle_radians " +
        text.substr(text.find("@observation shoulder_angle")).substr(
            std::string("@observation shoulder_angle angle_radians ").size(), 16) +
        "\n";
    orvd_contract::ObservationStream loose;
    Expect(orvd_contract::ParseObservationStream(tolerated, &loose, &parse_error),
           "extra directives, trailing fields and repeats must be tolerated");
    Expect(loose.observations.size() == 2,
           "a repeated observation must replace, not duplicate");

    // Strict on substance: something that will actually be used and cannot be
    // understood must fail loudly rather than be guessed at.
    for (const auto& [broken, why] : std::vector<std::pair<std::string, std::string>>{
             {"@observation x not_a_kind 3ff0000000000000\n", "unknown kind must fail"},
             {"@observation x angle_radians zzzz\n", "malformed payload must fail"},
             {"@observation x angle_radians 7ff8000000000000\n",
              "a non-finite value must fail"}}) {
        orvd_contract::ObservationStream rejected;
        std::string error_text;
        Expect(!orvd_contract::ParseObservationStream(broken, &rejected, &error_text),
               why);
        Expect(!error_text.empty(), "a rejection must say what was wrong");
    }
}

}  // namespace

int main() {
    CheckScenarioStatesAreDimensionallyConsistent();
    CheckScenarioCarriesNoVehicleConcept();
    CheckQuaternionStateIsUnitLength();
    CheckLooseParsingAcceptsFormAndRejectsSubstance();

    if (failure_count > 0) {
        std::fprintf(stderr, "%d contract check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("scenario and observation contract verified\n");
    return 0;
}
