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
        Expect(scenario.generalized_velocity_observation_kinds.size() ==
                   velocity_count,
               "velocity kinds must have one entry per velocity");
        Expect(
            scenario.generalized_position_derivative_observation_kinds.size() ==
                scenario.generalized_positions.size(),
            "position-derivative kinds must have one entry per position");
        Expect(
            scenario.differential_kinematics_probe_generalized_velocities
                    .size() == velocity_count,
            "the differential-kinematics probe must have one entry per velocity");
        Expect(
            scenario.inverse_mapping_probe_generalized_position_derivatives
                    .size() == scenario.generalized_positions.size(),
            "the inverse-mapping probe must have one entry per position");
        const std::vector<orvd_contract::ObservationKind> expected_position_kinds = {
            orvd_contract::ObservationKind::kAngleRadians,
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
        const std::vector<orvd_contract::ObservationKind>
            expected_velocity_kinds = {
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kTranslationalVelocityMetersPerSecond,
                orvd_contract::ObservationKind::
                    kTranslationalVelocityMetersPerSecond,
                orvd_contract::ObservationKind::
                    kTranslationalVelocityMetersPerSecond,
            };
        Expect(scenario.generalized_velocity_observation_kinds ==
                   expected_velocity_kinds,
               "velocity kinds must match the scenario topology");
        const std::vector<orvd_contract::ObservationKind>
            expected_position_derivative_kinds = {
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::
                    kAngularVelocityRadiansPerSecond,
                orvd_contract::ObservationKind::kQuaternionDerivativePerSecond,
                orvd_contract::ObservationKind::kQuaternionDerivativePerSecond,
                orvd_contract::ObservationKind::kQuaternionDerivativePerSecond,
                orvd_contract::ObservationKind::kQuaternionDerivativePerSecond,
                orvd_contract::ObservationKind::
                    kTranslationalVelocityMetersPerSecond,
                orvd_contract::ObservationKind::
                    kTranslationalVelocityMetersPerSecond,
                orvd_contract::ObservationKind::
                    kTranslationalVelocityMetersPerSecond,
            };
        Expect(scenario.generalized_position_derivative_observation_kinds ==
                   expected_position_derivative_kinds,
               "position-derivative kinds must match the wxyz/free-body "
               "topology");
        bool probe_has_positive_component = false;
        bool probe_has_negative_component = false;
        const auto& velocity_probe =
            scenario.differential_kinematics_probe_generalized_velocities;
        for (const double probe_component : velocity_probe) {
            Expect(std::isfinite(probe_component) && probe_component != 0.0,
                   "every differential-kinematics probe component must be "
                   "finite and nonzero");
            probe_has_positive_component =
                probe_has_positive_component || probe_component > 0.0;
            probe_has_negative_component =
                probe_has_negative_component || probe_component < 0.0;
        }
        for (std::size_t first = 0; first < velocity_probe.size(); ++first) {
            for (std::size_t second = first + 1; second < velocity_probe.size();
                 ++second) {
                Expect(velocity_probe[first] != velocity_probe[second],
                       "differential-kinematics probe components must be "
                       "pairwise distinct so column permutations are visible");
            }
        }
        Expect(probe_has_positive_component && probe_has_negative_component,
               "the differential-kinematics probe must contain both signs");
        Expect(velocity_probe != scenario.generalized_velocities,
               "the differential-kinematics probe must differ from the state "
               "velocity");

        for (const double derivative_component :
             scenario.inverse_mapping_probe_generalized_position_derivatives) {
            Expect(std::isfinite(derivative_component),
                   "every inverse-mapping probe component must be finite");
        }
        if (scenario.generalized_positions.size() >= 7 &&
            scenario.inverse_mapping_probe_generalized_position_derivatives
                    .size() >= 7) {
            double quaternion_radial_component = 0.0;
            for (std::size_t index = 3; index < 7; ++index) {
                quaternion_radial_component +=
                    scenario.generalized_positions[index] *
                    scenario
                        .inverse_mapping_probe_generalized_position_derivatives
                            [index];
            }
            Expect(std::abs(quaternion_radial_component) > 1.0e-2,
                   "the inverse-mapping probe must exercise quaternion radial "
                   "projection");
        }

        if (excitation == "dynamic_excitation") {
            for (const double generalized_acceleration :
                 scenario.generalized_accelerations) {
                Expect(std::isfinite(generalized_acceleration) &&
                           generalized_acceleration != 0.0,
                       "dynamic generalized accelerations must be finite and "
                       "nonzero");
            }
        } else {
            for (const double generalized_acceleration :
                 scenario.generalized_accelerations) {
                Expect(generalized_acceleration == 0.0,
                       "the near-zero scenario must prescribe zero generalized "
                       "accelerations");
            }
        }

        using orvd_contract::GeneralizedForceComponentKind;
        const std::vector<GeneralizedForceComponentKind> expected_force_kinds = {
            GeneralizedForceComponentKind::kTorqueNewtonMetres,
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

void CheckQuaternionStateIsNonDegenerateAndUnitLength() {
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario("dynamic_excitation");
    Expect(scenario.generalized_positions.size() >= 7,
           "the scenario must contain the floating quaternion");
    if (scenario.generalized_positions.size() < 7) return;
    // Positions [3..6] are the floating body's quaternion in this topology.
    double squared_norm = 0.0;
    for (std::size_t index = 3; index < 7; ++index)
        squared_norm += scenario.generalized_positions[index] *
                        scenario.generalized_positions[index];
    Expect(std::abs(squared_norm - 1.0) < 1e-12,
           "the prescribed floating quaternion must be unit length");

    // All four are distinct and one has a different sign, so exchanging any
    // pair changes the represented rotation instead of moving equal fixture
    // values around. Pin the deliberate wxyz order as part of the premise.
    const double scale = 1.0 / std::sqrt(30.0);
    const std::vector<double> expected_wxyz = {
        scale, 2.0 * scale, -3.0 * scale, 4.0 * scale};
    for (std::size_t offset = 0; offset < expected_wxyz.size(); ++offset) {
        Expect(std::abs(scenario.generalized_positions[3 + offset] -
                        expected_wxyz[offset]) < 1e-15,
               "the floating quaternion must use the non-degenerate wxyz "
               "fixture");
    }
}

void CheckTheElbowMountIsTurnedAndNotOnTheWorld() {
    // The rotated mount is what makes the cross-process comparison exercise a
    // non-identity fixed-frame rotation at all. Nothing downstream would notice
    // if it quietly became identity: both sides would degrade together and
    // agree. So the premise is stated here rather than assumed.
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(
            "dynamic_excitation");
    const orvd_contract::RevoluteJointDefinition* elbow = nullptr;
    for (const auto& joint : scenario.revolute_joints)
        if (joint.name == "elbow_joint") elbow = &joint;
    Expect(elbow != nullptr, "the fixture must contain the elbow joint");
    if (elbow == nullptr) return;

    Expect(!elbow->parent_link_name.empty(),
           "the elbow hangs off a link, not off the world: a frame fixed to the "
           "world is not something the modelling facade expresses");
    Expect(!elbow->parent_frame_rotation_in_parent.isIdentity(0.0),
           "the elbow's mount must be turned, or no non-identity fixed-frame "
           "rotation enters the position chain anywhere in this comparison");

    // The rotation itself, so that a change to it is a change somebody made on
    // purpose. A quarter of a radian about y, give or take.
    const double angle = 0.37;
    Eigen::Matrix3d expected;
    expected << std::cos(angle), 0.0, std::sin(angle), 0.0, 1.0, 0.0,
        -std::sin(angle), 0.0, std::cos(angle);
    Expect((elbow->parent_frame_rotation_in_parent - expected)
                   .cwiseAbs()
                   .maxCoeff() < 1e-15,
           "the elbow's mount is a rotation of 0.37 rad about y");
    Expect(elbow->axis_in_parent.isApprox(Eigen::Vector3d::UnitX()),
           "and its axis is stated in that turned frame, so a dropped rotation "
           "moves the axis as well as the frame");
}

void CheckRelativeVelocityObservationNamesThreeDistinctLinks() {
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(
            "dynamic_excitation");
    Expect(scenario.relative_spatial_velocity_observations.size() == 1,
           "the scenario must state one nontrivial relative velocity");
    if (scenario.relative_spatial_velocity_observations.size() != 1) return;
    const auto& relative =
        scenario.relative_spatial_velocity_observations.front();
    Expect(relative.moving_link_name != relative.reference_link_name &&
               relative.moving_link_name != relative.expressed_in_link_name &&
               relative.reference_link_name != relative.expressed_in_link_name,
           "moving, reference and expression links must be distinct");
    Expect(relative.moving_link_name == "lower_link" &&
               relative.reference_link_name == "floating_link" &&
               relative.expressed_in_link_name == "upper_link",
           "the relative observation must retain its named frame roles");
}

void CheckTheReverseBranchIsActuallyAReverseBranch() {
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(
            "dynamic_excitation");
    const orvd_contract::RevoluteJointDefinition* reverse_branch = nullptr;
    for (const auto& joint : scenario.revolute_joints) {
        if (joint.name == "reverse_branch_joint") reverse_branch = &joint;
    }
    Expect(reverse_branch != nullptr,
           "the fixture must contain the reverse branch joint");
    if (reverse_branch == nullptr) return;
    Expect(reverse_branch->parent_link_name == "reverse_branch_link" &&
               reverse_branch->child_link_name == "upper_link",
           "the unrooted branch must be stated as parent of an already "
           "world-connected child so the forest traverses it in reverse");
    Expect(!reverse_branch->parent_frame_rotation_in_parent.isIdentity(0.0) &&
               !reverse_branch->parent_frame_translation_in_parent_meters
                    .isZero(0.0),
           "the reverse branch must carry both a rotated and displaced mount");
    Expect(scenario.generalized_positions.size() > 2 &&
               scenario.generalized_positions[2] != 0.0 &&
               scenario.generalized_velocities.size() > 2 &&
               scenario.generalized_velocities[2] != 0.0,
           "the dynamic fixture must excite both the position and velocity of "
           "the reverse branch");
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
    CheckQuaternionStateIsNonDegenerateAndUnitLength();
    CheckTheElbowMountIsTurnedAndNotOnTheWorld();
    CheckRelativeVelocityObservationNamesThreeDistinctLinks();
    CheckTheReverseBranchIsActuallyAReverseBranch();
    CheckLooseObservationStreamParsing();

    if (failure_count > 0) {
        std::fprintf(stderr, "%d contract check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("scenario and observation contract verified\n");
    return 0;
}
