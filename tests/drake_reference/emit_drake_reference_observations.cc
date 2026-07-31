// Builds a scenario with unmodified upstream Drake and writes the raw
// observations to stdout.
//
// This emitter owns nothing but arithmetic. It does not know which observations
// are required, does not declare a tolerance, and cannot certify itself; the
// launcher and comparator hold those. That separation is the point: two sides
// that each declared their own acceptance rule could agree on a wrong one.
//
// Runs in its own process. Drake and a future ORVD candidate must never share an
// address space: libdrake exports the same drake:: symbols a vendored copy would
// keep, and co-linking is an ODR violation whose likely symptom is a comparison
// that appears to pass.
#include <cstdio>
#include <string>
#include <string_view>

#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"

#include "contract/observation_stream.h"
#include "contract/scenario_definition.h"

namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;

Eigen::VectorXd CopyToEigenVector(const std::vector<double>& values) {
    return Eigen::Map<const Eigen::VectorXd>(
        values.data(), static_cast<int>(values.size()));
}

void BuildDrakePlantFromScenario(const orvd_contract::ScenarioDefinition& scenario,
                                 MultibodyPlant<double>* plant) {
    for (const auto& link : scenario.links) {
        const Eigen::Vector3d& extents = link.solid_box_full_extents_meters;
        plant->AddRigidBody(
                link.name,
                SpatialInertia<double>::MakeFromCentralInertia(
                link.mass_kilograms, link.center_of_mass_in_link_frame_meters,
                link.mass_kilograms *
                    UnitInertia<double>::SolidBox(extents.x(), extents.y(), extents.z())));
    }
    for (const auto& joint : scenario.revolute_joints) {
        const auto& parent = joint.parent_link_name.empty()
                                 ? plant->world_body()
                                 : plant->GetBodyByName(joint.parent_link_name);
        plant->AddJoint<RevoluteJoint>(
            joint.name, parent,
            RigidTransformd(
                drake::math::RotationMatrixd(joint.parent_frame_rotation_in_parent),
                joint.parent_frame_translation_in_parent_meters),
            plant->GetBodyByName(joint.child_link_name), RigidTransformd::Identity(),
            joint.axis_in_parent);
    }
    // Gravity is set from the scenario rather than inherited, so a change in the
    // library default cannot silently move the results.
    plant->mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d(
            0.0, -scenario.gravity_acceleration_meters_per_second_squared, 0.0));
    plant->Finalize();
}

}  // namespace

int main(int argc, char** argv) {
    const std::string_view excitation =
        argc > 1 ? std::string_view{argv[1]} : std::string_view{"dynamic_excitation"};
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(excitation);

    MultibodyPlant<double> plant(0.0);
    BuildDrakePlantFromScenario(scenario, &plant);

    const Eigen::VectorXd generalized_positions =
        CopyToEigenVector(scenario.generalized_positions);
    const Eigen::VectorXd generalized_velocities =
        CopyToEigenVector(scenario.generalized_velocities);
    const Eigen::VectorXd generalized_accelerations =
        CopyToEigenVector(scenario.generalized_accelerations);
    if (generalized_positions.size() != plant.num_positions() ||
        generalized_velocities.size() != plant.num_velocities()) {
        std::fprintf(stderr,
                     "scenario state (%lld, %lld) does not match the built plant (%d, %d)\n",
                     static_cast<long long>(generalized_positions.size()),
                     static_cast<long long>(generalized_velocities.size()),
                     plant.num_positions(), plant.num_velocities());
        return 1;
    }

    auto context = plant.CreateDefaultContext();
    plant.SetPositions(context.get(), generalized_positions);
    plant.SetVelocities(context.get(), generalized_velocities);

    orvd_contract::ObservationStream stream;
    // The world is not one of the model's rigid bodies. Drake counts it and the
    // ORVD facade does not, so the fact is named for what it means rather than
    // left as a bare "num_bodies" that the two sides would answer differently
    // while both being right.
    stream.topology_facts = {
        {"num_positions", plant.num_positions()},
        {"num_velocities", plant.num_velocities()},
        {"num_rigid_bodies_excluding_world",
         static_cast<int>(plant.num_bodies()) - 1}};

    // Where each element's coordinates ended up, asked of the plant rather than
    // copied from the scenario. This is what makes the index-keyed state and
    // read-back observations mean the same thing on both sides instead of
    // agreeing by coincidence.
    for (const auto& joint_definition : scenario.revolute_joints) {
        const auto& joint =
            plant.GetJointByName(joint_definition.name);
        stream.topology_facts.push_back(
            {"joint_position_range_start[" + joint_definition.name + "]",
             joint.position_start()});
        stream.topology_facts.push_back(
            {"joint_position_range_size[" + joint_definition.name + "]",
             joint.num_positions()});
        stream.topology_facts.push_back(
            {"joint_velocity_range_start[" + joint_definition.name + "]",
             joint.velocity_start()});
        stream.topology_facts.push_back(
            {"joint_velocity_range_size[" + joint_definition.name + "]",
             joint.num_velocities()});
    }
    for (const auto& free_body_name : scenario.free_body_names) {
        const auto& body = plant.GetBodyByName(free_body_name);
        stream.topology_facts.push_back(
            {"free_body_position_range_start[" + free_body_name + "]",
             body.floating_positions_start()});
        stream.topology_facts.push_back(
            {"free_body_position_range_size[" + free_body_name + "]", 7});
        stream.topology_facts.push_back(
            {"free_body_velocity_range_start[" + free_body_name + "]",
             body.floating_velocities_start_in_v()});
        stream.topology_facts.push_back(
            {"free_body_velocity_range_size[" + free_body_name + "]", 6});
    }

    Eigen::MatrixXd mass_matrix(plant.num_velocities(), plant.num_velocities());
    plant.CalcMassMatrix(*context, &mass_matrix);
    for (int column = 0; column < plant.num_velocities(); ++column) {
        const Eigen::VectorXd column_generalized_force_response =
            mass_matrix.col(column) *
            scenario.mass_matrix_column_generalized_accelerations[
                static_cast<std::size_t>(column)];
        for (int generalized_force_index = 0;
             generalized_force_index < column_generalized_force_response.size();
             ++generalized_force_index)
            stream.observations.push_back(
                {"mass_matrix_column_generalized_force_response[" +
                     std::to_string(column) + "][" +
                     std::to_string(generalized_force_index) + "]",
                 column_generalized_force_response(generalized_force_index)});
    }

    drake::multibody::MultibodyForces<double> applied_forces(plant);
    plant.CalcForceElementsContribution(*context, &applied_forces);
    const Eigen::VectorXd inverse_dynamics =
        plant.CalcInverseDynamics(*context, generalized_accelerations, applied_forces);
    for (int velocity_index = 0; velocity_index < inverse_dynamics.size();
         ++velocity_index)
        stream.observations.push_back(
            {"inverse_dynamics[" + std::to_string(velocity_index) + "]",
             inverse_dynamics(velocity_index)});

    for (const auto& link : scenario.links) {
        const RigidTransformd pose =
            plant.EvalBodyPoseInWorld(*context, plant.GetBodyByName(link.name));
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                stream.observations.push_back(
                    {"pose_" + link.name + "[" + std::to_string(row) + "," +
                         std::to_string(column) + "]",
                     pose.rotation().matrix()(row, column)});
        for (int axis_index = 0; axis_index < 3; ++axis_index)
            stream.observations.push_back(
                {"pose_" + link.name + "_translation[" + std::to_string(axis_index) + "]",
                 pose.translation()(axis_index)});
    }

    // Read generalized positions back after evaluation, so an implementation
    // that rewrites or normalizes them in place cannot hide behind poses that
    // happen to look right.
    const Eigen::VectorXd positions_after_evaluation = plant.GetPositions(*context);
    for (int position_index = 0; position_index < positions_after_evaluation.size();
         ++position_index) {
        stream.observations.push_back(
            {"state_readback_position[" + std::to_string(position_index) + "]",
             positions_after_evaluation(position_index)});
    }

    std::fputs(orvd_contract::FormatObservationStream(stream).c_str(), stdout);
    return 0;
}
