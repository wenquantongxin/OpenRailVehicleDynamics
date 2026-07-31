// Builds a scenario with the configured installed Drake 1.54.0 and writes the
// raw observations to stdout.
//
// This emitter owns nothing but arithmetic. It does not know which observations
// are required, does not declare a tolerance, and cannot certify itself; the
// launcher and comparator hold those. That separation is the point: two sides
// that each declared their own acceptance rule could agree on a wrong one.
//
// Runs in its own process. Drake and the ORVD candidate must never share an
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
    const Eigen::VectorXd differential_kinematics_probe = CopyToEigenVector(
        scenario.differential_kinematics_probe_generalized_velocities);
    const Eigen::VectorXd inverse_mapping_probe = CopyToEigenVector(
        scenario.inverse_mapping_probe_generalized_position_derivatives);
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

    Eigen::VectorXd mapped_position_derivatives(plant.num_positions());
    plant.MapVelocityToQDot(*context, differential_kinematics_probe,
                            &mapped_position_derivatives);
    Eigen::VectorXd mapped_back_velocities(plant.num_velocities());
    plant.MapQDotToVelocity(*context, inverse_mapping_probe,
                            &mapped_back_velocities);
    std::vector<drake::multibody::SpatialAcceleration<double>>
        spatial_accelerations(plant.num_bodies());
    plant.CalcSpatialAccelerationsFromVdot(
        *context, generalized_accelerations, &spatial_accelerations);

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
        // Asked of the joint the plant actually gave this body, sizes included.
        // Writing 7 and 6 as literals would be this emitter agreeing with
        // itself about the one thing the fact exists to report.
        const drake::multibody::Joint<double>* floating_joint = nullptr;
        for (drake::multibody::JointIndex index{0};
             index < plant.num_joints(); ++index) {
            const auto& joint = plant.get_joint(index);
            if (joint.child_body().index() == body.index())
                floating_joint = &joint;
        }
        if (floating_joint == nullptr) {
            std::fprintf(stderr,
                         "the plant gave '%s' no inboard joint\n",
                         free_body_name.c_str());
            return 1;
        }
        stream.topology_facts.push_back(
            {"free_body_position_range_start[" + free_body_name + "]",
             floating_joint->position_start()});
        stream.topology_facts.push_back(
            {"free_body_position_range_size[" + free_body_name + "]",
             floating_joint->num_positions()});
        stream.topology_facts.push_back(
            {"free_body_velocity_range_start[" + free_body_name + "]",
             floating_joint->velocity_start()});
        stream.topology_facts.push_back(
            {"free_body_velocity_range_size[" + free_body_name + "]",
             floating_joint->num_velocities()});
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
        const auto& body = plant.GetBodyByName(link.name);
        const RigidTransformd pose =
            plant.EvalBodyPoseInWorld(*context, body);
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

        const auto& velocity =
            plant.EvalBodySpatialVelocityInWorld(*context, body);
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            stream.observations.push_back(
                {"velocity_" + link.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 velocity.rotational()(axis_index)});
            stream.observations.push_back(
                {"velocity_" + link.name +
                     "_translational_at_body_origin[" +
                     std::to_string(axis_index) + "]",
                 velocity.translational()(axis_index)});
            stream.observations.push_back(
                {"acceleration_" + link.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 spatial_accelerations[body.index()].rotational()(axis_index)});
            stream.observations.push_back(
                {"acceleration_" + link.name +
                     "_translational_at_body_origin[" +
                     std::to_string(axis_index) + "]",
                 spatial_accelerations[body.index()].translational()(
                     axis_index)});
        }

        Eigen::MatrixXd spatial_jacobian(6, plant.num_velocities());
        plant.CalcJacobianSpatialVelocity(
            *context, drake::multibody::JacobianWrtVariable::kV,
            body.body_frame(), link.center_of_mass_in_link_frame_meters,
            plant.world_frame(), plant.world_frame(), &spatial_jacobian);
        const Eigen::VectorXd response =
            spatial_jacobian * differential_kinematics_probe;
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            stream.observations.push_back(
                {"jacobian_probe_response_" + link.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 response(axis_index)});
            stream.observations.push_back(
                {"jacobian_probe_response_" + link.name +
                     "_translational_at_center_of_mass[" +
                     std::to_string(axis_index) + "]",
                 response(axis_index + 3)});
        }
    }

    for (const auto& relative :
         scenario.relative_spatial_velocity_observations) {
        const auto velocity =
            plant.GetBodyByName(relative.moving_link_name)
                .body_frame()
                .CalcSpatialVelocity(
                    *context,
                    plant.GetBodyByName(relative.reference_link_name)
                        .body_frame(),
                    plant.GetBodyByName(relative.expressed_in_link_name)
                        .body_frame());
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            stream.observations.push_back(
                {"relative_velocity_" + relative.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 velocity.rotational()(axis_index)});
            stream.observations.push_back(
                {"relative_velocity_" + relative.name +
                     "_translational_at_moving_origin[" +
                     std::to_string(axis_index) + "]",
                 velocity.translational()(axis_index)});
        }
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
    const Eigen::VectorXd velocities_after_evaluation =
        plant.GetVelocities(*context);
    for (int velocity_index = 0;
         velocity_index < velocities_after_evaluation.size(); ++velocity_index) {
        stream.observations.push_back(
            {"state_readback_velocity[" + std::to_string(velocity_index) + "]",
             velocities_after_evaluation(velocity_index)});
    }
    for (int position_index = 0;
         position_index < mapped_position_derivatives.size(); ++position_index) {
        stream.observations.push_back(
            {"mapped_position_derivative[" +
                 std::to_string(position_index) + "]",
             mapped_position_derivatives(position_index)});
    }
    for (int velocity_index = 0;
         velocity_index < mapped_back_velocities.size(); ++velocity_index) {
        stream.observations.push_back(
            {"mapped_back_generalized_velocity[" +
                 std::to_string(velocity_index) + "]",
             mapped_back_velocities(velocity_index)});
    }

    std::fputs(orvd_contract::FormatObservationStream(stream).c_str(), stdout);
    return 0;
}
