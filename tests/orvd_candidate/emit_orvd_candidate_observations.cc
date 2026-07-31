// Builds the scenario with ORVD's own modelling facade and writes the raw
// observations to stdout.
//
// The candidate side of the comparison. Like the reference emitter it owns
// nothing but arithmetic: it does not know which observations are required,
// declares no tolerance, and cannot certify itself. Two sides that each
// declared their own acceptance rule could agree on a wrong one.
//
// Runs in its own process and links no installed `libdrake.so`. It does link
// the landed tree, statically, which is the vendored Drake source — that is what
// the product is, and pretending otherwise here would be describing a program
// that does not exist. What must not happen is the installed library being in
// the same address space as the vendored copy: both export the same `drake::`
// symbols, and co-linking them is an ODR violation whose likely symptom is a
// comparison that appears to pass. The built-artefact test checks that, not this
// file: a claim a program makes about its own linkage is the one claim it cannot
// check.
//
// Only what G31-G36 have landed is emitted. The judge's capability-specific
// requirement set says which observations count, and it refuses a missing
// observation rather than skipping it.
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include "contract/observation_stream.h"
#include "contract/scenario_definition.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::AppliedRevoluteJointTorque;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

Eigen::VectorXd CopyToEigenVector(const std::vector<double>& values) {
    return Eigen::Map<const Eigen::VectorXd>(values.data(),
                                             static_cast<int>(values.size()));
}

/// The scenario's mass properties in the form the facade takes them.
///
/// The scenario states a solid box about its centre of mass and where that
/// centre sits; the facade takes a unit inertia about the body's own origin. The
/// shift is the caller's arithmetic — the public layer has no box helper, and
/// giving it one would be shaping the modelling interface around what this test
/// happens to build. Doing it here also keeps the two sides independent: the
/// reference reaches the same numbers through Drake's own helpers.
RigidBodyInertiaParameters InertiaOfSolidBox(
    const orvd_contract::LinkDefinition& link) {
    const Eigen::Vector3d& extents = link.solid_box_full_extents_meters;
    const Eigen::Vector3d& centre = link.center_of_mass_in_link_frame_meters;
    const Eigen::Vector3d squared = extents.cwiseProduct(extents) / 12.0;

    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = link.mass_kilograms;
    inertia.center_of_mass_in_body_frame = centre;
    // Central, then shifted out to the body origin by the parallel-axis term.
    inertia.unit_inertia_moments =
        Eigen::Vector3d(squared.y() + squared.z(), squared.x() + squared.z(),
                        squared.x() + squared.y()) +
        Eigen::Vector3d(centre.y() * centre.y() + centre.z() * centre.z(),
                        centre.x() * centre.x() + centre.z() * centre.z(),
                        centre.x() * centre.x() + centre.y() * centre.y());
    inertia.unit_inertia_products =
        Eigen::Vector3d(-centre.x() * centre.y(), -centre.x() * centre.z(),
                        -centre.y() * centre.z());
    return inertia;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string_view excitation =
        argc > 1 ? std::string_view{argv[1]}
                 : std::string_view{"dynamic_excitation"};
    const orvd_contract::ScenarioDefinition scenario =
        orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(excitation);

    MultibodyModel model;
    model.SetGravityVector(Eigen::Vector3d(
        0.0, -scenario.gravity_acceleration_meters_per_second_squared, 0.0));

    std::vector<std::pair<std::string, RigidBodyHandle>> bodies;
    for (const auto& link : scenario.links) {
        bodies.emplace_back(link.name,
                            model.AddRigidBody(link.name, InertiaOfSolidBox(link)));
    }
    const auto body_named = [&bodies](const std::string& name) {
        for (const auto& entry : bodies) {
            if (entry.first == name) return entry.second;
        }
        std::fprintf(stderr, "the scenario names a link '%s' it never defined\n",
                     name.c_str());
        std::exit(1);
    };

    for (const auto& joint : scenario.revolute_joints) {
        FrameHandle parent = model.world_frame();
        if (!joint.parent_link_name.empty()) {
            const RigidBodyHandle parent_body = body_named(joint.parent_link_name);
            FixedFramePoseParameters mount;
            mount.R_PF = joint.parent_frame_rotation_in_parent;
            mount.p_PoFo_P = joint.parent_frame_translation_in_parent_meters;
            parent = model.AddFixedFrame(joint.name + "_mount", parent_body, mount);
        } else if (!joint.parent_frame_rotation_in_parent.isIdentity(0.0) ||
                   !joint.parent_frame_translation_in_parent_meters.isZero(0.0)) {
            // The facade fixes frames to bodies, and the world is not one of
            // them. No scenario needs this yet; the first one that does gets a
            // refusal here rather than a silently dropped offset.
            std::fprintf(stderr,
                         "joint '%s' asks for an offset frame on the world, "
                         "which this emitter cannot express\n",
                         joint.name.c_str());
            return 1;
        }
        model.AddRevoluteJoint(joint.name, parent,
                               model.body_frame(body_named(joint.child_link_name)),
                               joint.axis_in_parent,
                               joint.damping_newton_metre_seconds_per_radian);
    }
    for (const auto& free_body_name : scenario.free_body_names) {
        model.DeclareFreeBody(body_named(free_body_name));
    }
    model.Finalize();

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
    if (generalized_positions.size() != model.num_generalized_positions() ||
        generalized_velocities.size() != model.num_generalized_velocities()) {
        std::fprintf(stderr,
                     "scenario state (%lld, %lld) does not match the built "
                     "model (%d, %d)\n",
                     static_cast<long long>(generalized_positions.size()),
                     static_cast<long long>(generalized_velocities.size()),
                     model.num_generalized_positions(),
                     model.num_generalized_velocities());
        return 1;
    }

    const auto context = model.CreateDefaultContext();
    model.SetGeneralizedPositions(context.get(), generalized_positions);
    model.SetGeneralizedVelocities(context.get(), generalized_velocities);

    Eigen::VectorXd mapped_position_derivatives(
        model.num_generalized_positions());
    model.MapGeneralizedVelocitiesToPositionDerivatives(
        *context, differential_kinematics_probe,
        &mapped_position_derivatives);
    Eigen::VectorXd mapped_back_velocities(model.num_generalized_velocities());
    model.MapGeneralizedPositionDerivativesToVelocities(
        *context, inverse_mapping_probe, &mapped_back_velocities);
    Eigen::MatrixXd angular_accelerations(3, model.num_rigid_bodies());
    Eigen::MatrixXd translational_accelerations(3, model.num_rigid_bodies());
    model.CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
        *context, generalized_accelerations, &angular_accelerations,
        &translational_accelerations);
    Eigen::MatrixXd generalized_mass_matrix(
        model.num_generalized_velocities(),
        model.num_generalized_velocities());
    model.CalcGeneralizedMassMatrix(*context, generalized_mass_matrix);

    Eigen::VectorXd velocity_bias(model.num_generalized_velocities());
    Eigen::VectorXd gravity_applied(model.num_generalized_velocities());
    Eigen::VectorXd damping_applied(model.num_generalized_velocities());
    Eigen::VectorXd required(model.num_generalized_velocities());
    model.CalcVelocityBiasGeneralizedForces(*context, velocity_bias);
    model.CalcGravityAppliedGeneralizedForces(*context, gravity_applied);
    model.CalcJointDampingAppliedGeneralizedForces(*context, damping_applied);
    model.CalcRequiredGeneralizedForces(*context, generalized_accelerations,
                                        required);

    std::vector<AppliedBodyWrench> applied_body_wrenches;
    for (const auto& definition : scenario.applied_body_wrenches) {
        const FrameHandle expressed = definition.expressed_in_frame_name.empty()
                                          ? model.world_frame()
                                          : model.body_frame(body_named(
                                                definition.expressed_in_frame_name));
        applied_body_wrenches.push_back(
            {body_named(definition.body_name),
             definition.point_position_in_body_frame_meters, expressed,
             definition.torque_about_point_newton_metres,
             definition.force_newtons});
    }
    std::vector<AppliedRevoluteJointTorque> applied_joint_torques;
    for (const auto& definition : scenario.applied_revolute_joint_torques) {
        applied_joint_torques.push_back(
            {model.GetJointByName(definition.joint_name),
             definition.torque_newton_metres});
    }
    auto forward_workspace = model.CreateForwardDynamicsWorkspace();
    Eigen::VectorXd forward_acceleration(model.num_generalized_velocities());
    model.CalcGeneralizedVelocityDerivatives(
        *context, applied_body_wrenches, applied_joint_torques, {},
        *forward_workspace, forward_acceleration);
    Eigen::VectorXd state_derivative(model.num_generalized_positions() +
                                     model.num_generalized_velocities());
    model.CalcStateTimeDerivatives(
        *context, applied_body_wrenches, applied_joint_torques, {},
        *forward_workspace, state_derivative);

    orvd_contract::ObservationStream stream;
    stream.topology_facts = {
        {"num_positions", model.num_generalized_positions()},
        {"num_velocities", model.num_generalized_velocities()},
        {"num_rigid_bodies_excluding_world", model.num_rigid_bodies()}};

    for (const auto& joint_definition : scenario.revolute_joints) {
        const JointHandle joint = model.GetJointByName(joint_definition.name);
        stream.topology_facts.push_back(
            {"joint_position_range_start[" + joint_definition.name + "]",
             model.GetJointPositionRange(joint).start()});
        stream.topology_facts.push_back(
            {"joint_position_range_size[" + joint_definition.name + "]",
             model.GetJointPositionRange(joint).size()});
        stream.topology_facts.push_back(
            {"joint_velocity_range_start[" + joint_definition.name + "]",
             model.GetJointVelocityRange(joint).start()});
        stream.topology_facts.push_back(
            {"joint_velocity_range_size[" + joint_definition.name + "]",
             model.GetJointVelocityRange(joint).size()});
    }
    for (const auto& free_body_name : scenario.free_body_names) {
        const RigidBodyHandle body = model.GetRigidBodyByName(free_body_name);
        stream.topology_facts.push_back(
            {"free_body_position_range_start[" + free_body_name + "]",
             model.GetFreeBodyPositionRange(body).start()});
        stream.topology_facts.push_back(
            {"free_body_position_range_size[" + free_body_name + "]",
             model.GetFreeBodyPositionRange(body).size()});
        stream.topology_facts.push_back(
            {"free_body_velocity_range_start[" + free_body_name + "]",
             model.GetFreeBodyVelocityRange(body).start()});
        stream.topology_facts.push_back(
            {"free_body_velocity_range_size[" + free_body_name + "]",
             model.GetFreeBodyVelocityRange(body).size()});
    }

    for (std::size_t link_index = 0; link_index < scenario.links.size();
         ++link_index) {
        const auto& link = scenario.links[link_index];
        const RigidBodyHandle body = model.GetRigidBodyByName(link.name);
        const auto pose =
            model.CalcPoseInWorld(*context, body);
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                stream.observations.push_back(
                    {"pose_" + link.name + "[" + std::to_string(row) + "," +
                         std::to_string(column) + "]",
                     pose.rotation()(row, column)});
        for (int axis_index = 0; axis_index < 3; ++axis_index)
            stream.observations.push_back(
                {"pose_" + link.name + "_translation[" +
                     std::to_string(axis_index) + "]",
                 pose.translation()(axis_index)});

        const auto velocity =
            model.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, body);
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            stream.observations.push_back(
                {"velocity_" + link.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 velocity.angular_velocity_radians_per_second()(axis_index)});
            stream.observations.push_back(
                {"velocity_" + link.name +
                     "_translational_at_body_origin[" +
                     std::to_string(axis_index) + "]",
                 velocity
                     .translational_velocity_at_frame_origin_meters_per_second()(
                         axis_index)});
            stream.observations.push_back(
                {"acceleration_" + link.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 angular_accelerations(axis_index,
                                       static_cast<int>(link_index))});
            stream.observations.push_back(
                {"acceleration_" + link.name +
                     "_translational_at_body_origin[" +
                     std::to_string(axis_index) + "]",
                 translational_accelerations(axis_index,
                                             static_cast<int>(link_index))});
        }

        Eigen::MatrixXd angular_jacobian(3,
                                         model.num_generalized_velocities());
        Eigen::MatrixXd translational_jacobian(
            3, model.num_generalized_velocities());
        model.CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
            *context, body, link.center_of_mass_in_link_frame_meters,
            &angular_jacobian, &translational_jacobian);
        const Eigen::Vector3d angular_response =
            angular_jacobian * differential_kinematics_probe;
        const Eigen::Vector3d translational_response =
            translational_jacobian * differential_kinematics_probe;
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            stream.observations.push_back(
                {"jacobian_probe_response_" + link.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 angular_response(axis_index)});
            stream.observations.push_back(
                {"jacobian_probe_response_" + link.name +
                     "_translational_at_center_of_mass[" +
                     std::to_string(axis_index) + "]",
                 translational_response(axis_index)});
        }
    }

    for (const auto& relative :
         scenario.relative_spatial_velocity_observations) {
        const auto velocity =
            model
                .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                    *context,
                    model.body_frame(body_named(relative.moving_link_name)),
                    model.body_frame(body_named(relative.reference_link_name)),
                    model.body_frame(
                        body_named(relative.expressed_in_link_name)));
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            stream.observations.push_back(
                {"relative_velocity_" + relative.name + "_angular[" +
                     std::to_string(axis_index) + "]",
                 velocity.angular_velocity_radians_per_second()(axis_index)});
            stream.observations.push_back(
                {"relative_velocity_" + relative.name +
                     "_translational_at_moving_origin[" +
                     std::to_string(axis_index) + "]",
                 velocity
                     .translational_velocity_at_frame_origin_meters_per_second()(
                         axis_index)});
        }
    }

    const Eigen::VectorXd positions_after_evaluation =
        context->generalized_positions();
    for (int position_index = 0;
         position_index < positions_after_evaluation.size(); ++position_index) {
        stream.observations.push_back(
            {"state_readback_position[" + std::to_string(position_index) + "]",
             positions_after_evaluation(position_index)});
    }
    const Eigen::VectorXd velocities_after_evaluation =
        context->generalized_velocities();
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
    for (int column = 0; column < generalized_mass_matrix.cols(); ++column) {
        const Eigen::VectorXd column_generalized_force_response =
            generalized_mass_matrix.col(column) *
            scenario.mass_matrix_column_generalized_accelerations[
                static_cast<std::size_t>(column)];
        for (int generalized_force_index = 0;
             generalized_force_index <
             column_generalized_force_response.size();
             ++generalized_force_index) {
            stream.observations.push_back(
                {"mass_matrix_column_generalized_force_response[" +
                     std::to_string(column) + "][" +
                     std::to_string(generalized_force_index) + "]",
                 column_generalized_force_response(generalized_force_index)});
        }
    }

    for (int index = 0; index < model.num_generalized_velocities(); ++index) {
        const std::array<std::pair<std::string_view, const Eigen::VectorXd*>, 5>
            generalized_dynamics_observations{{
                {"velocity_bias_generalized_force", &velocity_bias},
                {"gravity_applied_generalized_force", &gravity_applied},
                {"damping_applied_generalized_force", &damping_applied},
                {"required_generalized_force", &required},
                {"forward_dynamics_generalized_acceleration",
                 &forward_acceleration},
            }};
        for (const auto& [name, values] : generalized_dynamics_observations) {
            stream.observations.push_back(
                {std::string(name) + "[" + std::to_string(index) + "]",
                 (*values)(index)});
        }
    }
    for (int index = 0; index < state_derivative.size(); ++index) {
        stream.observations.push_back(
            {"state_time_derivative[" + std::to_string(index) + "]",
             state_derivative(index)});
    }

    std::fputs(orvd_contract::FormatObservationStream(stream).c_str(), stdout);
    return 0;
}
