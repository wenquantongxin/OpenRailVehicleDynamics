// Builds the scenario with ORVD's own modelling facade and writes the raw
// observations to stdout.
//
// The candidate side of the comparison. Like the reference emitter it owns
// nothing but arithmetic: it does not know which observations are required,
// declares no tolerance, and cannot certify itself. Two sides that each
// declared their own acceptance rule could agree on a wrong one.
//
// Runs in its own process, and links no Drake at all — not the library, not a
// vendored copy through some other target. That is checked by the launcher
// rather than asserted here, because a claim a program makes about itself is
// the one thing it cannot check.
//
// Only what G31 has landed is emitted. The mass matrix and the inverse dynamics
// belong to later goals; the judge's position-kinematics requirement set is what
// says so, and it refuses a missing observation rather than skipping it.
#include <cmath>
#include <cstdio>
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
                               joint.axis_in_parent);
    }
    for (const auto& free_body_name : scenario.free_body_names) {
        model.DeclareFreeBody(body_named(free_body_name));
    }
    model.Finalize();

    const Eigen::VectorXd generalized_positions =
        CopyToEigenVector(scenario.generalized_positions);
    const Eigen::VectorXd generalized_velocities =
        CopyToEigenVector(scenario.generalized_velocities);
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

    for (const auto& link : scenario.links) {
        const auto pose =
            model.CalcPoseInWorld(*context, model.GetRigidBodyByName(link.name));
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
    }

    const Eigen::VectorXd positions_after_evaluation =
        context->generalized_positions();
    for (int position_index = 0;
         position_index < positions_after_evaluation.size(); ++position_index) {
        stream.observations.push_back(
            {"state_readback_position[" + std::to_string(position_index) + "]",
             positions_after_evaluation(position_index)});
    }

    std::fputs(orvd_contract::FormatObservationStream(stream).c_str(), stdout);
    return 0;
}
