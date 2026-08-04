#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <span>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "orvd/system_assembly/system_instance.h"

namespace {

using orvd::configuration::AssembleVehicleMultibodyModel;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::configuration::VehicleDefinition;
using orvd::configuration::VehicleRigidBodyDefinition;
using orvd::multibody_model::MultibodyModel;

constexpr double kMachine = std::numeric_limits<double>::epsilon();
constexpr double kGravityMagnitude = 9.81;

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

// ---------------------------------------------------------------------------
// Gate 1 — the complete inertia conversion, read from the assembled model
// ---------------------------------------------------------------------------

void CheckCompleteSpatialInertiaAssembly() {
    VehicleDefinition probe;
    probe.vehicle_name = "complete_spatial_inertia_probe";
    VehicleRigidBodyDefinition body;
    body.name = "probe_body";
    body.moves_freely_in_world = true;
    body.mass_kilograms = 5.0;
    body.center_of_mass_in_body_frame_meters = Eigen::Vector3d(0.2, -0.3, 0.4);
    body.inertia_moments_about_center_of_mass_kilogram_square_meters =
        Eigen::Vector3d(2.1, 2.8, 3.4);
    body.inertia_products_about_center_of_mass_kilogram_square_meters =
        Eigen::Vector3d(0.11, -0.07, 0.09);
    probe.rigid_bodies.push_back(body);

    const auto model = AssembleVehicleMultibodyModel(probe, kGravityMagnitude);
    const auto context = model->CreateDefaultContext();
    Eigen::MatrixXd mass_matrix(model->num_generalized_velocities(),
                                model->num_generalized_velocities());
    model->CalcGeneralizedMassMatrix(*context, mass_matrix);
    Expect(mass_matrix.allFinite(),
           "the assembled synthetic mass matrix is finite");
    const auto range =
        model->GetFreeBodyVelocityRange(model->GetRigidBodyByName(body.name));
    Expect(range.size() == 6,
           "the synthetic free body exposes one complete spatial inertia block");

    Eigen::Matrix<double, 6, 6> expected;
    expected << 3.35, 0.41, -0.47, 0.0, -2.0, -1.5,
        0.41, 3.80, 0.69, 2.0, 0.0, -1.0,
        -0.47, 0.69, 4.05, 1.5, 1.0, 0.0,
        0.0, 2.0, 1.5, 5.0, 0.0, 0.0,
        -2.0, 0.0, 1.0, 0.0, 5.0, 0.0,
        -1.5, -1.0, 0.0, 0.0, 0.0, 5.0;
    const Eigen::Matrix<double, 6, 6> actual =
        mass_matrix.block<6, 6>(range.start(), range.start());
    const double scale = expected.cwiseAbs().maxCoeff();
    Expect((actual - expected).cwiseAbs().maxCoeff() <=
               64.0 * kMachine * scale,
           "the assembler carries mass, signed centre of mass, all three inertia "
           "matrix off-diagonal entries and the full parallel-axis shift into "
           "the finalized model");
}

void CheckRevoluteAxisAndDampingAssembly() {
    VehicleDefinition probe;
    probe.vehicle_name = "revolute_axis_and_damping_probe";

    VehicleRigidBodyDefinition carrier;
    carrier.name = "carrier";
    carrier.moves_freely_in_world = true;
    carrier.mass_kilograms = 1.0;
    carrier.inertia_moments_about_center_of_mass_kilogram_square_meters =
        Eigen::Vector3d::Ones();
    probe.rigid_bodies.push_back(carrier);

    VehicleRigidBodyDefinition rotor = carrier;
    rotor.name = "rotor";
    rotor.moves_freely_in_world = false;
    probe.rigid_bodies.push_back(rotor);
    const Eigen::Vector3d axis = Eigen::Vector3d(2.0, -3.0, 6.0).normalized();
    probe.revolute_joints.push_back(
        {"pivot", "carrier", "rotor", axis, 3.0});

    const auto model = AssembleVehicleMultibodyModel(probe, kGravityMagnitude);
    // The assembled model owns its values. Destroying the mutable source record
    // after assembly must not change or invalidate the finalized model.
    probe = VehicleDefinition{};
    const auto joint = model->GetJointByName("pivot");
    const auto rotor_handle = model->GetRigidBodyByName("rotor");
    auto context = model->CreateDefaultContext();

    constexpr double kAngle = 0.43;
    Eigen::VectorXd positions = context->generalized_positions();
    positions[model->GetJointPositionRange(joint).start()] = kAngle;
    model->SetGeneralizedPositions(context.get(), positions);
    const Eigen::Matrix3d actual_rotation =
        model->CalcPoseInWorld(*context, rotor_handle).rotation();
    Expect(actual_rotation.allFinite(),
           "the synthetic joint rotation is finite");
    const Eigen::Matrix3d expected_rotation =
        Eigen::AngleAxisd(kAngle, axis).toRotationMatrix();
    Expect((actual_rotation - expected_rotation).cwiseAbs().maxCoeff() <=
               32.0 * kMachine,
           "a positive revolute coordinate turns the child about the arbitrary "
           "three-component axis stated by the mutable record");
    Expect((actual_rotation -
            Eigen::AngleAxisd(kAngle, Eigen::Vector3d::UnitY())
                .toRotationMatrix())
               .cwiseAbs()
               .maxCoeff() > 0.1,
           "the axis check distinguishes the record value from a hard-coded y "
           "axis");

    constexpr double kJointVelocity = 2.0;
    Eigen::VectorXd velocities = context->generalized_velocities();
    const auto velocity_range = model->GetJointVelocityRange(joint);
    velocities[velocity_range.start()] = kJointVelocity;
    model->SetGeneralizedVelocities(context.get(), velocities);
    Eigen::VectorXd damping_force(model->num_generalized_velocities());
    model->CalcJointDampingAppliedGeneralizedForces(*context, damping_force);
    Expect(damping_force.allFinite(),
           "the synthetic joint damping force is finite");
    Eigen::VectorXd expected_force =
        Eigen::VectorXd::Zero(model->num_generalized_velocities());
    expected_force[velocity_range.start()] = -3.0 * kJointVelocity;
    Expect((damping_force - expected_force).cwiseAbs().maxCoeff() <=
               16.0 * kMachine,
           "the assembler carries a nonzero revolute damping coefficient and "
           "its opposing-force sign into the finalized model");
}

// ---------------------------------------------------------------------------
// Gate 2 — geometric invariants of the named frames, in the world
// ---------------------------------------------------------------------------

Eigen::Vector3d FramePositionInWorld(const MultibodyModel& model,
                                     const orvd::multibody_model::
                                         MultibodyEvaluationContext& context,
                                     std::string_view name) {
    return model.CalcPoseInWorld(context, model.GetFrameByName(name))
        .translation();
}

void CheckGeometricInvariants(const VehicleDefinition& vehicle,
                              const MultibodyModel& model) {
    const auto context = model.CreateDefaultContext();

    // A weld has no pose of its own. The parent mounting frame and the child
    // body frame must therefore coincide after assembly. This checks the actual
    // endpoint wiring directly, without rebuilding the rigid sub-assembly's
    // mass properties in test code.
    for (const auto& weld : vehicle.weld_joints) {
        const auto parent_pose =
            model.CalcPoseInWorld(*context,
                                  model.GetFrameByName(weld.parent_frame_name));
        const auto child_pose =
            model.CalcPoseInWorld(*context,
                                  model.GetFrameByName(weld.child_frame_name));
        Expect((parent_pose.translation() - child_pose.translation())
                       .cwiseAbs()
                       .maxCoeff() <=
                   1.0e-12 &&
                   (parent_pose.rotation() - child_pose.rotation())
                           .cwiseAbs()
                           .maxCoeff() <=
                       1.0e-12,
               "each welded child's body frame coincides with its named "
               "mounting frame");
    }

    // Mirror symmetry, over exactly the frames that have a lateral coordinate
    // to mirror. A frame on the longitudinal mid-plane has nothing to say here,
    // and asserting over it would make the discriminating-power proof below
    // pass on a frame whose sign cannot be flipped.
    std::map<std::string, Eigen::Vector3d> positions;
    for (const auto& frame : vehicle.fixed_frames) {
        positions.emplace(frame.name,
                          FramePositionInWorld(model, *context, frame.name));
    }

    int frames_naming_a_side = 0;
    for (const auto& frame : vehicle.fixed_frames) {
        if (frame.name.find("_right") != std::string::npos) {
            ++frames_naming_a_side;
        }
    }
    int mirrored_pairs = 0;
    int unpaired_lateral_frames = 0;
    double worst_mirror_residual = 0.0;
    for (const auto& [name, position] : positions) {
        if (position.y() <= 0.0) {
            continue;
        }
        const std::size_t right = name.rfind("_right_");
        std::string partner;
        if (right != std::string::npos) {
            partner = name.substr(0, right) + "_left_" +
                      name.substr(right + std::string("_right_").size());
        } else if (name.size() > 6 && name.compare(name.size() - 6, 6,
                                                   "_right") == 0) {
            partner = name.substr(0, name.size() - 6) + "_left";
        }
        const auto found = positions.find(partner);
        if (partner.empty() || found == positions.end()) {
            ++unpaired_lateral_frames;
            std::printf("     unpaired lateral frame: %s\n", name.c_str());
            continue;
        }
        ++mirrored_pairs;
        const Eigen::Vector3d mirrored(found->second.x(), -found->second.y(),
                                       found->second.z());
        worst_mirror_residual =
            std::max(worst_mirror_residual,
                     (position - mirrored).cwiseAbs().maxCoeff());
    }
    Expect(unpaired_lateral_frames == 0,
           "every frame with a lateral offset has a partner on the other side");
    Expect(mirrored_pairs == frames_naming_a_side,
           "every frame whose name states a side was mirrored, so the assertion "
           "covers the whole paired subset rather than whichever part happened "
           "to be found");
    Expect(mirrored_pairs > 20,
           "that subset is most of the frame family, not a token pair");
    Expect(worst_mirror_residual <= 1.0e-12,
           "left and right frames of the same name mirror about the vehicle's "
           "longitudinal mid-plane");

    // The wheelbase, between the two primary-suspension attachment frames of one
    // bogie. Not between the axlebox bushing frames: those are separated by
    // lt - 2*bAxle_x = 1.54 m, which is not the wheelbase and would be a
    // plausible-looking wrong answer.
    const double wheelbase =
        FramePositionInWorld(model, *context,
                             "front_leading_right_primary_suspension_"
                             "bogie_side_attachment")
            .x() -
        FramePositionInWorld(model, *context,
                             "front_trailing_right_primary_suspension_"
                             "bogie_side_attachment")
            .x();
    Expect(std::abs(wheelbase - 2.5) <= 1.0e-12,
           "the primary suspension frames of one bogie are one wheelbase apart");

    // The bogie centre distance, on the carbody-side weld mounting frames. In
    // the default configuration every free body sits at the world origin, so
    // the two bogie frames are not where this distance lives; the carbody's
    // carbody interface mounts are.
    const double bogie_centre_distance =
        FramePositionInWorld(model, *context, "front_carbody_interface_mount")
            .x() -
        FramePositionInWorld(model, *context, "rear_carbody_interface_mount")
            .x();
    Expect(std::abs(bogie_centre_distance - 15.7) <= 1.0e-12,
           "the two carbody interface mounts are one bogie centre distance apart");

    // Discriminating power, stated on a frame that actually has a lateral
    // coordinate to flip. On a mid-plane frame the same mutation changes
    // nothing, which is why the assertion above is scoped to the paired subset.
    const Eigen::Vector3d probe = FramePositionInWorld(
        model, *context,
        "front_leading_right_primary_suspension_bogie_side_attachment");
    Expect(std::abs(probe.y()) > 0.5,
           "the frame the mirror proof is stated on is far off the mid-plane, "
           "so flipping its lateral sign moves it by metres rather than by "
           "nothing");
}

// ---------------------------------------------------------------------------
// Gate 3 — the assembled model is all and only what the record states
// ---------------------------------------------------------------------------

void CheckAssemblyIsAllAndOnly(const VehicleDefinition& vehicle,
                               const MultibodyModel& model) {
    Expect(model.is_finalized(), "the model finalized");
    Expect(model.gravity_vector() == Eigen::Vector3d(0.0, 0.0, kGravityMagnitude),
           "the assembler stated gravity along the line's downward axis rather "
           "than leaving the multibody layer's own value, which points the "
           "other way");

    Expect(model.num_rigid_bodies() ==
               static_cast<int>(vehicle.rigid_bodies.size()),
           "every body the record states became a body, and no other body was "
           "added");
    // One world frame, one frame per body, and the record's named frames.
    Expect(model.num_frames() == 1 + static_cast<int>(
                                         vehicle.rigid_bodies.size() +
                                         vehicle.fixed_frames.size()),
           "every named frame the record states became a frame, and no other "
           "frame was added");
    Expect(model.num_joints() ==
               static_cast<int>(vehicle.revolute_joints.size() +
                                vehicle.weld_joints.size()),
           "free bodies produced no named joints, so the count is the record's "
           "joints and nothing else");
    Expect(model.num_generalized_positions() == 57 &&
               model.num_generalized_velocities() == 50,
           "seven free bodies and eight revolute joints give the declared GZ18 "
           "coordinate dimensions");

    int declared_free = 0;
    for (const auto& body : vehicle.rigid_bodies) {
        const auto handle = model.GetRigidBodyByName(body.name);
        Expect(model.IsFreeBody(handle) == body.moves_freely_in_world,
               "body '" + body.name + "' is free exactly when the record says");
        declared_free += body.moves_freely_in_world ? 1 : 0;
    }
    Expect(declared_free == 7, "the record declares seven free bodies");

    for (const auto& frame : vehicle.fixed_frames) {
        static_cast<void>(model.GetFrameByName(frame.name));
    }
    for (const auto& joint : vehicle.revolute_joints) {
        static_cast<void>(model.GetJointByName(joint.name));
    }
    Expect(vehicle.revolute_joints.size() == 8,
           "the GZ18 record carries all eight axlebox-carrier pivots");
    bool all_pivots_follow_the_source_type = true;
    for (const auto& joint : vehicle.revolute_joints) {
        all_pivots_follow_the_source_type =
            all_pivots_follow_the_source_type &&
            joint.axis_in_parent_frame == Eigen::Vector3d::UnitY() &&
            joint.damping_newton_metre_seconds_per_radian == 0.0;
    }
    Expect(all_pivots_follow_the_source_type,
           "every GZ18 type-2 pivot uses its derived positive-y axis and the "
           "source joint type's ideal zero damping, recorded explicitly in "
           "ORVD");
    for (const auto& joint : vehicle.weld_joints) {
        static_cast<void>(model.GetJointByName(joint.name));
    }

    double total_mass = 0.0;
    for (const auto& body : vehicle.rigid_bodies) {
        total_mass += body.mass_kilograms;
    }
    Expect(total_mass == 55695.0,
           "the ten placeholder bodies are still carrying their mass into the "
           "vehicle total");
}

void CheckRejections(const VehicleDefinition& vehicle) {
    const auto refuses = [](VehicleDefinition broken,
                            const std::string& description) {
        try {
            static_cast<void>(
                AssembleVehicleMultibodyModel(broken, kGravityMagnitude));
        } catch (const std::exception&) {
            return;
        }
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    };

    VehicleDefinition unknown_frame = vehicle;
    unknown_frame.revolute_joints.front().parent_frame_name = "no_such_frame";
    refuses(std::move(unknown_frame),
            "a joint naming a frame the assembler cannot resolve is refused");

    VehicleDefinition unnamed = vehicle;
    unnamed.vehicle_name.clear();
    refuses(std::move(unnamed),
            "a programmatically constructed record needs a vehicle identity");

    VehicleDefinition massless = vehicle;
    massless.rigid_bodies.front().mass_kilograms = 0.0;
    refuses(std::move(massless),
            "a programmatically modified body needs finite positive mass before "
            "the inertia conversion divides by it");

    VehicleDefinition orphan = vehicle;
    for (auto& body : orphan.rigid_bodies) {
        if (body.name == "front_bogie_frame") {
            body.moves_freely_in_world = false;
        }
    }
    refuses(std::move(orphan),
            "a body left with no path to the world is refused");

    VehicleDefinition doubly_related = vehicle;
    doubly_related.weld_joints.push_back(
        {"extra_weld", "carbody", "front_bogie_frame"});
    refuses(std::move(doubly_related),
            "a body both declared free and welded into the tree is refused");

    VehicleDefinition repeated_name = vehicle;
    repeated_name.rigid_bodies.push_back(repeated_name.rigid_bodies.front());
    refuses(std::move(repeated_name), "a repeated body name is refused");

    VehicleDefinition zero_axis = vehicle;
    zero_axis.revolute_joints.front().axis_in_parent_frame =
        Eigen::Vector3d::Zero();
    refuses(std::move(zero_axis), "a joint axis of zero length is refused");

    VehicleDefinition negative_damping = vehicle;
    negative_damping.revolute_joints.front()
        .damping_newton_metre_seconds_per_radian = -1.0;
    refuses(std::move(negative_damping), "a negative damping is refused");

    for (const double magnitude : {0.0, -9.81,
                                   std::numeric_limits<double>::infinity()}) {
        try {
            static_cast<void>(AssembleVehicleMultibodyModel(vehicle, magnitude));
            std::printf("FAIL a gravity magnitude of %g was accepted\n",
                        magnitude);
            ++failure_count;
        } catch (const std::invalid_argument&) {
        }
    }
}

// The assembled vehicle has to be something the rest of the product can pick up.
// This drives it once through the system-assembly chain and asks the only
// question that has a closed-form answer: with no force elements attached, does
// the whole vehicle fall, and does it fall the way this project's frame says
// down is. It is the end-to-end statement of the gravity direction — a model
// built with the multibody layer's own default would accelerate the other way.
//
// The system here deliberately carries no force plan. The record's suspension
// is not slack in the default configuration: every free body sits at the world
// origin, so an air spring's two ends are metres apart and the element is
// enormously stretched. That is a property of the configuration, not a defect,
// and a start-up state will place the bodies where the suspension is at rest.
// Until then, gravity alone is the only thing whose answer is known.
void CheckTheVehicleFallsTheWayDownIs(const VehicleDefinition& vehicle,
                                      const MultibodyModel& model) {
    using orvd::system_assembly::CompiledSystemPlan;
    using orvd::system_assembly::SystemAssemblyDescription;
    using orvd::system_assembly::SystemInstance;

    const SystemAssemblyDescription description(model);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);

    Expect(system.continuous_state_size() ==
               model.num_generalized_positions() +
                   model.num_generalized_velocities(),
           "a system with no force plan carries the vehicle's coordinates and "
           "nothing else, which is what makes the free fall below a closed-form "
           "answer");

    auto runtime = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd derivative(system.continuous_state_size());
    plan.CalcStateTimeDerivatives(*runtime, derivative);
    Expect(derivative.allFinite(),
           "the assembled GZ18 right-hand side is finite");

    const int velocity_start = model.num_generalized_positions();
    double worst_angular = 0.0;
    double worst_translational = 0.0;
    int checked_free_bodies = 0;
    for (const auto& body : vehicle.rigid_bodies) {
        if (!body.moves_freely_in_world) {
            continue;
        }
        ++checked_free_bodies;
        const auto range =
            model.GetFreeBodyVelocityRange(model.GetRigidBodyByName(body.name));
        for (int index = 0; index < 3; ++index) {
            worst_angular = std::max(
                worst_angular,
                std::abs(derivative[velocity_start + range.start() + index]));
        }
        const Eigen::Vector3d translational_acceleration(
            derivative[velocity_start + range.start() + 3],
            derivative[velocity_start + range.start() + 4],
            derivative[velocity_start + range.start() + 5]);
        worst_translational =
            std::max(worst_translational,
                     (translational_acceleration -
                      Eigen::Vector3d(0.0, 0.0, kGravityMagnitude))
                         .cwiseAbs()
                         .maxCoeff());
    }
    Expect(checked_free_bodies == 7,
           "the gravity smoke traversed every GZ18 free body");
    Expect(worst_angular <= 1.0e-12,
           "an unforced vehicle does not start rotating");
    Expect(worst_translational <= 1.0e-12,
           "every free body accelerates at g along the line's downward axis, "
           "which is the opposite of what the multibody layer's own default "
           "gravity would give");
}

// ---------------------------------------------------------------------------
// Gate 5 — the force plan, and who owns the nominal forces
// ---------------------------------------------------------------------------

void CheckForcePlanAndParameterOwnership(const VehicleDefinition& vehicle,
                                         const MultibodyModel& model) {
    using orvd::system_assembly::CompiledSystemPlan;
    using orvd::system_assembly::SystemAssemblyDescription;
    using orvd::system_assembly::SystemInstance;

    const auto plan = orvd::configuration::BuildVehicleForcePlan(vehicle, model);
    Expect(plan->translational_spring_damper_count() == 20 &&
               plan->roll_spring_damper_couple_count() == 2 &&
               plan->series_spring_viscous_damper_count() == 2 &&
               plan->saturated_piecewise_linear_damper_count() == 4,
           "the plan holds every element the record states, one per instance, "
           "grouped by real constitutive family");
    Expect(plan->series_spring_damper_force_state_count() == 2,
           "only the series family carries state, so the state block is two "
           "components wide");
    Expect(plan->body_wrench_count() == 56,
           "each of the twenty-eight elements produces exactly two body "
           "wrenches");

    const SystemAssemblyDescription description(model, *plan);
    const SystemInstance system(description);
    const CompiledSystemPlan compiled(system);
    Expect(system.continuous_state_size() == 109,
           "the system's state is the vehicle's coordinates plus the two "
           "series forces");
    Expect(system.series_spring_damper_force_state_range().start() == 107 &&
               system.series_spring_damper_force_state_range().size() == 2,
           "the force state sits after q and v, adjacent to them");

    // The nominal forces belong to a context, not to the record and not to the
    // compiled plan. Two contexts of one system hold their own, and neither the
    // record nor the plan changes when one is written.
    auto first = system.CreateDefaultRuntimeContext(0.0);
    auto second = system.CreateDefaultRuntimeContext(0.0);
    Expect(first->nominal_forces().size() == 60 &&
               first->nominal_forces() == Eigen::VectorXd::Zero(60),
           "a fresh context starts with every nominal force at zero: G50 makes "
           "the slot, a resolved start-up state fills it");
    system.SetNominalForce(*first, 0, Eigen::Vector3d(11.0, -22.0, 33.0));
    system.SetNominalForce(*second, 0, Eigen::Vector3d(-44.0, 55.0, -66.0));
    Expect(first->nominal_forces().segment<3>(0) ==
                   Eigen::Vector3d(11.0, -22.0, 33.0) &&
               second->nominal_forces().segment<3>(0) ==
                   Eigen::Vector3d(-44.0, 55.0, -66.0),
           "two contexts hold different nominal forces without contaminating "
           "each other");
    Expect(first->nominal_forces().segment<3>(3) == Eigen::Vector3d::Zero(),
           "writing one element's nominal force leaves the others alone");

    // And the difference reaches the dynamics: the same state under two
    // different preloads gives two different derivatives.
    Eigen::VectorXd first_derivative(109);
    Eigen::VectorXd second_derivative(109);
    compiled.CalcStateTimeDerivatives(*first, first_derivative);
    compiled.CalcStateTimeDerivatives(*second, second_derivative);
    Expect((first_derivative - second_derivative).cwiseAbs().maxCoeff() > 1.0e-6,
           "a context's nominal force reaches the right-hand side, so the slot "
           "is consumed rather than merely stored");

    // The state derivative of the series block is the constitutive law's, and
    // it is not zero merely because the vehicle is at rest: at zero relative
    // velocity and zero force it is zero, so give it a force to relax.
    Eigen::VectorXd state(109);
    system.CopyContinuousState(*first, state);
    state.segment<2>(107) << 5000.0, -3000.0;
    system.SetTimeAndContinuousState(*first, 0.0, state);
    Eigen::VectorXd relaxing(109);
    compiled.CalcStateTimeDerivatives(*first, relaxing);
    // dF/dt = K v - (K/C) F, and with the vehicle at rest v is zero, so the
    // rate is exactly -(K/C) F with K/C = 1e9 / 6e4.
    const double rate_constant = 1.0e9 / 6.0e4;
    Expect(std::abs(relaxing[107] + rate_constant * 5000.0) <=
                   1.0e-6 * rate_constant * 5000.0 &&
               std::abs(relaxing[108] - rate_constant * 3000.0) <=
                   1.0e-6 * rate_constant * 3000.0,
           "each series element relaxes at its own time constant, into its own "
           "slice of the state block");

    // A record edited after the plan was compiled cannot reach it.
    VehicleDefinition edited = vehicle;
    edited.translational_spring_dampers.front().stiffness_newtons_per_meter *=
        3.0;
    Eigen::VectorXd after_edit(109);
    compiled.CalcStateTimeDerivatives(*first, after_edit);
    Expect(after_edit == relaxing,
           "editing the record leaves an already compiled plan untouched");
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            throw std::invalid_argument("expected the GZ18 record path");
        }
        const VehicleDefinition vehicle =
            LoadVehicleDefinitionFromJsonFile(std::filesystem::path(argv[1]));
        const auto model =
            AssembleVehicleMultibodyModel(vehicle, kGravityMagnitude);

        CheckCompleteSpatialInertiaAssembly();
        CheckRevoluteAxisAndDampingAssembly();
        CheckGeometricInvariants(vehicle, *model);
        CheckAssemblyIsAllAndOnly(vehicle, *model);
        CheckTheVehicleFallsTheWayDownIs(vehicle, *model);
        CheckForcePlanAndParameterOwnership(vehicle, *model);
        CheckRejections(vehicle);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "GZ18 vehicle assembly failed: %s\n", error.what());
        return 1;
    }
    if (failure_count != 0) {
        std::printf("%d GZ18 vehicle assembly checks failed\n", failure_count);
        return 1;
    }
    std::printf("GZ18 vehicle assembly verified\n");
    return 0;
}
