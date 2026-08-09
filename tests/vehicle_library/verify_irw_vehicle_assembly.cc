#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/load_vehicle_definition.h"

namespace {

using orvd::configuration::AssembleVehicleSystem;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::configuration::VehicleDefinition;
using orvd::multibody_model::MultibodyModel;

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

template <typename Definition>
std::set<std::string> NamesOf(const std::vector<Definition>& definitions) {
    std::set<std::string> names;
    for (const Definition& definition : definitions) {
        names.insert(definition.name);
    }
    return names;
}

std::set<std::string> ExpectedBodyNames() {
    std::set<std::string> names{
        "carbody",       "frame_front",   "frame_rear",
        "axlebridge_ff", "axlebridge_fr", "axlebridge_rf",
        "axlebridge_rr", "dum_front",     "dum_rear"};
    for (const std::string axle : {"ff", "fr", "rf", "rr"}) {
        names.insert("wheel_" + axle + "_l");
        names.insert("wheel_" + axle + "_r");
    }
    for (const std::string bogie : {"front", "rear"}) {
        for (const char tag : {'A', 'B', 'C', 'D'}) {
            names.insert("longibar_" + bogie + "_" + tag);
        }
    }
    return names;
}

std::set<std::string> ExpectedWheelJointNames() {
    std::set<std::string> names;
    for (const std::string axle : {"ff", "fr", "rf", "rr"}) {
        names.insert("rev_wheel_" + axle + "_l");
        names.insert("rev_wheel_" + axle + "_r");
    }
    return names;
}

std::set<std::string> ExpectedBallJointNames() {
    std::set<std::string> names;
    for (const std::string bogie : {"front", "rear"}) {
        for (const char tag : {'A', 'B', 'C', 'D'}) {
            names.insert("ball_longibar_" + bogie + "_" + tag);
        }
    }
    return names;
}

std::set<std::string> ExpectedTranslationalNames() {
    std::set<std::string> names;
    for (const std::string bogie : {"front", "rear"}) {
        for (const std::string tag : {"A1", "B1", "C1", "D1", "A2",
                                      "B2", "C2", "D2"}) {
            names.insert(bogie + "_ps_spring_" + tag);
        }
        for (const char tag : {'A', 'B', 'C', 'D'}) {
            names.insert(bogie + "_ps_dmp_" + tag);
        }
        for (const char tag : {'A', 'B'}) {
            names.insert(bogie + "_ss_airspring_" + tag);
            names.insert(bogie + "_ss_tractionrod_" + tag);
            names.insert(bogie + "_ss_antihunting_" + tag);
        }
    }
    return names;
}

std::set<std::string> ExpectedBushingNames() {
    std::set<std::string> names;
    for (const std::string bogie : {"front", "rear"}) {
        for (const char tag : {'A', 'B', 'C', 'D'}) {
            names.insert(bogie + "_ps_barfixed_" + tag);
        }
    }
    return names;
}

void CheckRecordIdentity(const VehicleDefinition& vehicle) {
    Expect(vehicle.vehicle_name == "IRW", "IRW vehicle identity");
    Expect(vehicle.mechanical_definition_identifier ==
               "irw_reference_mechanical_definition",
           "IRW mechanical definition identity");
    Expect(NamesOf(vehicle.rigid_bodies) == ExpectedBodyNames(),
           "the record contains the frozen 25-body IRW name set");

    const std::set<std::string> expected_free{
        "carbody",       "frame_front",   "frame_rear", "axlebridge_ff",
        "axlebridge_fr", "axlebridge_rf", "axlebridge_rr"};
    std::set<std::string> actual_free;
    for (const auto& body : vehicle.rigid_bodies) {
        if (body.moves_freely_in_world) {
            actual_free.insert(body.name);
        }
    }
    Expect(actual_free == expected_free,
           "the record declares the frozen seven free bodies");
    Expect(std::set<std::string>(
               vehicle.mechanical_track_station_layout
                   .wheel_contact_carrier_body_names.begin(),
               vehicle.mechanical_track_station_layout
                   .wheel_contact_carrier_body_names.end()) ==
               std::set<std::string>{"axlebridge_ff", "axlebridge_fr",
                                     "axlebridge_rf", "axlebridge_rr"},
           "the four axle bridges, not the eight wheel bodies, are the IRW "
           "wheel-rail contact carriers");

    Expect(NamesOf(vehicle.revolute_joints) == ExpectedWheelJointNames(),
           "the record contains all and only eight independent wheel joints");
    Expect(NamesOf(vehicle.ball_rpy_joints) == ExpectedBallJointNames(),
           "the record contains all and only eight longibar Ball-RPY joints");
    Expect(NamesOf(vehicle.weld_joints) ==
               std::set<std::string>{"weld_dum_front", "weld_dum_rear"},
           "the record contains the two frozen DUM welds");
    Expect(vehicle.fixed_frames.size() == 122,
           "the record contains 106 suspension and weld frames plus 16 "
           "Ball-RPY joint frames");

    Expect(NamesOf(vehicle.translational_spring_dampers) ==
               ExpectedTranslationalNames(),
           "the 36 compiled translational elements have the frozen name set");
    Expect(NamesOf(vehicle.roll_spring_damper_couples) ==
               std::set<std::string>{"front_ss_antiroll",
                                     "rear_ss_antiroll"},
           "the two compiled roll couples have the frozen name set");
    Expect(NamesOf(vehicle.half_angle_midpoint_roll_pitch_yaw_bushings) ==
               ExpectedBushingNames(),
           "the eight compiled longibar bushings have the frozen name set");
    Expect(NamesOf(vehicle.series_spring_viscous_dampers) ==
               std::set<std::string>{"sslatdmp_0_mk_front_BF_SLD",
                                     "sslatdmp_1_mk_rear_BF_SLD"},
           "the two Maxwell channels have the frozen names");
}

void CheckSourceScalarsAndBindings(const VehicleDefinition& vehicle) {
    double total_mass = 0.0;
    std::map<std::string, const orvd::configuration::VehicleRigidBodyDefinition*>
        body_by_name;
    for (const auto& body : vehicle.rigid_bodies) {
        total_mass += body.mass_kilograms;
        body_by_name.emplace(body.name, &body);
    }
    Expect(std::abs(total_mass - 40193.67123394637) <= 5.0e-11,
           "the complete source mass ledger is preserved");
    const auto& carbody = *body_by_name.at("carbody");
    Expect(carbody.mass_kilograms == 25000.0 &&
               carbody.center_of_mass_in_body_frame_meters ==
                   Eigen::Vector3d(0.0, 0.0, 1.8) &&
               carbody
                       .inertia_moments_about_center_of_mass_kilogram_square_meters ==
                   Eigen::Vector3d(50000.0, 1200000.0, 1200000.0),
           "the carbody mass, positive COM height and central inertia are "
           "source exact");
    const auto& longibar = *body_by_name.at("longibar_front_A");
    Expect(longibar.mass_kilograms == 23.958904243295922 &&
               longibar
                       .inertia_moments_about_center_of_mass_kilogram_square_meters ==
                   Eigen::Vector3d(1.3841101530761446,
                                   0.014019477425825506,
                                   1.3841101530761446),
           "the polygonal longibar mass and central inertia are source exact");

    std::map<std::string, const orvd::configuration::
                              VehicleTranslationalSpringDamperDefinition*>
        translational;
    for (const auto& element : vehicle.translational_spring_dampers) {
        translational.emplace(element.name, &element);
    }
    const auto& primary = *translational.at("front_ps_spring_A1");
    Expect(primary.reference_frame_name ==
                   "mk_frame_front__M_Frame_PS_Spring_baseA1" &&
               primary.opposite_frame_name ==
                   "mk_axlebridge_ff__M_AxleBridge_PS_Spring_baseA" &&
               primary.stiffness_newtons_per_meter ==
                   Eigen::Vector3d(50000.0, 50000.0, 830000.0) &&
               primary.damping_newton_seconds_per_meter ==
                   Eigen::Vector3d(0.0, 0.0, 7000.0),
           "the first primary spring binds its real Frame-A and axlebridge "
           "markers with the frozen constants");
    const auto& crossed = *translational.at("rear_ps_spring_C2");
    Expect(crossed.reference_frame_name ==
                   "mk_frame_rear__M_Frame_PS_Spring_baseC2" &&
               crossed.opposite_frame_name ==
                   "mk_axlebridge_rr__M_AxleBridge_PS_Spring_baseA",
           "the nontrivial C2-to-rear-axle-A endpoint mapping is preserved");
    const auto& air = *translational.at("front_ss_airspring_A");
    Expect(air.reference_frame_name ==
                   "mk_frame_front__M_Frame_AirSpring_A" &&
               air.opposite_frame_name ==
                   "mk_dum_front__M_DUM_AirSpring_A" &&
               air.stiffness_newtons_per_meter ==
                   Eigen::Vector3d(15000.0, 15000.0, 50000.0),
           "the air spring keeps Frame A as its reference end");

    std::map<std::string, const orvd::configuration::
                              VehicleHalfAngleMidpointRollPitchYawBushingDefinition*>
        bushing_by_name;
    for (const auto& element :
         vehicle.half_angle_midpoint_roll_pitch_yaw_bushings) {
        bushing_by_name.emplace(element.name, &element);
    }
    for (const std::string bogie : {"front", "rear"}) {
        for (const char tag : {'A', 'B', 'C', 'D'}) {
            const auto& bushing =
                *bushing_by_name.at(bogie + "_ps_barfixed_" + tag);
            Expect(bushing.frame_a_name ==
                       "mk_longibar_" + bogie + "_" + tag +
                           "__M_LongiBar_" + tag + "_FrameSide" &&
                       bushing.frame_c_name ==
                           "mk_frame_" + bogie + "__M_Frame_LongiBar_" +
                               tag,
                   "each half-angle bushing binds its named longibar and "
                   "frame endpoints");
            Expect(bushing.rotational_stiffness_newton_meters_per_radian ==
                           Eigen::Vector3d::Zero() &&
                       bushing.rotational_damping_newton_meter_seconds_per_radian ==
                           Eigen::Vector3d::Constant(100.0) &&
                       bushing.translational_stiffness_newtons_per_meter ==
                           Eigen::Vector3d::Constant(1.0e8) &&
                       bushing.translational_damping_newton_seconds_per_meter ==
                           Eigen::Vector3d::Constant(100.0),
                   "each half-angle bushing keeps the frozen isotropic "
                   "constants and zero rotational stiffness");
        }
    }
}

void CheckAssembledTopology(
    const VehicleDefinition& vehicle,
    const orvd::configuration::AssembledVehicleSystem& system) {
    const MultibodyModel& model = system.model();
    Expect(model.num_rigid_bodies() == 25 && model.num_frames() == 148 &&
               model.num_joints() == 18,
           "the finalized graph contains 25 bodies, 122 fixed frames and 18 "
           "named joints without hidden vehicle elements");
    Expect(model.num_generalized_positions() == 81 &&
               model.num_generalized_velocities() == 74,
           "seven free bodies, eight wheel joints and eight Ball-RPY joints "
           "produce nq=81 and nv=74");
    Expect(system.system().continuous_state_size() == 157,
           "the assembled passive IRW state is q81 plus v74 plus z2");

    const std::set<std::string> free_names{
        "carbody",       "frame_front",   "frame_rear", "axlebridge_ff",
        "axlebridge_fr", "axlebridge_rf", "axlebridge_rr"};
    for (const std::string& name : ExpectedBodyNames()) {
        const auto body = model.GetRigidBodyByName(name);
        Expect(model.IsFreeBody(body) == free_names.contains(name),
               "body '" + name + "' has the frozen free/constrained role");
    }
    for (const std::string& name : ExpectedWheelJointNames()) {
        const auto joint = model.GetJointByName(name);
        Expect(model.GetJointPositionRange(joint).size() == 1 &&
                   model.GetJointVelocityRange(joint).size() == 1,
               "wheel joint '" + name + "' owns exactly one q and one v");
    }
    for (const std::string& name : ExpectedBallJointNames()) {
        const auto joint = model.GetJointByName(name);
        Expect(model.GetJointPositionRange(joint).size() == 3 &&
                   model.GetJointVelocityRange(joint).size() == 3,
               "Ball-RPY joint '" + name + "' owns exactly three q and three v");
    }
    for (const std::string name : {"weld_dum_front", "weld_dum_rear"}) {
        const auto joint = model.GetJointByName(name);
        Expect(model.GetJointPositionRange(joint).size() == 0 &&
                   model.GetJointVelocityRange(joint).size() == 0,
               "weld '" + name + "' contributes no generalized coordinate");
    }

    const auto context = model.CreateDefaultContext();
    for (const auto& joint : vehicle.ball_rpy_joints) {
        const auto parent =
            model.CalcPoseInWorld(*context,
                                  model.GetFrameByName(joint.parent_frame_name));
        const auto child =
            model.CalcPoseInWorld(*context,
                                  model.GetFrameByName(joint.child_frame_name));
        Expect((parent.translation() - child.translation()).norm() <= 1.0e-14,
               "Ball-RPY joint '" + joint.name +
                   "' closes its two real attachment points by default");
        const auto range =
            model.GetJointPositionRange(model.GetJointByName(joint.name));
        Expect(context->generalized_positions().segment(range.start(), 3) ==
                   joint.default_roll_pitch_yaw_angles_radians,
               "Ball-RPY joint '" + joint.name +
                   "' preserves the frozen Euler-angle branch exactly");
    }
    for (const auto& weld : vehicle.weld_joints) {
        const auto parent =
            model.CalcPoseInWorld(*context,
                                  model.GetFrameByName(weld.parent_frame_name));
        const auto child =
            model.CalcPoseInWorld(*context,
                                  model.GetFrameByName(weld.child_frame_name));
        Expect((parent.translation() - child.translation()).norm() <= 1.0e-14 &&
                   (parent.rotation() - child.rotation())
                           .cwiseAbs()
                           .maxCoeff() <= 1.0e-14,
               "DUM weld '" + weld.name + "' closes at its real carbody mount");
    }

    const auto& plan = system.force_plan();
    Expect(plan.translational_spring_damper_count() == 36 &&
               plan.roll_spring_damper_couple_count() == 2 &&
               plan.half_angle_midpoint_roll_pitch_yaw_bushing_count() == 8 &&
               plan.series_spring_viscous_damper_count() == 2 &&
               plan.saturated_piecewise_linear_damper_count() == 0,
           "the 46 ordinary suspensions compile as 36 translational, two "
           "roll and eight full bushings, beside two Maxwell channels");
    Expect(plan.body_wrench_count() == 96 &&
               plan.nominal_force_component_count() == 108 &&
               plan.series_spring_damper_force_state_count() == 2,
           "the compiled plan owns 96 body wrenches, 108 nominal-force "
           "components and two continuous force states");
}

Eigen::Vector3d WorldPoint(
    const MultibodyModel& model,
    const orvd::multibody_model::MultibodyEvaluationContext& context,
    const orvd::multibody_model::AppliedBodyWrench& wrench) {
    const auto pose = model.CalcPoseInWorld(context, wrench.body);
    return pose.translation() +
           pose.rotation() * wrench.point_position_in_body_frame_meters;
}

void CheckRealBushingExecution(const VehicleDefinition& vehicle,
                               const MultibodyModel& model) {
    // G65 qualified the constitutive law on an independent non-degenerate
    // fixture. This gate supplies the missing real-vehicle half: all eight
    // frozen IRW names and endpoint bindings must execute together against the
    // assembled topology. Keeping only this family makes each output pair's
    // identity unambiguous without exposing cross-family wrench ordinals as a
    // product API.
    VehicleDefinition bushing_only = vehicle;
    bushing_only.translational_spring_dampers.clear();
    bushing_only.roll_spring_damper_couples.clear();
    bushing_only.series_spring_viscous_dampers.clear();
    bushing_only.saturated_piecewise_linear_dampers.clear();
    const auto plan =
        orvd::configuration::BuildVehicleForcePlan(bushing_only, model);

    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions = context->generalized_positions();
    Eigen::VectorXd velocities =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    for (std::size_t index = 0; index < vehicle.ball_rpy_joints.size();
         ++index) {
        const auto joint =
            model.GetJointByName(vehicle.ball_rpy_joints[index].name);
        const auto q_range = model.GetJointPositionRange(joint);
        const auto v_range = model.GetJointVelocityRange(joint);
        const double scale = static_cast<double>(index + 1);
        positions.segment<3>(q_range.start()) +=
            scale * Eigen::Vector3d(0.001, -0.0007, 0.0005);
        velocities.segment<3>(v_range.start()) =
            scale * Eigen::Vector3d(0.012, -0.008, 0.005);
    }
    model.SetGeneralizedState(context.get(), positions, velocities);

    std::vector<orvd::multibody_model::AppliedBodyWrench> wrenches(
        static_cast<std::size_t>(plan->body_wrench_count()));
    Eigen::VectorXd no_series_state(0);
    Eigen::VectorXd no_nominal_force(0);
    Eigen::VectorXd no_series_derivative(0);
    plan->CalcAppliedForces(*context, no_series_state, no_nominal_force,
                            wrenches, no_series_derivative);

    std::map<std::string, std::string> frame_body;
    for (const auto& frame : vehicle.fixed_frames) {
        frame_body.emplace(frame.name, frame.body_name);
    }
    Expect(plan->half_angle_midpoint_roll_pitch_yaw_bushing_count() == 8 &&
               wrenches.size() == 16,
           "the real eight-bushing batch emits exactly eight named pairs");
    for (std::size_t index = 0;
         index < vehicle.half_angle_midpoint_roll_pitch_yaw_bushings.size();
         ++index) {
        const auto& definition =
            vehicle.half_angle_midpoint_roll_pitch_yaw_bushings[index];
        const auto& on_a = wrenches[2 * index];
        const auto& on_c = wrenches[2 * index + 1];
        const std::string name = definition.name;
        Expect(model.GetRigidBodyName(on_a.body) ==
                       frame_body.at(definition.frame_a_name) &&
                   model.GetRigidBodyName(on_c.body) ==
                       frame_body.at(definition.frame_c_name),
               "real bushing '" + name +
                   "' lands on its frozen longibar and frame bodies");
        Expect(on_a.expressed_in_frame == model.world_frame() &&
                   on_c.expressed_in_frame == model.world_frame(),
               "real bushing '" + name +
                   "' emits both wrenches in the world frame");
        Expect(on_a.force_newtons.allFinite() &&
                   on_a.torque_about_point_newton_metres.allFinite() &&
                   on_c.force_newtons.allFinite() &&
                   on_c.torque_about_point_newton_metres.allFinite() &&
                   on_c.force_newtons.norm() > 1.0 &&
                   on_c.torque_about_point_newton_metres.norm() > 1.0e-6,
               "real bushing '" + name +
                   "' produces finite non-degenerate six-component output");
        Expect(on_a.force_newtons == -on_c.force_newtons &&
                   on_a.torque_about_point_newton_metres ==
                       -on_c.torque_about_point_newton_metres,
               "real bushing '" + name +
                   "' emits an exact equal-and-opposite wrench pair");
        const double point_error =
            (WorldPoint(model, *context, on_a) -
             WorldPoint(model, *context, on_c))
                .norm();
        Expect(point_error <= 1.0e-13,
               "real bushing '" + name +
                   "' applies both wrenches at one common midpoint");
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            throw std::invalid_argument("expected the IRW vehicle definition");
        }
        const VehicleDefinition vehicle =
            LoadVehicleDefinitionFromJsonFile(std::filesystem::path(argv[1]));
        CheckRecordIdentity(vehicle);
        CheckSourceScalarsAndBindings(vehicle);
        const auto system = AssembleVehicleSystem(vehicle, 9.81);
        CheckAssembledTopology(vehicle, *system);
        CheckRealBushingExecution(vehicle, system->model());
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW vehicle assembly failed: %s\n", error.what());
        return 1;
    }
    return failure_count == 0 ? 0 : 1;
}
