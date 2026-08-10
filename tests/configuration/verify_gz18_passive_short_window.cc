// G55: the first real 10 ms passive GZ18 end-to-end slice.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "orvd/configuration/assembled_gz18_contact_scenario.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"

namespace {

using orvd::configuration::AssembledGz18ContactScenario;
using orvd::configuration::AssembleGz18ContactScenario;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::LoadTrackGeometryFromJsonFile;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::forces::WheelRailContactInterfaceObservation;
using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemContinuousStateAdvancer;
using orvd::multibody_model::AppliedBodyWrench;

constexpr std::size_t kSampleCount = 101;
constexpr std::size_t kCarrierCount = 4;
constexpr std::size_t kInterfaceCount = 8;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "GZ18 passive short window: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

double RelativeTolerance(double scale, double relative = 2.0e-9) {
    return relative * std::max(1.0, scale);
}

bool Near(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected,
          double relative = 2.0e-9) {
    return (actual - expected).lpNorm<Eigen::Infinity>() <=
           RelativeTolerance(expected.lpNorm<Eigen::Infinity>(), relative);
}

ContinuousStateErrorTolerances MakeGz18Tolerances() {
    Eigen::VectorXd absolute = Eigen::VectorXd::Constant(109, 1.0e-9);
    absolute.head(57).setConstant(1.0e-10);
    absolute.tail(2).setConstant(1.0e-2);
    return ContinuousStateErrorTolerances(1.0e-8, std::move(absolute));
}

std::array<double, kSampleCount> MakeSampleTimes() {
    std::array<double, kSampleCount> times{};
    for (std::size_t index = 0; index < times.size(); ++index) {
        times[index] = static_cast<double>(index) * 1.0e-4;
    }
    return times;
}

struct ShortWindowObservation {
    std::array<double, kCarrierCount> track_station_meters{};
    std::array<double, kCarrierCount> wheelset_lateral_meters{};
    std::array<double, kCarrierCount> wheelset_yaw_radians{};
    std::array<WheelRailContactInterfaceObservation, kInterfaceCount>
        interfaces{};
};

std::array<ShortWindowObservation, kSampleCount> BuildObservationBatch(
    const AssembledGz18ContactScenario& scenario,
    const std::array<double, kSampleCount>& sample_times,
    const Eigen::MatrixXd& dense_states) {
    const auto& assembled = scenario.vehicle_system();
    const auto* contact_plan = assembled.contact_force_plan();
    auto observation_context =
        assembled.system().CreateDefaultRuntimeContext(sample_times.front());
    assembled.system().CopyContextLocalParameters(
        scenario.initial_context().context(), *observation_context);
    auto workspace = contact_plan->CreateWorkspace();
    std::vector<AppliedBodyWrench> wrenches(kInterfaceCount);
    std::vector<WheelRailContactInterfaceObservation> interface_observations(
        kInterfaceCount);
    std::array<ShortWindowObservation, kSampleCount> batch{};

    for (std::size_t sample = 0; sample < sample_times.size(); ++sample) {
        assembled.system().SetTimeAndContinuousState(
            *observation_context, sample_times[sample],
            dense_states.col(static_cast<Eigen::Index>(sample)));
        assembled.system().UpdateWheelRailProjectionStationHints(
            *observation_context);
        auto component = assembled.system().GetMultibodyComponentView(
            *observation_context, assembled.system().multibody_component());
        contact_plan->CalcAppliedForcesAndObservations(
            component.context(), *workspace,
            observation_context
                ->wheel_rail_projection_station_hints_meters(),
            wrenches, interface_observations);

        ShortWindowObservation& output = batch[sample];
        std::copy(interface_observations.begin(),
                  interface_observations.end(), output.interfaces.begin());
        for (std::size_t carrier = 0; carrier < kCarrierCount; ++carrier) {
            const double station =
                observation_context
                    ->wheel_rail_projection_station_hints_meters()[carrier];
            output.track_station_meters[carrier] = station;
            const auto track =
                contact_plan->track_geometry().EvaluateTrackFrame(station);
            const auto body = assembled.model().GetRigidBodyByName(
                contact_plan->carrier_name(static_cast<int>(carrier)));
            const auto pose = assembled.model().CalcPoseInWorld(
                component.context(), body);
            const Eigen::Matrix3d rotation_track_from_inertial =
                track.pose().rotation_inertial_from_track().transpose();
            const Eigen::Vector3d origin_in_track =
                rotation_track_from_inertial *
                (pose.translation() -
                 track.pose().origin_in_inertial_meters());
            output.wheelset_lateral_meters[carrier] = origin_in_track.y();
            output.wheelset_yaw_radians[carrier] =
                orvd::wheel_rail_contact::ResolveRollYawPitch(
                    rotation_track_from_inertial * pose.rotation())
                    .yaw_radians;
        }
    }
    return batch;
}

void CheckInitialVerticalAcceleration(
    const AssembledGz18ContactScenario& scenario,
    const orvd::configuration::ResolvedStartupState& startup) {
    const auto& assembled = scenario.vehicle_system();
    auto& context = scenario.initial_context().context();
    Eigen::VectorXd derivatives(109);
    assembled.compiled_plan().CalcStateTimeDerivatives(context, derivatives);
    const int nq = assembled.model().num_generalized_positions();
    const int nv = assembled.model().num_generalized_velocities();
    auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());
    Eigen::MatrixXd angular(3, assembled.model().num_rigid_bodies());
    Eigen::MatrixXd translational(3, assembled.model().num_rigid_bodies());
    assembled.model()
        .CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
            component.context(), derivatives.segment(nq, nv), &angular,
            &translational);

    for (const auto& state : startup.free_body_startup_states) {
        const auto body = assembled.model().GetRigidBodyByName(state.body_name);
        int column = -1;
        for (int index = 0; index < assembled.model().num_rigid_bodies();
             ++index) {
            if (assembled.model().GetRigidBody(index) == body) {
                column = index;
                break;
            }
        }
        Require(column >= 0,
                "a start-up free body is absent from the acceleration order");
        if (column >= 0) {
            Require(std::abs(translational(2, column)) <= 1.0e-3,
                    "a start-up free body is not vertically supported");
        }
    }
}

void CheckPhysicalObservationBatch(
    const AssembledGz18ContactScenario& scenario,
    const std::array<ShortWindowObservation, kSampleCount>& observations) {
    const auto& placements =
        scenario.initial_context().wheel_pair_placements();
    const auto* contact_plan = scenario.vehicle_system().contact_force_plan();
    Require(placements.size() == kCarrierCount,
            "the resolved start-up did not retain four target wheelsets");

    for (std::size_t sample = 0; sample < observations.size(); ++sample) {
        for (std::size_t carrier = 0; carrier < kCarrierCount; ++carrier) {
            const double station =
                observations[sample].track_station_meters[carrier];
            Require(contact_plan->track_geometry()
                            .CurvatureRadiansPerMeter(station) ==
                        0.0 &&
                    contact_plan->track_geometry().SuperelevationMeters(
                        station) == 0.0 &&
                    contact_plan->track_geometry().CenterlineUpwardGrade(
                        station) == 0.0,
                    "a carrier left the qualified straight, level track span");
        }
        for (const auto& interface : observations[sample].interfaces) {
            Require(interface.contact_patch_count == 1 &&
                        interface.vertical_support_force_on_wheel_newtons >
                            0.0 &&
                        interface.normal_force_newtons > 0.0 &&
                        std::isfinite(
                            interface.longitudinal_force_on_wheel_newtons) &&
                        std::isfinite(interface.lateral_force_on_wheel_newtons),
                    "an interface lost its finite positive single-patch contact");
        }
    }

    if (placements.size() == kCarrierCount) {
        for (std::size_t carrier = 0; carrier < kCarrierCount; ++carrier) {
            const auto& placement = placements[carrier];
            Require(contact_plan->carrier_name(static_cast<int>(carrier)) ==
                        placement.station_reference_body_name,
                    "the target support order differs from the contact carriers");
            const std::string expected_prefix =
                placement.station_reference_body_name + ".";
            Require(contact_plan->interface_name(
                        static_cast<int>(2 * carrier)) ==
                            expected_prefix + "right" &&
                        contact_plan->interface_name(
                            static_cast<int>(2 * carrier + 1)) ==
                            expected_prefix + "left",
                    "the target support sides differ from the named contact "
                    "interfaces");
            const auto& right = observations.front().interfaces[2 * carrier];
            const auto& left =
                observations.front().interfaces[2 * carrier + 1];
            Require(std::abs(right.vertical_support_force_on_wheel_newtons -
                             placement.right_support_force_newtons) <= 1.0 &&
                        std::abs(left.vertical_support_force_on_wheel_newtons -
                                 placement.left_support_force_newtons) <= 1.0,
                    "the resolved target support does not match the real t=0 "
                    "contact result");
        }
    }
}

void CheckFinalDynamicWiring(
    const AssembledGz18ContactScenario& scenario) {
    const auto& assembled = scenario.vehicle_system();
    auto& context = scenario.initial_context().context();
    const auto* contact_plan = assembled.contact_force_plan();
    const int nq = assembled.model().num_generalized_positions();
    const int nv = assembled.model().num_generalized_velocities();

    Eigen::VectorXd rhs(109);
    assembled.compiled_plan().CalcStateTimeDerivatives(context, rhs);
    auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());

    std::vector<AppliedBodyWrench> wrenches(64);
    Eigen::VectorXd series_derivatives(2);
    assembled.force_plan().CalcAppliedForces(
        component.context(), context.series_spring_damper_forces(),
        context.nominal_forces(), std::span(wrenches).first(56),
        series_derivatives);
    auto contact_workspace = contact_plan->CreateWorkspace();
    std::array<WheelRailContactInterfaceObservation, kInterfaceCount>
        contact_observations{};
    contact_plan->CalcAppliedForcesAndObservations(
        component.context(), *contact_workspace,
        context.wheel_rail_projection_station_hints_meters(),
        std::span(wrenches).subspan(56, 8), contact_observations);

    Eigen::VectorXd projected_generalized_force = Eigen::VectorXd::Zero(nv);
    Eigen::MatrixXd angular_jacobian(3, nv);
    Eigen::MatrixXd translational_jacobian(3, nv);
    double spatial_power = 0.0;
    for (const AppliedBodyWrench& wrench : wrenches) {
        Require(wrench.expressed_in_frame == assembled.model().world_frame(),
                "a final body wrench is not expressed in the world frame");
        assembled.model()
            .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                component.context(), wrench.body,
                wrench.point_position_in_body_frame_meters, &angular_jacobian,
                &translational_jacobian);
        projected_generalized_force.noalias() +=
            angular_jacobian.transpose() *
                wrench.torque_about_point_newton_metres +
            translational_jacobian.transpose() * wrench.force_newtons;

        const auto pose = assembled.model().CalcPoseInWorld(
            component.context(), wrench.body);
        const auto velocity = assembled.model()
                                  .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                                      component.context(), wrench.body);
        const Eigen::Vector3d point_from_body_origin_in_world =
            pose.rotation() * wrench.point_position_in_body_frame_meters;
        const Eigen::Vector3d point_velocity =
            velocity
                .translational_velocity_at_frame_origin_meters_per_second() +
            velocity.angular_velocity_radians_per_second().cross(
                point_from_body_origin_in_world);
        spatial_power +=
            wrench.torque_about_point_newton_metres.dot(
                velocity.angular_velocity_radians_per_second()) +
            wrench.force_newtons.dot(point_velocity);
    }

    Eigen::VectorXd required_generalized_force(nv);
    assembled.model().CalcRequiredGeneralizedForces(
        component.context(), rhs.segment(nq, nv),
        required_generalized_force);
    Require(Near(projected_generalized_force, required_generalized_force),
            "the final 64 body wrenches do not close the required generalized "
            "force");
    Require(std::abs(projected_generalized_force.dot(
                         context.generalized_velocities()) -
                     spatial_power) <=
                RelativeTolerance(std::abs(spatial_power)),
            "the final spatial and generalized-force powers do not close");

    Eigen::VectorXd mapped_qdot(nq);
    assembled.model().MapGeneralizedVelocitiesToPositionDerivatives(
        component.context(), context.generalized_velocities(), &mapped_qdot);
    Require(Near(rhs.head(nq), mapped_qdot),
            "the final qdot slice is not N(q) times v");
    Require(Near(rhs.tail(2), series_derivatives),
            "the final Maxwell derivatives do not come from the vehicle plan");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr,
                     "usage: verify_gz18_passive_short_window VEHICLE STARTUP "
                     "LINE DATA_ROOT\n");
        return 2;
    }

    const auto vehicle = LoadVehicleDefinitionFromJsonFile(argv[1]);
    const auto startup = LoadResolvedStartupStateFromJsonFile(argv[2]);
    auto line = LoadTrackGeometryFromJsonFile(argv[3]);
    constexpr double kReferenceStationMeters = 0.0;
    constexpr double kProjectionHalfWidthMeters = 0.01;
    std::unique_ptr<AssembledGz18ContactScenario> scenario =
        AssembleGz18ContactScenario(
            vehicle, startup, std::move(line), std::filesystem::path(argv[4]),
            kReferenceStationMeters, kProjectionHalfWidthMeters);

    CheckInitialVerticalAcceleration(*scenario, startup);
    auto& accepted = scenario->initial_context().context();
    Eigen::VectorXd initial_state(109);
    scenario->vehicle_system().system().CopyContinuousState(accepted,
                                                            initial_state);
    const auto sample_times = MakeSampleTimes();
    SystemContinuousStateAdvancer advancer(
        scenario->vehicle_system().system(),
        scenario->vehicle_system().compiled_plan(), accepted,
        MakeGz18Tolerances(), NoCallTimeAppliedForces{});
    const Eigen::MatrixXd dense_states =
        advancer.AdvanceToWithDenseStateSamples(sample_times.back(),
                                                sample_times);
    Eigen::VectorXd final_state(109);
    scenario->vehicle_system().system().CopyContinuousState(accepted,
                                                            final_state);
    Require(dense_states.rows() == 109 &&
                dense_states.cols() ==
                    static_cast<Eigen::Index>(kSampleCount) &&
                dense_states.allFinite(),
            "the 10 ms advance did not return a finite 109 by 101 state batch");
    Require(dense_states.col(0) == initial_state &&
                dense_states.col(dense_states.cols() - 1) == final_state &&
                accepted.time_seconds() == sample_times.back(),
            "the state batch and accepted 10 ms endpoint do not share exact "
            "boundary states");

    const auto observations =
        BuildObservationBatch(*scenario, sample_times, dense_states);
    CheckPhysicalObservationBatch(*scenario, observations);
    CheckFinalDynamicWiring(*scenario);

    if (failures != 0) {
        std::fprintf(stderr, "%d GZ18 short-window assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts("GZ18 passive 10 ms short window verified");
    return 0;
}
