#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "orvd/track_geometry/track_geometry.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h"
#include "system_coordinate_problem.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "system_integration_test_configuration.h"

namespace {

namespace forces = orvd::forces;
namespace contact = orvd::wheel_rail_contact;
namespace track = orvd::track_geometry;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemRhsBridge;
using orvd::integrators::internal::CoordinateState;
using orvd::integrators::internal::SystemCoordinateProblem;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::system_assembly::SystemRuntimeContext;

constexpr double kRadius = 0.42;

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool SameBits(const Eigen::VectorXd& first, const Eigen::VectorXd& second) {
    return first.size() == second.size() &&
           std::memcmp(first.data(), second.data(),
                       static_cast<std::size_t>(first.size()) * sizeof(double)) == 0;
}

template <class Height>
contact::ProfilePoints Sample(contact::ProfileRole role, const char* name,
                              double half_width, int count, Height height) {
    std::vector<double> lateral;
    std::vector<double> vertical;
    for (int i = 0; i < count; ++i) {
        const double y = -half_width + 2.0 * half_width * i / (count - 1);
        lateral.push_back(y);
        vertical.push_back(height(y));
    }
    return contact::ProfilePoints::FromAuthoredOrder(
        role, name, std::move(lateral), std::move(vertical));
}

std::unique_ptr<contact::WheelRailContactRuntimePersonality> MakePersonality() {
    const auto wheel = Sample(contact::ProfileRole::kWheel, "synthetic_cone",
                              0.05, 1001, [](double y) { return 0.05 * y; });
    const auto rail = Sample(contact::ProfileRole::kRail, "synthetic_crown",
                             0.06, 1201, [](double y) { return 1.667 * y * y; });
    contact::WheelRailContactConfiguration configuration;
    configuration.geometry.nominal_rolling_radius_meters = kRadius;
    configuration.geometry.outline_sample_count = 1000;
    configuration.geometry.island_quadrature_stations = 120;
    configuration.geometry.rail_cant_radians = 0.0;
    configuration.tangential.longitudinal_cells = 21;
    configuration.tangential.lateral_strips = 21;
    configuration.tangential.refinement_width = 0.01;
    const contact::WheelProfilePreprocessingConfiguration preparation;
    auto right = std::make_unique<contact::WheelRailContactModel>(
        wheel, rail, contact::WheelSide::kRight, preparation, configuration);
    auto left = std::make_unique<contact::WheelRailContactModel>(
        wheel, rail, contact::WheelSide::kLeft, preparation, configuration);
    contact::WheelRailPoseConstants constants;
    constants.nominal_rolling_radius_meters = kRadius;
    return std::make_unique<contact::WheelRailContactRuntimePersonality>(
        std::move(right), std::move(left), constants, constants,
        contact::RailProfileOriginMode::kTrackStation);
}

track::TrackGeometry MakeStraightLine() {
    track::TrackScalarSegment zero;
    zero.length_meters = 100.0;
    zero.shape = track::TrackScalarSegmentShape::kConstant;
    return track::TrackGeometry(
        track::TrackScalarProfile(0.0, {zero}, {}),
        track::TrackScalarProfile(0.0, {zero}, {}),
        track::TrackVerticalProfile(0.0, {track::ConstantGradeSegment{100.0, 0.0}}, {}),
        1.5, 1.0);
}

struct Snapshot {
    double time;
    Eigen::VectorXd physical;
    std::vector<double> hints;
};

Snapshot TakeSnapshot(const SystemInstance& system,
                      const SystemRuntimeContext& context) {
    Snapshot result{context.time_seconds(),
                    Eigen::VectorXd(system.continuous_state_size()), {}};
    system.CopyContinuousState(context, result.physical);
    const auto hints = context.wheel_rail_projection_station_hints_meters();
    result.hints.assign(hints.begin(), hints.end());
    return result;
}

void RequireHints(const SystemRuntimeContext& context,
                  const std::vector<double>& expected) {
    const auto actual = context.wheel_rail_projection_station_hints_meters();
    Require(!actual.empty() && actual.size() == expected.size() &&
                std::equal(actual.begin(), actual.end(), expected.begin()),
            "coordinate contact bridge changed nonempty projection history");
}

void RequireUnchanged(const SystemInstance& system,
                      const SystemRuntimeContext& context,
                      const Snapshot& before) {
    const auto after = TakeSnapshot(system, context);
    Require(after.time == before.time && after.physical == before.physical,
            "coordinate geometry work changed the physical context");
    RequireHints(context, before.hints);
}

void CheckContactIsolation(bool singular_inertia) {
    MultibodyModel model;
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = singular_inertia ? 0.0 : 100.0;
    inertia.unit_inertia_moments = singular_inertia
                                      ? Eigen::Vector3d::Zero()
                                      : Eigen::Vector3d::Constant(0.2);
    const auto wheel = model.AddRigidBody("wheel", inertia);
    inertia.mass_kilograms = 1.0;
    inertia.unit_inertia_moments.setConstant(0.2);
    const auto anchor = model.AddRigidBody("anchor", inertia);
    model.DeclareFreeBody(wheel);
    model.AddWeldJoint("anchor_weld", model.world_frame(), model.body_frame(anchor));
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();

    forces::VehicleForceElementCollection elements;
    elements.series_spring_viscous_dampers = {
        forces::SeriesSpringViscousDamper{
            "small_series", {model.body_frame(wheel)}, {model.body_frame(anchor)},
            forces::ForceElementAxis::kLongitudinal, 8.0, 2.0}};
    const forces::VehicleForcePlan vehicle_plan(model, std::move(elements));
    const forces::WheelRailContactForcePlan contact_plan(
        model, MakeStraightLine(), MakePersonality(), nullptr,
        {forces::WheelRailContactCarrierDefinition{"carrier", "wheel", 8.125}},
        {forces::WheelRailContactInterfaceDefinition{
            "right_contact", "carrier", "wheel", contact::WheelSide::kRight,
            std::nullopt}});
    const SystemAssemblyDescription description(model, vehicle_plan, contact_plan);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);
    auto accepted = system.CreateDefaultRuntimeContext(1.0);
    auto trial = system.CreateDefaultRuntimeContext(2.0);
    auto reference = system.CreateDefaultRuntimeContext(3.0);
    const int nq = model.num_generalized_positions();
    const int nv = model.num_generalized_velocities();
    const auto q_range = model.GetFreeBodyPositionRange(wheel);
    const auto v_range = model.GetFreeBodyVelocityRange(wheel);

    Eigen::VectorXd physical = TakeSnapshot(system, *trial).physical;
    physical.segment<4>(q_range.start()) << 1.7, 0.0, 0.0, 0.0;
    physical.segment<3>(q_range.start() + 4) << 10.25, 0.0, -kRadius - 2.75e-4;
    physical.segment<6>(nq + v_range.start()) << 0.0, -1.0 / kRadius, 0.0,
                                                 1.0, 0.0, 0.0;
    physical[nq + nv] = 3.4;
    const std::array<double, 1> trial_hints{8.125};
    system.SetTimeContinuousStateAndWheelRailProjectionHints(
        *trial, 2.0, physical, trial_hints);
    Eigen::VectorXd accepted_physical = physical;
    accepted_physical[q_range.start() + 4] = 3.0;
    const std::array<double, 1> accepted_hints{2.25};
    system.SetTimeContinuousStateAndWheelRailProjectionHints(
        *accepted, 1.0, accepted_physical, accepted_hints);
    const auto accepted_before = TakeSnapshot(system, *accepted);
    const auto trial_before = TakeSnapshot(system, *trial);
    auto expected_trial_hints = trial_before.hints;

    SystemCoordinateProblem bridge(system, plan, *trial, NoCallTimeAppliedForces{});
    physical[q_range.start() + 4] = 14.5;
    CoordinateState coordinates = bridge.MakeCoordinateState(9.5, physical);
    bridge.ValidateInitialState(coordinates.time_seconds, coordinates.q,
                                 coordinates.s, coordinates.z);
    Eigen::VectorXd round_trip(physical.size());
    bridge.CopyPhysicalState(coordinates, round_trip);
    Require((round_trip - physical).norm() < 1e-13,
            "contact bridge coordinate import/export disagrees with physical state");
    CoordinateState projected = coordinates;
    projected.q.segment<4>(q_range.start()) *= 1.25;
    projected.s.segment<4>(q_range.start()) *= 1.25;
    projected.s.segment<4>(q_range.start()) +=
        0.17 * projected.q.segment<4>(q_range.start());
    Require(bridge.ProjectEndpoint(coordinates.q, projected.q, projected.s),
            "contact geometry fixture did not exercise paired projection");
    bridge.CopyPhysicalState(projected, round_trip);
    Require((round_trip - physical).norm() < 1e-13,
            "paired projection changed the intended physical contact state");
    Eigen::VectorXd observed(bridge.physical_state_size());
    bridge.CopyLinearlyInterpolatedPhysicalState(
        coordinates.q, trial_before.physical, round_trip, 0.37, observed);
    Require(observed.allFinite() &&
                observed[q_range.start() + 4] ==
                    std::lerp(trial_before.physical[q_range.start() + 4],
                              round_trip[q_range.start() + 4], 0.37),
            "contact dense observation did not interpolate physical storage");
    RequireUnchanged(system, *trial, trial_before);
    RequireUnchanged(system, *accepted, accepted_before);

    Eigen::VectorXd b = Eigen::VectorXd::Constant(nq, 73.0);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(1, -29.0);
    if (singular_inertia) {
        std::string failure;
        try {
            bridge.Evaluate(projected.time_seconds, projected.q, projected.s,
                            projected.z, b, g);
        } catch (const std::exception& error) {
            failure = error.what();
        }
        Require(failure.find("positive-definite") != std::string::npos,
                "contact failure fixture did not reach real singular dynamics");
        Require(b == Eigen::VectorXd::Constant(nq, 73.0) && g[0] == -29.0,
                "real contact/dynamics failure partially wrote b or g");
    } else {
        bridge.Evaluate(projected.time_seconds, projected.q, projected.s,
                        projected.z, b, g);
        system.SetTimeContinuousStateAndWheelRailProjectionHints(
            *reference, 3.0, round_trip, trial_hints);
        SystemRhsBridge rhs(system, plan, *reference, NoCallTimeAppliedForces{});
        Eigen::VectorXd expected_rhs(physical.size());
        rhs.CalcTimeDerivatives(projected.time_seconds, round_trip, expected_rhs);
        const auto component = system.GetMultibodyComponentView(
            *reference, system.multibody_component());
        const Eigen::VectorXd acceleration = expected_rhs.segment(nq, nv);
        Eigen::VectorXd expected_b(nq);
        model.MapGeneralizedVelocityDerivativesToPositionSecondDerivatives(
            component.context(), acceleration, &expected_b);
        Require((b - expected_b).norm() < 1e-11 * std::max(1.0, expected_b.norm()) &&
                    std::abs(g[0] - expected_rhs[nq + nv]) < 1e-12,
                "actual contact bridge b/g differs from independent full RHS");
        Require(std::abs(acceleration[v_range.start() + 5]) > 1e-4,
                "synthetic wheel/rail fixture carried no actual vertical contact force");
        Require(trial->time_seconds() == projected.time_seconds,
                "actual contact evaluation did not install trial time");
        const Eigen::VectorXd endpoint_b = b;
        const Eigen::VectorXd endpoint_g = g;
        const auto require_same_endpoint = [&](SystemCoordinateProblem& problem) {
            Eigen::VectorXd next_b(nq), next_g(1);
            problem.Evaluate(projected.time_seconds, projected.q, projected.s,
                             projected.z, next_b, next_g);
            Require(SameBits(endpoint_b, next_b) && SameBits(endpoint_g, next_g),
                    "same-branch contact projection history changed endpoint b/g bits");
        };
        // Repeating the real contact evaluation exercises the populated contact
        // workspace. Updating hints must actually move its station seed.
        require_same_endpoint(bridge);
        const auto before_hint_refresh = TakeSnapshot(system, *trial);
        system.UpdateWheelRailProjectionStationHints(*trial);
        const auto after_hint_refresh = TakeSnapshot(system, *trial);
        Require(after_hint_refresh.hints != before_hint_refresh.hints,
                "contact hint-refresh qualification did not change a real station seed");
        Require(SameBits(after_hint_refresh.physical, before_hint_refresh.physical) &&
                    after_hint_refresh.time == before_hint_refresh.time,
                "contact hint refresh changed physical endpoint state/time");
        expected_trial_hints = after_hint_refresh.hints;
        require_same_endpoint(bridge);
        require_same_endpoint(bridge);
        RequireHints(*trial, expected_trial_hints);

        // A fresh physical workspace with a separately refreshed seed must
        // produce the same endpoint, including the non-unit quaternion path.
        auto cold_context = system.CreateDefaultRuntimeContext(0.0);
        system.SetTimeContinuousStateAndWheelRailProjectionHints(
            *cold_context, projected.time_seconds, round_trip, trial_hints);
        const auto cold_before = TakeSnapshot(system, *cold_context);
        system.UpdateWheelRailProjectionStationHints(*cold_context);
        const auto cold_after = TakeSnapshot(system, *cold_context);
        Require(cold_after.hints != cold_before.hints &&
                    std::abs(cold_after.physical.segment<4>(q_range.start()).norm() - 1.7) < 1e-14,
                "cold hint-refresh qualification lost its changed seed or nonunit quaternion");
        SystemCoordinateProblem cold_bridge(system, plan, *cold_context,
                                            NoCallTimeAppliedForces{});
        require_same_endpoint(cold_bridge);
        require_same_endpoint(cold_bridge);
        RequireHints(*cold_context, cold_after.hints);
        const auto trial_after_success = TakeSnapshot(system, *trial);
        CoordinateState invalid = projected;
        invalid.q.segment<4>(q_range.start()).setZero();
        b.setConstant(73.0);
        g.setConstant(-29.0);
        bool refused = false;
        try {
            bridge.Evaluate(invalid.time_seconds, invalid.q, invalid.s,
                            invalid.z, b, g);
        } catch (const std::invalid_argument&) {
            refused = true;
        }
        Require(refused && b == Eigen::VectorXd::Constant(nq, 73.0) && g[0] == -29.0,
                "invalid contact trial partially wrote b/g or was not refused");
        RequireUnchanged(system, *trial, trial_after_success);
    }
    RequireUnchanged(system, *accepted, accepted_before);
    RequireHints(*trial, expected_trial_hints);
}

void CheckHeldTorqueSynchronization(bool use_newmark) {
    using namespace orvd::integrators;
    MultibodyModel model;
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 100.0;
    inertia.unit_inertia_moments.setConstant(0.2);
    const auto wheel = model.AddRigidBody("wheel", inertia);
    const auto axis_provider = model.AddRigidBody("axis_provider", inertia);
    const auto reaction = model.AddRigidBody("reaction", inertia);
    model.DeclareFreeBody(wheel);
    model.AddWeldJoint("fixed_axis", model.world_frame(), model.body_frame(axis_provider));
    model.AddWeldJoint("fixed_reaction", model.world_frame(), model.body_frame(reaction));
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();
    const forces::VehicleForcePlan vehicle_plan(model, {});
    const forces::WheelRailContactForcePlan contact_plan(
        model, MakeStraightLine(), MakePersonality(), nullptr,
        {forces::WheelRailContactCarrierDefinition{"carrier", "wheel", 8.125}},
        {forces::WheelRailContactInterfaceDefinition{
            "right_contact", "carrier", "wheel", contact::WheelSide::kRight,
            std::nullopt}});
    const forces::IndependentWheelActiveTorquePlan torque_plan(
        model, {{"held_spin", "axis_provider", "wheel", "reaction"}});
    const SystemAssemblyDescription description(model, vehicle_plan, contact_plan, torque_plan);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);
    auto accepted = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd initial = TakeSnapshot(system, *accepted).physical;
    const auto qr = model.GetFreeBodyPositionRange(wheel);
    const auto vr = model.GetFreeBodyVelocityRange(wheel);
    const int physical_velocity = model.num_generalized_positions() + vr.start();
    initial.segment<4>(qr.start()) << 1.7, 0.0, 0.0, 0.0;
    // Keep the wheel clear of the rail: the actual contact plan still projects
    // its moving station, while the isolated spin response is analytically known.
    initial.segment<3>(qr.start() + 4) << 14.5, 0.0, -2.0 * kRadius;
    initial[physical_velocity + 3] = 1.0;
    const std::array<double, 1> initial_hints{8.125};
    system.SetTimeContinuousStateAndWheelRailProjectionHints(
        *accepted, 0.0, initial, initial_hints);
    constexpr double h = 1.0 / 1024.0;
    SystemIntegrationConfiguration configuration{ZhaiConfiguration{h}};
    if (use_newmark) configuration.method = orvd::integrators::test::NewmarkSettings(h);
    auto advancer = std::make_unique<SystemContinuousStateAdvancer>(
        system, plan, *accepted, std::move(configuration), NoCallTimeAppliedForces{});
    const std::array<double, 1> held_torque{20.0};
    system.SetHeldIndependentWheelActiveTorques(*accepted, held_torque);
    // Merely changing accepted holds must not change the backend's frozen input.
    advancer->AdvanceTo(4.0 * h);
    const auto before_sync = TakeSnapshot(system, *accepted);
    Require(before_sync.physical.segment<3>(physical_velocity).norm() < 1e-13,
            "unsynchronized accepted wheel torque leaked into the mechanical backend");
    Require(before_sync.hints != std::vector<double>(initial_hints.begin(), initial_hints.end()),
            "mechanical factory run did not refresh real nonempty projection history");
    Require(accepted->held_independent_wheel_active_torques_newton_metres()[0] == 20.0,
            "successful state publication overwrote accepted held wheel torque");
    Require(advancer->integration_statistics().successful_internal_step_count == 4,
            "station updates preserve successful-step accounting");

    advancer->SynchronizeAfterAcceptedContextChange();
    Require(advancer->integration_statistics().successful_internal_step_count == 0 &&
                advancer->integration_statistics().right_hand_side_evaluation_count == 1,
            "held-torque synchronization rebuilds initial derivatives");
    const auto synced = TakeSnapshot(system, *accepted);
    Require(SameBits(synced.physical, before_sync.physical) &&
                synced.time == before_sync.time && synced.hints == before_sync.hints,
            "backend torque synchronization changed accepted state/time/projection history");
    advancer->AdvanceTo(8.0 * h);
    const auto driven = TakeSnapshot(system, *accepted);
    // Iyy = mass * unit moment = 20 kg m², so tau/Iyy = 1 rad/s².
    Require(std::abs(driven.physical[physical_velocity + 1] - 4.0 * h) < 1e-9 &&
                std::abs(driven.physical[physical_velocity]) < 1e-13 &&
                std::abs(driven.physical[physical_velocity + 2]) < 1e-13,
            "synchronized held wheel torque did not produce its real multibody spin response");
    Require(advancer->integration_statistics().successful_internal_step_count == 4,
            "post-synchronization steps retain correct accounting");
    Require(accepted->held_independent_wheel_active_torques_newton_metres()[0] == 20.0,
            "mechanical trials or accepted publication changed the held active torque");
}

}  // namespace

void VerifySystemCoordinateProblemContactIsolation() {
    CheckContactIsolation(false);
    CheckContactIsolation(true);
}

void VerifyBasicSystemHeldTorqueSynchronization() {
    CheckHeldTorqueSynchronization(false);
    CheckHeldTorqueSynchronization(true);
}
