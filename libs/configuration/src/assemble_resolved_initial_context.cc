#include "orvd/configuration/assemble_resolved_initial_context.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "resolved_startup_state_invariants.h"

namespace orvd::configuration {
namespace {

using multibody_model::MultibodyModel;
using system_assembly::SystemInstance;
using system_assembly::SystemRuntimeContext;

std::string Describe(double value) { return std::to_string(value); }

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("resolved start-up assembly: " + detail);
}

// Every name family the record states must be exactly the family the assembled
// system has: not a subset, not a superset. A missing entry is the dangerous
// direction — the slot it would have filled keeps its default and nothing else
// notices — so it is named as plainly as an unknown one.
void RequireSameNames(const std::vector<std::string>& stated,
                      const std::vector<std::string>& expected,
                      const std::string& what) {
    const std::unordered_set<std::string> stated_set(stated.begin(),
                                                     stated.end());
    const std::unordered_set<std::string> expected_set(expected.begin(),
                                                       expected.end());
    for (const std::string& name : expected) {
        if (!stated_set.contains(name)) {
            Reject("the record states no " + what + " for '" + name +
                   "', which this vehicle has; a start-up state must state "
                   "every one of them, because an omitted entry would leave "
                   "its slot at a default nothing else would question");
        }
    }
    for (const std::string& name : stated) {
        if (!expected_set.contains(name)) {
            Reject("the record states a " + what + " for '" + name +
                   "', which this vehicle does not have");
        }
    }
}

std::vector<std::string> NamesOfFreeBodies(const ResolvedStartupState& state) {
    std::vector<std::string> names;
    names.reserve(state.free_body_startup_states.size());
    for (const FreeBodyStartupState& body : state.free_body_startup_states) {
        names.push_back(body.body_name);
    }
    return names;
}

// One block of the whole-model coordinate vectors, claimed by one element.
struct ClaimedRange {
    int start{0};
    int size{0};
    std::string owner;
};

// The claimed ranges must tile the vector: no coordinate claimed twice, none
// left unclaimed.
//
// For a vehicle whose every free body and every coordinate-carrying joint owns
// exactly one range, this follows from the name checks above, and no record can
// make it fail — it is a structural guard, not a gate a mutation of the record
// could trip. It stays because the implication runs through the model's
// arrangement, not through anything stated here: an element that ever owned two
// ranges, or a family whose ranges were not disjoint, would be caught here and
// nowhere else.
void RequireExactCover(std::vector<ClaimedRange> claims, int total,
                       const std::string& what) {
    std::sort(claims.begin(), claims.end(),
              [](const ClaimedRange& left, const ClaimedRange& right) {
                  return left.start < right.start;
              });
    int next = 0;
    for (const ClaimedRange& claim : claims) {
        if (claim.size == 0) {
            continue;
        }
        if (claim.start < next) {
            Reject("'" + claim.owner + "' claims " + what + " from index " +
                   std::to_string(claim.start) +
                   ", which another element already claims");
        }
        if (claim.start > next) {
            Reject(what + " index " + std::to_string(next) +
                   " is claimed by no element of this start-up state, so a "
                   "coordinate would keep its default value");
        }
        next = claim.start + claim.size;
    }
    if (next != total) {
        Reject(what + " has " + std::to_string(total) +
               " entries but the start-up state's named entities cover only " +
               std::to_string(next));
    }
}

void RequireStationInsideLine(double station, const std::string& body_name,
                              const track_geometry::TrackGeometry& line) {
    if (!std::isfinite(station)) {
        Reject("the station of '" + body_name + "' is " + Describe(station) +
               ", which is not finite");
    }
    if (station < line.start_track_station_meters() ||
        station > line.end_track_station_meters()) {
        Reject("'" + body_name + "' would start at station " +
               Describe(station) + " m, outside this line's domain [" +
               Describe(line.start_track_station_meters()) + ", " +
               Describe(line.end_track_station_meters()) +
               "] m; the whole vehicle has to stand on the line it starts on");
    }
}

}  // namespace

ResolvedInitialContext::ResolvedInitialContext(
    std::unique_ptr<SystemRuntimeContext> context,
    std::vector<ResolvedWheelsetPlacement> wheelset_placements,
    double rail_profile_reference_vertical_offset_meters,
    StartupWheelRailBinding wheel_rail_binding,
    std::string load_condition_identifier)
    : context_(std::move(context)),
      wheelset_placements_(std::move(wheelset_placements)),
      rail_profile_reference_vertical_offset_meters_(
          rail_profile_reference_vertical_offset_meters),
      wheel_rail_binding_(std::move(wheel_rail_binding)),
      load_condition_identifier_(std::move(load_condition_identifier)) {}

ResolvedInitialContext::ResolvedInitialContext(
    ResolvedInitialContext&&) noexcept = default;
ResolvedInitialContext& ResolvedInitialContext::operator=(
    ResolvedInitialContext&&) noexcept = default;
ResolvedInitialContext::~ResolvedInitialContext() = default;

ResolvedInitialContext AssembleResolvedInitialContext(
    const AssembledVehicleSystem& system,
    const ResolvedStartupState& startup_state,
    const track_geometry::TrackGeometry& line,
    double vehicle_layout_reference_track_station_meters) {
    // 1. The record's own invariants. A malformed value gets its own
    // diagnostic here rather than a misleading identity mismatch below.
    internal::RequireResolvedStartupStateInvariants(startup_state,
                                                    "resolved start-up state");
    if (!std::isfinite(vehicle_layout_reference_track_station_meters)) {
        Reject("the vehicle layout reference track station is " +
               Describe(vehicle_layout_reference_track_station_meters) +
               ", which is not finite");
    }

    // 2. Identity, including the gravity this state was resolved under.
    const VehicleBinding& binding = system.binding();
    if (startup_state.vehicle_binding.vehicle_name != binding.vehicle_name) {
        Reject("the record was resolved for vehicle '" +
               startup_state.vehicle_binding.vehicle_name +
               "', but this system was assembled from '" + binding.vehicle_name +
               "'");
    }
    if (startup_state.vehicle_binding.mechanical_definition_identifier !=
        binding.mechanical_definition_identifier) {
        Reject("the record was resolved for mechanical definition '" +
               startup_state.vehicle_binding.mechanical_definition_identifier +
               "', but this system was assembled from '" +
               binding.mechanical_definition_identifier +
               "'; identifier equality is what binds a resolved state to a "
               "mechanical definition, and keeping the identifier current is "
               "the caller's responsibility");
    }
    const MultibodyModel& model = system.model();
    const Eigen::Vector3d expected_gravity(
        0.0, 0.0,
        startup_state.gravitational_acceleration_meters_per_second_squared);
    if (model.gravity_vector() != expected_gravity) {
        Reject(
            "the record was resolved under gravity " +
            Describe(
                startup_state
                    .gravitational_acceleration_meters_per_second_squared) +
            " m/s^2 along +z, but this system was assembled with (" +
            Describe(model.gravity_vector().x()) + ", " +
            Describe(model.gravity_vector().y()) + ", " +
            Describe(model.gravity_vector().z()) +
            ") m/s^2; every preload, every target wheel load and the rail "
            "offset that produces them follow from that value, so this is "
            "compared exactly rather than within a tolerance");
    }

    // 3. Every name family, all and only.
    std::vector<std::string> stated;
    stated.reserve(startup_state.target_wheel_support_forces.size());
    for (const WheelsetTargetSupportForces& forces :
         startup_state.target_wheel_support_forces) {
        stated.push_back(forces.wheelset_body_name);
    }
    RequireSameNames(stated, binding.wheelset_body_names,
                     "target wheel support force");

    stated.clear();
    for (const WheelsetStartupKinematics& wheelset :
         startup_state.wheelset_startup_kinematics) {
        stated.push_back(wheelset.wheelset_body_name);
    }
    RequireSameNames(stated, binding.wheelset_body_names,
                     "wheelset start-up kinematics");

    std::vector<std::string> free_body_names;
    for (const VehicleFreeBodyStationOffset& offset :
         binding.free_body_station_offsets) {
        free_body_names.push_back(offset.body_name);
    }
    RequireSameNames(NamesOfFreeBodies(startup_state), free_body_names,
                     "free-body start-up state");

    stated.clear();
    for (const RevoluteJointStartupState& joint :
         startup_state.revolute_joint_startup_states) {
        stated.push_back(joint.joint_name);
    }
    RequireSameNames(stated, binding.generalized_coordinate_joint_names,
                     "joint start-up state");

    stated.clear();
    for (const SeriesSpringViscousDamperForceState& element :
         startup_state.series_spring_viscous_damper_force_states) {
        stated.push_back(element.element_name);
    }
    RequireSameNames(stated, binding.series_spring_viscous_damper_names,
                     "series force state");

    stated.clear();
    for (const TranslationalSpringDamperNominalForce& element :
         startup_state.translational_spring_damper_nominal_forces) {
        stated.push_back(element.element_name);
    }
    RequireSameNames(stated, binding.translational_spring_damper_names,
                     "nominal force");

    // 4. Where each free body actually ends up.
    std::unordered_map<std::string, double> mechanical_offset_of_body;
    for (const VehicleFreeBodyStationOffset& offset :
         binding.free_body_station_offsets) {
        mechanical_offset_of_body.emplace(offset.body_name,
                                          offset.station_offset_meters);
    }
    std::unordered_map<std::string, double> station_of_body;
    for (const FreeBodyStartupState& body :
         startup_state.free_body_startup_states) {
        const double station =
            vehicle_layout_reference_track_station_meters +
            mechanical_offset_of_body.at(body.body_name) +
            body.resolved_track_station_offset_from_mechanical_layout_meters;
        RequireStationInsideLine(station, body.body_name, line);
        station_of_body.emplace(body.body_name, station);
    }

    // 5. Expand the two shared GZ18 start-up authorities before touching a
    // context. The record remains editable C++, so arithmetic overflow is
    // rejected here even when no JSON loader was involved.
    const double common_wheel_spin_magnitude =
        startup_state.initial_longitudinal_speed_meters_per_second /
        startup_state.common_startup_effective_rolling_radius_meters;
    if (!std::isfinite(common_wheel_spin_magnitude)) {
        Reject("initial_longitudinal_speed_meters_per_second divided by "
               "common_startup_effective_rolling_radius_meters is not "
               "finite");
    }
    std::unordered_map<std::string, Eigen::Vector3d>
        wheelset_spin_of_body;
    for (const WheelsetStartupKinematics& wheelset :
         startup_state.wheelset_startup_kinematics) {
        const Eigen::Vector3d spin =
            wheelset.forward_spin_axis_in_body_frame *
            common_wheel_spin_magnitude;
        if (!spin.allFinite()) {
            Reject("the generated spin of wheelset '" +
                   wheelset.wheelset_body_name + "' is not finite");
        }
        wheelset_spin_of_body.emplace(wheelset.wheelset_body_name, spin);
    }

    // 6. Names to handles, indices and ranges. Nothing below indexes by
    // declaration order: the model's arrangement is its own business.
    const SystemInstance& instance = system.system();
    const int position_count = model.num_generalized_positions();
    const int velocity_count = model.num_generalized_velocities();

    // A coordinate range can only come from the model, so these are built by
    // aggregate initialisation rather than declared and then filled in.
    struct PlacedBody {
        const FreeBodyStartupState* state;
        multibody_model::GeneralizedPositionRange positions;
        multibody_model::GeneralizedVelocityRange velocities;
        double station_meters;
    };
    std::vector<PlacedBody> placed_bodies;
    std::vector<ClaimedRange> position_claims;
    std::vector<ClaimedRange> velocity_claims;
    placed_bodies.reserve(startup_state.free_body_startup_states.size());
    for (const FreeBodyStartupState& body :
         startup_state.free_body_startup_states) {
        const multibody_model::RigidBodyHandle handle =
            model.GetRigidBodyByName(body.body_name);
        if (!model.IsFreeBody(handle)) {
            Reject("'" + body.body_name +
                   "' has a free-body start-up state, but this model does not "
                   "declare it free; its coordinates belong to the joints that "
                   "relate it");
        }
        const PlacedBody placed{&body, model.GetFreeBodyPositionRange(handle),
                                model.GetFreeBodyVelocityRange(handle),
                                station_of_body.at(body.body_name)};
        position_claims.push_back(ClaimedRange{
            placed.positions.start(), placed.positions.size(), body.body_name});
        velocity_claims.push_back(
            ClaimedRange{placed.velocities.start(), placed.velocities.size(),
                         body.body_name});
        placed_bodies.push_back(placed);
    }

    struct PlacedJoint {
        const RevoluteJointStartupState* state;
        multibody_model::GeneralizedPositionRange positions;
        multibody_model::GeneralizedVelocityRange velocities;
    };
    std::vector<PlacedJoint> placed_joints;
    placed_joints.reserve(startup_state.revolute_joint_startup_states.size());
    for (const RevoluteJointStartupState& joint :
         startup_state.revolute_joint_startup_states) {
        const multibody_model::JointHandle handle =
            model.GetJointByName(joint.joint_name);
        const PlacedJoint placed{&joint, model.GetJointPositionRange(handle),
                                 model.GetJointVelocityRange(handle)};
        if (placed.positions.size() != 1 || placed.velocities.size() != 1) {
            Reject("'" + joint.joint_name + "' has " +
                   std::to_string(placed.positions.size()) +
                   " generalized positions and " +
                   std::to_string(placed.velocities.size()) +
                   " generalized velocities, but a revolute start-up state "
                   "states one of each");
        }
        position_claims.push_back(ClaimedRange{placed.positions.start(),
                                               placed.positions.size(),
                                               joint.joint_name});
        velocity_claims.push_back(ClaimedRange{placed.velocities.start(),
                                               placed.velocities.size(),
                                               joint.joint_name});
        placed_joints.push_back(placed);
    }

    struct PlacedSeriesForce {
        const SeriesSpringViscousDamperForceState* state;
        system_assembly::SystemContinuousStateRange range;
    };
    std::vector<PlacedSeriesForce> placed_series;
    std::vector<ClaimedRange> force_state_claims;
    const int position_and_velocity_count = position_count + velocity_count;
    const int force_state_count =
        instance.continuous_state_size() - position_and_velocity_count;
    placed_series.reserve(
        startup_state.series_spring_viscous_damper_force_states.size());
    for (const SeriesSpringViscousDamperForceState& element :
         startup_state.series_spring_viscous_damper_force_states) {
        const system_assembly::SeriesSpringViscousDamperIndex index =
            instance.GetSeriesSpringViscousDamperIndexByName(
                element.element_name);
        const system_assembly::SystemContinuousStateRange range =
            instance.series_spring_damper_force_state_range(index);
        force_state_claims.push_back(
            ClaimedRange{range.start() - position_and_velocity_count,
                         range.size(), element.element_name});
        placed_series.push_back(PlacedSeriesForce{&element, range});
    }

    std::vector<
        std::pair<system_assembly::TranslationalSpringDamperIndex,
                  Eigen::Vector3d>>
        resolved_nominal_forces;
    resolved_nominal_forces.reserve(
        startup_state.translational_spring_damper_nominal_forces.size());
    for (const TranslationalSpringDamperNominalForce& element :
         startup_state.translational_spring_damper_nominal_forces) {
        resolved_nominal_forces.emplace_back(
            instance.GetTranslationalSpringDamperIndexByName(
                element.element_name),
            element.force_on_reference_end_in_reference_frame_newtons);
    }

    // 7. The three blocks must be covered exactly, with nothing claimed twice
    // and nothing left at a default.
    RequireExactCover(position_claims, position_count, "generalized position");
    RequireExactCover(velocity_claims, velocity_count, "generalized velocity");
    RequireExactCover(force_state_claims, force_state_count,
                      "series force state");

    // 8. The whole continuous state, built before any of it is committed.
    Eigen::VectorXd continuous_state =
        Eigen::VectorXd::Zero(instance.continuous_state_size());
    for (const PlacedBody& placed : placed_bodies) {
        // The complete formula is used on every line. The bundled GZ18 demo is
        // qualified only on straight, level, zero-superelevation track, but a
        // researcher may deliberately place the same input on another line;
        // this layer does not prohibit or silently simplify that experiment.
        const track_geometry::TrackFramePose pose =
            line.EvaluateTrackFrame(placed.station_meters).pose();
        const Eigen::Quaterniond rotation_inertial_from_track(
            pose.rotation_inertial_from_track());
        const Eigen::Quaterniond rotation_inertial_from_body =
            rotation_inertial_from_track *
            placed.state->rotation_local_track_from_body;
        const Eigen::Vector3d offset_in_track(
            0.0, placed.state->lateral_offset_in_local_track_frame_meters,
            placed.state->vertical_offset_in_local_track_frame_meters);
        const Eigen::Vector3d origin_in_inertial =
            pose.origin_in_inertial_meters() +
            pose.rotation_inertial_from_track() * offset_in_track;

        const int start = placed.positions.start();
        continuous_state[start + 0] = rotation_inertial_from_body.w();
        continuous_state[start + 1] = rotation_inertial_from_body.x();
        continuous_state[start + 2] = rotation_inertial_from_body.y();
        continuous_state[start + 3] = rotation_inertial_from_body.z();
        continuous_state.segment<3>(start + 4) = origin_in_inertial;

        // Both stated velocities are inertial velocities; only the basis they
        // are written in differs, so this is a change of basis and nothing
        // else. No transport term of the track frame enters.
        Eigen::Vector3d angular_velocity_in_body =
            placed.state
                ->additional_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second;
        const auto wheelset_spin =
            wheelset_spin_of_body.find(placed.state->body_name);
        if (wheelset_spin != wheelset_spin_of_body.end()) {
            angular_velocity_in_body += wheelset_spin->second;
        }
        if (!angular_velocity_in_body.allFinite()) {
            Reject("the expanded angular velocity of '" +
                   placed.state->body_name + "' is not finite");
        }
        const Eigen::Vector3d angular_velocity_in_inertial =
            rotation_inertial_from_body * angular_velocity_in_body;
        const Eigen::Vector3d origin_velocity_in_track(
            startup_state.initial_longitudinal_speed_meters_per_second,
            placed.state
                ->body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second,
            placed.state
                ->body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second);
        const Eigen::Vector3d origin_velocity_in_inertial =
            pose.rotation_inertial_from_track() *
            origin_velocity_in_track;
        const int velocity_start =
            position_count + placed.velocities.start();
        continuous_state.segment<3>(velocity_start) =
            angular_velocity_in_inertial;
        continuous_state.segment<3>(velocity_start + 3) =
            origin_velocity_in_inertial;
    }
    for (const PlacedJoint& placed : placed_joints) {
        const double rate =
            placed.state->rate_per_common_wheel_spin_magnitude *
            common_wheel_spin_magnitude;
        if (!std::isfinite(rate)) {
            Reject("the generated rate of joint '" +
                   placed.state->joint_name + "' is not finite");
        }
        continuous_state[placed.positions.start()] =
            placed.state->position_radians;
        continuous_state[position_count + placed.velocities.start()] =
            rate;
    }
    for (const PlacedSeriesForce& placed : placed_series) {
        continuous_state[placed.range.start()] =
            placed.state->reference_end_axial_force_newtons;
    }

    // 9-11. A context the caller has never seen, one state transaction, and
    // the already-resolved nominal forces. Every way this could fail was
    // exhausted above, so the writes below cannot leave a partial state; if one
    // did, the context would be destroyed with the exception and never
    // published.
    std::unique_ptr<SystemRuntimeContext> context =
        instance.CreateDefaultRuntimeContext(0.0);
    instance.SetTimeAndContinuousState(*context, 0.0, continuous_state);
    for (const auto& [index, force] : resolved_nominal_forces) {
        instance.SetNominalForce(*context, index, force);
    }

    // 12. The metadata this assembly derived, beside the context it belongs
    // with.
    std::vector<ResolvedWheelsetPlacement> placements;
    placements.reserve(startup_state.target_wheel_support_forces.size());
    for (const WheelsetTargetSupportForces& forces :
         startup_state.target_wheel_support_forces) {
        placements.push_back(ResolvedWheelsetPlacement{
            forces.wheelset_body_name, station_of_body.at(
                                           forces.wheelset_body_name),
            forces.left_support_force_newtons,
            forces.right_support_force_newtons});
    }

    return ResolvedInitialContext(
        std::move(context), std::move(placements),
        startup_state.rail_profile_reference_vertical_offset_meters,
        startup_state.wheel_rail_binding,
        startup_state.load_condition_identifier);
}

ResolvedInitialContext AssembleResolvedInitialContext(
    const AssembledVehicleSystem& system,
    const ResolvedStartupState& startup_state,
    double vehicle_layout_reference_track_station_meters) {
    const forces::WheelRailContactForcePlan* contact_plan =
        system.contact_force_plan();
    if (contact_plan == nullptr) {
        Reject("the assembled system has no wheel-rail contact plan and thus "
               "owns no line to place this start-up state on");
    }
    return AssembleResolvedInitialContext(
        system, startup_state, contact_plan->track_geometry(),
        vehicle_layout_reference_track_station_meters);
}

}  // namespace orvd::configuration
