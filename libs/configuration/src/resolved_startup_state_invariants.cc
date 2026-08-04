#include "resolved_startup_state_invariants.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace orvd::configuration::internal {
namespace {

// A unit quaternion is what makes a rotation. The tolerance is a few rounding
// steps of the four-term norm, not a repair budget: a record outside it is
// refused rather than normalised, because normalising answers a different
// question from the one the author asked.
constexpr double kUnitQuaternionNormTolerance = 1.0e-12;

std::string Describe(double value) { return std::to_string(value); }

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("resolved start-up state: " + detail);
}

void RequireFinite(double value, const std::string& what) {
    if (!std::isfinite(value)) {
        Reject(what + " is " + Describe(value) + ", which is not finite");
    }
}

void RequireFiniteVector(const Eigen::Vector3d& value,
                         const std::string& what) {
    if (!value.allFinite()) {
        Reject(what + " has a component that is not finite");
    }
}

void RequireDistinctNames(const std::vector<std::string>& names,
                          const std::string& what) {
    std::unordered_set<std::string> seen;
    for (const std::string& name : names) {
        if (name.empty()) {
            Reject(what + " contains an empty name");
        }
        if (!seen.insert(name).second) {
            Reject(what + " names '" + name + "' more than once");
        }
    }
}

}  // namespace

void RequireResolvedStartupStateInvariants(const ResolvedStartupState& state,
                                           const std::string& origin) {
    const auto field = [&origin](const std::string& name) {
        return origin + "." + name;
    };

    if (state.vehicle_binding.vehicle_name.empty()) {
        Reject(field("vehicle_binding.vehicle_name") + " is empty");
    }
    if (state.vehicle_binding.mechanical_definition_identifier.empty()) {
        Reject(field("vehicle_binding.mechanical_definition_identifier") +
               " is empty");
    }
    if (state.wheel_rail_binding.wheel_profile_identifier.empty()) {
        Reject(field("wheel_rail_binding.wheel_profile_identifier") +
               " is empty");
    }
    if (state.wheel_rail_binding.rail_profile_identifier.empty()) {
        Reject(field("wheel_rail_binding.rail_profile_identifier") +
               " is empty");
    }
    if (state.wheel_rail_binding.wheel_rail_contact_strategy_identifier
            .empty()) {
        Reject(field(
                   "wheel_rail_binding.wheel_rail_contact_strategy_identifier") +
               " is empty");
    }
    if (state.load_condition_identifier.empty()) {
        Reject(field("load_condition_identifier") + " is empty");
    }

    const std::string gravity_field =
        field("gravitational_acceleration_meters_per_second_squared");
    RequireFinite(state.gravitational_acceleration_meters_per_second_squared,
                  gravity_field);
    if (!(state.gravitational_acceleration_meters_per_second_squared > 0.0)) {
        Reject(gravity_field + " is " +
               Describe(
                   state.gravitational_acceleration_meters_per_second_squared) +
               " m/s^2, but the gravity a state was resolved under is "
               "strictly positive");
    }

    const std::string speed_field =
        field("initial_longitudinal_speed_meters_per_second");
    RequireFinite(state.initial_longitudinal_speed_meters_per_second,
                  speed_field);
    if (!(state.initial_longitudinal_speed_meters_per_second > 0.0)) {
        Reject(speed_field + " is " +
               Describe(state.initial_longitudinal_speed_meters_per_second) +
               " m/s; this is a magnitude, and the direction it runs in is "
               "stated by running_direction, so zero and negative speeds are "
               "refused rather than reinterpreted");
    }

    RequireFinite(state.rail_profile_reference_vertical_offset_meters,
                  field("rail_profile_reference_vertical_offset_meters"));

    std::vector<std::string> names;
    names.reserve(state.target_wheel_support_forces.size());
    for (const WheelsetTargetSupportForces& forces :
         state.target_wheel_support_forces) {
        const std::string what =
            field("target_wheel_support_forces['" + forces.wheelset_body_name +
                  "']");
        RequireFinite(forces.left_support_force_newtons, what + ".left");
        RequireFinite(forces.right_support_force_newtons, what + ".right");
        if (!(forces.left_support_force_newtons > 0.0) ||
            !(forces.right_support_force_newtons > 0.0)) {
            Reject(what +
                   " states a support force that is not strictly positive; "
                   "these are magnitudes of compressive support along -z_T, "
                   "and a wheel that carries nothing is not a resolved "
                   "start-up state");
        }
        names.push_back(forces.wheelset_body_name);
    }
    RequireDistinctNames(names, field("target_wheel_support_forces"));

    names.clear();
    for (const FreeBodyStartupState& body : state.free_body_startup_states) {
        const std::string what =
            field("free_body_startup_states['" + body.body_name + "']");
        const double norm = body.rotation_local_track_from_body.norm();
        RequireFinite(norm, what + ".rotation_local_track_from_body norm");
        if (std::abs(norm - 1.0) > kUnitQuaternionNormTolerance) {
            Reject(what + ".rotation_local_track_from_body has norm " +
                   Describe(norm) +
                   ", not one; it is refused rather than normalised, because "
                   "normalising would answer a different question from the "
                   "one the record asks");
        }
        RequireFinite(
            body.resolved_track_station_offset_from_mechanical_layout_meters,
            what + ".resolved_track_station_offset_from_mechanical_layout_"
                   "meters");
        RequireFinite(body.lateral_offset_in_local_track_frame_meters,
                      what + ".lateral_offset_in_local_track_frame_meters");
        RequireFinite(body.vertical_offset_in_local_track_frame_meters,
                      what + ".vertical_offset_in_local_track_frame_meters");
        RequireFiniteVector(
            body
                .body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second,
            what + ".body_angular_velocity_in_inertial_expressed_in_body_"
                   "frame_radians_per_second");
        RequireFiniteVector(
            body
                .body_origin_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second,
            what + ".body_origin_velocity_in_inertial_expressed_in_local_"
                   "track_frame_meters_per_second");
        names.push_back(body.body_name);
    }
    RequireDistinctNames(names, field("free_body_startup_states"));

    names.clear();
    for (const RevoluteJointStartupState& joint :
         state.revolute_joint_startup_states) {
        const std::string what =
            field("revolute_joint_startup_states['" + joint.joint_name + "']");
        RequireFinite(joint.position_radians, what + ".position_radians");
        RequireFinite(joint.rate_radians_per_second,
                      what + ".rate_radians_per_second");
        names.push_back(joint.joint_name);
    }
    RequireDistinctNames(names, field("revolute_joint_startup_states"));

    names.clear();
    for (const SeriesSpringViscousDamperForceState& element :
         state.series_spring_viscous_damper_force_states) {
        RequireFinite(element.reference_end_axial_force_newtons,
                      field("series_spring_viscous_damper_force_states['" +
                            element.element_name +
                            "'].reference_end_axial_force_newtons"));
        names.push_back(element.element_name);
    }
    RequireDistinctNames(names,
                         field("series_spring_viscous_damper_force_states"));

    names.clear();
    for (const TranslationalSpringDamperNominalForce& element :
         state.translational_spring_damper_nominal_forces) {
        RequireFiniteVector(
            element.force_on_reference_end_in_reference_frame_newtons,
            field("translational_spring_damper_nominal_forces['" +
                  element.element_name +
                  "'].force_on_reference_end_in_reference_frame_newtons"));
        names.push_back(element.element_name);
    }
    RequireDistinctNames(
        names, field("translational_spring_damper_nominal_forces"));
}

}  // namespace orvd::configuration::internal
