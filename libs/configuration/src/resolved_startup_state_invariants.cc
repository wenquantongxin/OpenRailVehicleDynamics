#include "resolved_startup_state_invariants.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
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

void RequireIdentifier(std::string_view identifier, const std::string& what) {
    if (identifier.empty()) {
        Reject(what + " is empty");
    }
    for (const char character : identifier) {
        const bool admitted = (character >= 'A' && character <= 'Z') ||
                              (character >= 'a' && character <= 'z') ||
                              (character >= '0' && character <= '9') ||
                              character == '.' || character == '_' ||
                              character == '-';
        if (!admitted) {
            Reject(what + " is '" + std::string(identifier) +
                   "', which contains a character outside [A-Za-z0-9._-]");
        }
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
    RequireIdentifier(
        state.vehicle_binding.mechanical_definition_identifier,
        field("vehicle_binding.mechanical_definition_identifier"));
    RequireIdentifier(state.wheel_rail_binding.wheel_profile_identifier,
                      field("wheel_rail_binding.wheel_profile_identifier"));
    RequireIdentifier(state.wheel_rail_binding.rail_profile_identifier,
                      field("wheel_rail_binding.rail_profile_identifier"));
    RequireIdentifier(
        state.wheel_rail_binding.wheel_rail_contact_strategy_identifier,
        field("wheel_rail_binding.wheel_rail_contact_strategy_identifier"));
    RequireIdentifier(state.load_condition_identifier,
                      field("load_condition_identifier"));

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

    switch (state.running_direction) {
        case StartupRunningDirection::kIncreasingTrackStation:
            break;
        default:
            Reject(field("running_direction") +
                   " is not a supported running-direction value");
    }

    if (state.common_wheel_spin_generation.has_value()) {
        const std::string radius_field =
            field("common_wheel_spin_generation."
                  "effective_rolling_radius_meters");
        const double radius = state.common_wheel_spin_generation
                                  ->effective_rolling_radius_meters;
        RequireFinite(radius, radius_field);
        if (!(radius > 0.0)) {
            Reject(radius_field + " is " + Describe(radius) +
                   " m, but the effective rolling radius must be strictly "
                   "positive");
        }
    }

    RequireFinite(state.rail_profile_reference_vertical_offset_meters,
                  field("rail_profile_reference_vertical_offset_meters"));

    std::vector<std::string> names;
    names.reserve(state.wheel_pair_target_support_forces.size());
    for (const WheelPairTargetSupportForces& forces :
         state.wheel_pair_target_support_forces) {
        const std::string what =
            field("wheel_pair_target_support_forces['" +
                  forces.station_reference_body_name + "']");
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
        names.push_back(forces.station_reference_body_name);
    }
    RequireDistinctNames(names,
                         field("wheel_pair_target_support_forces"));

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
                .explicit_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second,
            what + ".explicit_body_angular_velocity_in_inertial_expressed_in_body_"
                   "frame_radians_per_second");
        RequireFiniteVector(body.common_wheel_spin_coefficient_in_body_frame,
                            what +
                                ".common_wheel_spin_coefficient_in_body_frame");
        if (!state.common_wheel_spin_generation.has_value() &&
            body.common_wheel_spin_coefficient_in_body_frame !=
                Eigen::Vector3d::Zero()) {
            Reject(what +
                   ".common_wheel_spin_coefficient_in_body_frame is nonzero, "
                   "but this record has no common wheel-spin generation");
        }
        RequireFinite(
            body
                .body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second,
            what + ".body_origin_lateral_velocity_in_inertial_expressed_in_"
                   "local_track_frame_meters_per_second");
        RequireFinite(
            body
                .body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second,
            what + ".body_origin_vertical_velocity_in_inertial_expressed_in_"
                   "local_track_frame_meters_per_second");
        names.push_back(body.body_name);
    }
    RequireDistinctNames(names, field("free_body_startup_states"));

    names.clear();
    for (const RevoluteJointStartupState& joint :
         state.revolute_joint_startup_states) {
        const std::string what =
            field("revolute_joint_startup_states['" + joint.joint_name + "']");
        RequireFinite(joint.position_radians, what + ".position_radians");
        if (const auto* explicit_rate =
                std::get_if<ExplicitRevoluteJointRate>(&joint.rate)) {
            RequireFinite(explicit_rate->angular_rate_radians_per_second,
                          what +
                              ".rate.angular_rate_radians_per_second");
        } else {
            const auto& generated_rate =
                std::get<RevoluteJointRatePerCommonWheelSpin>(joint.rate);
            RequireFinite(generated_rate.multiplier,
                          what + ".rate.multiplier");
            if (!state.common_wheel_spin_generation.has_value()) {
                Reject(what +
                       ".rate uses the common wheel-spin magnitude, but this "
                       "record has no common wheel-spin generation");
            }
        }
        names.push_back(joint.joint_name);
    }
    RequireDistinctNames(names, field("revolute_joint_startup_states"));

    names.clear();
    for (const BallRpyJointStartupState& joint :
         state.ball_rpy_joint_startup_states) {
        const std::string what =
            field("ball_rpy_joint_startup_states['" + joint.joint_name +
                  "']");
        RequireFiniteVector(joint.roll_pitch_yaw_angles_radians,
                            what + ".roll_pitch_yaw_angles_radians");
        RequireFiniteVector(
            joint
                .angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second,
            what +
                ".angular_velocity_of_child_in_parent_expressed_in_parent_"
                "frame_radians_per_second");
        names.push_back(joint.joint_name);
    }
    RequireDistinctNames(names, field("ball_rpy_joint_startup_states"));

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
