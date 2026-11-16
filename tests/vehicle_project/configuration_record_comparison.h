#pragma once

#include <algorithm>
#include <cstddef>
#include <tuple>
#include <utility>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"

// Member-by-member comparison of the configuration records a vehicle project
// holds and generates: the vehicle definition and the wheel-rail contact
// record, with every record they contain.

namespace orvd::configuration_record_comparison {

// The members of each record type the comparison descends into, in
// declaration order. A structured binding has to name every member of its
// type, so a member added to one of these records stops every test using this
// comparison compiling until the comparison covers it.

inline auto Members(const configuration::VehicleRigidBodyDefinition& body) {
    const auto& [name, moves_freely_in_world, mass, center_of_mass, inertia_moments,
                 inertia_products] = body;
    return std::tie(name, moves_freely_in_world, mass, center_of_mass, inertia_moments,
                    inertia_products);
}

inline auto Members(const configuration::VehicleFixedFrameDefinition& frame) {
    const auto& [name, body_name, position, rotation] = frame;
    return std::tie(name, body_name, position, rotation);
}

inline auto Members(const configuration::VehicleRevoluteJointDefinition& joint) {
    const auto& [name, parent_frame_name, child_frame_name, axis, damping] = joint;
    return std::tie(name, parent_frame_name, child_frame_name, axis, damping);
}

inline auto Members(const configuration::VehicleBallRpyJointDefinition& joint) {
    const auto& [name, parent_frame_name, child_frame_name, default_angles] = joint;
    return std::tie(name, parent_frame_name, child_frame_name, default_angles);
}

inline auto Members(const configuration::VehicleWeldJointDefinition& joint) {
    const auto& [name, parent_frame_name, child_frame_name] = joint;
    return std::tie(name, parent_frame_name, child_frame_name);
}

inline auto Members(const configuration::VehicleTranslationalSpringDamperDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, stiffness, damping] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, stiffness, damping);
}

inline auto Members(const configuration::VehicleRollSpringDamperCoupleDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, stiffness, damping] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, stiffness, damping);
}

inline auto Members(const configuration::VehicleSeriesSpringViscousDamperDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, axis, stiffness, damping] =
        element;
    return std::tie(name, reference_frame_name, opposite_frame_name, axis, stiffness, damping);
}

inline auto Members(
    const configuration::VehiclePointToPointSeriesSpringBilinearDamperDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, stiffness, damping_below_knee,
                 knee_force, damping_above_knee] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, stiffness, damping_below_knee,
                    knee_force, damping_above_knee);
}

inline auto Members(
    const configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition& point) {
    const auto& [relative_velocity, force] = point;
    return std::tie(relative_velocity, force);
}

inline auto Members(const configuration::VehicleSaturatedPiecewiseLinearDamperDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, axis, curve] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, axis, curve);
}

inline auto Members(const configuration::VehicleReferenceFrameLinearBushingDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, translational_stiffness,
                 translational_damping, rotational_damping] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, translational_stiffness,
                    translational_damping, rotational_damping);
}

inline auto Members(const configuration::PointToPointLinearSpringDamperDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, nominal_length, stiffness,
                 damping] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, nominal_length, stiffness,
                    damping);
}

inline auto Members(const configuration::PointToPointHystereticSpringDefinition& element) {
    const auto& [name, reference_frame_name, opposite_frame_name, nominal_length, stiffness,
                 hysteresis_ratio, hysteresis_transition_speed] = element;
    return std::tie(name, reference_frame_name, opposite_frame_name, nominal_length, stiffness,
                    hysteresis_ratio, hysteresis_transition_speed);
}

inline auto Members(const configuration::VehicleFreeBodyStationOffsetDefinition& offset) {
    const auto& [body_name, station_offset] = offset;
    return std::tie(body_name, station_offset);
}

inline auto Members(const configuration::VehicleMechanicalTrackStationLayoutDefinition& layout) {
    const auto& [reference_body_name, free_body_station_offsets,
                 wheel_pair_station_reference_body_names] = layout;
    return std::tie(reference_body_name, free_body_station_offsets,
                    wheel_pair_station_reference_body_names);
}

inline auto Members(const configuration::VehicleDefinition& vehicle) {
    const auto& [vehicle_name, mechanical_definition_identifier, mechanical_track_station_layout,
                 rigid_bodies, fixed_frames, revolute_joints, ball_rpy_joints, weld_joints,
                 translational_spring_dampers, roll_spring_damper_couples,
                 series_spring_viscous_dampers, point_to_point_series_spring_bilinear_dampers,
                 saturated_piecewise_linear_dampers, reference_frame_linear_bushings,
                 point_to_point_linear_spring_dampers, point_to_point_hysteretic_springs] = vehicle;
    return std::tie(vehicle_name, mechanical_definition_identifier, mechanical_track_station_layout,
                    rigid_bodies, fixed_frames, revolute_joints, ball_rpy_joints, weld_joints,
                    translational_spring_dampers, roll_spring_damper_couples,
                    series_spring_viscous_dampers, point_to_point_series_spring_bilinear_dampers,
                    saturated_piecewise_linear_dampers, reference_frame_linear_bushings,
                    point_to_point_linear_spring_dampers, point_to_point_hysteretic_springs);
}

inline auto Members(const configuration::VehicleContactCarrierDeclaration& carrier) {
    const auto& [carrier_name, body_name, rotation_body_from_profile] = carrier;
    return std::tie(carrier_name, body_name, rotation_body_from_profile);
}

inline auto Members(const configuration::VehicleContactInterfaceDeclaration& interface) {
    const auto& [interface_name, carrier_name, wheel_body_name, side, revolute_joint_name] =
        interface;
    return std::tie(interface_name, carrier_name, wheel_body_name, side, revolute_joint_name);
}

inline auto Members(const configuration::VehicleContactDeclaration& declaration) {
    const auto& [carriers, interfaces] = declaration;
    return std::tie(carriers, interfaces);
}

inline auto Members(const configuration::ContactProfileReference& profile) {
    const auto& [profile_identifier, profile_path] = profile;
    return std::tie(profile_identifier, profile_path);
}

inline auto Members(const configuration::VehicleWheelContactConfiguration& wheel) {
    const auto& [profile, nominal_rolling_radius, lateral_datum_magnitude] = wheel;
    return std::tie(profile, nominal_rolling_radius, lateral_datum_magnitude);
}

inline auto Members(const configuration::VehicleRailContactConfiguration& rail) {
    const auto& [profile, track_gauge, gauge_measuring_depth, pose_rail_cant, geometry_rail_cant,
                 profile_origin_mode] = rail;
    return std::tie(profile, track_gauge, gauge_measuring_depth, pose_rail_cant, geometry_rail_cant,
                    profile_origin_mode);
}

inline auto Members(const configuration::VehicleContactGeometrySettings& geometry) {
    const auto& [envelope_bin_width, outline_sample_count, contact_gap_epsilon,
                 island_merge_gap_tolerance, island_quadrature_stations] = geometry;
    return std::tie(envelope_bin_width, outline_sample_count, contact_gap_epsilon,
                    island_merge_gap_tolerance, island_quadrature_stations);
}

inline auto Members(const configuration::VehicleNormalContactSettings& normal) {
    const auto& [penetration_equivalence_factor, reference_damping, reference_stiffness,
                 damping_activation_penetration] = normal;
    return std::tie(penetration_equivalence_factor, reference_damping, reference_stiffness,
                    damping_activation_penetration);
}

inline auto Members(const configuration::VehicleTangentialContactSettings& tangential) {
    const auto& [longitudinal_cells, lateral_strips, refinement_width] = tangential;
    return std::tie(longitudinal_cells, lateral_strips, refinement_width);
}

inline auto Members(const wheel_rail_contact::ContactMaterial& material) {
    const auto& [youngs_modulus, poisson_ratio] = material;
    return std::tie(youngs_modulus, poisson_ratio);
}

inline auto Members(const wheel_rail_contact::CreepageConfiguration& creepage) {
    const auto& [minimum_reference_speed] = creepage;
    return std::tie(minimum_reference_speed);
}

inline auto Members(const wheel_rail_contact::FrictionLaw& friction) {
    const auto& [static_coefficient, limiting_fraction, decay, minimum_reference_speed] = friction;
    return std::tie(static_coefficient, limiting_fraction, decay, minimum_reference_speed);
}

inline auto Members(
    const wheel_rail_contact::WheelProfilePreprocessingConfiguration& preprocessing) {
    const auto& [equal_arc_length_rescan_step] = preprocessing;
    return std::tie(equal_arc_length_rescan_step);
}

inline auto Members(const configuration::VehicleContactComputationSettings& computation) {
    const auto& [contact_strategy_identifier, material, geometry, normal, creepage, friction,
                 tangential, creep_coefficients_outside_table, wheel_profile_preprocessing] =
        computation;
    return std::tie(contact_strategy_identifier, material, geometry, normal, creepage, friction,
                    tangential, creep_coefficients_outside_table, wheel_profile_preprocessing);
}

inline auto Members(const configuration::VehicleContactRecord& record) {
    const auto& [declaration, wheel, rail, computation] = record;
    return std::tie(declaration, wheel, rail, computation);
}

template <typename Value>
bool Same(const Value& expected, const Value& actual);

template <typename Value>
bool Same(const std::vector<Value>& expected, const std::vector<Value>& actual) {
    return std::ranges::equal(
        expected, actual, [](const Value& left, const Value& right) { return Same(left, right); });
}

template <typename Tuple, std::size_t... Index>
bool SameMembers(const Tuple& expected, const Tuple& actual, std::index_sequence<Index...>) {
    return (Same(std::get<Index>(expected), std::get<Index>(actual)) && ...);
}

// Records compare member by member; every other value by its own equality,
// exactly, because a generated record is a copy and no arithmetic separates it
// from its source.
template <typename Value>
bool Same(const Value& expected, const Value& actual) {
    if constexpr (requires { Members(expected); }) {
        const auto expected_members = Members(expected);
        return SameMembers(
            expected_members, Members(actual),
            std::make_index_sequence<std::tuple_size_v<decltype(expected_members)>>{});
    } else {
        return expected == actual;
    }
}

}  // namespace orvd::configuration_record_comparison
