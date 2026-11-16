#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "orvd/wheel_rail_contact/contact_creepage.h"
#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"
#include "orvd/wheel_rail_contact/normal_contact_force.h"
#include "orvd/wheel_rail_contact/profile_points.h"
#include "orvd/wheel_rail_contact/tangential_contact_force.h"
#include "orvd/wheel_rail_contact/wheel_profile_preprocessing.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

namespace orvd::configuration {

struct VehicleContactCarrierDeclaration {
    std::string carrier_name;
    std::string body_name;
    Eigen::Matrix3d rotation_body_from_nonspinning_wheel_profile{Eigen::Matrix3d::Identity()};
};

struct VehicleContactInterfaceDeclaration {
    std::string interface_name;
    std::string carrier_name;
    std::string wheel_body_name;
    wheel_rail_contact::WheelSide side{wheel_rail_contact::WheelSide::kRight};
    std::optional<std::string> independent_wheel_revolute_joint_name;
};

// Declarations in interface order, independent of initial projection stations.
struct VehicleContactDeclaration {
    std::vector<VehicleContactCarrierDeclaration> carriers;
    std::vector<VehicleContactInterfaceDeclaration> interfaces;
};

struct ContactProfileReference {
    std::string profile_identifier;
    // Resolved relative to the explicitly supplied asset root.
    std::filesystem::path profile_path;
};

// These quantities accompany the reusable wheel definition.
struct VehicleWheelContactConfiguration {
    ContactProfileReference profile;
    double nominal_rolling_radius_meters{};
    double wheel_lateral_datum_magnitude_meters{};
};

// The root project supplies the rail environment.
struct VehicleRailContactConfiguration {
    ContactProfileReference profile;
    double track_gauge_meters{};
    double gauge_measuring_depth_meters{};
    // Used by wheel/rail pose and rail gauge-datum construction. Reference
    // records retain atan(1/40) on this path.
    double pose_rail_cant_radians{};
    // Used by contact geometry and force evaluation. Reference records retain
    // the qualified rounded value 0.024994792 on this path. Both cant values
    // are active inputs; preserve them separately rather than merging them.
    double geometry_rail_cant_radians{};
    wheel_rail_contact::RailProfileOriginMode profile_origin_mode{
        wheel_rail_contact::RailProfileOriginMode::kProfileCoordinate};
};

struct VehicleContactGeometrySettings {
    double envelope_bin_width_meters{};
    std::size_t outline_sample_count{};
    double contact_gap_epsilon_meters{};
    double island_merge_gap_tolerance_meters{};
    std::size_t island_quadrature_stations{};
};

struct VehicleNormalContactSettings {
    // The reference EEC law uses 1.0 / 0.55 as its penetration multiplier;
    // reference records store the double-precision value of that expression.
    double penetration_equivalence_factor{};
    double reference_damping_newton_seconds_per_meter{};
    double reference_stiffness_newtons_per_meter{};
    double damping_activation_penetration_meters{};
};

struct VehicleTangentialContactSettings {
    std::size_t longitudinal_cells{};
    std::size_t lateral_strips{};
    double refinement_width{};
};

// Shared by all interfaces. Material, wheel radius and rail cant each have
// their own source fields rather than copies in the individual force laws.
struct VehicleContactComputationSettings {
    std::string contact_strategy_identifier;
    wheel_rail_contact::ContactMaterial material;
    VehicleContactGeometrySettings geometry;
    VehicleNormalContactSettings normal;
    wheel_rail_contact::CreepageConfiguration creepage;
    wheel_rail_contact::FrictionLaw friction;
    VehicleTangentialContactSettings tangential;
    wheel_rail_contact::OutsideTableRule creep_coefficients_outside_table{
        wheel_rail_contact::OutsideTableRule::kAsymptotic};
    wheel_rail_contact::WheelProfilePreprocessingConfiguration wheel_profile_preprocessing;
};

struct VehicleContactRecord {
    VehicleContactDeclaration declaration;
    VehicleWheelContactConfiguration wheel;
    VehicleRailContactConfiguration rail;
    VehicleContactComputationSettings computation;
};

// The project derives the mechanical layout's wheel-pair station body list
// from these carriers, in carrier order, instead of editing a second list.
[[nodiscard]] std::vector<std::string> CollectWheelPairStationReferenceBodyNames(
    const VehicleContactDeclaration& declaration);

}  // namespace orvd::configuration
