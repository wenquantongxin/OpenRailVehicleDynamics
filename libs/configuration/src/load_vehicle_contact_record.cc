#include "orvd/configuration/load_vehicle_contact_record.h"

#include <cstddef>
#include <string>

#include "strict_json.h"

namespace orvd::configuration {
namespace {

using strict_json::ElementPath;
using strict_json::Json;
using strict_json::RequireArray;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireFiniteVector3;
using strict_json::RequireIdentifier;
using strict_json::RequireIndex;
using strict_json::RequireString;
using strict_json::ThrowExpected;

Eigen::Matrix3d ParseRotationRows(const Json& value, const std::string& path) {
    RequireArray(value, path);
    if (value.size() != 3) {
        ThrowExpected(path, "three finite row vectors");
    }
    Eigen::Matrix3d rotation;
    for (std::size_t row = 0; row < 3; ++row) {
        rotation.row(static_cast<Eigen::Index>(row)) =
            RequireFiniteVector3(value[row], ElementPath(path, row));
    }
    return rotation;
}

VehicleContactCarrierDeclaration ParseCarrier(const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"carrier_name", "body_name", "rotation_body_from_nonspinning_wheel_profile"});
    return {
        .carrier_name = RequireString(value.at("carrier_name"), path + ".carrier_name"),
        .body_name = RequireString(value.at("body_name"), path + ".body_name"),
        .rotation_body_from_nonspinning_wheel_profile =
            ParseRotationRows(value.at("rotation_body_from_nonspinning_wheel_profile"),
                              path + ".rotation_body_from_nonspinning_wheel_profile"),
    };
}

VehicleContactInterfaceDeclaration ParseInterface(const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"interface_name", "carrier_name", "wheel_body_name", "side",
                      "independent_wheel_revolute_joint_name"});
    VehicleContactInterfaceDeclaration interface;
    interface.interface_name = RequireString(value.at("interface_name"), path + ".interface_name");
    interface.carrier_name = RequireString(value.at("carrier_name"), path + ".carrier_name");
    interface.wheel_body_name =
        RequireString(value.at("wheel_body_name"), path + ".wheel_body_name");
    const std::string side = RequireString(value.at("side"), path + ".side");
    if (side == "left") {
        interface.side = wheel_rail_contact::WheelSide::kLeft;
    } else if (side == "right") {
        interface.side = wheel_rail_contact::WheelSide::kRight;
    } else {
        ThrowExpected(path + ".side", "'left' or 'right'");
    }
    const Json& joint = value.at("independent_wheel_revolute_joint_name");
    if (!joint.is_null()) {
        interface.independent_wheel_revolute_joint_name =
            RequireString(joint, path + ".independent_wheel_revolute_joint_name");
    }
    return interface;
}

VehicleContactDeclaration ParseDeclaration(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"carriers", "interfaces"});
    const Json& carriers = value.at("carriers");
    const Json& interfaces = value.at("interfaces");
    RequireArray(carriers, path + ".carriers");
    RequireArray(interfaces, path + ".interfaces");
    VehicleContactDeclaration declaration;
    declaration.carriers.reserve(carriers.size());
    for (std::size_t index = 0; index < carriers.size(); ++index) {
        declaration.carriers.push_back(
            ParseCarrier(carriers[index], ElementPath(path + ".carriers", index)));
    }
    declaration.interfaces.reserve(interfaces.size());
    for (std::size_t index = 0; index < interfaces.size(); ++index) {
        declaration.interfaces.push_back(
            ParseInterface(interfaces[index], ElementPath(path + ".interfaces", index)));
    }
    return declaration;
}

ContactProfileReference ParseProfile(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"profile_identifier", "profile_path"});
    return {
        .profile_identifier = RequireIdentifier(value.at("profile_identifier"),
                                                path + ".profile_identifier", "profile identifier"),
        .profile_path = RequireString(value.at("profile_path"), path + ".profile_path"),
    };
}

VehicleWheelContactConfiguration ParseWheel(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"profile", "nominal_rolling_radius_meters", "wheel_lateral_datum_magnitude_meters"});
    return {
        .profile = ParseProfile(value.at("profile"), path + ".profile"),
        .nominal_rolling_radius_meters = RequireFiniteNumber(
            value.at("nominal_rolling_radius_meters"), path + ".nominal_rolling_radius_meters"),
        .wheel_lateral_datum_magnitude_meters =
            RequireFiniteNumber(value.at("wheel_lateral_datum_magnitude_meters"),
                                path + ".wheel_lateral_datum_magnitude_meters"),
    };
}

VehicleRailContactConfiguration ParseRail(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"profile", "track_gauge_meters", "gauge_measuring_depth_meters", "pose_rail_cant_radians",
         "geometry_rail_cant_radians", "profile_origin_mode"});
    VehicleRailContactConfiguration rail{
        .profile = ParseProfile(value.at("profile"), path + ".profile"),
        .track_gauge_meters =
            RequireFiniteNumber(value.at("track_gauge_meters"), path + ".track_gauge_meters"),
        .gauge_measuring_depth_meters = RequireFiniteNumber(
            value.at("gauge_measuring_depth_meters"), path + ".gauge_measuring_depth_meters"),
        .pose_rail_cant_radians = RequireFiniteNumber(value.at("pose_rail_cant_radians"),
                                                      path + ".pose_rail_cant_radians"),
        .geometry_rail_cant_radians = RequireFiniteNumber(value.at("geometry_rail_cant_radians"),
                                                          path + ".geometry_rail_cant_radians"),
    };
    const std::string origin =
        RequireString(value.at("profile_origin_mode"), path + ".profile_origin_mode");
    if (origin == "track_station") {
        rail.profile_origin_mode = wheel_rail_contact::RailProfileOriginMode::kTrackStation;
    } else if (origin == "profile_coordinate") {
        rail.profile_origin_mode = wheel_rail_contact::RailProfileOriginMode::kProfileCoordinate;
    } else {
        ThrowExpected(path + ".profile_origin_mode", "'track_station' or 'profile_coordinate'");
    }
    return rail;
}

wheel_rail_contact::ContactMaterial ParseMaterial(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"youngs_modulus_pascals", "poisson_ratio"});
    return {
        .youngs_modulus_pascals = RequireFiniteNumber(value.at("youngs_modulus_pascals"),
                                                      path + ".youngs_modulus_pascals"),
        .poisson_ratio = RequireFiniteNumber(value.at("poisson_ratio"), path + ".poisson_ratio"),
    };
}

VehicleContactGeometrySettings ParseGeometry(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"envelope_bin_width_meters", "outline_sample_count", "contact_gap_epsilon_meters",
         "island_merge_gap_tolerance_meters", "island_quadrature_stations"});
    return {
        .envelope_bin_width_meters = RequireFiniteNumber(value.at("envelope_bin_width_meters"),
                                                         path + ".envelope_bin_width_meters"),
        .outline_sample_count =
            RequireIndex(value.at("outline_sample_count"), path + ".outline_sample_count"),
        .contact_gap_epsilon_meters = RequireFiniteNumber(value.at("contact_gap_epsilon_meters"),
                                                          path + ".contact_gap_epsilon_meters"),
        .island_merge_gap_tolerance_meters =
            RequireFiniteNumber(value.at("island_merge_gap_tolerance_meters"),
                                path + ".island_merge_gap_tolerance_meters"),
        .island_quadrature_stations = RequireIndex(value.at("island_quadrature_stations"),
                                                   path + ".island_quadrature_stations"),
    };
}

VehicleNormalContactSettings ParseNormal(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"penetration_equivalence_factor", "reference_damping_newton_seconds_per_meter",
         "reference_stiffness_newtons_per_meter", "damping_activation_penetration_meters"});
    return {
        .penetration_equivalence_factor = RequireFiniteNumber(
            value.at("penetration_equivalence_factor"), path + ".penetration_equivalence_factor"),
        .reference_damping_newton_seconds_per_meter =
            RequireFiniteNumber(value.at("reference_damping_newton_seconds_per_meter"),
                                path + ".reference_damping_newton_seconds_per_meter"),
        .reference_stiffness_newtons_per_meter =
            RequireFiniteNumber(value.at("reference_stiffness_newtons_per_meter"),
                                path + ".reference_stiffness_newtons_per_meter"),
        .damping_activation_penetration_meters =
            RequireFiniteNumber(value.at("damping_activation_penetration_meters"),
                                path + ".damping_activation_penetration_meters"),
    };
}

wheel_rail_contact::CreepageConfiguration ParseCreepage(const Json& value,
                                                        const std::string& path) {
    RequireExactKeys(value, path, {"minimum_reference_speed_meters_per_second"});
    return {
        .minimum_reference_speed_meters_per_second =
            RequireFiniteNumber(value.at("minimum_reference_speed_meters_per_second"),
                                path + ".minimum_reference_speed_meters_per_second"),
    };
}

wheel_rail_contact::FrictionLaw ParseFriction(const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"static_coefficient", "limiting_fraction", "decay_per_meter_per_second",
                      "minimum_reference_speed_meters_per_second"});
    return {
        .static_coefficient =
            RequireFiniteNumber(value.at("static_coefficient"), path + ".static_coefficient"),
        .limiting_fraction =
            RequireFiniteNumber(value.at("limiting_fraction"), path + ".limiting_fraction"),
        .decay_per_meter_per_second = RequireFiniteNumber(value.at("decay_per_meter_per_second"),
                                                          path + ".decay_per_meter_per_second"),
        .minimum_reference_speed_meters_per_second =
            RequireFiniteNumber(value.at("minimum_reference_speed_meters_per_second"),
                                path + ".minimum_reference_speed_meters_per_second"),
    };
}

VehicleTangentialContactSettings ParseTangential(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"longitudinal_cells", "lateral_strips", "refinement_width"});
    return {
        .longitudinal_cells =
            RequireIndex(value.at("longitudinal_cells"), path + ".longitudinal_cells"),
        .lateral_strips = RequireIndex(value.at("lateral_strips"), path + ".lateral_strips"),
        .refinement_width =
            RequireFiniteNumber(value.at("refinement_width"), path + ".refinement_width"),
    };
}

wheel_rail_contact::WheelProfilePreprocessingConfiguration ParsePreprocessing(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"equal_arc_length_rescan_step_meters"});
    return {
        .equal_arc_length_rescan_step_meters =
            RequireFiniteNumber(value.at("equal_arc_length_rescan_step_meters"),
                                path + ".equal_arc_length_rescan_step_meters"),
    };
}

VehicleContactComputationSettings ParseComputation(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"contact_strategy_identifier", "material", "geometry", "normal", "creepage", "friction",
         "tangential", "creep_coefficients_outside_table", "wheel_profile_preprocessing"});
    VehicleContactComputationSettings computation{
        .contact_strategy_identifier =
            RequireIdentifier(value.at("contact_strategy_identifier"),
                              path + ".contact_strategy_identifier", "contact strategy identifier"),
        .material = ParseMaterial(value.at("material"), path + ".material"),
        .geometry = ParseGeometry(value.at("geometry"), path + ".geometry"),
        .normal = ParseNormal(value.at("normal"), path + ".normal"),
        .creepage = ParseCreepage(value.at("creepage"), path + ".creepage"),
        .friction = ParseFriction(value.at("friction"), path + ".friction"),
        .tangential = ParseTangential(value.at("tangential"), path + ".tangential"),
        .wheel_profile_preprocessing = ParsePreprocessing(value.at("wheel_profile_preprocessing"),
                                                          path + ".wheel_profile_preprocessing"),
    };
    const std::string outside = RequireString(value.at("creep_coefficients_outside_table"),
                                              path + ".creep_coefficients_outside_table");
    if (outside == "refuse") {
        computation.creep_coefficients_outside_table =
            wheel_rail_contact::OutsideTableRule::kRefuse;
    } else if (outside == "asymptotic") {
        computation.creep_coefficients_outside_table =
            wheel_rail_contact::OutsideTableRule::kAsymptotic;
    } else {
        ThrowExpected(path + ".creep_coefficients_outside_table", "'refuse' or 'asymptotic'");
    }
    return computation;
}

}  // namespace

VehicleContactRecord LoadVehicleContactRecordFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const Json value = strict_json::ParseStrictJson(
        strict_json::ReadWholeFile(configuration_path, "vehicle contact record"));
    const std::string path = "vehicle contact record";
    RequireExactKeys(value, path, {"declaration", "wheel", "rail", "computation"});
    return {
        .declaration = ParseDeclaration(value.at("declaration"), path + ".declaration"),
        .wheel = ParseWheel(value.at("wheel"), path + ".wheel"),
        .rail = ParseRail(value.at("rail"), path + ".rail"),
        .computation = ParseComputation(value.at("computation"), path + ".computation"),
    };
}

}  // namespace orvd::configuration
