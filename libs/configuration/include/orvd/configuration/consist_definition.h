#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/configuration/point_to_point_spring_definitions.h"

namespace orvd::configuration {

struct ConsistVehicleInstanceDefinition {
    std::string instance_name;
    std::string vehicle_name;
    std::string mechanical_definition_identifier;
    double station_offset_meters{};
};

struct ConsistDefinition {
    std::vector<ConsistVehicleInstanceDefinition> vehicles;
    std::vector<PointToPointLinearSpringDamperDefinition> point_to_point_linear_spring_dampers;
    std::vector<PointToPointHystereticSpringDefinition> point_to_point_hysteretic_springs;
};

[[nodiscard]] ConsistDefinition LoadConsistDefinition(const std::filesystem::path& path);

[[nodiscard]] inline std::string QualifyVehicleName(std::string_view instance_name,
                                                    std::string_view local_name) {
    if (instance_name.empty())
        return std::string(local_name);
    return std::string(instance_name) + "." + std::string(local_name);
}

}  // namespace orvd::configuration
