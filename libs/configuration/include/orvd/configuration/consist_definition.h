#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace orvd::configuration {

struct ConsistVehicleInstanceDefinition {
    std::string instance_name;
    std::string vehicle_name;
    std::string mechanical_definition_identifier;
    double station_offset_meters{};
};

struct ConsistCouplerEndDefinition {
    std::string instance_name;
    std::string frame_name;
};

struct ConsistCouplerDefinition {
    std::string name;
    ConsistCouplerEndDefinition reference_end;
    ConsistCouplerEndDefinition opposite_end;
    double nominal_length_meters{};
    double stiffness_newtons_per_meter{};
    double damping_newton_seconds_per_meter{};
};

struct ConsistDefinition {
    std::vector<ConsistVehicleInstanceDefinition> vehicles;
    std::vector<ConsistCouplerDefinition> couplers;
};

[[nodiscard]] ConsistDefinition LoadConsistDefinition(const std::filesystem::path& path);

[[nodiscard]] inline std::string QualifyVehicleName(std::string_view instance_name,
                                                    std::string_view local_name) {
    if (instance_name.empty())
        return std::string(local_name);
    return std::string(instance_name) + "." + std::string(local_name);
}

}  // namespace orvd::configuration
