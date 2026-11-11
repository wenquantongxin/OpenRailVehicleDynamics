#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "orvd/configuration/consist_definition.h"
#include "strict_json.h"

namespace orvd::configuration {
namespace {
using strict_json::ElementPath;
using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireIdentifier;
using strict_json::RequireString;

ConsistCouplerEndDefinition ReadEnd(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"instance_name", "frame_name"});
    return {RequireIdentifier(value.at("instance_name"), path + ".instance_name", "instance name"),
            RequireString(value.at("frame_name"), path + ".frame_name")};
}
}  // namespace

ConsistDefinition LoadConsistDefinition(const std::filesystem::path& path) {
    const auto root = ParseStrictJson(ReadWholeFile(path, "consist definition"));
    RequireExactKeys(root, "consist", {"vehicles", "couplers"});
    RequireArray(root.at("vehicles"), "consist.vehicles");
    RequireArray(root.at("couplers"), "consist.couplers");
    ConsistDefinition result;
    std::unordered_set<std::string> instance_names;
    for (std::size_t i = 0; i < root.at("vehicles").size(); ++i) {
        const auto& item = root.at("vehicles")[i];
        const auto where = ElementPath("consist.vehicles", i);
        RequireExactKeys(item, where,
                         {"instance_name", "vehicle_name", "mechanical_definition_identifier",
                          "station_offset_meters"});
        ConsistVehicleInstanceDefinition instance{
            RequireIdentifier(item.at("instance_name"), where + ".instance_name", "instance name"),
            RequireString(item.at("vehicle_name"), where + ".vehicle_name"),
            RequireIdentifier(item.at("mechanical_definition_identifier"),
                              where + ".mechanical_definition_identifier", "mechanical definition"),
            RequireFiniteNumber(item.at("station_offset_meters"),
                                where + ".station_offset_meters")};
        if (!instance_names.insert(instance.instance_name).second) {
            throw std::invalid_argument(where + " repeats an instance name");
        }
        result.vehicles.push_back(std::move(instance));
    }
    if (result.vehicles.empty()) {
        throw std::invalid_argument("consist.vehicles must not be empty");
    }
    std::unordered_set<std::string> force_names;
    for (std::size_t i = 0; i < root.at("couplers").size(); ++i) {
        const auto& item = root.at("couplers")[i];
        const auto where = ElementPath("consist.couplers", i);
        RequireExactKeys(item, where,
                         {"name", "reference_end", "opposite_end", "nominal_length_meters",
                          "stiffness_newtons_per_meter", "damping_newton_seconds_per_meter"});
        ConsistCouplerDefinition force{
            RequireString(item.at("name"), where + ".name"),
            ReadEnd(item.at("reference_end"), where + ".reference_end"),
            ReadEnd(item.at("opposite_end"), where + ".opposite_end"),
            RequireFiniteNumber(item.at("nominal_length_meters"), where + ".nominal_length_meters"),
            RequireFiniteNumber(item.at("stiffness_newtons_per_meter"),
                                where + ".stiffness_newtons_per_meter"),
            RequireFiniteNumber(item.at("damping_newton_seconds_per_meter"),
                                where + ".damping_newton_seconds_per_meter")};
        if (force.name.empty() || !force_names.insert(force.name).second ||
            !instance_names.contains(force.reference_end.instance_name) ||
            !instance_names.contains(force.opposite_end.instance_name) ||
            force.reference_end.instance_name == force.opposite_end.instance_name ||
            force.reference_end.frame_name.empty() || force.opposite_end.frame_name.empty() ||
            !(force.nominal_length_meters > 0.0) || force.stiffness_newtons_per_meter < 0.0 ||
            force.damping_newton_seconds_per_meter < 0.0) {
            throw std::invalid_argument(where +
                                        " has invalid endpoints or constitutive parameters");
        }
        result.couplers.push_back(std::move(force));
    }
    return result;
}
}  // namespace orvd::configuration
