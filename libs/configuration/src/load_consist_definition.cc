#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "orvd/configuration/consist_definition.h"
#include "point_to_point_spring_definitions_json.h"
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

}  // namespace

ConsistDefinition LoadConsistDefinition(const std::filesystem::path& path) {
    const auto root = ParseStrictJson(ReadWholeFile(path, "consist definition"));
    RequireExactKeys(
        root, "consist",
        {"vehicles", "point_to_point_linear_spring_dampers", "point_to_point_hysteretic_springs"});
    RequireArray(root.at("vehicles"), "consist.vehicles");
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
    const auto read_elements = [&](const char* key, auto parse, auto& destination) {
        const auto& array = root.at(key);
        const std::string path = std::string("consist.") + key;
        RequireArray(array, path);
        for (std::size_t index = 0; index < array.size(); ++index) {
            const auto where = ElementPath(path, index);
            auto element = parse(array[index], where);
            if (!force_names.insert(element.name).second) {
                throw std::invalid_argument(where + " repeats force-element name '" + element.name +
                                            "'");
            }
            destination.push_back(std::move(element));
        }
    };
    read_elements("point_to_point_linear_spring_dampers",
                  internal::ParsePointToPointLinearSpringDamper,
                  result.point_to_point_linear_spring_dampers);
    read_elements("point_to_point_hysteretic_springs", internal::ParsePointToPointHystereticSpring,
                  result.point_to_point_hysteretic_springs);
    return result;
}
}  // namespace orvd::configuration
