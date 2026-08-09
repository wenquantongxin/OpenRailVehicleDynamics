#include "orvd/configuration/load_profile_points.h"

#include <cstddef>
#include <string>
#include <vector>

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
using strict_json::ThrowExpected;

wheel_rail_contact::ProfileRole RequireProfileRole(const Json& value,
                                                   const std::string& path) {
    const std::string role = RequireString(value, path);
    if (role == "wheel") {
        return wheel_rail_contact::ProfileRole::kWheel;
    }
    if (role == "rail") {
        return wheel_rail_contact::ProfileRole::kRail;
    }
    ThrowExpected(path, "either 'wheel' or 'rail'");
}

std::vector<double> RequireCoordinateColumn(const Json& root,
                                            const std::string& key) {
    const std::string path = "$." + key;
    const Json& array = root.at(key);
    RequireArray(array, path);
    std::vector<double> column;
    column.reserve(array.size());
    for (std::size_t index = 0; index < array.size(); ++index) {
        column.push_back(
            RequireFiniteNumber(array[index], ElementPath(path, index)));
    }
    return column;
}

}  // namespace

wheel_rail_contact::ProfilePoints LoadProfilePointsFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const std::string document = ReadWholeFile(configuration_path, "profile");
    const Json root = ParseStrictJson(document);
    RequireExactKeys(root, "$",
                     {"profile_identifier", "profile_role", "coordinate_frame",
                      "length_unit", "lateral_meters", "vertical_meters"});

    // The frame and the unit are stated so that a reader of the file knows what
    // its numbers mean, and are checked rather than merely recorded so that a
    // file stating something this loader does not implement is refused instead
    // of being read as if it had said the admitted thing.
    const std::string frame =
        RequireString(root.at("coordinate_frame"), "$.coordinate_frame");
    if (frame != "lateral_right_vertical_down") {
        ThrowExpected("$.coordinate_frame",
                      "'lateral_right_vertical_down', the only convention this "
                      "loader implements");
    }
    const std::string unit = RequireString(root.at("length_unit"), "$.length_unit");
    if (unit != "meter") {
        ThrowExpected("$.length_unit",
                      "'meter'; a profile is converted to metres when it is "
                      "written, not when it is read");
    }

    std::string identifier = RequireIdentifier(
        root.at("profile_identifier"), "$.profile_identifier", "profile identifier");
    const wheel_rail_contact::ProfileRole role =
        RequireProfileRole(root.at("profile_role"), "$.profile_role");

    std::vector<double> lateral = RequireCoordinateColumn(root, "lateral_meters");
    std::vector<double> vertical = RequireCoordinateColumn(root, "vertical_meters");

    return wheel_rail_contact::ProfilePoints::FromAuthoredOrder(
        role, std::move(identifier), std::move(lateral), std::move(vertical));
}

}  // namespace orvd::configuration
