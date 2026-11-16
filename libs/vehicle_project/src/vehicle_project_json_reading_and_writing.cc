#include "vehicle_project_json_reading_and_writing.h"

#include <Eigen/Geometry>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_contact_record_json_reading_and_writing.h"
#include "vehicle_definition_json_reading_and_writing.h"

namespace orvd::vehicle_project::internal {
namespace {

using configuration::strict_json::ElementPath;
using configuration::strict_json::Json;
using configuration::strict_json::RequireArray;
using configuration::strict_json::RequireExactKeys;
using configuration::strict_json::RequireFiniteNumber;
using configuration::strict_json::RequireObject;
using configuration::strict_json::RequireString;

constexpr const char* kGravityKey = "gravitational_acceleration_meters_per_second_squared";

Json EncodePlacement(const FreeBodyPlacement& placement) {
    const Eigen::Quaterniond& rotation = placement.rotation_local_track_from_body;
    return {{"body_name", placement.body_name},
            {"rotation_local_track_from_body",
             {{"w", rotation.w()}, {"x", rotation.x()}, {"y", rotation.y()}, {"z", rotation.z()}}},
            {"lateral_offset_in_local_track_frame_meters",
             placement.lateral_offset_in_local_track_frame_meters},
            {"vertical_offset_in_local_track_frame_meters",
             placement.vertical_offset_in_local_track_frame_meters}};
}

FreeBodyPlacement DecodePlacement(const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"body_name", "rotation_local_track_from_body",
                      "lateral_offset_in_local_track_frame_meters",
                      "vertical_offset_in_local_track_frame_meters"});
    const Json& rotation = value.at("rotation_local_track_from_body");
    const std::string rotation_path = path + ".rotation_local_track_from_body";
    RequireExactKeys(rotation, rotation_path, {"w", "x", "y", "z"});
    return {.body_name = RequireString(value.at("body_name"), path + ".body_name"),
            .rotation_local_track_from_body =
                Eigen::Quaterniond(RequireFiniteNumber(rotation.at("w"), rotation_path + ".w"),
                                   RequireFiniteNumber(rotation.at("x"), rotation_path + ".x"),
                                   RequireFiniteNumber(rotation.at("y"), rotation_path + ".y"),
                                   RequireFiniteNumber(rotation.at("z"), rotation_path + ".z")),
            .lateral_offset_in_local_track_frame_meters =
                RequireFiniteNumber(value.at("lateral_offset_in_local_track_frame_meters"),
                                    path + ".lateral_offset_in_local_track_frame_meters"),
            .vertical_offset_in_local_track_frame_meters =
                RequireFiniteNumber(value.at("vertical_offset_in_local_track_frame_meters"),
                                    path + ".vertical_offset_in_local_track_frame_meters")};
}

Json EncodeVisualDefinitionBodyName(const VisualDefinitionBodyName& named) {
    return {{"body_name", named.body_name},
            {"visual_definition_body_name", named.visual_definition_body_name}};
}

VisualDefinitionBodyName DecodeVisualDefinitionBodyName(const Json& value,
                                                        const std::string& path) {
    RequireExactKeys(value, path, {"body_name", "visual_definition_body_name"});
    return {.body_name = RequireString(value.at("body_name"), path + ".body_name"),
            .visual_definition_body_name = RequireString(value.at("visual_definition_body_name"),
                                                         path + ".visual_definition_body_name")};
}

Json EncodeGroups(const std::vector<ElementGroup>& groups) {
    Json encoded = Json::array();
    for (const ElementGroup& group : groups) {
        Json members = Json::array();
        for (const ElementReference& member : group.members) {
            members.push_back(
                {{"kind", std::string(ElementKindWord(member.kind))}, {"name", member.name}});
        }
        encoded.push_back({{"name", group.name},
                           {"members", std::move(members)},
                           {"subgroups", EncodeGroups(group.subgroups)}});
    }
    return encoded;
}

ElementReference DecodeGroupMember(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"kind", "name"});
    const std::optional<ElementKind> kind =
        FindElementKindWrittenAs(RequireString(value.at("kind"), path + ".kind"));
    if (!kind.has_value()) {
        configuration::strict_json::ThrowExpected(path + ".kind", "an element kind word");
    }
    return {*kind, RequireString(value.at("name"), path + ".name")};
}

std::vector<ElementGroup> DecodeGroups(const Json& value, const std::string& path) {
    RequireArray(value, path);
    std::vector<ElementGroup> groups;
    for (std::size_t index = 0; index < value.size(); ++index) {
        const Json& group = value[index];
        const std::string group_path = ElementPath(path, index);
        RequireExactKeys(group, group_path, {"name", "members", "subgroups"});
        const Json& members = group.at("members");
        const std::string members_path = group_path + ".members";
        RequireArray(members, members_path);
        std::vector<ElementReference> decoded_members;
        for (std::size_t member = 0; member < members.size(); ++member) {
            decoded_members.push_back(
                DecodeGroupMember(members[member], ElementPath(members_path, member)));
        }
        groups.push_back(
            {.name = RequireString(group.at("name"), group_path + ".name"),
             .members = std::move(decoded_members),
             .subgroups = DecodeGroups(group.at("subgroups"), group_path + ".subgroups")});
    }
    return groups;
}

// The visual definition the project carries as text, as the value the
// document holds: a JSON object, or null when there is none.
Json EncodeVisualDefinition(const std::string& document) {
    const std::string path = "$.visual_definition";
    if (document.empty()) {
        return nullptr;
    }
    Json value;
    try {
        value = configuration::strict_json::ParseStrictJson(document);
    } catch (const std::invalid_argument& refusal) {
        throw std::invalid_argument(path + " is not the text of a JSON object: " + refusal.what());
    }
    RequireObject(value, path);
    return value;
}

template <typename Element, typename Encode>
Json EncodeArray(const std::vector<Element>& elements, Encode encode) {
    Json encoded = Json::array();
    for (const Element& element : elements) {
        encoded.push_back(encode(element));
    }
    return encoded;
}

template <typename Element, typename Decode>
std::vector<Element> DecodeArray(const Json& value, const std::string& path, Decode decode) {
    RequireArray(value, path);
    std::vector<Element> elements;
    elements.reserve(value.size());
    for (std::size_t index = 0; index < value.size(); ++index) {
        elements.push_back(decode(value[index], ElementPath(path, index)));
    }
    return elements;
}

}  // namespace

Json EncodeVehicleProjectToJson(const VehicleProject& project) {
    const std::string gravity_path = std::string("$.") + kGravityKey;
    if (!project.gravitational_acceleration_meters_per_second_squared.has_value()) {
        throw std::invalid_argument(gravity_path +
                                    " cannot be written: the project states no gravitational "
                                    "acceleration");
    }
    Json document = {
        {"vehicle_definition", configuration::internal::EncodeVehicleDefinitionToJson(
                                   project.vehicle_definition, "$.vehicle_definition")},
        {"contact_record", project.contact_record.has_value()
                               ? configuration::internal::EncodeVehicleContactRecordToJson(
                                     *project.contact_record, "$.contact_record")
                               : Json(nullptr)},
        {kGravityKey, *project.gravitational_acceleration_meters_per_second_squared},
        {"free_body_placements", EncodeArray(project.free_body_placements, EncodePlacement)},
        {"visual_definition_body_names",
         EncodeArray(project.visual_definition_body_names, EncodeVisualDefinitionBodyName)},
        {"element_groups", EncodeGroups(project.element_groups)},
        {"visual_definition", EncodeVisualDefinition(project.visual_definition_document)}};
    // The configuration encoders check their own records; these are the
    // numbers this document adds.
    configuration::strict_json::RequireFiniteJsonNumbers(document.at(kGravityKey), gravity_path);
    configuration::strict_json::RequireFiniteJsonNumbers(document.at("free_body_placements"),
                                                         "$.free_body_placements");
    return document;
}

VehicleProject DecodeVehicleProjectFromJson(const Json& document) {
    RequireExactKeys(document, "$",
                     {"vehicle_definition", "contact_record", kGravityKey, "free_body_placements",
                      "visual_definition_body_names", "element_groups", "visual_definition"});
    VehicleProject project{.vehicle_definition =
                               configuration::internal::DecodeVehicleDefinitionFromJson(
                                   document.at("vehicle_definition"), "$.vehicle_definition")};
    if (const Json& contact_record = document.at("contact_record"); !contact_record.is_null()) {
        project.contact_record = configuration::internal::DecodeVehicleContactRecordFromJson(
            contact_record, "$.contact_record");
    }
    project.gravitational_acceleration_meters_per_second_squared =
        RequireFiniteNumber(document.at(kGravityKey), std::string("$.") + kGravityKey);
    project.free_body_placements = DecodeArray<FreeBodyPlacement>(
        document.at("free_body_placements"), "$.free_body_placements", DecodePlacement);
    project.visual_definition_body_names = DecodeArray<VisualDefinitionBodyName>(
        document.at("visual_definition_body_names"), "$.visual_definition_body_names",
        DecodeVisualDefinitionBodyName);
    project.element_groups = DecodeGroups(document.at("element_groups"), "$.element_groups");
    const Json& visual_definition = document.at("visual_definition");
    if (visual_definition.is_object()) {
        project.visual_definition_document = visual_definition.dump();
    } else if (!visual_definition.is_null()) {
        configuration::strict_json::ThrowExpected("$.visual_definition", "a JSON object or null");
    }
    return project;
}

}  // namespace orvd::vehicle_project::internal
