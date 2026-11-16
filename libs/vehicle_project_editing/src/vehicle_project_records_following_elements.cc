#include "vehicle_project_records_following_elements.h"

#include <vector>

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::ElementGroup;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;

void RenameGroupMembers(std::vector<ElementGroup>& groups, const ElementReference& element,
                        const std::string& new_name) {
    for (ElementGroup& group : groups) {
        for (ElementReference& member : group.members) {
            if (member == element) {
                member.name = new_name;
            }
        }
        RenameGroupMembers(group.subgroups, element, new_name);
    }
}

void RemoveGroupMembers(std::vector<ElementGroup>& groups, const ElementReference& element) {
    for (ElementGroup& group : groups) {
        std::erase(group.members, element);
        RemoveGroupMembers(group.subgroups, element);
    }
}

void EraseStationOffsetEntry(VehicleProject& project, const std::string& body_name) {
    std::erase_if(
        project.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets,
        [&body_name](const configuration::VehicleFreeBodyStationOffsetDefinition& offset) {
            return offset.body_name == body_name;
        });
}

void ErasePlacement(VehicleProject& project, const std::string& body_name) {
    std::erase_if(project.free_body_placements,
                  [&body_name](const vehicle_project::FreeBodyPlacement& placement) {
                      return placement.body_name == body_name;
                  });
}

}  // namespace

void RenameRecordsFollowingElement(VehicleProject& project, const ElementReference& element,
                                   const std::string& new_name) {
    RenameGroupMembers(project.element_groups, element, new_name);
    if (element.kind != ElementKind::kRigidBody) {
        return;
    }
    const auto rename = [&element, &new_name](std::string& body_name) {
        if (body_name == element.name) {
            body_name = new_name;
        }
    };
    auto& layout = project.vehicle_definition.mechanical_track_station_layout;
    rename(layout.reference_body_name);
    for (configuration::VehicleFreeBodyStationOffsetDefinition& offset :
         layout.free_body_station_offsets) {
        rename(offset.body_name);
    }
    std::ranges::for_each(layout.wheel_pair_station_reference_body_names, rename);
    for (vehicle_project::FreeBodyPlacement& placement : project.free_body_placements) {
        rename(placement.body_name);
    }
    for (vehicle_project::VisualDefinitionBodyName& named : project.visual_definition_body_names) {
        rename(named.body_name);
    }
}

void RemoveRecordsFollowingElement(VehicleProject& project, const ElementReference& element) {
    RemoveGroupMembers(project.element_groups, element);
    if (element.kind != ElementKind::kRigidBody) {
        return;
    }
    EraseStationOffsetEntry(project, element.name);
    std::erase(project.vehicle_definition.mechanical_track_station_layout
                   .wheel_pair_station_reference_body_names,
               element.name);
    ErasePlacement(project, element.name);
    std::erase_if(project.visual_definition_body_names,
                  [&element](const vehicle_project::VisualDefinitionBodyName& named) {
                      return named.body_name == element.name;
                  });
}

void AddFreeBodyRecords(VehicleProject& project, const std::string& body_name) {
    if (FindStationOffsetEntry(project, body_name) == nullptr) {
        project.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets
            .push_back({.body_name = body_name, .station_offset_meters = 0.0});
    }
    if (FindPlacement(project, body_name) == nullptr) {
        project.free_body_placements.push_back({.body_name = body_name});
    }
}

void RemoveFreeBodyRecords(VehicleProject& project, const std::string& body_name) {
    EraseStationOffsetEntry(project, body_name);
    ErasePlacement(project, body_name);
}

std::expected<void, std::string> RequireLayoutKeepsReferenceBody(const VehicleProject& project,
                                                                 const std::string& body_name,
                                                                 std::string_view edit) {
    if (body_name ==
        project.vehicle_definition.mechanical_track_station_layout.reference_body_name) {
        return std::unexpected("rigid body '" + body_name +
                               "' is the mechanical track station layout's reference body, which "
                               "every station is measured from, so it cannot be " +
                               std::string(edit));
    }
    return {};
}

}  // namespace orvd::vehicle_project_editing::internal
