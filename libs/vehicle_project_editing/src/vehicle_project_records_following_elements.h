#pragma once

#include <algorithm>
#include <expected>
#include <memory>
#include <string>
#include <string_view>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"

// The records that follow an element without being elements, kept in step by
// every edit that touches the element: its memberships in groups, and for a
// rigid body its entry in the mechanical track station layout, the layout's
// reference body name, the layout's wheel-pair station reference list, its
// drawing placement and the name its shapes are stated under.

namespace orvd::vehicle_project_editing::internal {

// The body's entry in the mechanical track station layout, or null when the
// layout has none for it.
template <vehicle_project::ConstOrMutableVehicleProject Project>
auto* FindStationOffsetEntry(Project& project, const std::string& body_name) {
    auto& offsets =
        project.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets;
    const auto found = std::ranges::find(
        offsets, body_name, &configuration::VehicleFreeBodyStationOffsetDefinition::body_name);
    return found == offsets.end() ? nullptr : std::to_address(found);
}

// The body's placement, or null when it has none.
template <vehicle_project::ConstOrMutableVehicleProject Project>
auto* FindPlacement(Project& project, const std::string& body_name) {
    auto& placements = project.free_body_placements;
    const auto found =
        std::ranges::find(placements, body_name, &vehicle_project::FreeBodyPlacement::body_name);
    return found == placements.end() ? nullptr : std::to_address(found);
}

// The record holding the body-keyed member, chosen by the member's record.
template <vehicle_project::ConstOrMutableVehicleProject Project>
auto* FindBodyKeyedRecord(Project& project, const std::string& body_name,
                          double configuration::VehicleFreeBodyStationOffsetDefinition::*) {
    return FindStationOffsetEntry(project, body_name);
}

template <vehicle_project::ConstOrMutableVehicleProject Project>
auto* FindBodyKeyedRecord(Project& project, const std::string& body_name,
                          double vehicle_project::FreeBodyPlacement::*) {
    return FindPlacement(project, body_name);
}

// Renames `element` to `new_name` in the records that follow it.
void RenameRecordsFollowingElement(vehicle_project::VehicleProject& project,
                                   const vehicle_project::ElementReference& element,
                                   const std::string& new_name);

// Removes the records that follow `element`. Groups stay, emptied or not.
void RemoveRecordsFollowingElement(vehicle_project::VehicleProject& project,
                                   const vehicle_project::ElementReference& element);

// Gives a body that now moves freely the station layout entry and the
// placement a free body has, where it has none: a zero station offset, and
// zero offsets with the identity rotation.
void AddFreeBodyRecords(vehicle_project::VehicleProject& project, const std::string& body_name);

// Takes the station layout entry and the placement from a body that no longer
// moves freely.
void RemoveFreeBodyRecords(vehicle_project::VehicleProject& project, const std::string& body_name);

// Refuses an edit that would take the station layout's reference body out of
// the layout, which `edit` completes the reason with: "removed", or "attached
// by a joint". Every other body may leave it.
[[nodiscard]] std::expected<void, std::string> RequireLayoutKeepsReferenceBody(
    const vehicle_project::VehicleProject& project, const std::string& body_name,
    std::string_view edit);

}  // namespace orvd::vehicle_project_editing::internal
