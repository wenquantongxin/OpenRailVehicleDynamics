#pragma once

#include <algorithm>
#include <vector>

#include "configuration_record_comparison.h"
#include "orvd/vehicle_project/vehicle_project.h"

// Part-by-part comparison of two vehicle projects: the configuration records
// member by member (configuration_record_comparison.h), and the gravity, the
// placements, the names bodies' shapes are stated under, the grouping tree and
// the visual definition's text exactly. A structured binding names every
// member of each type it compares, so a member added to one stops the tests
// using it compiling until it is compared too.

namespace orvd::vehicle_project_comparison {

inline bool SamePlacement(const vehicle_project::FreeBodyPlacement& expected,
                          const vehicle_project::FreeBodyPlacement& actual) {
    const auto& [body_name, rotation, lateral_offset, vertical_offset] = expected;
    return body_name == actual.body_name &&
           rotation.coeffs() == actual.rotation_local_track_from_body.coeffs() &&
           lateral_offset == actual.lateral_offset_in_local_track_frame_meters &&
           vertical_offset == actual.vertical_offset_in_local_track_frame_meters;
}

inline bool SameVisualDefinitionBodyName(const vehicle_project::VisualDefinitionBodyName& expected,
                                         const vehicle_project::VisualDefinitionBodyName& actual) {
    const auto& [body_name, visual_definition_body_name] = expected;
    return body_name == actual.body_name &&
           visual_definition_body_name == actual.visual_definition_body_name;
}

inline bool SameGroups(const std::vector<vehicle_project::ElementGroup>& expected,
                       const std::vector<vehicle_project::ElementGroup>& actual) {
    return std::ranges::equal(
        expected, actual,
        [](const vehicle_project::ElementGroup& left, const vehicle_project::ElementGroup& right) {
            const auto& [name, members, subgroups] = left;
            return name == right.name && members == right.members &&
                   SameGroups(subgroups, right.subgroups);
        });
}

inline bool SameProject(const vehicle_project::VehicleProject& expected,
                        const vehicle_project::VehicleProject& actual) {
    const auto& [vehicle_definition, contact_record, gravitational_acceleration,
                 free_body_placements, visual_definition_body_names, element_groups,
                 visual_definition_document] = expected;
    return configuration_record_comparison::Same(vehicle_definition, actual.vehicle_definition) &&
           contact_record.has_value() == actual.contact_record.has_value() &&
           (!contact_record.has_value() ||
            configuration_record_comparison::Same(*contact_record, *actual.contact_record)) &&
           gravitational_acceleration ==
               actual.gravitational_acceleration_meters_per_second_squared &&
           std::ranges::equal(free_body_placements, actual.free_body_placements, SamePlacement) &&
           std::ranges::equal(visual_definition_body_names, actual.visual_definition_body_names,
                              SameVisualDefinitionBodyName) &&
           SameGroups(element_groups, actual.element_groups) &&
           visual_definition_document == actual.visual_definition_document;
}

}  // namespace orvd::vehicle_project_comparison
