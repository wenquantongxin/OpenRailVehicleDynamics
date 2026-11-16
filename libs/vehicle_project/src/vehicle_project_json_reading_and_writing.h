#pragma once

#include "orvd/vehicle_project/vehicle_project.h"
#include "strict_json.h"

// A vehicle project as the JSON document of its project file:
//
//   {"vehicle_definition": <a vehicle definition file's object>,
//    "contact_record": <a contact record file's object> or null,
//    "gravitational_acceleration_meters_per_second_squared": <number>,
//    "free_body_placements": [{"body_name", "rotation_local_track_from_body":
//                              {"w", "x", "y", "z"},
//                              "lateral_offset_in_local_track_frame_meters",
//                              "vertical_offset_in_local_track_frame_meters"}, ...],
//    "visual_definition_body_names": [{"body_name",
//                                      "visual_definition_body_name"}, ...],
//    "element_groups": [{"name", "members": [{"kind", "name"}, ...],
//                        "subgroups": [<group>, ...]}, ...],
//    "visual_definition": <object> or null}
//
// The vehicle definition and the contact record are configuration's records,
// which configuration encodes and decodes at the paths $.vehicle_definition
// and $.contact_record; this document adds no knowledge of their fields. A
// placement is written as a start-up state writes a free body's pose, a group
// member's kind as the element kind's word, and arrays keep their order.

namespace orvd::vehicle_project::internal {

// The document of `project`.
//
// Throws std::invalid_argument, naming the place in the document, when a
// number is not finite, when the project states no gravitational acceleration,
// and when its visual definition is not the text of a JSON object.
[[nodiscard]] configuration::strict_json::Json EncodeVehicleProjectToJson(
    const VehicleProject& project);

// The project a document states. Checks the document's own keys and types,
// all and only, and nothing about the vehicle: an unfinished project reads
// back as it was written.
//
// Throws std::invalid_argument naming the place in the document.
[[nodiscard]] VehicleProject DecodeVehicleProjectFromJson(
    const configuration::strict_json::Json& document);

}  // namespace orvd::vehicle_project::internal
