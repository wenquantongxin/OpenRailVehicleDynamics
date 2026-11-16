#pragma once

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"

// One vehicle project as one file of a directory of projects
// (vehicle_project_files_in_directory.h): UTF-8 JSON with two-space
// indentation and one final newline, holding the vehicle definition as the
// project holds it, the contact record or null, the gravitational
// acceleration, the placements, the names the bodies' shapes are stated under,
// the groups, and the visual definition or null.
//
// Saving does not depend on whether the vehicle passes a check: neither
// function judges the vehicle, so an unfinished project, a zero mass or an
// empty curve for instance, is written and read back as it is. Reading gives
// back the project that was written, every typed field equal and the visual
// definition equal as a JSON value, and writing what was read gives the same
// bytes again.

namespace orvd::vehicle_project {

// Writes `project` as the project file at `location`, replacing the file
// there whole or not at all: the document is completed in memory, written to
// a pending file beside the target and renamed over it.
//
// Throws, leaving the target as it was and no pending file behind, when the
// name is not a project name (std::invalid_argument), when the document
// cannot be completed (std::invalid_argument naming the place: a number that
// is not finite, no gravitational acceleration, a visual definition that is
// not the text of a JSON object), and when the file cannot be written
// (std::runtime_error naming it).
void WriteVehicleProjectFile(const VehicleProject& project,
                             const VehicleProjectFileLocation& location);

// Reads the project file at `location`. It refuses only what is wrong with the
// document: syntax, a repeated, unknown or missing key, a wrong type, a number
// binary64 cannot hold, an element kind word that names no kind.
//
// Throws std::invalid_argument naming the name or the place in the document,
// and std::runtime_error naming the file when it cannot be read.
[[nodiscard]] VehicleProject ReadVehicleProjectFile(const VehicleProjectFileLocation& location);

}  // namespace orvd::vehicle_project
