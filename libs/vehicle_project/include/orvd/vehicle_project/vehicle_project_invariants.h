#pragma once

#include "orvd/vehicle_project/vehicle_project.h"

namespace orvd::vehicle_project {

// Checks the parts of a project this library owns: its placements and its
// grouping tree. The vehicle definition and the contact record belong to
// configuration, whose loaders and assemblies check them; they are not checked
// a second time here.
//
// Each placement must name a free body of the vehicle definition, and no body
// may be named twice. Its offsets must be finite and its rotation a unit
// quaternion to within a few rounding steps, the criterion a resolved start-up
// state applies, so every start-up pose is an admissible placement. A rotation
// outside it is refused rather than normalised.
//
// Each group name must be non-empty and differ from its siblings'. Each member
// must name an element the project contains, a contact carrier or interface
// only while the project has a contact record, and no element may be a member
// twice, whether of one group or of two.
//
// Generation does not call this: placements and groups cannot change what is
// generated, so a mistake in them does not block it.
//
// Throws std::invalid_argument on the first violation. The message starts with
// a path into the project, for example
// "vehicle project.element_groups['bogies'].subgroups['front'].members", and
// names the offending element, group or value.
void RequireVehicleProjectInvariants(const VehicleProject& project);

}  // namespace orvd::vehicle_project
