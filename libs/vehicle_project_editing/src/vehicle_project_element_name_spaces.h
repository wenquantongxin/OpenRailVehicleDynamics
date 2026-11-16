#pragma once

#include <expected>
#include <span>
#include <string>

#include "orvd/vehicle_project/vehicle_project.h"

// The name spaces elements are named in, as the kernel has them: rigid bodies
// and fixed frames share one, since a frame is named by either; the three
// joint kinds share one; the eight force element kinds share one; contact
// carriers and contact interfaces have one each. Within a name space a name
// identifies one element, so a reference, which names kinds of one name space,
// identifies the element it names. Kinds of different name spaces may share a
// name: GZ18's wheelset bodies and their contact carriers do.

namespace orvd::vehicle_project_editing::internal {

// Whether the project holds an element of one of `kinds` named `name`.
[[nodiscard]] bool ContainsElementOfKinds(const vehicle_project::VehicleProject& project,
                                          std::span<const vehicle_project::ElementKind> kinds,
                                          const std::string& name);

// Refuses `name` for a new or renamed element of `kind`: an empty name, a name
// an element of the same name space holds, and the world's own name among
// rigid bodies and fixed frames.
[[nodiscard]] std::expected<void, std::string> RequireNameAvailable(
    const vehicle_project::VehicleProject& project, vehicle_project::ElementKind kind,
    const std::string& name);

}  // namespace orvd::vehicle_project_editing::internal
