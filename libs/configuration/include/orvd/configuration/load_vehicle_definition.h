#pragma once

#include <filesystem>

#include "orvd/configuration/vehicle_definition.h"

namespace orvd::configuration {

// Loads one human-authored strict JSON document from the exact path supplied by
// the caller and returns the vehicle description it states.
//
// The parser refuses duplicate, unknown and missing keys, wrong types,
// unsupported schema versions, embedded NUL bytes and numbers the document
// states but binary64 cannot hold. It performs no path search, environment
// substitution, default insertion or retained DOM storage: the returned
// description owns everything it carries.
//
// Two further refusals are made here because they are properties of the record
// that only this layer can report against a JSON path: a name referred to but
// never declared, and a mass that is not positive. The second is not a
// restatement of what the multibody layer checks — that layer accepts a massless
// body, which is a legitimate frame carrier, but a record's inertia is stated
// about the centre of mass and dividing it by zero produces a diagnostic
// pointing at the unit inertia rather than at the mass the author wrote.
//
// Everything else the multibody layer already decides is left to it: repeated
// names, a body with no path to the world, a relation that would close a loop,
// an unusable joint axis, a negative damping, a rotation that is not a rotation.
// Checking those twice would give two diagnostics for one mistake and let the
// two drift apart.
//
// Throws std::runtime_error when the file cannot be read. Throws
// std::invalid_argument for invalid JSON or an inconsistent description.
[[nodiscard]] VehicleDefinition LoadVehicleDefinitionFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
