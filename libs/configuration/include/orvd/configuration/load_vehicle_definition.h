#pragma once

#include <filesystem>

#include "orvd/configuration/vehicle_definition.h"

namespace orvd::configuration {

// Loads one human-authored strict JSON document from the exact path supplied by
// the caller and returns the vehicle description it states.
//
// The parser refuses duplicate, unknown and missing keys, wrong types,
// embedded NUL bytes and numbers the document states but binary64 cannot hold.
// It performs no path search, environment substitution, default insertion or
// retained DOM storage: the returned description owns everything it carries.
//
// The returned value is intentionally mutable. Strictness applies to this one
// JSON-to-record conversion; a C++ caller may subsequently make explicit edits
// before passing the record to the assembler.
//
// Further refusals are made here when only this layer can identify the offending
// JSON path: references to undeclared names, an empty vehicle or force-element
// name, force-element names repeated across constitutive families, a body mass
// that is not positive, and a freely moving body's singular centre-of-mass
// inertia. A six-degree-of-freedom body needs nonsingular rotational inertia;
// constrained bodies remain subject to the multibody layer's full topology-
// aware mass-property checks. A fixed-frame matrix form is checked here for a
// finite right-handed orthonormal `body <- frame` rotation because that
// direction and shape are part of the JSON contract.
//
// Everything else the multibody layer already decides is left to it: repeated
// topology names, a body with no path to the world, a relation that would close
// a loop, an unusable joint axis, or a negative damping.
// Checking those twice would give two diagnostics for one mistake and let the
// two drift apart.
//
// Throws std::runtime_error when the file cannot be read. Throws
// std::invalid_argument for invalid JSON or an inconsistent description.
[[nodiscard]] VehicleDefinition LoadVehicleDefinitionFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
