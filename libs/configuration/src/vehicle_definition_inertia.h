#pragma once

#include <string_view>

#include "orvd/configuration/vehicle_definition.h"

namespace orvd::configuration::internal {

// A freely moving body's six rotational degrees of freedom require a
// nonsingular inertia at its centre of mass. Check that before a parallel-axis
// shift can make a missing tensor look nonzero. Constrained bodies remain the
// multibody layer's responsibility because a point mass can be valid once its
// admitted motion is known. `subject` identifies either a JSON path or a
// programmatically edited body in the diagnostic.
void ThrowIfSingularFreeBodyCenterOfMassInertia(
    const VehicleRigidBodyDefinition& body, std::string_view subject);

}  // namespace orvd::configuration::internal
