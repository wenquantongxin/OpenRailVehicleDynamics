#pragma once

#include <string>
#include <vector>

#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/vehicle_project/vehicle_project.h"

namespace orvd::vehicle_project {

// Places the free bodies at the poses a resolved start-up state states,
// replacing every placement the project had.
//
// Each free-body start-up state becomes one placement carrying exactly its
// rotation and its lateral and vertical offsets. Its resolved station offset
// and its velocities are not copied: a placement's station is the mechanical
// one, and a placement is a pose to draw, not a state to start from. A free
// body the state does not mention is unplaced afterwards.
//
// The state must be bound to this project's vehicle: its vehicle name and its
// mechanical definition identifier must both equal the definition's, which is
// the binding a start-up assembly requires. Whether every placement then names
// a free body of the definition is checked by RequireVehicleProjectInvariants,
// not here.
//
// Throws std::invalid_argument, leaving the project unchanged, when the state
// is bound to another vehicle.
void PlaceFreeBodiesAtStartupPoses(VehicleProject& project,
                                   const configuration::ResolvedStartupState& startup_state);

// The free bodies of the project's vehicle definition that no placement names
// yet, in the order the definition declares them.
[[nodiscard]] std::vector<std::string> CollectUnplacedFreeBodyNames(const VehicleProject& project);

}  // namespace orvd::vehicle_project
