#pragma once

#include <string>
#include <vector>

// What checking a vehicle project found, in words a user can be shown.
//
// A check runs, in order, the project's own invariants
// (vehicle_project::RequireVehicleProjectInvariants), generation of the
// vehicle definition (vehicle_project::GenerateVehicleDefinition) and the
// start-up-free assembly (configuration::AssembleVehicleSystem) under the
// session's gravity, including the contact declaration when one is provided.
// Whatever a step refuses becomes a refusal carrying that step's own message;
// nothing here restates or anticipates their rules, and a faulty project never
// makes a check throw. The check
// passes when nothing was refused.

namespace orvd::vehicle_project_editing {

struct MechanicalCheckReport {
    // The message of each refusing step, exactly as the step stated it. Empty
    // when the check passed.
    std::vector<std::string> refusals{};

    // Facts that do not stop the check from passing: each free body without a
    // placement, and the absence of the wheel-rail contact record.
    std::vector<std::string> information{};

    // What the check covered and what passing means. The same for every check.
    std::vector<std::string> coverage_statements{
        "The check validates the project's free-body placements, the names its bodies' shapes "
        "are stated under and its element groups, and assembles the vehicle definition into a "
        "multibody model with its force elements.",
        "When a contact record is provided, its carriers and interfaces are checked against "
        "the mechanical definition. Contact profiles, wheel-rail forces and start-up conditions "
        "are not evaluated; no track or wheel-rail contact runtime is assembled.",
        "Passing does not mean that a start-up state exists for this vehicle or that the "
        "vehicle can run."};
};

}  // namespace orvd::vehicle_project_editing
