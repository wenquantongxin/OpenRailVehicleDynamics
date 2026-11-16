#pragma once

#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"

namespace orvd::configuration {

struct VehicleContactTopology {
    std::vector<forces::WheelRailContactCarrierDefinition> carriers;
    std::vector<forces::WheelRailContactInterfaceDefinition> interfaces;
    std::vector<forces::IndependentWheelActiveTorqueCoupleDefinition> active_torque_couples;
};

// Checks the declaration against the call-time mechanical description and
// builds contact bindings in declaration order. Projection stations remain
// zero until the run places each carrier body on its line. Active-torque
// bindings are supplied by the control configuration, not by contact.
//
// Each carrier is a distinct free body with one left and one right interface.
// A rigid-wheelset interface acts on its carrier. An independent-wheel
// interface names the carrier-to-wheel revolute joint, whose two ends are the
// bodies' own frames and whose unit axis is -R_BP * e_y in the carrier frame.
// Thus the wheel's relative spin has the sign the contact law expects for
// any stated non-spinning profile basis R_BP.
//
// Throws std::invalid_argument, naming the offending carrier or interface,
// when a declaration does not satisfy these geometric and naming contracts.
[[nodiscard]] VehicleContactTopology BuildVehicleContactTopology(
    const VehicleDefinition& vehicle, const VehicleContactDeclaration& declaration);

}  // namespace orvd::configuration
