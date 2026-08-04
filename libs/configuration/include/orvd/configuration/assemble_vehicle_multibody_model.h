#pragma once

#include <memory>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"

namespace orvd::configuration {

// Builds the multibody model one vehicle description states, and nothing else.
//
// Every topology entity of the current description becomes exactly one modelling
// call: a rigid body, a fixed frame, a revolute or weld joint, or a free-body
// declaration. Nothing is inferred or added. The function consumes the record's
// call-time value and retains no reference to it. New record fields require an
// explicit consumer and a direct test; the compiler cannot prove that by itself.
//
// The inertia the description states is about each body's centre of mass, and
// the multibody layer wants a unit inertia about the body origin. The parallel
// axis theorem is applied here, once, on the way in. The converted numbers are
// not written anywhere a record could later disagree with.
//
// Gravity is not part of a vehicle's identity — the same vehicle is the same
// vehicle on any planet — so it is not in the description and the caller states
// its magnitude here. The direction is not the caller's to choose: the track
// inertial frame has already fixed which way is down, and the model is given
// `magnitude * downward`. This is stated rather than defaulted because the
// multibody layer carries a gravity vector from the moment a model exists, and
// that value points the other way in this project's convention; a caller who
// forgot would get a vehicle that falls upward without any diagnostic.
//
// The model is returned by unique pointer because a multibody model can neither
// be copied nor moved: it hands out handles that are only valid against the one
// model that made them.
//
// Throws std::invalid_argument when the vehicle name is empty, a body mass is not
// finite and positive, the magnitude is not finite and positive, or when the
// multibody layer refuses any part of the description — a repeated
// name, an unusable joint axis, a negative damping, a body with no path to the
// world, a relation that would close a loop, or mass properties no body can
// have. Those diagnostics come from the layer that owns the rule and are not
// restated here.
[[nodiscard]] std::unique_ptr<multibody_model::MultibodyModel>
AssembleVehicleMultibodyModel(
    const VehicleDefinition& vehicle,
    double gravitational_acceleration_magnitude_meters_per_second_squared);

// Compiles the vehicle's force elements against an assembled model.
//
// Every element the description states becomes exactly one entry of the plan,
// with each endpoint reduced to the model frame its name resolves to.  During
// evaluation the finalized model and current context resolve that frame's
// carrier body and body-fixed origin; those facts are never reconstructed from
// the mutable description. The description is read once here and not retained:
// a record edited afterwards cannot reach a plan already compiled from it.
//
// The model must be the one this description was assembled into, and must
// outlive the plan.
//
// Throws std::invalid_argument when an element name is empty or repeated, when
// an element names a frame the model does not have, when its two ends are the
// same frame or lie on one body, or when a constitutive constant is not usable.
// Passive algebraic elements require non-negative stiffness and damping; a
// series element requires both strictly positive, since with either at zero it
// has no time constant and is an algebraic element of a different family.  A
// clipped damper curve must be an ascending run of finite, non-negative forces
// starting at the origin.
[[nodiscard]] std::unique_ptr<forces::VehicleForcePlan> BuildVehicleForcePlan(
    const VehicleDefinition& vehicle,
    const multibody_model::MultibodyModel& model);

}  // namespace orvd::configuration
