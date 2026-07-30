#pragma once

/// @file
/// The two conversions between the first-party inertia record and the vendored
/// spatial inertia value type.
///
/// These replace what upstream did with a ten-element anonymous vector: mass at
/// index 0, centre of mass at 1..3, unit inertia moments and products at 4..9,
/// with a table of named constants to say which was which. That was a transport
/// format standing in for a type. `RigidBodyInertiaParameters` says what each
/// number is in its own field names, so the packing has nothing left to do —
/// what remains is this, a translation between a first-party record and the
/// mathematical value type the rigid tree computes with.
///
/// Only these two. There is no `GetMass(vector)`, no index table, no partial
/// accessor that reads one field out of a packed blob: a caller that wants the
/// mass reads `parameters.mass_kilograms`.

#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace orvd::rigid_multibody_tree::internal {

/// The record as the rigid tree's value type, about the body origin.
///
/// The validity check is skipped, and that is a considered decision rather than
/// a saving. The only way a record reaches this function is out of
/// `MultibodyStateInstance`, whose sole write path validates the same physics
/// this check would: shift to the centre of mass, reduce to principal moments,
/// require non-negativity and the triangle inequality. Running it again would
/// mean an eigenvalue decomposition per body per evaluation to re-establish
/// something that was established when the value was stored and cannot have
/// changed since — the record is a value, and the store hands out const views.
///
/// This is not the argument that "the vendored type checks it, so we need not":
/// that argument was wrong when it was used to justify a weaker check at the
/// store, and the check at the store is now the complete one. This is the
/// narrower claim that the same check should not run twice on the same value.
inline drake::multibody::SpatialInertia<double> ToSpatialInertia(
    const multibody_runtime::RigidBodyInertiaParameters& parameters) {
    const Eigen::Vector3d& moments = parameters.unit_inertia_moments;
    const Eigen::Vector3d& products = parameters.unit_inertia_products;
    const drake::multibody::UnitInertia<double> G_BBo_B(
        moments.x(), moments.y(), moments.z(), products.x(), products.y(),
        products.z());
    return drake::multibody::SpatialInertia<double>(
        parameters.mass_kilograms, parameters.center_of_mass_in_body_frame,
        G_BBo_B, /* skip_validity_check = */ true);
}

/// The value type as a record, ready to be handed to the store.
///
/// No validation here either, for the opposite reason: the store validates what
/// it is given, and a check here would either duplicate that or — worse —
/// disagree with it, leaving a value that one layer accepts and the other
/// refuses with nothing to say which is right.
inline multibody_runtime::RigidBodyInertiaParameters ToInertiaParameters(
    const drake::multibody::SpatialInertia<double>& M_BBo_B) {
    multibody_runtime::RigidBodyInertiaParameters parameters;
    parameters.mass_kilograms = M_BBo_B.get_mass();
    parameters.center_of_mass_in_body_frame = M_BBo_B.get_com();
    parameters.unit_inertia_moments = M_BBo_B.get_unit_inertia().get_moments();
    parameters.unit_inertia_products = M_BBo_B.get_unit_inertia().get_products();
    return parameters;
}

}  // namespace orvd::rigid_multibody_tree::internal
