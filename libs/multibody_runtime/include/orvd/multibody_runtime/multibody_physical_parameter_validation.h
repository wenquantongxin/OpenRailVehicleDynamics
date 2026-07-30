#pragma once

/// @file
/// What a physical parameter has to be, checked once and used by everyone.
///
/// The state store checks these on the way in, and so does the modelling facade
/// before it hands a description to the rigid tree. One implementation, because
/// two would be two statements of the same physics with nothing keeping them
/// equal — and the day they diverged, which one was consulted would depend on
/// which entry point the caller happened to use.
///
/// These throw. They are not assertions: some vendored constructors only check
/// in Debug, while adapter conversions intentionally skip a check already made
/// at this boundary. Neither is an acceptance contract for first-party input.

#include <string>

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace orvd::multibody_runtime {

/// @throws std::invalid_argument if `pose` contains a non-finite translation or
/// if its rotation is not orthonormal within a few units of round-off and
/// right-handed. It is validated and never repaired — orthonormalising a
/// caller's matrix turns a modelling mistake into a plausible answer they will
/// never be told about.
void ThrowIfNotAValidFixedFramePose(
    const FixedFramePoseParameters& pose, const std::string& what);

/// @throws std::invalid_argument if no real distribution of mass has these
/// properties: a negative mass, a non-finite entry, or central principal moments
/// that are negative or violate the triangle inequality.
///
/// The moments are checked after shifting to the centre of mass and reducing to
/// principal axes. The diagonal about the body origin is the wrong three
/// numbers: a large product of inertia makes the matrix indefinite while leaving
/// the diagonal looking entirely ordinary.
void ThrowIfNotRealisableInertia(const RigidBodyInertiaParameters& parameters,
                                 const std::string& what);

}  // namespace orvd::multibody_runtime
