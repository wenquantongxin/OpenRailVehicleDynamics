// Observation semantics for the model-neutral cross-process comparison.
//
// Scope: this is a test contract, not a product API. It exists so that two
// independently built processes can be driven from the same scenario and judged
// on the same quantities. It is deliberately narrow — enough for the checks the
// project runs today, and no more. It is not a configuration language.
//
// The judge keeps scalar kinds separate because an absolute limit is meaningful
// only in its own dimension. A generalized force vector on a mixed floating
// joint carries newton-metres in rotational components and newtons in
// translational components; neither emitter is allowed to relabel them.
#pragma once

#include <string>

namespace orvd_contract {

// Scalar physical kinds. Each names a dimension, not a storage layout: two
// quantities share a kind only if one tolerance is meaningful for both.
// Rotations are required and compared as complete 3×3 groups.
enum class ObservationKind {
    kAngleRadians,          // a joint angle or any bare rotation coordinate
    kUnitQuaternionComponent,  // one component of a unit quaternion; O(1), dimensionless
    kTranslationMeters,
    kAngularVelocityRadiansPerSecond,
    kTranslationalVelocityMetersPerSecond,
    kForceNewtons,          // translational generalized force
    kTorqueNewtonMetres,    // rotational generalized force
};

// One raw scalar emitted by a process. Physical meaning belongs only to the
// comparison requirements; an emitter must not declare the rule that judges it.
struct Observation {
    std::string name;
    double value;
};

// A discrete fact about the model that must match exactly. Sizes, counts and
// coordinate starts are not approximations, so "close" has no meaning for them.
struct TopologyFact {
    std::string name;
    long long value;
};

}  // namespace orvd_contract
