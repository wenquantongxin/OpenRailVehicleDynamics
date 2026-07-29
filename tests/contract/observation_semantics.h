// Observation semantics for the model-neutral cross-process comparison.
//
// Scope: this is a test contract, not a product API. It exists so that two
// independently built processes can be driven from the same scenario and judged
// on the same quantities. It is deliberately narrow — enough for the checks the
// project runs today, and no more. It is not a configuration language.
//
// Why the kinds below are separate: a comparison tolerance is only meaningful
// against a magnitude of the same dimension. A generalized force vector on a
// mixed floating joint carries newton-metres in its rotational components and
// newtons in its translational ones; collapsing those into one "force" category
// with one scale lets an error in the smaller component hide behind the larger
// one. So every observed scalar names the kind it belongs to, and the kind
// decides which reference magnitude normalizes it.
#pragma once

#include <string>
#include <string_view>

namespace orvd_contract {

// The physical kinds this contract can compare. Each names a dimension, not a
// storage layout: two quantities share a kind only if a tolerance expressed in
// one is meaningful for the other.
enum class ObservationKind {
    kAngleRadians,          // a joint angle or any bare rotation coordinate
    kUnitQuaternionComponent,  // one component of a unit quaternion; O(1), dimensionless
    kTranslationMeters,
    kForceNewtons,          // translational generalized force
    kTorqueNewtonMetres,    // rotational generalized force
    kRotationMatrixElement,  // an element of a rotation, compared as SO(3) angle
};

// Rotation is never compared element by element: nine element-wise differences
// do not compose into a meaningful attitude error, and a matrix that is not a
// rotation at all can pass such a comparison. Elements are transported so the
// judge can rebuild the rotation and take the geodesic angle between them.
[[nodiscard]] constexpr bool IsRotationMatrixElement(ObservationKind kind) {
    return kind == ObservationKind::kRotationMatrixElement;
}

[[nodiscard]] constexpr std::string_view ObservationKindName(ObservationKind kind) {
    switch (kind) {
        case ObservationKind::kAngleRadians:             return "angle_radians";
        case ObservationKind::kUnitQuaternionComponent:  return "unit_quaternion_component";
        case ObservationKind::kTranslationMeters:        return "translation_meters";
        case ObservationKind::kForceNewtons:             return "force_newtons";
        case ObservationKind::kTorqueNewtonMetres:       return "torque_newton_metres";
        case ObservationKind::kRotationMatrixElement:    return "rotation_matrix_element";
    }
    return "unknown";
}

[[nodiscard]] bool ParseObservationKind(std::string_view name, ObservationKind* kind);

// One observed scalar. `name` identifies the quantity across processes;
// `kind` selects the comparison rule.
struct Observation {
    std::string name;
    ObservationKind kind{ObservationKind::kTranslationMeters};
    double value{};
};

// A discrete fact about the model that must match exactly. Sizes, counts and
// coordinate starts are not approximations, so "close" has no meaning for them.
struct TopologyFact {
    std::string name;
    long long value{};
};

}  // namespace orvd_contract
