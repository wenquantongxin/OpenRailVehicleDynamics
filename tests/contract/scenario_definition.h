// The model-neutral scenario that both sides of a cross-process comparison are
// driven from.
//
// Scope: a test contract, not a product API. It describes only the rigid-body
// constructs the current checks need — links, revolute joints, one free-floating
// body, a prescribed state and a prescribed excitation. There is no vehicle
// concept here and none is coming: a scenario that knew about wheelsets would
// shape the runtime interfaces around one consumer.
//
// Comparison scales live with the scenario because they are derived from the
// same inputs that build the model: link lengths, masses and gravity. A scale
// recovered by looking at output would be fitted to whatever the reference
// happened to produce, which is why the judge never computes one.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace orvd_contract {

struct LinkDefinition {
    std::string name;
    double mass_kilograms{1.0};
    Eigen::Vector3d center_of_mass_in_link_frame{Eigen::Vector3d::Zero()};
    Eigen::Vector3d solid_box_full_extents_meters{Eigen::Vector3d::Ones()};
};

struct RevoluteJointDefinition {
    std::string name;
    std::string parent_link_name;  // empty means the world link
    std::string child_link_name;
    Eigen::Vector3d parent_frame_offset_in_parent{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis_in_parent{Eigen::Vector3d::UnitZ()};
};

// Per-component reference magnitudes. Rotational and translational entries are
// kept apart because they carry different dimensions; one shared scale would let
// an error in the smaller component hide behind the larger one.
struct ComparisonScales {
    double angle_radians{1.0};
    double translation_meters{0.0};
    std::vector<double> generalized_force_component_newtons;         // per velocity index
    std::vector<double> generalized_torque_component_newton_metres;  // per velocity index
};

struct ScenarioDefinition {
    std::string name;
    double gravity_acceleration_meters_per_second_squared{9.81};
    std::vector<LinkDefinition> links;
    std::vector<RevoluteJointDefinition> revolute_joints;
    std::string free_floating_link_name;

    // Prescribed state and excitation. Both sides are driven from these exact
    // numbers rather than from each implementation's own defaults.
    std::vector<double> generalized_positions;
    std::vector<double> generalized_velocities;
    std::vector<double> generalized_accelerations;

    // Strictly positive in every entry: a column of the mass matrix excited by
    // zero is never observed at all. This is separate from the prescribed
    // accelerations above, which drive inverse dynamics and may be zero.
    std::vector<double> mass_matrix_excitation_scales;

    ComparisonScales comparison_scales;
};

// A two-link revolute chain plus one free-floating body. The joint axes are
// deliberately non-parallel: a chain whose joints share an axis cannot expose a
// sign or frame error in the off-diagonal inertia coupling.
//
// `excitation` selects the working point:
//   "near_zero_cancellation" — rest state; generalized forces collapse towards
//                              zero, which is where a spurious bias term shows.
//   "dynamic_excitation"     — non-zero velocities and accelerations chosen so
//                              the generalized forces are firmly away from zero.
[[nodiscard]] ScenarioDefinition MakeRevoluteChainWithFloatingBodyScenario(
    std::string_view excitation);

}  // namespace orvd_contract
