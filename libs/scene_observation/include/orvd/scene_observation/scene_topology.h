#pragma once

/// @file
/// The static identity of a scene: which rigid bodies exist, and where each
/// wheel's profile datum sits on its own body.

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/wheel_rail_contact/profile_points.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

namespace orvd::scene_observation {

struct SceneBody {
    std::string name;
    bool moves_freely_in_world{false};
};

/// Where one wheel's profile datum sits on its rigid body, in that body's own
/// frame, derived from the contact plan's frozen constants rather than authored
/// by hand. A viewer places its wheel geometry at the datum, along the spin
/// axis, with the nominal rolling radius; the contact point observed by the
/// force plan then lies on that wheel.
///
/// The derivation states the datum in the carrier's non-spinning wheel-profile
/// axes and maps it into the body through the carrier's fixed profile basis.
/// This is exact for a rigid wheelset, whose wheel body is the carrier, and for
/// an independently rotating wheel joined to its carrier by a revolute joint
/// about the profile lateral axis with coincident frames at zero angle, which
/// is the relation the closed IRW assembly validates.
struct SceneWheelPlacement {
    std::string interface_name;
    std::string wheel_body_name;
    wheel_rail_contact::WheelSide side{wheel_rail_contact::WheelSide::kRight};
    Eigen::Vector3d datum_in_wheel_body_frame_meters{Eigen::Vector3d::Zero()};
    Eigen::Vector3d spin_axis_in_wheel_body_frame{Eigen::Vector3d::UnitY()};
    double nominal_rolling_radius_meters{0.0};
    /// The revolute joint whose generalized position is this wheel's spin
    /// relative to its carrier; absent for a rigid wheelset, whose spin lives
    /// in the wheelset body's own orientation.
    std::optional<std::string> spin_joint_name;
};

struct SceneTopology {
    /// Model rigid-body order. A frame's body slots follow this order.
    std::vector<SceneBody> bodies;
    /// Contact-plan interface order. Empty for a system without a contact
    /// plan.
    std::vector<SceneWheelPlacement> wheel_placements;
};

[[nodiscard]] SceneWheelPlacement DeriveWheelPlacement(
    const forces::WheelRailContactInterfaceDefinition& interface,
    const forces::WheelRailContactCarrierDefinition& carrier,
    const wheel_rail_contact::WheelRailPoseConstants& constants);

/// Describes a finalized model and, when present, its contact plan. Body slots
/// follow `BodyStateSampler::names()`.
[[nodiscard]] SceneTopology DescribeSceneTopology(
    const multibody_model::MultibodyModel& model,
    const forces::WheelRailContactForcePlan* contact_plan);

}  // namespace orvd::scene_observation
