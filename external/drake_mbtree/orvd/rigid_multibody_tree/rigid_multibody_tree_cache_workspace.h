#pragma once

/// @file
/// The rigid tree's cache directory, as eleven named members.
///
/// This is where the runtime contract's retained caches become storage. Each one
/// is a `VersionedCacheSlot` whose template parameters name the state sources
/// that make it stale — the same sets the contract works out and the dependency
/// matrix checks. Reading a member's declaration tells you what invalidates it
/// without going anywhere else.
///
/// Eleven members, not a registry. The directory is frozen because it is a
/// type: nothing can append a slot to a struct, which is a stronger statement
/// than a table that declines to grow. There is no ticket graph, no lookup by
/// name and no type erasure — a cache is reached by naming it.
///
/// The workspace belongs to one evaluation context and is constructed with it.
/// A workspace that could be moved between contexts would carry freshness
/// snapshots that refer to versions of a state it no longer sits beside, and
/// those snapshots would still compare equal often enough to be believed.
///
/// Everything is sized once, from the finalized model. Nothing here allocates
/// on an evaluation.
///
/// Two entries of the contract are deliberately absent. The block system
/// Jacobian (#5) is computed directly in the first version, so pre-allocating
/// storage for it would be a cache nobody fills. The ABA force cache and the
/// accelerations (#14, #15) depend on the forces applied on the call that asks
/// for them, so they are that call's workspace rather than something to keep;
/// a slot for them would record a freshness that says nothing about the forces
/// the value actually came from.

#include <vector>

#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/frame_body_pose_cache.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "orvd/multibody_runtime/multibody_cache_slot.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace orvd::rigid_multibody_tree::internal {

namespace slot_sources {
inline constexpr auto kQ = multibody_runtime::MultibodyStateVersionSource::
    kGeneralizedPositions;
inline constexpr auto kV = multibody_runtime::MultibodyStateVersionSource::
    kGeneralizedVelocities;
inline constexpr auto kFramePoses =
    multibody_runtime::MultibodyStateVersionSource::kFixedFramePoses;
inline constexpr auto kInertias =
    multibody_runtime::MultibodyStateVersionSource::kRigidBodyInertias;
inline constexpr auto kActuators = multibody_runtime::
    MultibodyStateVersionSource::kJointActuatorParameters;
}  // namespace slot_sources

/// The eleven retained caches of the runtime contract, pre-allocated.
class RigidMultibodyTreeCacheWorkspace {
   public:
    /// Sizes every slot from the finalized model.
    ///
    /// `state` is the state these slots read their versions from. It must be the
    /// state of the same context that owns this workspace: the pairing is what
    /// makes a snapshot mean anything, and it is established here, once,
    /// rather than at each evaluation.
    RigidMultibodyTreeCacheWorkspace(
        const multibody_runtime::MultibodyStateInstance& state,
        const drake::multibody::internal::SpanningForest& forest,
        int num_links, int num_frames)
        // #3: the poses of every frame in its link and body frames.
        : frame_body_poses(state, std::in_place, num_links, num_frames,
                           forest.num_mobods()),
          // #4: X_FM, X_PB and X_WB for every mobilized body.
          position_kinematics(state, std::in_place, forest),
          // #9: the across-node hinge matrix H_PB_W, one column per velocity.
          across_node_jacobian(state, std::in_place,
                               static_cast<std::size_t>(forest.num_velocities())),
          // #8: V_WB and V_PB_W for every mobilized body.
          velocity_kinematics(state, std::in_place, forest),
          // #6: each body's spatial inertia re-expressed in the world frame.
          //
          // NaN rather than zero, here and for the composite inertias below. A
          // zero-filled inertia is a legitimate value — a massless frame carrier
          // has one — so a pass that failed to write an entry would leave
          // something indistinguishable from a body that is genuinely massless.
          // NaN cannot be mistaken for an answer.
          spatial_inertia_in_world(
              state, std::in_place,
              static_cast<std::size_t>(forest.num_mobods()),
              drake::multibody::SpatialInertia<double>::NaN()),
          // #7: the inertia of each body plus everything outboard of it.
          composite_body_inertia_in_world(
              state, std::in_place,
              static_cast<std::size_t>(forest.num_mobods()),
              drake::multibody::SpatialInertia<double>::NaN()),
          // #10: the velocity-dependent bias force on each body.
          dynamic_bias(state, std::in_place,
                       static_cast<std::size_t>(forest.num_mobods())),
          // #12: the bias spatial acceleration of each body.
          spatial_acceleration_bias(
              state, std::in_place,
              static_cast<std::size_t>(forest.num_mobods())),
          // #1: the reflected inertia contributed by each actuated velocity.
          reflected_inertia(state, std::in_place, forest.num_velocities()),
          // #11: the articulated body inertia of each body's outboard subtree.
          articulated_body_inertia(state, std::in_place, forest),
          // #13: the articulated body bias force of each body.
          articulated_body_force_bias(
              state, std::in_place,
              static_cast<std::size_t>(forest.num_mobods())) {}

    // A workspace is one context's private world, for the same reason a state
    // is. Copying it would produce a second set of slots claiming the first
    // set's committed snapshots.
    RigidMultibodyTreeCacheWorkspace(const RigidMultibodyTreeCacheWorkspace&) =
        delete;
    RigidMultibodyTreeCacheWorkspace& operator=(
        const RigidMultibodyTreeCacheWorkspace&) = delete;
    RigidMultibodyTreeCacheWorkspace(RigidMultibodyTreeCacheWorkspace&&) =
        delete;
    RigidMultibodyTreeCacheWorkspace& operator=(
        RigidMultibodyTreeCacheWorkspace&&) = delete;

    // The dependency sets below are transcribed from the runtime contract's §2
    // table. The contract entry number is on each member.

    /// #3 — frame pose in link and body frames.
    multibody_runtime::VersionedCacheSlot<
        drake::multibody::internal::FrameBodyPoseCache<double>,
        slot_sources::kFramePoses, slot_sources::kInertias>
        frame_body_poses;

    /// #4 — position kinematics.
    multibody_runtime::VersionedCacheSlot<
        drake::multibody::internal::PositionKinematicsCache<double>,
        slot_sources::kQ, slot_sources::kFramePoses>
        position_kinematics;

    /// #9 — H_PB_W(q).
    multibody_runtime::VersionedCacheSlot<std::vector<drake::Vector6<double>>,
                                          slot_sources::kQ,
                                          slot_sources::kFramePoses>
        across_node_jacobian;

    /// #8 — velocity kinematics.
    multibody_runtime::VersionedCacheSlot<
        drake::multibody::internal::VelocityKinematicsCache<double>,
        slot_sources::kQ, slot_sources::kV, slot_sources::kFramePoses>
        velocity_kinematics;

    /// #6 — mobod spatial inertia in world.
    multibody_runtime::VersionedCacheSlot<
        std::vector<drake::multibody::SpatialInertia<double>>, slot_sources::kQ,
        slot_sources::kFramePoses, slot_sources::kInertias>
        spatial_inertia_in_world;

    /// #7 — composite mobod inertia in world.
    multibody_runtime::VersionedCacheSlot<
        std::vector<drake::multibody::SpatialInertia<double>>, slot_sources::kQ,
        slot_sources::kFramePoses, slot_sources::kInertias>
        composite_body_inertia_in_world;

    /// #10 — mobod dynamic bias.
    multibody_runtime::VersionedCacheSlot<
        std::vector<drake::multibody::SpatialForce<double>>, slot_sources::kQ,
        slot_sources::kV, slot_sources::kFramePoses, slot_sources::kInertias>
        dynamic_bias;

    /// #12 — spatial acceleration bias.
    multibody_runtime::VersionedCacheSlot<
        std::vector<drake::multibody::SpatialAcceleration<double>>,
        slot_sources::kQ, slot_sources::kV, slot_sources::kFramePoses>
        spatial_acceleration_bias;

    /// #1 — reflected inertia.
    multibody_runtime::VersionedCacheSlot<drake::VectorX<double>,
                                          slot_sources::kActuators>
        reflected_inertia;

    /// #11 — articulated body inertia.
    multibody_runtime::VersionedCacheSlot<
        drake::multibody::internal::ArticulatedBodyInertiaCache<double>,
        slot_sources::kQ, slot_sources::kFramePoses, slot_sources::kInertias,
        slot_sources::kActuators>
        articulated_body_inertia;

    /// #13 — articulated body force bias.
    multibody_runtime::VersionedCacheSlot<
        std::vector<drake::multibody::SpatialForce<double>>, slot_sources::kQ,
        slot_sources::kV, slot_sources::kFramePoses, slot_sources::kInertias,
        slot_sources::kActuators>
        articulated_body_force_bias;
};

}  // namespace orvd::rigid_multibody_tree::internal
