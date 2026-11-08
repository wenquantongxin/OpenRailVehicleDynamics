// Checks eight cache dependency subsets against five state version sources.
// The independently specified expectations below cover the eleven caches in
// external/drake_mbtree/include/orvd/rigid_multibody_tree/
// rigid_multibody_tree_cache_workspace.h. Deriving them from the slot templates
// would make a missing or extra dependency invisible to this check.
//
// Actual cache calculations are checked against cold-context results in
// verify_rigid_tree_cache_binding.cc.
#include <array>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_cache_slot.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"

namespace {

using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::JointActuatorParameters;
using orvd::multibody_runtime::MultibodyStateInstance;
using orvd::multibody_runtime::MultibodyStateLayout;
using orvd::multibody_runtime::MultibodyStateLayoutDescription;
using orvd::multibody_runtime::MultibodyStateVersionSource;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::multibody_runtime::VersionedCacheSlot;

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

struct SyntheticPass {
    explicit SyntheticPass(int entry_count)
        : entries(static_cast<std::size_t>(entry_count), 0.0) {}
    std::vector<double> entries;
};

/// Independently specified invalidation sources for one cache subset.
///
/// Five named fields rather than an array indexed by the enum: an index would
/// make a reordering of the enum silently shift every row's meaning, and the
/// rows can be reviewed directly against the quantities read by each cache.
struct ContractRow {
    const char* semantic_name;
    /// Corresponding numbered entries in the rigid-tree cache workspace.
    const char* contract_entries;
    bool generalized_positions;
    bool generalized_velocities;
    bool fixed_frame_poses;
    bool rigid_body_inertias;
    bool joint_actuator_parameters;
};

// Eleven retained caches share these eight dependency subsets.
constexpr ContractRow kReflectedInertia{
    "reflected inertia", "#1", false, false, false, false, true};
constexpr ContractRow kFramePoseInLinkAndBodyFrames{
    "frame pose in link and body frames", "#3", false, false, true, true, false};
constexpr ContractRow kPositionKinematics{
    "position kinematics, H_PB_W(q)", "#4/#9", true, false, true, false, false};
constexpr ContractRow kMobodSpatialInertia{
    "mobod spatial inertia, composite mobod inertia", "#6/#7",
    true, false, true, true, false};
constexpr ContractRow kVelocityKinematics{
    "velocity kinematics, spatial acceleration bias", "#8/#12",
    true, true, true, false, false};
constexpr ContractRow kMobodDynamicBias{
    "mobod dynamic bias", "#10", true, true, true, true, false};
constexpr ContractRow kArticulatedBodyInertia{
    "articulated body inertia", "#11", true, false, true, true, true};
constexpr ContractRow kArticulatedBodyForceBias{
    "articulated body force bias", "#13", true, true, true, true, true};

// The corresponding slot declarations. These are what is under test; the rows
// above are what they are tested against.
using ReflectedInertiaSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kJointActuatorParameters>;
using FramePoseSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kFixedFramePoses,
                       MultibodyStateVersionSource::kRigidBodyInertias>;
using PositionKinematicsSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kFixedFramePoses>;
using MobodSpatialInertiaSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kFixedFramePoses,
                       MultibodyStateVersionSource::kRigidBodyInertias>;
using VelocityKinematicsSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kGeneralizedVelocities,
                       MultibodyStateVersionSource::kFixedFramePoses>;
using MobodDynamicBiasSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kGeneralizedVelocities,
                       MultibodyStateVersionSource::kFixedFramePoses,
                       MultibodyStateVersionSource::kRigidBodyInertias>;
using ArticulatedBodyInertiaSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kFixedFramePoses,
                       MultibodyStateVersionSource::kRigidBodyInertias,
                       MultibodyStateVersionSource::kJointActuatorParameters>;
using ArticulatedBodyForceBiasSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kGeneralizedVelocities,
                       MultibodyStateVersionSource::kFixedFramePoses,
                       MultibodyStateVersionSource::kRigidBodyInertias,
                       MultibodyStateVersionSource::kJointActuatorParameters>;

MultibodyStateLayoutDescription SampleDescription() {
    MultibodyStateLayoutDescription description;
    description.generalized_position_count = 9;
    description.generalized_velocity_count = 8;
    description.rigid_body_count = 3;
    description.fixed_frame_count = 2;
    description.joint_velocity_counts = {6, 1, 1};
    description.joint_actuator_count = 2;
    description.revolute_spring_count = 1;
    description.linear_bushing_count = 1;
    return description;
}

/// Writes one source with a legitimate value, advancing exactly that version.
void WriteSource(MultibodyStateInstance& state,
                 MultibodyStateVersionSource source, double marker) {
    switch (source) {
        case MultibodyStateVersionSource::kGeneralizedPositions:
            state.set_generalized_positions(Eigen::VectorXd::Constant(9, marker));
            return;
        case MultibodyStateVersionSource::kGeneralizedVelocities:
            state.set_generalized_velocities(Eigen::VectorXd::Constant(8, marker));
            return;
        case MultibodyStateVersionSource::kFixedFramePoses: {
            FixedFramePoseParameters pose;
            pose.p_PoFo_P = Eigen::Vector3d::Constant(marker);
            state.set_fixed_frame_pose_parameters(0, pose);
            return;
        }
        case MultibodyStateVersionSource::kRigidBodyInertias: {
            RigidBodyInertiaParameters inertia;
            inertia.mass_kilograms = marker;
            state.set_rigid_body_inertia_parameters(0, inertia);
            return;
        }
        case MultibodyStateVersionSource::kJointActuatorParameters: {
            JointActuatorParameters actuator;
            actuator.rotor_inertia = marker;
            state.set_joint_actuator_parameters(0, actuator);
            return;
        }
    }
}

const char* NameOf(MultibodyStateVersionSource source) {
    switch (source) {
        case MultibodyStateVersionSource::kGeneralizedPositions:
            return "generalized positions";
        case MultibodyStateVersionSource::kGeneralizedVelocities:
            return "generalized velocities";
        case MultibodyStateVersionSource::kFixedFramePoses:
            return "fixed frame poses";
        case MultibodyStateVersionSource::kRigidBodyInertias:
            return "rigid body inertias";
        case MultibodyStateVersionSource::kJointActuatorParameters:
            return "joint actuator parameters";
    }
    return "unknown source";
}

/// Recomputes only if the slot says it is stale, and counts how often it did.
///
/// It decides on `is_fresh()` and nothing else. In particular it never consults
/// the expectation table — a driver that recomputed when the table said to would
/// be checking the table against itself. Only the success path exists here;
/// failure handling and retry are the evaluator's, in G25.
template <typename Slot>
void EvaluateIfStale(Slot& slot, int& compute_count) {
    if (!slot.is_fresh()) {
        SyntheticPass& storage = slot.mutable_value_for_recomputation();
        ++compute_count;
        storage.entries[0] = static_cast<double>(compute_count);
        slot.CommitRecomputation();
    }
    (void)slot.value();
}

/// Runs one cell: one dependency subset against one source, on a state and slot
/// that have never been touched before.
template <typename Slot>
void CheckCell(const ContractRow& row, MultibodyStateVersionSource source,
               bool expected_to_invalidate) {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    Slot slot(state, std::in_place, 4);
    int compute_count = 0;

    const std::string cell = std::string(row.semantic_name) + " (" +
                             row.contract_entries + ") against a " +
                             NameOf(source) + " write";

    EvaluateIfStale(slot, compute_count);
    ExpectTrue(compute_count == 1, cell + ": a cold slot computes exactly once");
    EvaluateIfStale(slot, compute_count);
    ExpectTrue(compute_count == 1,
               cell + ": querying a fresh slot does not recompute");

    WriteSource(state, source, 0.5);

    // Freshness is asserted before the next query: the write itself must not
    // have driven a recomputation. A design where the setter pushed work would
    // recompute values nobody went on to ask for.
    ExpectTrue(slot.is_fresh() != expected_to_invalidate,
               cell + (expected_to_invalidate
                           ? ": the write leaves the slot stale"
                           : ": the write leaves the slot fresh"));
    ExpectTrue(compute_count == 1,
               cell + ": the write itself recomputes nothing");

    EvaluateIfStale(slot, compute_count);
    ExpectTrue(compute_count == (expected_to_invalidate ? 2 : 1),
               cell + ": the next query recomputes " +
                   (expected_to_invalidate ? "once" : "not at all"));

    EvaluateIfStale(slot, compute_count);
    ExpectTrue(compute_count == (expected_to_invalidate ? 2 : 1),
               cell + ": and the slot is fresh again afterwards");

    // Advance the same source again after the slot has committed a non-zero
    // version. Without this second write, an implementation that reduced every
    // snapshot to "zero or non-zero" would pass: it would notice 0 -> 1, then
    // mistake every later revision for the same state.
    WriteSource(state, source, 0.75);
    ExpectTrue(slot.is_fresh() != expected_to_invalidate,
               cell + (expected_to_invalidate
                           ? ": a second write makes the slot stale again"
                           : ": a second write still leaves the slot fresh"));
    ExpectTrue(compute_count == (expected_to_invalidate ? 2 : 1),
               cell + ": the second write itself recomputes nothing");

    EvaluateIfStale(slot, compute_count);
    ExpectTrue(compute_count == (expected_to_invalidate ? 3 : 1),
               cell + ": the query after a second write recomputes " +
                   (expected_to_invalidate ? "once" : "not at all"));
    EvaluateIfStale(slot, compute_count);
    ExpectTrue(compute_count == (expected_to_invalidate ? 3 : 1),
               cell + ": the second recomputation is also followed by a hot read");
}

/// Runs one row: one dependency subset against all five sources.
template <typename Slot>
void CheckRow(const ContractRow& row) {
    const std::array<
        std::pair<MultibodyStateVersionSource, bool ContractRow::*>, 5>
        sources{{{MultibodyStateVersionSource::kGeneralizedPositions,
                  &ContractRow::generalized_positions},
                 {MultibodyStateVersionSource::kGeneralizedVelocities,
                  &ContractRow::generalized_velocities},
                 {MultibodyStateVersionSource::kFixedFramePoses,
                  &ContractRow::fixed_frame_poses},
                 {MultibodyStateVersionSource::kRigidBodyInertias,
                  &ContractRow::rigid_body_inertias},
                 {MultibodyStateVersionSource::kJointActuatorParameters,
                  &ContractRow::joint_actuator_parameters}}};
    for (const auto& [source, expectation] : sources) {
        CheckCell<Slot>(row, source, row.*expectation);
    }
}

void CheckPositionsAndVelocitiesInvalidateSeparately() {
    // Position kinematics (#4/#9) must survive a velocity write, while
    // velocity kinematics (#8/#12) must expire. Checking both distinguishes
    // separate position and velocity versions from a shared version.
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionKinematicsSlot position_slot(state, std::in_place, 4);
    VelocityKinematicsSlot velocity_slot(state, std::in_place, 4);
    int position_computes = 0;
    int velocity_computes = 0;
    EvaluateIfStale(position_slot, position_computes);
    EvaluateIfStale(velocity_slot, velocity_computes);

    state.set_generalized_velocities(Eigen::VectorXd::Constant(8, 1.0));
    ExpectTrue(position_slot.is_fresh(),
               "a velocity write leaves position kinematics fresh — q and v are "
               "not one version here");
    ExpectTrue(!velocity_slot.is_fresh(),
               "the same write does expire velocity kinematics");
    EvaluateIfStale(position_slot, position_computes);
    EvaluateIfStale(velocity_slot, velocity_computes);
    ExpectTrue(position_computes == 1 && velocity_computes == 2,
               "only the cache that reads velocities pays for a velocity write");

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    ExpectTrue(!position_slot.is_fresh() && !velocity_slot.is_fresh(),
               "a position write expires both, because both read positions");
}

}  // namespace

int main() {
    CheckRow<ReflectedInertiaSlot>(kReflectedInertia);
    CheckRow<FramePoseSlot>(kFramePoseInLinkAndBodyFrames);
    CheckRow<PositionKinematicsSlot>(kPositionKinematics);
    CheckRow<MobodSpatialInertiaSlot>(kMobodSpatialInertia);
    CheckRow<VelocityKinematicsSlot>(kVelocityKinematics);
    CheckRow<MobodDynamicBiasSlot>(kMobodDynamicBias);
    CheckRow<ArticulatedBodyInertiaSlot>(kArticulatedBodyInertia);
    CheckRow<ArticulatedBodyForceBiasSlot>(kArticulatedBodyForceBias);
    CheckPositionsAndVelocitiesInvalidateSeparately();

    if (failure_count > 0) {
        std::printf("%d dependency matrix check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "all eight dependency subsets of the cache workspace expire for exactly"
        " the sources they declare and for no others, across forty cells\n");
    return 0;
}
