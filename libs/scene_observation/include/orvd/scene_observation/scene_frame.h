#pragma once

/// @file
/// One sampled scene frame: its identity in the run's timeline, every body's
/// world-frame state and the scalars observed at the same sample.

#include <cstdint>
#include <vector>

#include "orvd/scene_observation/body_state.h"
#include "orvd/scene_observation/scalar_definition.h"

namespace orvd::scene_observation {

/// Where a sample sits in the run's data timeline. This is sampling identity,
/// not a presentation state.
enum class SamplePhase : std::uint8_t {
    kInitialAcceptedState = 0,
    kDenseIntermediateSample = 1,
    kAcceptedEndpoint = 2,
};

struct SceneFrameIdentity {
    /// The integrator time of the sample, bit for bit.
    double time_seconds{0.0};
    /// The run's integer clock tick for this sample, or -1 when the run has no
    /// integer clock.
    std::int64_t time_nanoseconds{-1};
    /// The run's sample ordinal, or -1 when the run has none.
    std::int64_t sample_index{-1};
    SamplePhase phase{SamplePhase::kInitialAcceptedState};
};

struct SceneFrame {
    SceneFrameIdentity identity;
    /// One state per body, in the slot order of `SceneTopology::bodies`.
    std::vector<BodyState> bodies;
    /// The unwrapped rotation of each wheel about its
    /// `spin_axis_in_wheel_body_frame`, in radians, one per
    /// `SceneTopology::wheel_placements` entry, continuous in time so that the
    /// difference between two frames is the true rotation including whole
    /// turns. Empty when the run provides none; a wheel's orientation in
    /// `bodies` already contains its spin, so this value only tells a display
    /// which way and how far the wheel turned between samples.
    std::vector<double> wheel_spin_angles_radians;
    /// One value and status per scalar definition, in definition order.
    ScalarValues scalars;
};

}  // namespace orvd::scene_observation
