#pragma once

#include <cstddef>
#include <vector>

#include "orvd/wheel_rail_contact/profile_points.h"

// The two admitted ways of turning an authored wheel profile into the outline
// the contact geometry samples, and the policy that picks one.
//
// Both exist because two reference vehicles were qualified against two
// different preparations of the same kind of asset, and reproducing either one
// means reproducing its preparation. They are not a compatibility layer and
// neither is the successor of the other: when a single preparation is qualified
// for both vehicles, the losing one is deleted together with this selection
// layer in one change.
//
//   * Equal arc-length rescan. The authored points are replaced by points laid
//     at a constant step along the curve itself, and a second spline is built
//     through those. Where the profile turns sharply the new points crowd
//     together, so the outline resolves the flange root at the same density as
//     the tread instead of at the density the asset author happened to write.
//     The rescan runs on the physical right-hand ordering and the mirror is
//     applied to its result; running it on an already-mirrored list moves the
//     phase of the whole node set to the other end of the profile, which leaves
//     the right wheel correct and the left wheel silently different.
//
//   * Source lateral rediscretisation. The authored points are kept, and the
//     outline is sampled from a control polygon laid at a constant step in the
//     lateral coordinate, marching from the authored first point. Between
//     control nodes the outline is a straight line, so its slope is piecewise
//     constant. This one is sensitive to the authored order because the step
//     marches from the authored first element and the remainder therefore lands
//     at the authored last one.
//
// The policy carries both steps as numbers rather than as a closed choice, and
// refuses at construction when both are set. Making the combination
// unrepresentable in the type would look tidier and would remove the only case
// a test can point at: a rule nobody can violate is a rule nobody can show is
// enforced.

namespace orvd::wheel_rail_contact {

// Which preparation the policy resolved to.
enum class WheelProfilePreparation {
    kAuthoredNodes,
    kEqualArcLengthRescan,
    kSourceLateralRediscretisation,
};

struct WheelProfilePreprocessingConfiguration {
    // A positive step selects the equal arc-length rescan; zero disables it.
    double equal_arc_length_rescan_step_meters{0.0};
    // A positive step selects the source lateral rediscretisation; zero
    // disables it.
    double source_lateral_rediscretisation_step_meters{0.0};
};

// The outline the contact geometry consumes: a fixed number of stations
// spanning the side-resolved profile, with the surface height and its slope at
// each.
struct WheelProfileOutline {
    std::vector<double> station_meters;
    std::vector<double> height_meters;
    std::vector<double> height_slope;
};

// The nodes the chosen preparation laid down, kept beside the outline because
// they are the thing that differs between the two preparations and a consumer
// comparing them needs to see them, not infer them.
//
// Empty when no preparation is selected. For the rescan they are the replaced
// profile nodes; for the rediscretisation they are control nodes for the
// outline only and are not profile nodes at all.
struct WheelProfileControlNodes {
    std::vector<double> lateral_meters;
    std::vector<double> vertical_meters;
    // The curve length the rescan measured, in metres. Zero for the other two
    // preparations, which never measure it.
    double total_arc_length_meters{0.0};
};

class WheelProfilePreprocessing {
   public:
    // Throws std::invalid_argument when either step is not finite or is
    // negative, and when both steps are positive: the two preparations rewrite
    // the same outline and stacking them has no meaning.
    explicit WheelProfilePreprocessing(
        WheelProfilePreprocessingConfiguration configuration);

    [[nodiscard]] WheelProfilePreparation preparation() const {
        return preparation_;
    }
    [[nodiscard]] double step_meters() const { return step_meters_; }

    // Lays the control nodes this preparation calls for.
    //
    // Throws std::invalid_argument when the profile is not a wheel profile, and
    // std::runtime_error when the authored points do not admit the spline the
    // rescan needs.
    [[nodiscard]] WheelProfileControlNodes LayControlNodes(
        const ProfilePoints& authored, WheelSide side) const;

    // Samples the visible outline at `sample_count` stations spanning the
    // side-resolved profile.
    //
    // This is the observable the two preparations share: they produce different
    // kinds of artefact, and this is the one place their results are directly
    // comparable point by point.
    //
    // Throws std::invalid_argument when the profile is not a wheel profile or
    // when fewer than two stations are asked for.
    [[nodiscard]] WheelProfileOutline SampleVisibleOutline(
        const ProfilePoints& authored, WheelSide side,
        std::size_t sample_count) const;

   private:
    WheelProfilePreparation preparation_{WheelProfilePreparation::kAuthoredNodes};
    double step_meters_{0.0};
};

}  // namespace orvd::wheel_rail_contact
