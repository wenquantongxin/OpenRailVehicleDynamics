#pragma once

#include <memory>

#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

// The immutable part of a vehicle's wheel-rail contact personality that an
// assembled right-hand side consumes.
//
// Asset identities, source-facing configuration and scientific inspection
// remain the configuration layer's concern.  This narrower value owns exactly
// the two side models, fixed pose constants and rail-origin convention the
// runtime needs, so a compiled force plan can own all of its dependencies
// without depending back on configuration.

namespace orvd::wheel_rail_contact {

class WheelRailContactRuntimePersonality {
   public:
    WheelRailContactRuntimePersonality(
        std::unique_ptr<WheelRailContactModel> right_model,
        std::unique_ptr<WheelRailContactModel> left_model,
        WheelRailPoseConstants right_pose_constants,
        WheelRailPoseConstants left_pose_constants,
        RailProfileOriginMode rail_profile_origin_mode);

    WheelRailContactRuntimePersonality(
        const WheelRailContactRuntimePersonality&) = delete;
    WheelRailContactRuntimePersonality& operator=(
        const WheelRailContactRuntimePersonality&) = delete;
    WheelRailContactRuntimePersonality(WheelRailContactRuntimePersonality&&) =
        delete;
    WheelRailContactRuntimePersonality& operator=(
        WheelRailContactRuntimePersonality&&) = delete;

    [[nodiscard]] const WheelRailContactModel& model(WheelSide side) const {
        return side == WheelSide::kRight ? *right_model_ : *left_model_;
    }
    [[nodiscard]] const WheelRailPoseConstants& pose_constants(
        WheelSide side) const {
        return side == WheelSide::kRight ? right_pose_constants_
                                         : left_pose_constants_;
    }
    [[nodiscard]] RailProfileOriginMode rail_profile_origin_mode() const {
        return rail_profile_origin_mode_;
    }

   private:
    std::unique_ptr<WheelRailContactModel> right_model_;
    std::unique_ptr<WheelRailContactModel> left_model_;
    WheelRailPoseConstants right_pose_constants_;
    WheelRailPoseConstants left_pose_constants_;
    RailProfileOriginMode rail_profile_origin_mode_;
};

}  // namespace orvd::wheel_rail_contact
