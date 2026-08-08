#include "orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::wheel_rail_contact {
namespace {

void RequirePoseConstants(const WheelRailPoseConstants& constants,
                          const char* side) {
    if (!std::isfinite(constants.rail_lateral_datum_meters) ||
        !std::isfinite(constants.rail_vertical_datum_meters) ||
        !std::isfinite(constants.rail_roll_radians) ||
        !std::isfinite(constants.wheel_lateral_datum_meters) ||
        !std::isfinite(constants.nominal_rolling_radius_meters) ||
        !(constants.nominal_rolling_radius_meters > 0.0)) {
        throw std::invalid_argument(
            std::string("wheel-rail runtime personality: the ") + side +
            " pose constants are not finite with a positive nominal rolling "
            "radius");
    }
}

}  // namespace

WheelRailContactRuntimePersonality::WheelRailContactRuntimePersonality(
    std::unique_ptr<WheelRailContactModel> right_model,
    std::unique_ptr<WheelRailContactModel> left_model,
    WheelRailPoseConstants right_pose_constants,
    WheelRailPoseConstants left_pose_constants,
    ProfileTrackRollTransportStrategy roll_transport_strategy,
    RailProfileOriginMode rail_profile_origin_mode)
    : right_model_(std::move(right_model)),
      left_model_(std::move(left_model)),
      right_pose_constants_(right_pose_constants),
      left_pose_constants_(left_pose_constants),
      roll_transport_strategy_(roll_transport_strategy),
      rail_profile_origin_mode_(rail_profile_origin_mode) {
    if (right_model_ == nullptr || left_model_ == nullptr) {
        throw std::invalid_argument(
            "wheel-rail runtime personality: both side models are required");
    }
    RequirePoseConstants(right_pose_constants_, "right");
    RequirePoseConstants(left_pose_constants_, "left");
}

}  // namespace orvd::wheel_rail_contact
