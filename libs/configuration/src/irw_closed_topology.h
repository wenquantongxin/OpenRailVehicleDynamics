#pragma once

#include <array>
#include <string_view>

#include <Eigen/Core>

#include "orvd/wheel_rail_contact/profile_points.h"

namespace orvd::configuration::internal {

struct IrwClosedInterfaceTopology {
    std::string_view interface_name;
    std::string_view carrier_body_name;
    std::string_view wheel_body_name;
    std::string_view revolute_joint_name;
    std::string_view reaction_frame_body_name;
    wheel_rail_contact::WheelSide side;
};

inline constexpr std::array<std::string_view, 4> kIrwClosedCarrierNames{
    "axlebridge_ff", "axlebridge_fr", "axlebridge_rf", "axlebridge_rr"};

// The frozen WRL/SIMPACK physical axle-bridge body basis and ORVD's retained
// WRL/Drake axle-bridge body basis differ by this half turn:
// R_TB,ORVD = R_TB,source * C. Since C == C.transpose() == C.inverse(), the
// same value restores the source body basis for control observation.
inline const Eigen::Matrix3d kIrwBodyBasisHalfTurn =
    Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();

// Frozen WRL order: each axle from front to rear, left before right. Contact,
// active torque and the G76 control observation binding all consume this one
// table; none reconstructs the order independently.
inline constexpr std::array<IrwClosedInterfaceTopology, 8>
    kIrwClosedInterfaces{{
        {"wheel_ff_l", "axlebridge_ff", "wheel_ff_l", "rev_wheel_ff_l",
         "frame_front", wheel_rail_contact::WheelSide::kLeft},
        {"wheel_ff_r", "axlebridge_ff", "wheel_ff_r", "rev_wheel_ff_r",
         "frame_front", wheel_rail_contact::WheelSide::kRight},
        {"wheel_fr_l", "axlebridge_fr", "wheel_fr_l", "rev_wheel_fr_l",
         "frame_front", wheel_rail_contact::WheelSide::kLeft},
        {"wheel_fr_r", "axlebridge_fr", "wheel_fr_r", "rev_wheel_fr_r",
         "frame_front", wheel_rail_contact::WheelSide::kRight},
        {"wheel_rf_l", "axlebridge_rf", "wheel_rf_l", "rev_wheel_rf_l",
         "frame_rear", wheel_rail_contact::WheelSide::kLeft},
        {"wheel_rf_r", "axlebridge_rf", "wheel_rf_r", "rev_wheel_rf_r",
         "frame_rear", wheel_rail_contact::WheelSide::kRight},
        {"wheel_rr_l", "axlebridge_rr", "wheel_rr_l", "rev_wheel_rr_l",
         "frame_rear", wheel_rail_contact::WheelSide::kLeft},
        {"wheel_rr_r", "axlebridge_rr", "wheel_rr_r", "rev_wheel_rr_r",
         "frame_rear", wheel_rail_contact::WheelSide::kRight},
    }};

}  // namespace orvd::configuration::internal
