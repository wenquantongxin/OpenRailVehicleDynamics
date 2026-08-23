#pragma once

/// @file
/// Curve operating points selected by the source better2 cross-line schedule.

#include <string_view>

#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

/// One constant curve-region operating point, independent of route and speed.
struct IrwCurveFullStateGuidanceProfile final {
    std::string_view identity;
    control::IrwGuidanceAxleValues feedforward_gains{};
    control::IrwGuidanceAxleValues yaw_rate_feedback_gains{};
    control::IrwGuidanceAxleValues yaw_feedback_gains{};
    control::IrwGuidanceAxleValues lateral_velocity_feedback_gains{};
    control::IrwGuidanceAxleValues lateral_displacement_feedback_gains{};
    control::IrwGuidanceAxleValues lateral_integral_feedback_gains{};
    control::IrwGuidanceAxleValues wheel_speed_difference_feedback_gains{};
    control::IrwGuidanceAxleValues yaw_rate_filter_time_constants_seconds{};
    control::IrwGuidanceAxleValues
        lateral_velocity_filter_time_constants_seconds{};
    control::IrwGuidanceAxleValues equilibrium_yaw_angles_radians{};
    control::IrwGuidanceAxleValues
        equilibrium_lateral_displacements_meters{};
    double wheel_speed_difference_outer_filter_time_constant_seconds{};
    double wheel_speed_difference_reference_absolute_limit_radians_per_second{};
};

inline constexpr IrwCurveFullStateGuidanceProfile kBetter2R300Profile{
    .identity = "r300_pd_m30_m6",
    .feedforward_gains = {2.2, 0.1, -1.45, 1.6},
    .yaw_rate_feedback_gains = {-6.0, 0.0, 0.0, 0.0},
    .yaw_feedback_gains = {-30.0, 0.0, 0.0, 1.3},
    .lateral_velocity_feedback_gains = {},
    .lateral_displacement_feedback_gains = {},
    .lateral_integral_feedback_gains = {0.15, 0.15, 0.15, 0.15},
    .wheel_speed_difference_feedback_gains = {0.3, 0.3, 0.3, 0.81},
    .yaw_rate_filter_time_constants_seconds = {0.002, 0.002, 0.002, 0.002},
    .lateral_velocity_filter_time_constants_seconds =
        {0.002, 0.002, 0.002, 0.002},
    .equilibrium_yaw_angles_radians = {0.0, 0.0, 0.0, 0.00284688},
    .equilibrium_lateral_displacements_meters =
        {0.01337, -0.01175, 0.00854, 0.01316},
    .wheel_speed_difference_outer_filter_time_constant_seconds = 0.026,
    .wheel_speed_difference_reference_absolute_limit_radians_per_second = 1.0,
};

/// R300 wear champion recorded by the historical SCBP campaign.  The same
/// operating point is present in each archived champion schedule that names
/// `r300_wear_champion_t320`.
inline constexpr IrwCurveFullStateGuidanceProfile kScbpR300Profile{
    .identity = "r300_wear_champion_t320",
    .feedforward_gains = {3.8, 0.1, -1.45, 1.6},
    .yaw_rate_feedback_gains = {},
    .yaw_feedback_gains = {0.0, 0.0, 0.0, 1.3},
    .lateral_velocity_feedback_gains = {},
    .lateral_displacement_feedback_gains = {},
    .lateral_integral_feedback_gains = {0.15, 0.15, 0.15, 0.15},
    .wheel_speed_difference_feedback_gains = {0.3, 0.3, 0.3, 0.81},
    .yaw_rate_filter_time_constants_seconds = {0.002, 0.002, 0.002, 0.002},
    .lateral_velocity_filter_time_constants_seconds =
        {0.002, 0.002, 0.002, 0.002},
    .equilibrium_yaw_angles_radians = {0.0, 0.0, 0.0, 0.00284688},
    .equilibrium_lateral_displacements_meters =
        {0.01337, -0.01175, 0.00854, 0.01316},
    .wheel_speed_difference_outer_filter_time_constant_seconds = 0.026,
    .wheel_speed_difference_reference_absolute_limit_radians_per_second = 1.0,
};

inline constexpr IrwCurveFullStateGuidanceProfile kBetter2R600Profile{
    .identity = "r600_wear_champion_t343",
    .feedforward_gains = {3.85, 0.0, -1.0, 1.5},
    .yaw_rate_feedback_gains = {0.02, 0.0, -0.04, 0.0},
    .yaw_feedback_gains = {0.0, 0.0, 0.0, 0.9},
    .lateral_velocity_feedback_gains = {},
    .lateral_displacement_feedback_gains = {},
    .lateral_integral_feedback_gains = {0.15, 0.15, 0.15, 0.15},
    .wheel_speed_difference_feedback_gains = {0.3, 0.3, 0.3, 0.45},
    .yaw_rate_filter_time_constants_seconds = {0.002, 0.002, 0.002, 0.002},
    .lateral_velocity_filter_time_constants_seconds =
        {0.002, 0.002, 0.002, 0.002},
    .equilibrium_yaw_angles_radians = {0.0, 0.0, 0.0, 0.004},
    .equilibrium_lateral_displacements_meters =
        {0.01337, -0.01175, 0.00854, 0.01316},
    .wheel_speed_difference_outer_filter_time_constant_seconds = 0.03,
    .wheel_speed_difference_reference_absolute_limit_radians_per_second = 1.0,
};

inline constexpr IrwCurveFullStateGuidanceProfile kBetter2R800Profile{
    .identity = "r800_wear_champion_t853",
    .feedforward_gains = {3.4, 0.1, -1.05, 1.5},
    .yaw_rate_feedback_gains = {0.05, 0.0, 0.0, -0.1},
    .yaw_feedback_gains = {0.1, 0.0, 0.0, 0.7},
    .lateral_velocity_feedback_gains = {},
    .lateral_displacement_feedback_gains = {},
    .lateral_integral_feedback_gains = {0.15, 0.15, 0.15, 0.15},
    .wheel_speed_difference_feedback_gains = {0.45, 0.15, 0.21, 0.18},
    .yaw_rate_filter_time_constants_seconds = {0.002, 0.002, 0.002, 0.002},
    .lateral_velocity_filter_time_constants_seconds =
        {0.002, 0.002, 0.002, 0.002},
    .equilibrium_yaw_angles_radians = {0.0, 0.0, 0.0, 0.004},
    .equilibrium_lateral_displacements_meters =
        {0.01337, -0.01175, 0.00854, 0.01316},
    .wheel_speed_difference_outer_filter_time_constant_seconds = 0.03,
    .wheel_speed_difference_reference_absolute_limit_radians_per_second = 1.0,
};

/// The historical SCBP champions selected for R600 and R800 are exactly the
/// same operating points retained by better2.  Keep aliases instead of a
/// second transcription so the duplicated experiment arms cannot drift.
inline constexpr auto kScbpR600Profile = kBetter2R600Profile;
inline constexpr auto kScbpR800Profile = kBetter2R800Profile;

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
