#pragma once

#include <array>
#include <cstddef>
#include <filesystem>

#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

/// Compact outcome shared by the installed-API IRW guidance experiments.
struct IrwGuidanceExperimentRunSummary final {
    std::size_t observation_count{};
    std::size_t contact_patch_observation_count{};
    std::size_t control_event_count{};
    std::size_t backend_synchronization_count{};
    double simulated_duration_seconds{};
    control::IrwGuidanceAxleValues final_axle_track_stations_meters{};
    double advance_and_synchronization_wall_seconds{};
    double control_wall_seconds{};
    double observation_and_streaming_wall_seconds{};
    double finalization_wall_seconds{};
};

/// Runs the experiment from the installed ORVD assets below `orvd_data_root`.
///
/// The output path must not exist. Files are first written to an exact sibling
/// partial directory and become visible together through one final rename.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwCrosslineR300R600R800V60V80V100FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the historical normal curvature-differential wheel-speed baseline on
/// the isolated R300/V60/AAR5 route.  The terminal event is the first 10 ms
/// boundary at which all four axle stations have reached 600 m.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR300Aar5V60NormalDifferentialWheelSpeed(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the same normal baseline on the isolated R600/V80/AAR5 route. The
/// bundled resolved V60 start-up is scaled only in longitudinal speed and the
/// eight explicit independent-wheel rates.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR600Aar5V80NormalDifferentialWheelSpeed(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the same normal baseline on the isolated R800/V100/AAR5 route. The
/// bundled resolved V60 start-up is scaled only in longitudinal speed and the
/// eight explicit independent-wheel rates.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR800Aar5V100NormalDifferentialWheelSpeed(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the better2 R300 curve operating point on the isolated V60/AAR5
/// route, ending at the first control event with all axles at 600 m.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR300Aar5V60Better2FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the better2 R600 curve operating point on the isolated V80/AAR5
/// route, ending at the first control event with all axles at 600 m.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR600Aar5V80Better2FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the better2 R800 curve operating point on the isolated V100/AAR5
/// route, ending at the first control event with all axles at 600 m.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR800Aar5V100Better2FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the R300 wear champion recorded by the historical SCBP campaign on
/// the isolated V60/AAR5 route, ending at the first control event with all
/// axles at 600 m.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR300Aar5V60ScbpRecordedWearChampionFullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the R600 wear champion recorded by the historical SCBP campaign on
/// the isolated V80/AAR5 route.  Its operating point is intentionally
/// identical to the better2 R600 operating point.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR600Aar5V80ScbpRecordedWearChampionFullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

/// Runs the R800 wear champion recorded by the historical SCBP campaign on
/// the isolated V100/AAR5 route.  Its operating point is intentionally
/// identical to the better2 R800 operating point.
[[nodiscard]] IrwGuidanceExperimentRunSummary
RunIrwR800Aar5V100ScbpRecordedWearChampionFullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
