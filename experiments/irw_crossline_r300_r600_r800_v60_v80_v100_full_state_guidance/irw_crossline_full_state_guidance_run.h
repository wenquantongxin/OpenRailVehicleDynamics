#pragma once

#include <cstddef>
#include <filesystem>

namespace orvd::experiments::irw_crossline_full_state_guidance {

/// Compact outcome of the fixed 100-second cross-line experiment.
struct IrwCrosslineFullStateGuidanceRunSummary final {
    std::size_t observation_count{};
    std::size_t contact_patch_observation_count{};
    std::size_t control_event_count{};
    std::size_t backend_synchronization_count{};
    double advance_and_synchronization_wall_seconds{};
    double control_wall_seconds{};
    double observation_and_streaming_wall_seconds{};
    double finalization_wall_seconds{};
};

/// Runs the experiment from the installed ORVD assets below `orvd_data_root`.
///
/// The output path must not exist. Files are first written to an exact sibling
/// partial directory and become visible together through one final rename.
[[nodiscard]] IrwCrosslineFullStateGuidanceRunSummary
RunIrwCrosslineR300R600R800V60V80V100FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory);

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
