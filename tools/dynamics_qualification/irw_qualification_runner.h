#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>

#include "qualification_run_summary.h"
#include "qualification_sample_clock.h"

namespace orvd::dynamics_qualification {

// One internal IRW qualification recipe. Unlike the GZ18 recipe, this type has
// no irregularity identifier: G70/G71 deliberately assemble the explicit
// no-track-irregularity scenario. It is not installed or exported.
struct IrwQualificationRunConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
    std::int64_t sample_period_nanoseconds{};
    std::optional<QualificationSampleRefinement> local_sample_refinement;
};

// Runs one passive IRW qualification in a private assembled system. A
// successful call publishes one complete directory atomically; failure leaves
// no destination and cannot mutate another qualification context.
[[nodiscard]] QualificationRunSummary RunIrwQualification(
    const IrwQualificationRunConfiguration& configuration);

}  // namespace orvd::dynamics_qualification
