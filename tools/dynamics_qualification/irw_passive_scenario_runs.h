#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include <Eigen/Core>

#include "qualification_run_summary.h"

namespace orvd::dynamics_qualification {

inline constexpr std::string_view
    kIrwR300NoIrregularityV60PassiveScenarioIdentifier =
        "irw_r300_no_irregularity_v60_passive";
inline constexpr std::string_view
    kIrwR300Aar5V60PassiveScenarioIdentifier =
        "irw_r300_aar5_v60_passive";

// One internal passive IRW run configuration. The no-irregularity recipe
// leaves the field identity empty; the R300/AAR5 recipe supplies it. The
// runner selects a closed required/forbidden recipe rather than exposing an
// arbitrary vehicle API. This type is not installed or exported.
struct IrwPassiveScenarioRunConfiguration final {
    std::string scenario_identifier;
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::optional<std::string> track_irregularity_identifier;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
    std::int64_t sample_period_nanoseconds{};
};

// Runs one passive IRW qualification in a private assembled system. A
// successful call publishes one complete directory atomically; failure leaves
// no destination and cannot mutate another qualification context.
[[nodiscard]] QualificationRunSummary RunIrwPassiveScenario(
    const IrwPassiveScenarioRunConfiguration& configuration);

}  // namespace orvd::dynamics_qualification
