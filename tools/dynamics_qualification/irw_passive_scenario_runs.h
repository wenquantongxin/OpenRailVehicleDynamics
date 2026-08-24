#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include <Eigen/Core>

#include "qualification_run_summary.h"
#include "time_integrator_qualification_case.h"

namespace orvd::dynamics_qualification {

inline constexpr std::string_view
    kIrwR300NoIrregularityV60PassiveScenarioIdentifier =
        "irw_r300_no_irregularity_v60_passive";
inline constexpr std::string_view
    kIrwR300Aar5V60PassiveScenarioIdentifier =
        "irw_r300_aar5_v60_passive";
inline constexpr std::string_view
    kIrwStraightAar5V80PassiveScenarioIdentifier =
        "irw_straight_aar5_v80_passive";
inline constexpr std::string_view
    kIrwR600Aar5V80PassiveScenarioIdentifier =
        "irw_r600_aar5_v80_passive";
inline constexpr std::string_view
    kIrwR800Aar5V100PassiveScenarioIdentifier =
        "irw_r800_aar5_v100_passive";
inline constexpr std::string_view
    kIrwStraightAar6V120PassiveScenarioIdentifier =
        "irw_straight_aar6_v120_passive";
inline constexpr std::string_view
    kIrwR1000Aar6V120PassiveScenarioIdentifier =
        "irw_r1000_aar6_v120_passive";
inline constexpr std::string_view
    kIrwStraightAar6V160PassiveScenarioIdentifier =
        "irw_straight_aar6_v160_passive";
inline constexpr std::string_view
    kIrwStraightErriLowV200PassiveScenarioIdentifier =
        "irw_straight_erri_low_v200_passive";

// One internal passive IRW run configuration. The scenario identity binds the
// line, irregularity requirement, initial speed and private numerical recipe.
// The 80, 100, 120, 160 and 200 km/h identities additionally bind eight
// explicit wheel rates derived by scaling the bundled 60 km/h resolved state.
// The runner does not infer an identity from an irregularity name. This type
// is not installed or exported.
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
    std::optional<TimeIntegratorQualificationCase>
        time_integrator_qualification_case;
};

// Runs one passive IRW qualification in a private assembled system. A
// successful call publishes one complete directory atomically; failure leaves
// no destination and cannot mutate another qualification context.
[[nodiscard]] QualificationRunSummary RunIrwPassiveScenario(
    const IrwPassiveScenarioRunConfiguration& configuration);

}  // namespace orvd::dynamics_qualification
