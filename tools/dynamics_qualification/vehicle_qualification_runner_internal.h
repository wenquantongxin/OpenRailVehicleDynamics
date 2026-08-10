#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "qualification_run_summary.h"
#include "qualification_sample_clock.h"

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"

namespace orvd::dynamics_qualification::internal {

inline constexpr std::size_t kRepresentativeBodyCount = 3;

using ScenarioAssembler =
    decltype(&configuration::AssembleGz18ContactScenario);

enum class TrackIrregularityRequirement {
    kForbidden,
    kRequired,
};

struct QualificationRunConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::optional<std::string> track_irregularity_identifier;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
    std::int64_t sample_period_nanoseconds{};
    std::optional<QualificationSampleRefinement> local_sample_refinement;
};

// A closed, private recipe used only by the two migration executables. It is
// deliberately not a public arbitrary-vehicle simulation interface.
struct VehicleQualificationRecipe final {
    std::string_view vehicle_label;
    std::array<std::string_view, kRepresentativeBodyCount>
        representative_body_names;
    double relative_tolerance{};
    double generalized_position_absolute_tolerance{};
    double generalized_velocity_absolute_tolerance{};
    double series_force_absolute_tolerance_newtons{};
    int expected_generalized_position_count{};
    int expected_generalized_velocity_count{};
    int expected_series_force_state_count{};
    int expected_vehicle_wrench_count{};
    TrackIrregularityRequirement track_irregularity_requirement{};
    ScenarioAssembler assemble_scenario{};
};

[[nodiscard]] QualificationRunSummary RunVehicleQualification(
    const QualificationRunConfiguration& configuration,
    const VehicleQualificationRecipe& recipe);

}  // namespace orvd::dynamics_qualification::internal
