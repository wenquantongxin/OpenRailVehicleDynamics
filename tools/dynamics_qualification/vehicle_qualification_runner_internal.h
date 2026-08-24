#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "qualification_run_summary.h"
#include "qualification_sample_clock.h"
#include "time_integrator_qualification_case.h"

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "system_continuous_state_integration_recipe.h"

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
    std::optional<TimeIntegratorQualificationCase>
        time_integrator_qualification_case;
};

// A closed, private recipe used only by the two migration executables. It is
// deliberately not a public arbitrary-vehicle simulation interface.
struct VehicleQualificationRecipe final {
    constexpr VehicleQualificationRecipe(
        std::string_view vehicle_label_value,
        std::array<std::string_view, kRepresentativeBodyCount>
            representative_body_names_value,
        double relative_tolerance_value,
        double generalized_position_absolute_tolerance_value,
        double generalized_velocity_absolute_tolerance_value,
        double series_force_absolute_tolerance_newtons_value,
        integrators::internal::SystemContinuousStateIntegrationRecipe
            default_integration_recipe_value,
        int expected_generalized_position_count_value,
        int expected_generalized_velocity_count_value,
        int expected_series_force_state_count_value,
        int expected_vehicle_wrench_count_value,
        TrackIrregularityRequirement track_irregularity_requirement_value,
        ScenarioAssembler assemble_scenario_value)
        : vehicle_label(vehicle_label_value),
          representative_body_names(representative_body_names_value),
          relative_tolerance(relative_tolerance_value),
          generalized_position_absolute_tolerance(
              generalized_position_absolute_tolerance_value),
          generalized_velocity_absolute_tolerance(
              generalized_velocity_absolute_tolerance_value),
          series_force_absolute_tolerance_newtons(
              series_force_absolute_tolerance_newtons_value),
          default_integration_recipe(default_integration_recipe_value),
          expected_generalized_position_count(
              expected_generalized_position_count_value),
          expected_generalized_velocity_count(
              expected_generalized_velocity_count_value),
          expected_series_force_state_count(
              expected_series_force_state_count_value),
          expected_vehicle_wrench_count(
              expected_vehicle_wrench_count_value),
          track_irregularity_requirement(
              track_irregularity_requirement_value),
          assemble_scenario(assemble_scenario_value) {}
    VehicleQualificationRecipe() = delete;

    std::string_view vehicle_label;
    std::array<std::string_view, kRepresentativeBodyCount>
        representative_body_names;
    double relative_tolerance{};
    double generalized_position_absolute_tolerance{};
    double generalized_velocity_absolute_tolerance{};
    double series_force_absolute_tolerance_newtons{};
    integrators::internal::SystemContinuousStateIntegrationRecipe
        default_integration_recipe;
    int expected_generalized_position_count{};
    int expected_generalized_velocity_count{};
    int expected_series_force_state_count{};
    int expected_vehicle_wrench_count{};
    TrackIrregularityRequirement track_irregularity_requirement{};
    ScenarioAssembler assemble_scenario{};
};

// Observes one real OpenMP team at the contact batch's fixed eight-interface
// request. A multi-worker request that the runtime serializes is rejected;
// a resolved one-worker request remains a valid serial execution identity.
// The returned value is the number of distinct workers actually observed.
[[nodiscard]] int RequireRealContactBatchParallelTeam();

[[nodiscard]] QualificationRunSummary RunVehicleQualification(
    const QualificationRunConfiguration& configuration,
    const VehicleQualificationRecipe& recipe);

}  // namespace orvd::dynamics_qualification::internal
