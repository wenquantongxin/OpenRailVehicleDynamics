#pragma once

#include <cmath>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string_view>

#include "system_continuous_state_integration_recipe.h"

namespace orvd::dynamics_qualification {

// One source-tree-only comparison coordinate. A case composes a real backend
// with a named refinement tier; it is not an installed integrator selector.
enum class TimeIntegratorQualificationBackend {
    kScenarioDefaultCvode,
    kRadau5,
};

enum class TimeIntegratorQualificationToleranceTier {
    kCoarse,
    kNominal,
    kFine,
    kReference,
};

struct TimeIntegratorQualificationCase final {
    constexpr TimeIntegratorQualificationCase(
        TimeIntegratorQualificationBackend backend_value,
        TimeIntegratorQualificationToleranceTier tolerance_tier_value)
        : backend(backend_value), tolerance_tier(tolerance_tier_value) {}
    TimeIntegratorQualificationCase() = delete;

    TimeIntegratorQualificationBackend backend;
    TimeIntegratorQualificationToleranceTier tolerance_tier;

    friend constexpr bool operator==(
        const TimeIntegratorQualificationCase&,
        const TimeIntegratorQualificationCase&) = default;
};

struct ResolvedTimeIntegratorQualificationNumerics final {
    integrators::internal::SystemContinuousStateIntegrationRecipe
        integration_recipe;
    std::optional<TimeIntegratorQualificationCase> qualification_case;
    std::string_view qualification_case_identifier;
    std::string_view tolerance_tier_identifier;
    double tolerance_scale_from_scenario_recipe;
    double relative_tolerance;
    double generalized_position_absolute_tolerance;
    double generalized_velocity_absolute_tolerance;
    double series_force_absolute_tolerance_newtons;
};

[[nodiscard]] constexpr std::string_view
TimeIntegratorQualificationToleranceTierIdentifier(
    TimeIntegratorQualificationToleranceTier tier) {
    switch (tier) {
        case TimeIntegratorQualificationToleranceTier::kCoarse:
            return "coarse";
        case TimeIntegratorQualificationToleranceTier::kNominal:
            return "nominal";
        case TimeIntegratorQualificationToleranceTier::kFine:
            return "fine";
        case TimeIntegratorQualificationToleranceTier::kReference:
            return "reference";
    }
    throw std::invalid_argument(
        "time-integrator qualification: unsupported tolerance tier");
}

[[nodiscard]] constexpr double TimeIntegratorQualificationToleranceScale(
    TimeIntegratorQualificationToleranceTier tier) {
    switch (tier) {
        case TimeIntegratorQualificationToleranceTier::kCoarse:
            return 10.0;
        case TimeIntegratorQualificationToleranceTier::kNominal:
            return 1.0;
        case TimeIntegratorQualificationToleranceTier::kFine:
            return 0.1;
        case TimeIntegratorQualificationToleranceTier::kReference:
            return 0.01;
    }
    throw std::invalid_argument(
        "time-integrator qualification: unsupported tolerance tier");
}

[[nodiscard]] constexpr std::string_view
TimeIntegratorQualificationCaseIdentifier(
    TimeIntegratorQualificationCase qualification_case) {
    switch (qualification_case.backend) {
        case TimeIntegratorQualificationBackend::kScenarioDefaultCvode:
            switch (qualification_case.tolerance_tier) {
                case TimeIntegratorQualificationToleranceTier::kCoarse:
                    return "scenario_default_cvode_coarse";
                case TimeIntegratorQualificationToleranceTier::kNominal:
                    return "scenario_default_cvode_nominal";
                case TimeIntegratorQualificationToleranceTier::kFine:
                    return "scenario_default_cvode_fine";
                case TimeIntegratorQualificationToleranceTier::kReference:
                    return "scenario_default_cvode_reference";
            }
            break;
        case TimeIntegratorQualificationBackend::kRadau5:
            switch (qualification_case.tolerance_tier) {
                case TimeIntegratorQualificationToleranceTier::kCoarse:
                    return "radau5_coarse";
                case TimeIntegratorQualificationToleranceTier::kNominal:
                    return "radau5_nominal";
                case TimeIntegratorQualificationToleranceTier::kFine:
                    return "radau5_fine";
                case TimeIntegratorQualificationToleranceTier::kReference:
                    return "radau5_reference";
            }
            break;
    }
    throw std::invalid_argument(
        "time-integrator qualification: unsupported case");
}

[[nodiscard]] constexpr std::optional<TimeIntegratorQualificationCase>
ParseTimeIntegratorQualificationCase(std::string_view identifier) {
    for (const auto backend : {
             TimeIntegratorQualificationBackend::kScenarioDefaultCvode,
             TimeIntegratorQualificationBackend::kRadau5}) {
        for (const auto tier : {
                 TimeIntegratorQualificationToleranceTier::kCoarse,
                 TimeIntegratorQualificationToleranceTier::kNominal,
                 TimeIntegratorQualificationToleranceTier::kFine,
                 TimeIntegratorQualificationToleranceTier::kReference}) {
            const TimeIntegratorQualificationCase candidate{backend, tier};
            if (TimeIntegratorQualificationCaseIdentifier(candidate) ==
                identifier) {
                return candidate;
            }
        }
    }
    return std::nullopt;
}

[[nodiscard]] inline ResolvedTimeIntegratorQualificationNumerics
ResolveTimeIntegratorQualificationNumerics(
    std::optional<TimeIntegratorQualificationCase> qualification_case,
    integrators::internal::SystemContinuousStateIntegrationRecipe
        scenario_default_integration_recipe,
    double relative_tolerance,
    double generalized_position_absolute_tolerance,
    double generalized_velocity_absolute_tolerance,
    double series_force_absolute_tolerance_newtons) {
    using IntegrationRecipe =
        integrators::internal::SystemContinuousStateIntegrationRecipe;
    IntegrationRecipe integration_recipe = scenario_default_integration_recipe;
    double scale = 1.0;
    std::string_view case_identifier;
    std::string_view tier_identifier = "scenario_default";
    if (qualification_case.has_value()) {
        switch (qualification_case->backend) {
            case TimeIntegratorQualificationBackend::kScenarioDefaultCvode:
                if (scenario_default_integration_recipe !=
                        IntegrationRecipe::kCvodeBdf2 &&
                    scenario_default_integration_recipe !=
                        IntegrationRecipe::kCvodeBdf5) {
                    throw std::logic_error(
                        "time-integrator qualification: the scenario default "
                        "is not CVODE");
                }
                break;
            case TimeIntegratorQualificationBackend::kRadau5:
                integration_recipe = IntegrationRecipe::kRadau5;
                break;
        }
        scale = TimeIntegratorQualificationToleranceScale(
            qualification_case->tolerance_tier);
        case_identifier =
            TimeIntegratorQualificationCaseIdentifier(*qualification_case);
        tier_identifier = TimeIntegratorQualificationToleranceTierIdentifier(
            qualification_case->tolerance_tier);
    }

    ResolvedTimeIntegratorQualificationNumerics result{
        integration_recipe,
        qualification_case,
        case_identifier,
        tier_identifier,
        scale,
        relative_tolerance * scale,
        generalized_position_absolute_tolerance * scale,
        generalized_velocity_absolute_tolerance * scale,
        series_force_absolute_tolerance_newtons * scale,
    };
    for (const double tolerance : {
             result.relative_tolerance,
             result.generalized_position_absolute_tolerance,
             result.generalized_velocity_absolute_tolerance,
             result.series_force_absolute_tolerance_newtons}) {
        if (!std::isfinite(tolerance) || !(tolerance > 0.0)) {
            throw std::invalid_argument(
                "time-integrator qualification: a resolved tolerance is not "
                "positive and finite");
        }
    }
    return result;
}

}  // namespace orvd::dynamics_qualification
