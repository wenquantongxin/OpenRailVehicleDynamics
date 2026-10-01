#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <variant>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/integrators/system_integration_configuration.h"
#include "time_integrator_qualification_case.h"

namespace orvd::dynamics_qualification {

struct ScenarioDefaultRequest final {};

enum class OdeIntegrationMethod { kCvodeBdf2, kCvodeBdf5, kRadau5 };

struct ExplicitOdeRequest final {
    OdeIntegrationMethod method;
    double relative_tolerance;
    double generalized_position_absolute_tolerance;
    double generalized_velocity_absolute_tolerance;
    double series_force_absolute_tolerance_newtons;
};

using integrators::CoordinateNewtonScales;
using integrators::SeriesForceNewtonScales;

struct NewmarkRequest final {
    std::uint64_t step_size_nanoseconds{};
    int maximum_iterations{12};
    CoordinateNewtonScales translation;
    CoordinateNewtonScales angle;
    CoordinateNewtonScales quaternion;
    SeriesForceNewtonScales force;
};

struct ZhaiRequest final {
    std::uint64_t step_size_nanoseconds{};
};

struct IntegrationRequest final {
    std::variant<ScenarioDefaultRequest, TimeIntegratorQualificationCase,
                 ExplicitOdeRequest, NewmarkRequest, ZhaiRequest> method;
    std::size_t maximum_internal_steps_per_advance{integrators::kDefaultMaximumInternalStepsPerAdvance};
};

struct ScenarioOdeDefaults final {
    dynamics_qualification::QualificationIntegrationMethod recipe;
    double relative_tolerance;
    double generalized_position_absolute_tolerance;
    double generalized_velocity_absolute_tolerance;
    double series_force_absolute_tolerance_newtons;
};

struct ResolvedIntegrationConfiguration final {
    integrators::SystemIntegrationConfiguration
        configuration;
    nlohmann::json metadata;
    // Compatibility data only for the scenario default and eight legacy ODE
    // cases. An explicit ODE request has no legacy tier/case identity.
    std::optional<ResolvedTimeIntegratorQualificationNumerics> ode_numerics;
    std::optional<std::uint64_t> step_size_nanoseconds;
};

// Each of the five JSON method tags has its own closed field set. The eight
// legacy ODE cases retain their existing CLI spelling and numerical meaning.
[[nodiscard]] IntegrationRequest ReadIntegrationConfiguration(
    const std::filesystem::path& path);
[[nodiscard]] IntegrationRequest RequestIntegrationConfiguration(
    const std::optional<std::filesystem::path>& configuration_path,
    const std::optional<TimeIntegratorQualificationCase>& legacy_case);
[[nodiscard]] nlohmann::json RequestMetadata(const IntegrationRequest& request);

// Resolves scenario defaults and physical tolerances into the public contract.
// The library binds Newton scale families to the model; the tool records only
// declared families and the independent output state layout.
[[nodiscard]] ResolvedIntegrationConfiguration ResolveIntegrationConfiguration(
    const IntegrationRequest& request,
    const configuration::AssembledVehicleSystem& assembled,
    const Eigen::Ref<const Eigen::VectorXd>& initial_physical_state,
    const ScenarioOdeDefaults& scenario_defaults);

}  // namespace orvd::dynamics_qualification
