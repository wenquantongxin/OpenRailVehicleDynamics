#include "qualification_integration_configuration.h"

#include "qualification_state_layout.h"

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace orvd::dynamics_qualification {
namespace {
using Json = nlohmann::json;
using Recipe = dynamics_qualification::QualificationIntegrationMethod;

[[noreturn]] void Invalid(const std::string& message) {
    throw std::invalid_argument("qualification integration configuration: " + message);
}

void Positive(double value, std::string_view field) {
    if (!std::isfinite(value) || value <= 0.0) {
        Invalid(std::string(field) + " must be positive and finite");
    }
}

double StepSeconds(std::uint64_t nanoseconds) {
    if (nanoseconds == 0) Invalid("step_size_nanoseconds must be positive");
    const double seconds = static_cast<double>(nanoseconds) * 1e-9;
    Positive(seconds, "step_size_seconds");
    return seconds;
}

void Fields(const Json& value, std::initializer_list<std::string_view> required,
            std::initializer_list<std::string_view> optional = {}) {
    if (!value.is_object()) Invalid("expected a JSON object");
    std::set<std::string> allowed;
    for (const auto name : required) {
        allowed.emplace(name);
        if (!value.contains(std::string(name))) Invalid("missing field " + std::string(name));
    }
    for (const auto name : optional) allowed.emplace(name);
    for (const auto& item : value.items()) {
        if (!allowed.contains(item.key())) Invalid("unknown field " + item.key());
    }
}

std::uint64_t PositiveInteger(const Json& value, const char* field) {
    if (!value.is_number_integer() ||
        (!value.is_number_unsigned() && value.get<std::int64_t>() <= 0)) {
        Invalid(std::string(field) + " must be a positive integer");
    }
    const auto result = value.get<std::uint64_t>();
    if (result == 0) Invalid(std::string(field) + " must be a positive integer");
    return result;
}

double PositiveNumber(const Json& value, const char* field) {
    if (!value.is_number()) Invalid(std::string(field) + " must be a number");
    const double result = value.get<double>();
    Positive(result, field);
    return result;
}

Recipe OdeRecipe(OdeIntegrationMethod method) {
    switch (method) {
        case OdeIntegrationMethod::kCvodeBdf2: return Recipe::kCvodeBdf2;
        case OdeIntegrationMethod::kCvodeBdf5: return Recipe::kCvodeBdf5;
        case OdeIntegrationMethod::kRadau5: return Recipe::kRadau5;
    }
    Invalid("unsupported explicit ODE method");
}

void ExplicitOdeMetadata(const ExplicitOdeRequest& request, Json& result) {
    const auto recipe = OdeRecipe(request.method);
    Positive(request.relative_tolerance, "relative_tolerance");
    Positive(request.generalized_position_absolute_tolerance,
             "generalized_position_absolute_tolerance");
    Positive(request.generalized_velocity_absolute_tolerance,
             "generalized_velocity_absolute_tolerance");
    Positive(request.series_force_absolute_tolerance_newtons,
             "series_force_absolute_tolerance_newtons");
    result["integrator_recipe_identifier"] = dynamics_qualification::IntegrationRecipeIdentifier(recipe);
    result["relative_tolerance"] = request.relative_tolerance;
    result["generalized_position_absolute_tolerance"] = request.generalized_position_absolute_tolerance;
    result["generalized_velocity_absolute_tolerance"] = request.generalized_velocity_absolute_tolerance;
    result["series_force_absolute_tolerance_newtons"] = request.series_force_absolute_tolerance_newtons;
    if (const auto order = dynamics_qualification::MaximumBdfOrderForRecipe(recipe)) {
        result["maximum_bdf_order"] = *order;
    }
}

integrators::ContinuousStateErrorTolerances PhysicalTolerances(
    const system_assembly::SystemInstance& system, double relative,
    double position, double velocity, double force) {
    Eigen::VectorXd absolute(system.continuous_state_size());
    const auto q = system.generalized_positions_state_range();
    const auto v = system.generalized_velocities_state_range();
    const auto z = system.series_spring_damper_force_state_range();
    absolute.segment(q.start(), q.size()).setConstant(position);
    absolute.segment(v.start(), v.size()).setConstant(velocity);
    absolute.segment(z.start(), z.size()).setConstant(force);
    return {relative, std::move(absolute)};
}

CoordinateNewtonScales ReadCoordinateScales(const Json& value) {
    Fields(value, {"position_correction", "velocity_correction",
                   "acceleration_residual", "acceleration_reference"});
    return {PositiveNumber(value.at("position_correction"), "position_correction"),
            PositiveNumber(value.at("velocity_correction"), "velocity_correction"),
            PositiveNumber(value.at("acceleration_residual"), "acceleration_residual"),
            PositiveNumber(value.at("acceleration_reference"), "acceleration_reference")};
}

Json CoordinateScalesJson(const CoordinateNewtonScales& scales) {
    Positive(scales.position_correction, "position_correction");
    Positive(scales.velocity_correction, "velocity_correction");
    Positive(scales.acceleration_residual, "acceleration_residual");
    Positive(scales.acceleration_reference, "acceleration_reference");
    return {{"position_correction", scales.position_correction},
            {"velocity_correction", scales.velocity_correction},
            {"acceleration_residual", scales.acceleration_residual},
            {"acceleration_reference", scales.acceleration_reference}};
}

Json NewmarkJson(const NewmarkRequest& request) {
    if (request.maximum_iterations <= 0) Invalid("maximum_iterations must be positive");
    Positive(request.force.correction, "force correction");
    Positive(request.force.residual, "force residual");
    Positive(request.force.reference, "force reference");
    return {{"maximum_iterations", request.maximum_iterations},
            {"quaternion_scale_convention", "reference_norm_multiple"},
            {"scales", {{"translation", CoordinateScalesJson(request.translation)},
                        {"angle", CoordinateScalesJson(request.angle)},
                        {"quaternion", CoordinateScalesJson(request.quaternion)},
                        {"force", {{"correction", request.force.correction},
                                   {"residual", request.force.residual},
                                   {"reference", request.force.reference}}}}}};
}

auto MakeOdeMethodConfiguration(Recipe recipe,
    integrators::ContinuousStateErrorTolerances tolerances)
    -> integrators::SystemIntegrationMethodConfiguration {
    switch (recipe) {
        case Recipe::kCvodeBdf2: return integrators::CvodeBdf2Configuration{std::move(tolerances)};
        case Recipe::kCvodeBdf5: return integrators::CvodeBdf5Configuration{std::move(tolerances)};
        case Recipe::kRadau5: return integrators::Radau5Configuration{std::move(tolerances)};
        case Recipe::kNewmark:
        case Recipe::kZhai: Invalid("ODE preset selected a mechanical method");
    }
    Invalid("unsupported ODE preset");
}
}  // namespace

IntegrationRequest ReadIntegrationConfiguration(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open integration configuration: " + path.string());
    // Reject duplicates instead of silently allowing a later value to replace
    // the configuration that a reader sees first.
    std::vector<std::set<std::string>> object_keys;
    Json document;
    try {
        document = Json::parse(input, [&](int, Json::parse_event_t event, Json& value) {
            if (event == Json::parse_event_t::object_start) object_keys.emplace_back();
            if (event == Json::parse_event_t::key &&
                !object_keys.back().insert(value.get<std::string>()).second) {
                Invalid("duplicate field " + value.get<std::string>());
            }
            if (event == Json::parse_event_t::object_end) object_keys.pop_back();
            return true;
        });
    } catch (const Json::exception& error) {
        Invalid(std::string("invalid JSON: ") + error.what());
    }
    if (!document.is_object() || !document.contains("schema_version") ||
        !document.contains("method")) {
        Invalid("expected an object with schema_version and method");
    }
    if (PositiveInteger(document.at("schema_version"), "schema_version") != 1) {
        Invalid("unsupported schema_version");
    }
    if (!document.at("method").is_string()) Invalid("method must be a string");
    const auto method = document.at("method").get<std::string>();
    const bool is_ode = method == "cvode_bdf2" || method == "cvode_bdf5" || method == "radau5";
    if (is_ode) {
        Fields(document, {"schema_version", "method", "relative_tolerance",
                          "generalized_position_absolute_tolerance",
                          "generalized_velocity_absolute_tolerance",
                          "series_force_absolute_tolerance_newtons"},
               {"maximum_internal_steps_per_advance"});
    } else if (method == "newmark") {
        Fields(document, {"schema_version", "method", "step_size_nanoseconds", "newton"},
               {"maximum_internal_steps_per_advance"});
    } else if (method == "zhai") {
        Fields(document, {"schema_version", "method", "step_size_nanoseconds"},
               {"maximum_internal_steps_per_advance"});
    } else {
        Invalid("unsupported method; expected cvode_bdf2, cvode_bdf5, radau5, newmark or zhai");
    }
    IntegrationRequest result;
    if (document.contains("maximum_internal_steps_per_advance")) {
        const auto budget = PositiveInteger(document.at("maximum_internal_steps_per_advance"),
                                            "maximum_internal_steps_per_advance");
        if (budget > std::numeric_limits<std::size_t>::max()) Invalid("advance budget is too large");
        result.maximum_internal_steps_per_advance = static_cast<std::size_t>(budget);
    }
    if (is_ode) {
        const auto ode_method = method == "cvode_bdf2" ? OdeIntegrationMethod::kCvodeBdf2 :
                                method == "cvode_bdf5" ? OdeIntegrationMethod::kCvodeBdf5 :
                                                        OdeIntegrationMethod::kRadau5;
        result.method = ExplicitOdeRequest{
            ode_method,
            PositiveNumber(document.at("relative_tolerance"), "relative_tolerance"),
            PositiveNumber(document.at("generalized_position_absolute_tolerance"),
                           "generalized_position_absolute_tolerance"),
            PositiveNumber(document.at("generalized_velocity_absolute_tolerance"),
                           "generalized_velocity_absolute_tolerance"),
            PositiveNumber(document.at("series_force_absolute_tolerance_newtons"),
                           "series_force_absolute_tolerance_newtons")};
        return result;
    }
    const auto step = PositiveInteger(document.at("step_size_nanoseconds"), "step_size_nanoseconds");
    if (method == "zhai") {
        if (document.contains("newton")) Invalid("zhai does not accept newton configuration");
        result.method = ZhaiRequest{step};
    } else if (method == "newmark") {
        if (!document.contains("newton")) Invalid("newmark requires newton configuration");
        const auto& newton = document.at("newton");
        Fields(newton, {"maximum_iterations", "quaternion_scale_convention", "scales"});
        const auto iterations = PositiveInteger(newton.at("maximum_iterations"), "maximum_iterations");
        if (iterations > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
            Invalid("maximum_iterations is too large");
        }
        if (newton.at("quaternion_scale_convention") != "reference_norm_multiple") {
            Invalid("quaternion_scale_convention must be reference_norm_multiple");
        }
        const auto& scales = newton.at("scales");
        Fields(scales, {"translation", "angle", "quaternion", "force"});
        const auto& force = scales.at("force");
        Fields(force, {"correction", "residual", "reference"});
        result.method = NewmarkRequest{
            step, static_cast<int>(iterations),
            ReadCoordinateScales(scales.at("translation")),
            ReadCoordinateScales(scales.at("angle")),
            ReadCoordinateScales(scales.at("quaternion")),
            {PositiveNumber(force.at("correction"), "force correction"),
             PositiveNumber(force.at("residual"), "force residual"),
             PositiveNumber(force.at("reference"), "force reference")}};
    }
    return result;
}

IntegrationRequest RequestIntegrationConfiguration(
    const std::optional<std::filesystem::path>& configuration_path,
    const std::optional<TimeIntegratorQualificationCase>& legacy_case) {
    if (configuration_path && legacy_case) Invalid("integration config and legacy case are mutually exclusive");
    if (configuration_path) return ReadIntegrationConfiguration(*configuration_path);
    IntegrationRequest result;
    if (legacy_case) result.method = *legacy_case;
    return result;
}

Json RequestMetadata(const IntegrationRequest& request) {
    if (request.maximum_internal_steps_per_advance == 0) Invalid("advance budget must be positive");
    Json result{{"integration_configuration_schema_version", 1},
                {"maximum_internal_steps_per_advance", request.maximum_internal_steps_per_advance},
                {"step_size_nanoseconds", nullptr}, {"step_size_seconds", nullptr},
                {"newton", nullptr}};
    for (const char* field : {"tolerance_tier_identifier", "tolerance_scale_from_scenario_recipe",
                              "relative_tolerance", "generalized_position_absolute_tolerance",
                              "generalized_velocity_absolute_tolerance", "series_force_absolute_tolerance_newtons",
                              "qualification_case_identifier", "integrator_recipe_identifier", "maximum_bdf_order"}) {
        result[field] = nullptr;
    }
    if (const auto* newmark = std::get_if<NewmarkRequest>(&request.method)) {
        result["integrator_recipe_identifier"] = "newmark";
        result["step_size_nanoseconds"] = newmark->step_size_nanoseconds;
        result["step_size_seconds"] = StepSeconds(newmark->step_size_nanoseconds);
        result["newton"] = NewmarkJson(*newmark);
    } else if (const auto* zhai = std::get_if<ZhaiRequest>(&request.method)) {
        result["integrator_recipe_identifier"] = "zhai";
        result["step_size_nanoseconds"] = zhai->step_size_nanoseconds;
        result["step_size_seconds"] = StepSeconds(zhai->step_size_nanoseconds);
    } else if (const auto* legacy = std::get_if<TimeIntegratorQualificationCase>(&request.method)) {
        result["qualification_case_identifier"] = TimeIntegratorQualificationCaseIdentifier(*legacy);
    } else if (const auto* ode = std::get_if<ExplicitOdeRequest>(&request.method)) {
        ExplicitOdeMetadata(*ode, result);
    }
    return result;
}

ResolvedIntegrationConfiguration ResolveIntegrationConfiguration(
    const IntegrationRequest& request,
    const configuration::AssembledVehicleSystem& assembled,
    const Eigen::Ref<const Eigen::VectorXd>& initial_physical_state,
    const ScenarioOdeDefaults& scenario_defaults) {
    Json metadata = RequestMetadata(request);
    const auto& system = assembled.system();
    if (initial_physical_state.size() != system.continuous_state_size() ||
        !initial_physical_state.allFinite()) Invalid("invalid initial physical state size or value");
    metadata["coordinate_layout"] = BuildQualificationStateLayout(assembled, initial_physical_state);
    const auto budget = request.maximum_internal_steps_per_advance;
    if (const auto* newmark = std::get_if<NewmarkRequest>(&request.method)) {
        integrators::NewmarkNewtonConfiguration newton{
            newmark->maximum_iterations, newmark->translation, newmark->angle,
            newmark->quaternion, newmark->force};
        const double step_seconds = metadata.at("step_size_seconds").get<double>();
        return {integrators::SystemIntegrationConfiguration{integrators::NewmarkConfiguration{
                     step_seconds, std::move(newton)}, budget},
                std::move(metadata), std::nullopt, newmark->step_size_nanoseconds};
    }
    if (const auto* zhai = std::get_if<ZhaiRequest>(&request.method)) {
        const double step_seconds = metadata.at("step_size_seconds").get<double>();
        return {integrators::SystemIntegrationConfiguration{integrators::ZhaiConfiguration{step_seconds}, budget},
                std::move(metadata), std::nullopt, zhai->step_size_nanoseconds};
    }
    if (const auto* ode = std::get_if<ExplicitOdeRequest>(&request.method)) {
        return {integrators::SystemIntegrationConfiguration{MakeOdeMethodConfiguration(
                     OdeRecipe(ode->method), PhysicalTolerances(system, ode->relative_tolerance,
                         ode->generalized_position_absolute_tolerance,
                         ode->generalized_velocity_absolute_tolerance,
                         ode->series_force_absolute_tolerance_newtons)), budget},
                std::move(metadata), std::nullopt, std::nullopt};
    }
    std::optional<TimeIntegratorQualificationCase> legacy;
    if (const auto* value = std::get_if<TimeIntegratorQualificationCase>(&request.method)) legacy = *value;
    const auto ode = ResolveTimeIntegratorQualificationNumerics(
        legacy, scenario_defaults.recipe, scenario_defaults.relative_tolerance,
        scenario_defaults.generalized_position_absolute_tolerance,
        scenario_defaults.generalized_velocity_absolute_tolerance,
        scenario_defaults.series_force_absolute_tolerance_newtons);
    metadata["tolerance_tier_identifier"] = ode.tolerance_tier_identifier;
    metadata["tolerance_scale_from_scenario_recipe"] = ode.tolerance_scale_from_scenario_recipe;
    metadata["relative_tolerance"] = ode.relative_tolerance;
    metadata["generalized_position_absolute_tolerance"] = ode.generalized_position_absolute_tolerance;
    metadata["generalized_velocity_absolute_tolerance"] = ode.generalized_velocity_absolute_tolerance;
    metadata["series_force_absolute_tolerance_newtons"] = ode.series_force_absolute_tolerance_newtons;
    metadata["integrator_recipe_identifier"] = dynamics_qualification::IntegrationRecipeIdentifier(ode.integration_recipe);
    if (const auto order = dynamics_qualification::MaximumBdfOrderForRecipe(ode.integration_recipe)) {
        metadata["maximum_bdf_order"] = *order;
    }
    return {integrators::SystemIntegrationConfiguration{MakeOdeMethodConfiguration(
                 ode.integration_recipe, PhysicalTolerances(system, ode.relative_tolerance,
                     ode.generalized_position_absolute_tolerance,
                     ode.generalized_velocity_absolute_tolerance,
                     ode.series_force_absolute_tolerance_newtons)), budget},
            std::move(metadata), ode, std::nullopt};
}

}  // namespace orvd::dynamics_qualification
