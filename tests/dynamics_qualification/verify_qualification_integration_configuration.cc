#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "orvd/configuration/load_vehicle_definition.h"
#include "qualification_integration_configuration.h"
#include "qualification_state_layout.h"

namespace {
namespace dq = orvd::dynamics_qualification;
namespace ci = orvd::integrators;
using Json = nlohmann::json;

void Require(bool condition, std::string_view message) {
    if (!condition) throw std::runtime_error(std::string(message));
}

template <class Function>
void Reject(Function&& function, std::string_view message) {
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error(std::string(message));
}

void Write(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path);
    output << text;
    Require(static_cast<bool>(output), "could not write configuration test input");
}

Json Read(const std::filesystem::path& path) {
    std::ifstream input(path);
    return Json::parse(input);
}

Json ExplicitOdeDocument(const char* method) {
    return {{"schema_version", 1}, {"method", method},
            {"relative_tolerance", 2e-11},
            {"generalized_position_absolute_tolerance", 3e-12},
            {"generalized_velocity_absolute_tolerance", 4e-12},
            {"series_force_absolute_tolerance_newtons", 5e-9},
            {"maximum_internal_steps_per_advance", 5000000}};
}


void Parsing(const std::filesystem::path& examples, const std::filesystem::path& work) {
    const auto newmark_path = examples / "newmark_explicit_trial.json";
    const auto zhai_path = examples / "zhai_explicit_trial.json";
    const auto newmark = dq::ReadIntegrationConfiguration(newmark_path);
    const auto zhai = dq::ReadIntegrationConfiguration(zhai_path);
    Require(std::holds_alternative<dq::NewmarkRequest>(newmark.method), "Newmark tag was lost");
    Require(std::holds_alternative<dq::ZhaiRequest>(zhai.method), "Zhai tag was lost");
    Require(std::get<dq::NewmarkRequest>(newmark.method).step_size_nanoseconds == 25000,
            "integer step changed during parsing");
    Require(!dq::RequestMetadata(newmark).at("newton").contains("quaternion_scale_convention"),
            "tool configuration must not duplicate the library quaternion convention");
    const auto path = work / "invalid_configuration.json";
    const auto bad = [&](Json json) {
        Write(path, json.dump());
        Reject([&] { static_cast<void>(dq::ReadIntegrationConfiguration(path)); },
               "malformed configuration was accepted");
    };
    const auto good = Read(newmark_path);
    auto altered = good; altered["method"] = "cvode_bdf2"; bad(altered);
    altered = good; altered["schema_version"] = 2; bad(altered);
    altered = good; altered["schema_version"] = 1.0; bad(altered);
    altered = good; altered["unknown"] = 1; bad(altered);
    altered = good; altered["step_size_nanoseconds"] = 0; bad(altered);
    altered = good; altered["step_size_nanoseconds"] = -1; bad(altered);
    altered = good; altered["step_size_nanoseconds"] = 25000.0; bad(altered);
    altered = good; altered["maximum_internal_steps_per_advance"] = 0; bad(altered);
    altered = good; altered["maximum_internal_steps_per_public_advance"] = 1000; bad(altered);
    altered = good; altered.erase("newton"); bad(altered);
    altered = good; altered["newton"]["maximum_iterations"] = 0; bad(altered);
    altered = good; altered["newton"]["maximum_iterations"] = 2147483648ULL; bad(altered);
    altered = good; altered["newton"]["quaternion_scale_convention"] = "unit_norm"; bad(altered);
    altered = good; altered["newton"]["quaternion_scale_convention"] = "reference_norm_multiple"; bad(altered);
    altered = good; altered["newton"]["scales"].erase("angle"); bad(altered);
    altered = good; altered["newton"]["scales"]["quaternion"]["position_correction"] = 0; bad(altered);
    altered = good; altered["newton"]["scales"]["force"]["residual"] = "0.01"; bad(altered);
    altered = good; altered["newton"]["scales"]["angle"]["physical_velocity"] = 1; bad(altered);
    altered = Read(zhai_path); altered["newton"] = good.at("newton"); bad(altered);
    Write(path, R"({"schema_version":1,"method":"zhai","method":"newmark","step_size_nanoseconds":1})");
    Reject([&] { static_cast<void>(dq::ReadIntegrationConfiguration(path)); }, "duplicate key accepted");
    std::string duplicate = good.dump();
    const auto key = duplicate.find("\"position_correction\":");
    Require(key != std::string::npos, "missing nested-key fixture");
    duplicate.insert(key, R"("\u0070osition_correction":1,)");
    Write(path, duplicate);
    Reject([&] { static_cast<void>(dq::ReadIntegrationConfiguration(path)); },
           "nested escape-equivalent duplicate key accepted");
    Write(path, R"({"schema_version":1,"method":"zhai","step_size_nanoseconds":1e500})");
    Reject([&] { static_cast<void>(dq::ReadIntegrationConfiguration(path)); }, "non-finite JSON accepted");
    altered = Read(zhai_path); altered.erase("maximum_internal_steps_per_advance");
    Write(path, altered.dump());
    Require(dq::ReadIntegrationConfiguration(path).maximum_internal_steps_per_advance == ci::kDefaultMaximumInternalStepsPerAdvance,
            "default common budget changed");
    Require(std::holds_alternative<dq::ScenarioDefaultRequest>(
                dq::RequestIntegrationConfiguration(std::nullopt).method),
            "no-option request lost scenario default");
    Require(std::holds_alternative<dq::NewmarkRequest>(
                dq::RequestIntegrationConfiguration(newmark_path).method),
            "explicit configuration path lost its requested method");
    auto invalid = newmark;
    std::get<dq::NewmarkRequest>(invalid.method).force.reference =
        std::numeric_limits<double>::infinity();
    Reject([&] { static_cast<void>(dq::RequestMetadata(invalid)); }, "non-finite C++ scale accepted");
    invalid = zhai; invalid.maximum_internal_steps_per_advance = 0;
    Reject([&] { static_cast<void>(dq::RequestMetadata(invalid)); }, "zero C++ budget accepted");

    for (const char* method : {"cvode_bdf2", "cvode_bdf5", "radau5"}) {
        const auto ode_json = ExplicitOdeDocument(method);
        Write(path, ode_json.dump());
        const auto ode_request = dq::ReadIntegrationConfiguration(path);
        const auto& ode = std::get<dq::ExplicitOdeRequest>(ode_request.method);
        Require(ode.relative_tolerance == 2e-11 &&
                    ode.generalized_position_absolute_tolerance == 3e-12 &&
                    ode.generalized_velocity_absolute_tolerance == 4e-12 &&
                    ode.series_force_absolute_tolerance_newtons == 5e-9 &&
                    ode_request.maximum_internal_steps_per_advance == 5000000,
                "explicit ODE payload changed during parsing");
        const auto metadata = dq::RequestMetadata(ode_request);
        Require(metadata.at("integrator_recipe_identifier") == method,
                "explicit ODE request lost its method identity");
        for (const char* field : {"relative_tolerance", "generalized_position_absolute_tolerance",
                                 "generalized_velocity_absolute_tolerance", "series_force_absolute_tolerance_newtons"}) {
            altered = ode_json; altered.erase(field); bad(altered);
            altered = ode_json; altered[field] = 0; bad(altered);
            altered = ode_json; altered[field] = -1; bad(altered);
            altered = ode_json; altered[field] = "1e-12"; bad(altered);
            altered = ode_json; altered[field] = true; bad(altered);
        }
        altered = ode_json; altered["step_size_nanoseconds"] = 12500; bad(altered);
        altered = ode_json; altered["newton"] = good.at("newton"); bad(altered);
        altered = ode_json; altered["tolerance_tier_identifier"] = "reference"; bad(altered);
        altered = ode_json; altered["unexpected"] = 1; bad(altered);
        altered = ode_json; altered["maximum_internal_steps_per_advance"] = 0; bad(altered);
        altered = ode_json; altered["maximum_internal_steps_per_advance"] = 5e6; bad(altered);
        altered = ode_json; altered.erase("maximum_internal_steps_per_advance");
        Write(path, altered.dump());
        Require(dq::ReadIntegrationConfiguration(path).maximum_internal_steps_per_advance == ci::kDefaultMaximumInternalStepsPerAdvance,
                "explicit ODE changed the default budget");
        duplicate = ode_json.dump();
        duplicate.insert(1, R"("relative_tolerance":1e-10,)");
        Write(path, duplicate);
        Reject([&] { static_cast<void>(dq::ReadIntegrationConfiguration(path)); },
               "duplicate explicit ODE tolerance accepted");
    }
    altered = good; altered["relative_tolerance"] = 1e-6; bad(altered);
    altered = Read(zhai_path); altered["relative_tolerance"] = 1e-6; bad(altered);
    dq::IntegrationRequest invalid_ode{dq::ExplicitOdeRequest{
        dq::OdeIntegrationMethod::kCvodeBdf2, 1e-6, 1e-7, 1e-8,
        std::numeric_limits<double>::infinity()}, 5000000};
    Reject([&] { static_cast<void>(dq::RequestMetadata(invalid_ode)); },
           "non-finite C++ explicit ODE tolerance accepted");
}

void CheckScales(const dq::CoordinateNewtonScales& expected,
                 const orvd::integrators::CoordinateNewtonScales& actual) {
    Require(actual.position_correction == expected.position_correction &&
                actual.velocity_correction == expected.velocity_correction &&
                actual.acceleration_residual == expected.acceleration_residual &&
                actual.acceleration_reference == expected.acceleration_reference,
            "public coordinate family differs from the declared tool request");
}

void Vehicle(const std::filesystem::path& vehicle_path, const dq::IntegrationRequest& input,
             bool empty_force_state) {
    auto definition = orvd::configuration::LoadVehicleDefinitionFromJsonFile(vehicle_path);
    if (empty_force_state) definition.series_spring_viscous_dampers.clear();
    // Reverse the caller's declaration order: resolution must follow actual
    // model ranges, not vector position or assumptions about a particular car.
    std::reverse(definition.mechanical_track_station_layout.free_body_station_offsets.begin(),
                 definition.mechanical_track_station_layout.free_body_station_offsets.end());
    auto assembled = orvd::configuration::AssembleVehicleSystem(definition, 9.81);
    const auto& system = assembled->system();
    auto context = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd initial(system.continuous_state_size());
    system.CopyContinuousState(*context, initial);
    const auto& model = assembled->model();
    const auto& binding = assembled->binding();
    // Each free body has a distinct legal non-unit norm and alternating sign.
    double norm = 1.25;
    for (const auto& body : binding.free_body_station_offsets) {
        const auto range = model.GetFreeBodyPositionRange(model.GetRigidBodyByName(body.body_name));
        initial.segment(range.start(), 4) *= norm;
        norm = norm > 0.0 ? -(norm + 0.25) : -norm + 0.25;
    }
    auto request = input;
    auto& newmark = std::get<dq::NewmarkRequest>(request.method);
    // Distinct units/families make accidental cross-family reuse visible.
    newmark.translation = {1e-8, 2e-7, 3e-6, 4.0};
    newmark.angle = {5e-8, 6e-7, 7e-6, 8.0};
    newmark.quaternion = {9e-8, 1e-6, 2e-5, 3.0};
    newmark.force = {0.02, 0.03, 40000};
    const dq::ExplicitOdeRequest defaults{dq::OdeIntegrationMethod::kCvodeBdf2,
                                         2e-7, 3e-8, 4e-8, 5e-3};
    const auto resolved = dq::ResolveIntegrationConfiguration(request, *assembled, initial, defaults);
    const auto layout = dq::BuildQualificationStateLayout(*assembled, initial);
    Require(resolved.metadata.at("coordinate_layout") == layout,
            "Newmark did not use the shared physical layout");
    const auto& actual = std::get<ci::NewmarkConfiguration>(resolved.configuration.method);
    Require(resolved.metadata.at("step_size_nanoseconds") == newmark.step_size_nanoseconds,
            "declared integer step was not preserved in metadata");
    Require(actual.nominal_step_size_seconds == resolved.metadata.at("step_size_seconds").get<double>(),
            "recorded step does not equal the configured step");
    const int nq = system.generalized_positions_state_range().size();
    const auto zr = system.series_spring_damper_force_state_range();
    const int nz = zr.size();
    const auto& scales = actual.nonlinear_solver;
    CheckScales(newmark.translation, scales.translation);
    CheckScales(newmark.angle, scales.angle);
    CheckScales(newmark.quaternion, scales.quaternion);
    Require(scales.force.correction == 0.02 && scales.force.residual == 0.03 &&
                scales.force.reference == 40000,
            "public force family differs from the declared tool request");
    for (const char* name : {"relative_tolerance", "generalized_position_absolute_tolerance",
                            "generalized_velocity_absolute_tolerance", "series_force_absolute_tolerance_newtons",
                            "maximum_bdf_order"}) {
        Require(resolved.metadata.at(name).is_null(), "mechanical metadata fabricated ODE values");
    }
    Require(!resolved.metadata.at("newton").contains("expanded_scales"),
            "tool metadata must not expose private solver arrays");
    Require(resolved.metadata.at("coordinate_layout").at("initial_quaternion_norms").size() ==
                binding.free_body_station_offsets.size(), "metadata lost per-body quaternion norms");
    const auto vr = system.generalized_velocities_state_range();
    std::vector<bool> velocity_covered(static_cast<std::size_t>(vr.size()), false);
    for (const auto& block : resolved.metadata.at("coordinate_layout").at("coordinate_blocks")) {
        const auto family = block.at("family").get<std::string>();
        if (family == "force") {
            Require(block.at("joint_kind") == "series_force", "force state lost its layout kind");
            continue;
        }
        const auto owner = block.at("owner").get<std::string>();
        int start;
        int size;
        if (family == "angle") {
            const auto range = model.GetJointVelocityRange(model.GetJointByName(owner));
            start = range.start();
            size = range.size();
            if (std::find(binding.ball_rpy_joint_names.begin(), binding.ball_rpy_joint_names.end(), owner) !=
                binding.ball_rpy_joint_names.end()) {
                Require(block.at("joint_kind") == "ball_rpy" &&
                            block.at("angle_order") == "roll_pitch_yaw" &&
                            block.at("rotation_convention") == "R_FM=Rz(yaw)*Ry(pitch)*Rx(roll)" &&
                            block.at("physical_velocity_kind") == "angular_velocity" &&
                            block.at("physical_velocity_expression_frame") == "parent_joint_frame",
                        "Ball-RPY layout lost its pose or physical velocity convention");
            } else {
                Require(block.at("joint_kind") == "revolute" &&
                            block.at("physical_velocity_kind") == "joint_angular_rate",
                        "revolute layout was conflated with Ball-RPY");
            }
        } else {
            const auto range = model.GetFreeBodyVelocityRange(model.GetRigidBodyByName(owner));
            start = range.start() + (family == "translation" ? 3 : 0);
            size = 3;
            Require(block.at("physical_velocity_expression_frame") == "world",
                    "free-body velocity expression frame changed");
            if (family == "quaternion") {
                Require(block.at("joint_kind") == "free_quaternion" &&
                            block.at("quaternion_order") == "wxyz" &&
                            block.at("rotation_convention") == "R_WB",
                        "free quaternion layout lost its coefficient order or rotation direction");
            } else {
                Require(block.at("joint_kind") == "free_translation" &&
                            block.at("position_expression_frame") == "world",
                        "free translation layout lost its physical convention");
            }
        }
        Require(block.at("physical_velocity_start") == vr.start() + start &&
                    block.at("physical_velocity_size") == size,
                "metadata velocity range differs from actual model ownership");
        for (int i = start; i < start + size; ++i) {
            Require(!velocity_covered.at(static_cast<std::size_t>(i)), "metadata velocity blocks overlap");
            velocity_covered[static_cast<std::size_t>(i)] = true;
        }
    }
    Require(std::all_of(velocity_covered.begin(), velocity_covered.end(), [](bool value) { return value; }),
            "metadata velocity blocks omit a physical velocity");
    dq::IntegrationRequest zhai{dq::ZhaiRequest{25000}, 1000000};
    const auto zrsl = dq::ResolveIntegrationConfiguration(zhai, *assembled, initial, defaults);
    Require(zrsl.metadata.at("newton").is_null() &&
                zrsl.metadata.at("coordinate_layout") == resolved.metadata.at("coordinate_layout"),
            "Zhai acquired Newton configuration or lost coordinate ownership");
    auto wrong_initial = initial.head(initial.size() - 1).eval();
    Reject([&] { static_cast<void>(dq::ResolveIntegrationConfiguration(request, *assembled, wrong_initial, defaults)); },
           "wrong physical state dimension accepted");
    auto invalid_initial = initial;
    invalid_initial[0] = std::numeric_limits<double>::quiet_NaN();
    Reject([&] { static_cast<void>(dq::ResolveIntegrationConfiguration(request, *assembled, invalid_initial, defaults)); },
           "non-finite physical state accepted");
    const auto first = model.GetFreeBodyPositionRange(model.GetRigidBodyByName(binding.free_body_station_offsets.front().body_name));
    invalid_initial = initial; invalid_initial.segment(first.start(), 4).setZero();
    Reject([&] { static_cast<void>(dq::ResolveIntegrationConfiguration(request, *assembled, invalid_initial, defaults)); },
           "zero quaternion silently normalized");
    const auto scenario_default = dq::ResolveIntegrationConfiguration(
        dq::RequestIntegrationConfiguration(std::nullopt), *assembled, initial, defaults);
    Require(scenario_default.metadata.at("relative_tolerance") == defaults.relative_tolerance &&
                scenario_default.metadata.at("maximum_bdf_order") == 2,
            "no-option ODE defaults changed");
    Require(scenario_default.metadata.at("coordinate_layout") == layout,
            "scenario-default ODE lost the shared layout");
    for (const auto method : {dq::OdeIntegrationMethod::kCvodeBdf2,
                              dq::OdeIntegrationMethod::kCvodeBdf5,
                              dq::OdeIntegrationMethod::kRadau5}) {
        const dq::IntegrationRequest explicit_request{dq::ExplicitOdeRequest{
            method, 2e-11, 3e-12, 4e-12, 5e-9}, 5000000};
        const auto explicit_result = dq::ResolveIntegrationConfiguration(
            explicit_request, *assembled, initial, defaults);
        Require(explicit_result.configuration.maximum_internal_steps_per_advance == 5000000 &&
                    explicit_result.metadata.at("coordinate_layout") == layout &&
                    explicit_result.metadata.at("step_size_seconds").is_null() &&
                    explicit_result.metadata.at("newton").is_null(),
                "explicit ODE acquired unrelated method data or lost its physical layout");
        Require(explicit_result.metadata.at("relative_tolerance") == 2e-11 &&
                    explicit_result.metadata.at("generalized_position_absolute_tolerance") == 3e-12 &&
                    explicit_result.metadata.at("generalized_velocity_absolute_tolerance") == 4e-12 &&
                    explicit_result.metadata.at("series_force_absolute_tolerance_newtons") == 5e-9,
                "explicit ODE metadata did not preserve the requested tolerances");
        std::visit([&](const auto& configuration) {
            using Method = std::decay_t<decltype(configuration)>;
            if constexpr (requires { configuration.tolerances; }) {
                Require(configuration.tolerances.relative_tolerance() == 2e-11,
                        "explicit relative tolerance did not reach the strong configuration");
                const auto& values = configuration.tolerances.component_absolute_tolerances();
                Require(values.size() == initial.size() && values.head(nq).isConstant(3e-12) &&
                            values.segment(vr.start(), vr.size()).isConstant(4e-12) &&
                            values.segment(zr.start(), nz).isConstant(5e-9),
                        "explicit physical tolerances did not reach their actual ranges");
                if constexpr (std::is_same_v<Method, ci::CvodeBdf2Configuration>) {
                    Require(method == dq::OdeIntegrationMethod::kCvodeBdf2 &&
                                explicit_result.metadata.at("maximum_bdf_order") == 2,
                            "explicit BDF2 method identity changed");
                } else if constexpr (std::is_same_v<Method, ci::CvodeBdf5Configuration>) {
                    Require(method == dq::OdeIntegrationMethod::kCvodeBdf5 &&
                                explicit_result.metadata.at("maximum_bdf_order") == 5,
                            "explicit BDF5 method identity changed");
                } else {
                    Require(method == dq::OdeIntegrationMethod::kRadau5 &&
                                explicit_result.metadata.at("maximum_bdf_order").is_null(),
                            "explicit Radau5 method identity changed");
                }
            } else { Require(false, "explicit ODE request resolved to a mechanical method"); }
        }, explicit_result.configuration.method);
    }
    if (empty_force_state) Require(nz == 0, "empty-z fixture was not empty");
}
}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: verify_qualification_integration_configuration <source_root> <temporary_directory>\n");
        return 2;
    }
    try {
        const std::filesystem::path source(argv[1]);
        const std::filesystem::path work(argv[2]);
        std::filesystem::create_directories(work);
        const auto examples = source / "tools/dynamics_qualification/integration_configurations";
        Parsing(examples, work);
        const auto request = dq::ReadIntegrationConfiguration(examples / "newmark_explicit_trial.json");
        Vehicle(source / "vehicle_library/gz18/vehicle_definition.json", request, false);
        Vehicle(source / "vehicle_library/irw/vehicle_definition.json", request, false);
        Vehicle(source / "vehicle_library/gz18/vehicle_definition.json", request, true);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "qualification integration configuration: %s\n", error.what());
        return 1;
    }
    std::puts("qualification integration configuration verified");
    return 0;
}
