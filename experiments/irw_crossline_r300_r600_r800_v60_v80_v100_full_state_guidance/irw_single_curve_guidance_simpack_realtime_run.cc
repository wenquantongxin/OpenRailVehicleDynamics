#include "irw_single_curve_guidance_simpack_realtime_run.h"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "irw_guidance_control_transaction.h"
#include "simpack_realtime_instance.h"

#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

using Clock = std::chrono::steady_clock;
using actuation::WheelDriveTorqueChannelValues;
using simpack_realtime::SimpackRealtimeInstance;

constexpr std::size_t kAxleCount = control::kIrwGuidanceAxleCount;
constexpr std::size_t kWheelCount = control::kIrwGuidanceWheelCount;
constexpr std::size_t kMaximumPatchCount = 8;
constexpr double kObservationPeriodSeconds = 0.001;
constexpr double kControlPeriodSeconds = 0.01;
constexpr std::uint64_t kObservationSamplesPerControlPeriod = 10;

constexpr std::array<std::string_view, kWheelCount> kInputNames{
    "$S_IRWBogie_Front.$UI_UA_Simat",
    "$S_IRWBogie_Front.$UI_UB_Simat",
    "$S_IRWBogie_Front.$UI_UC_Simat",
    "$S_IRWBogie_Front.$UI_UD_Simat",
    "$S_IRWBogie_Rear.$UI_UA_Simat",
    "$S_IRWBogie_Rear.$UI_UB_Simat",
    "$S_IRWBogie_Rear.$UI_UC_Simat",
    "$S_IRWBogie_Rear.$UI_UD_Simat"};
constexpr std::array<std::string_view, kWheelCount> kSimpackWheelNames{
    "axle01_left", "axle01_right", "axle02_left", "axle02_right",
    "axle03_left", "axle03_right", "axle04_left", "axle04_right"};

struct AbiBinding final {
    std::array<std::size_t, kWheelCount> input_indices{};
    std::array<std::size_t, kAxleCount> station_indices{};
    std::array<std::size_t, kAxleCount> lateral_indices{};
    std::array<std::size_t, kAxleCount> yaw_indices{};
    std::array<std::size_t, kWheelCount> wheel_rate_indices{};
    std::array<std::array<std::array<std::size_t, 3>, kMaximumPatchCount>,
               kWheelCount>
        patch_force_indices{};
};

struct Observation final {
    control::IrwGuidanceAxleValues axle_stations_meters{};
    control::IrwGuidanceAxleValues lateral_displacements_meters{};
    control::IrwGuidanceAxleValues yaw_angles_radians{};
    control::IrwGuidanceWheelValues
        wheel_rates_in_frozen_scalar_convention_radians_per_second{};
    control::IrwGuidanceWheelValues normal_forces_newtons{};
    control::IrwGuidanceWheelValues longitudinal_forces_newtons{};
    control::IrwGuidanceWheelValues lateral_forces_newtons{};
};

class AtomicOutputDirectory final {
   public:
    explicit AtomicOutputDirectory(std::filesystem::path final_path)
        : final_path_(std::filesystem::absolute(std::move(final_path))
                          .lexically_normal()),
          working_path_(final_path_.parent_path() /
                        (final_path_.filename().string() + ".partial")) {
        if (final_path_.filename().empty() ||
            !std::filesystem::is_directory(final_path_.parent_path())) {
            Reject("the output path must name a new directory below an "
                   "existing parent");
        }
        if (std::filesystem::exists(final_path_) ||
            std::filesystem::exists(working_path_)) {
            Reject("the output directory or its partial sibling already "
                   "exists");
        }
        if (!std::filesystem::create_directory(working_path_)) {
            Reject("could not create the partial output directory");
        }
        owns_working_path_ = true;
    }

    ~AtomicOutputDirectory() {
        if (owns_working_path_) {
            std::error_code ignored;
            std::filesystem::remove_all(working_path_, ignored);
        }
    }

    AtomicOutputDirectory(const AtomicOutputDirectory&) = delete;
    AtomicOutputDirectory& operator=(const AtomicOutputDirectory&) = delete;

    [[nodiscard]] const std::filesystem::path& working_path() const noexcept {
        return working_path_;
    }

    void Publish() {
        std::error_code error;
        std::filesystem::rename(working_path_, final_path_, error);
        if (error) {
            Reject("could not atomically publish the SIMPACK result: " +
                   error.message());
        }
        owns_working_path_ = false;
    }

   private:
    [[noreturn]] static void Reject(const std::string& detail) {
        throw std::runtime_error("SIMPACK single-curve guidance output: " +
                                 detail);
    }

    std::filesystem::path final_path_;
    std::filesystem::path working_path_;
    bool owns_working_path_{false};
};

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("SIMPACK single-curve guidance experiment: " +
                             detail);
}

[[nodiscard]] double ElapsedSeconds(Clock::time_point begin,
                                    Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

[[nodiscard]] std::string_view Trim(std::string_view value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

[[nodiscard]] std::string ReadAssignment(
    const std::filesystem::path& model_path, std::string_view property) {
    std::ifstream input(model_path);
    if (!input) {
        Reject("could not read the temporary SIMPACK model");
    }
    std::string result;
    std::size_t count{};
    std::string line;
    while (std::getline(input, line)) {
        const std::string_view view = Trim(line);
        if (!view.starts_with(property) ||
            (view.size() > property.size() &&
             view[property.size()] != ' ' &&
             view[property.size()] != '\t' &&
             view[property.size()] != '(')) {
            continue;
        }
        const std::size_t equals = view.find('=');
        if (equals == std::string_view::npos) {
            Reject("malformed model assignment for " +
                   std::string(property));
        }
        const std::size_t comment = view.find('!', equals + 1);
        result = std::string(Trim(view.substr(
            equals + 1, comment == std::string_view::npos
                            ? std::string_view::npos
                            : comment - equals - 1)));
        ++count;
    }
    if (!input.eof() || count != 1) {
        Reject("the model must contain exactly one assignment for " +
               std::string(property));
    }
    return result;
}

[[nodiscard]] int ReadIntegerAssignment(
    const std::filesystem::path& model_path, std::string_view property) {
    const std::string text = ReadAssignment(model_path, property);
    int result{};
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), result);
    if (error != std::errc{} || end != text.data() + text.size()) {
        Reject("model assignment is not an integer: " +
               std::string(property));
    }
    return result;
}

[[nodiscard]] double ReadScalarAssignment(
    const std::filesystem::path& model_path, std::string_view property) {
    const std::string text = ReadAssignment(model_path, property);
    double result{};
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), result,
        std::chars_format::general);
    if (error != std::errc{} || end != text.data() + text.size() ||
        !std::isfinite(result)) {
        Reject("model assignment is not a finite scalar: " +
               std::string(property));
    }
    return result;
}

void ValidateTemporaryModel(
    const std::filesystem::path& model_path,
    const IrwSingleCurveGuidanceSimpackRealtimeDefinition& definition) {
    if (ReadAssignment(model_path, "track.active") !=
        definition.active_track_name) {
        Reject("the temporary model does not select the requested track");
    }
    if (ReadIntegerAssignment(model_path, "vehicle.applystartvel") != 1 ||
        ReadIntegerAssignment(model_path, "slv.integ.type") != 5 ||
        ReadIntegerAssignment(model_path, "slv.integ.meetop") != 1 ||
        ReadIntegerAssignment(model_path, "slv.rt.enable") != 1 ||
        ReadIntegerAssignment(model_path, "slv.rt.log.rate") != 0) {
        Reject("the temporary model does not satisfy the frozen startup and "
               "fixed-step Realtime communication-point contract");
    }
    if (!SameBits(ReadScalarAssignment(model_path, "slv.rt.stepsize"),
                  kObservationPeriodSeconds) ||
        !SameBits(ReadScalarAssignment(model_path, "slv.integ.fix.h"),
                  kObservationPeriodSeconds)) {
        Reject("the SIMPACK fixed and Realtime communication steps must both "
               "be 1 ms");
    }
}

[[nodiscard]] std::string PatchOutputName(std::string_view wheel,
                                          std::size_t patch,
                                          std::string_view quantity) {
    std::ostringstream result;
    result << "$Y_contact_" << wheel << "_patch" << std::setw(2)
           << std::setfill('0') << patch + 1 << '_' << quantity;
    return result.str();
}

[[nodiscard]] std::unordered_map<std::string, std::size_t> IndexNames(
    const std::vector<std::string>& names, std::string_view kind) {
    std::unordered_map<std::string, std::size_t> result;
    result.reserve(names.size());
    for (std::size_t index = 0; index < names.size(); ++index) {
        if (!result.emplace(names[index], index).second) {
            Reject(std::string(kind) + " contains a duplicate name: " +
                   names[index]);
        }
    }
    return result;
}

[[nodiscard]] std::size_t RequireIndex(
    const std::unordered_map<std::string, std::size_t>& indices,
    const std::string& name) {
    const auto iterator = indices.find(name);
    if (iterator == indices.end()) {
        Reject("SIMPACK Realtime ABI is missing " + name);
    }
    return iterator->second;
}

[[nodiscard]] AbiBinding BindAbi(const SimpackRealtimeInstance& instance) {
    const auto input_indices = IndexNames(instance.input_names(), "u-Input");
    const auto output_indices =
        IndexNames(instance.output_names(), "y-Output");
    if (input_indices.size() != kInputNames.size()) {
        Reject("the SIMPACK model must expose exactly eight torque inputs");
    }

    AbiBinding result;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        result.input_indices[wheel] =
            RequireIndex(input_indices, std::string(kInputNames[wheel]));
        result.wheel_rate_indices[wheel] = RequireIndex(
            output_indices,
            "$Y_" + std::string(kSimpackWheelNames[wheel]) + "_w");
        for (std::size_t patch = 0; patch < kMaximumPatchCount; ++patch) {
            result.patch_force_indices[wheel][patch][0] = RequireIndex(
                output_indices,
                PatchOutputName(kSimpackWheelNames[wheel], patch, "normal"));
            result.patch_force_indices[wheel][patch][1] = RequireIndex(
                output_indices,
                PatchOutputName(kSimpackWheelNames[wheel], patch, "tx"));
            result.patch_force_indices[wheel][patch][2] = RequireIndex(
                output_indices,
                PatchOutputName(kSimpackWheelNames[wheel], patch, "ty"));
        }
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        std::ostringstream prefix;
        prefix << "$Y_axle" << std::setw(2) << std::setfill('0') << axle + 1;
        result.station_indices[axle] =
            RequireIndex(output_indices, prefix.str() + "_s");
        result.lateral_indices[axle] =
            RequireIndex(output_indices, prefix.str() + "_y");
        result.yaw_indices[axle] =
            RequireIndex(output_indices, prefix.str() + "_yaw");
    }
    return result;
}

template <std::size_t Size>
void RequireFinite(const std::array<double, Size>& values,
                   std::string_view description) {
    for (const double value : values) {
        if (!std::isfinite(value)) {
            Reject(std::string(description) + " contains a non-finite value");
        }
    }
}

[[nodiscard]] Observation DecodeObservation(
    std::span<const double> values, const AbiBinding& binding) {
    Observation result;
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        result.axle_stations_meters[axle] =
            values[binding.station_indices[axle]];
        result.lateral_displacements_meters[axle] =
            values[binding.lateral_indices[axle]];
        result.yaw_angles_radians[axle] = values[binding.yaw_indices[axle]];
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        result.wheel_rates_in_frozen_scalar_convention_radians_per_second
            [wheel] = values[binding.wheel_rate_indices[wheel]];
        for (std::size_t patch = 0; patch < kMaximumPatchCount; ++patch) {
            result.normal_forces_newtons[wheel] +=
                values[binding.patch_force_indices[wheel][patch][0]];
            result.longitudinal_forces_newtons[wheel] +=
                values[binding.patch_force_indices[wheel][patch][1]];
            result.lateral_forces_newtons[wheel] +=
                values[binding.patch_force_indices[wheel][patch][2]];
        }
    }
    RequireFinite(result.axle_stations_meters, "axle stations");
    RequireFinite(result.lateral_displacements_meters, "axle lateral values");
    RequireFinite(result.yaw_angles_radians, "axle yaw values");
    RequireFinite(
        result.wheel_rates_in_frozen_scalar_convention_radians_per_second,
        "wheel rates");
    RequireFinite(result.normal_forces_newtons, "normal forces");
    RequireFinite(result.longitudinal_forces_newtons,
                  "longitudinal forces");
    RequireFinite(result.lateral_forces_newtons, "lateral forces");
    return result;
}

[[nodiscard]] control::IrwFullStateWheelSpeedGuidanceMechanicalInput
MechanicalInput(const Observation& observation) {
    return control::IrwFullStateWheelSpeedGuidanceMechanicalInput{
        .axle_lateral_displacements_meters =
            observation.lateral_displacements_meters,
        .axle_yaw_angles_radians = observation.yaw_angles_radians,
        .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second =
            observation
                .wheel_rates_in_frozen_scalar_convention_radians_per_second,
    };
}

[[nodiscard]] bool AllAxlesReachedTerminalStation(
    const Observation& observation, double terminal_station_meters) {
    return std::ranges::all_of(
        observation.axle_stations_meters,
        [terminal_station_meters](double station) {
            return station >= terminal_station_meters;
        });
}

[[nodiscard]] std::vector<double> BindInputs(
    const SimpackRealtimeInstance& instance, const AbiBinding& binding,
    const WheelDriveTorqueChannelValues& torques) {
    std::vector<double> result = instance.initial_input_values();
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        result[binding.input_indices[wheel]] = torques[wheel];
    }
    return result;
}

[[nodiscard]] std::ofstream OpenOutput(const std::filesystem::path& path) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open output file: " + path.string());
    }
    output << std::setprecision(17);
    return output;
}

void CloseChecked(std::ofstream* output,
                  const std::filesystem::path& path) {
    output->flush();
    if (!*output) {
        Reject("could not flush output file: " + path.string());
    }
    output->close();
    if (!*output) {
        Reject("could not close output file: " + path.string());
    }
}

template <std::size_t Size>
void WriteValues(std::ofstream* output,
                 const std::array<double, Size>& values) {
    for (const double value : values) {
        *output << '\t' << value;
    }
}

void WriteObservationHeader(std::ofstream* output) {
    *output << "sample_index\ttime_seconds";
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        *output << "\taxle_" << axle + 1 << "_track_station_meters";
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        *output << "\taxle_" << axle + 1 << "_lateral_displacement_meters";
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        *output << "\taxle_" << axle + 1 << "_yaw_angle_radians";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output << "\twheel_" << wheel + 1
                << "_frozen_angular_speed_radians_per_second";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output << "\twheel_" << wheel + 1
                << "_actual_drive_torque_newton_metres";
    }
    for (const std::string_view quantity :
         {"normal_force_newtons", "longitudinal_force_newtons",
          "lateral_force_newtons"}) {
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            *output << "\twheel_" << wheel + 1 << '_' << quantity;
        }
    }
    *output << '\n';
}

void WriteObservation(std::ofstream* output, std::uint64_t sample_index,
                      double time_seconds, const Observation& observation,
                      const WheelDriveTorqueChannelValues& held_torques) {
    *output << sample_index << '\t' << time_seconds;
    WriteValues(output, observation.axle_stations_meters);
    WriteValues(output, observation.lateral_displacements_meters);
    WriteValues(output, observation.yaw_angles_radians);
    WriteValues(
        output,
        observation
            .wheel_rates_in_frozen_scalar_convention_radians_per_second);
    WriteValues(output, held_torques);
    WriteValues(output, observation.normal_forces_newtons);
    WriteValues(output, observation.longitudinal_forces_newtons);
    WriteValues(output, observation.lateral_forces_newtons);
    *output << '\n';
}

void WriteControlHeader(std::ofstream* output) {
    *output << "event_ordinal\tevent_kind\tevent_time_seconds";
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        *output << "\taxle_" << axle + 1 << "_track_station_meters";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output << "\twheel_" << wheel + 1
                << "_base_speed_reference_meters_per_second";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output << "\twheel_" << wheel + 1
                << "_raw_torque_request_newton_metres";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output << "\twheel_" << wheel + 1
                << "_prioritized_torque_request_newton_metres";
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output << "\twheel_" << wheel + 1
                << "_actual_drive_torque_newton_metres";
    }
    *output << '\n';
}

void WriteControlEvent(
    std::ofstream* output, std::uint64_t event_ordinal,
    std::string_view event_kind, double event_time_seconds,
    const Observation& observation,
    const control::IrwFullStateWheelSpeedGuidanceOperatingPoint&
        operating_point,
    const IrwGuidanceControlTransactionResult& transaction) {
    *output << event_ordinal << '\t' << event_kind << '\t'
            << event_time_seconds;
    WriteValues(output, observation.axle_stations_meters);
    WriteValues(output,
                operating_point.base_wheel_speed_references_meters_per_second);
    WriteValues(
        output,
        transaction.controller_result.requested_wheel_torques_newton_metres);
    WriteValues(output,
                transaction.prioritized_wheel_torque_requests_newton_metres);
    WriteValues(
        output,
        transaction.conditioning_result.actual_wheel_torques_newton_metres);
    *output << '\n';
}

[[nodiscard]] std::string JsonString(std::string_view text) {
    std::ostringstream result;
    result << '"';
    for (const char character : text) {
        if (character == '"' || character == '\\') {
            result << '\\';
        }
        result << character;
    }
    result << '"';
    return result.str();
}

void WriteMetadata(
    const std::filesystem::path& path,
    const std::filesystem::path& model_path,
    const actuation::WheelDriveTorqueCommandConditioner& conditioner,
    const IrwSingleCurveGuidanceSimpackRealtimeDefinition& definition,
    const IrwSingleCurveGuidanceSimpackRealtimeRunSummary& summary) {
    auto output = OpenOutput(path);
    output << "{\n"
           << "  \"completed\": true,\n"
           << "  \"experiment\": "
           << JsonString(definition.experiment_name)
           << ",\n"
           << "  \"model_path\": " << JsonString(model_path.string())
           << ",\n"
           << "  \"active_track\": "
           << JsonString(definition.active_track_name) << ",\n"
           << "  \"control_profile_identity\": "
           << JsonString(definition.control_profile_identity) << ",\n"
           << "  \"base_speed_meters_per_second\": "
           << definition.base_speed_meters_per_second << ",\n"
           << "  \"planned_curve_radius_meters\": "
           << definition.curve_radius_meters << ",\n"
           << "  \"integrator\": \"fixed-step\",\n"
           << "  \"fixed_integrator_step_seconds\": "
           << kObservationPeriodSeconds << ",\n"
           << "  \"meet_output_points\": true,\n"
           << "  \"realtime_communication_period_seconds\": "
           << kObservationPeriodSeconds << ",\n"
           << "  \"control_period_seconds\": " << kControlPeriodSeconds
           << ",\n"
           << "  \"terminal_minimum_all_axle_station_meters\": "
           << definition.terminal_minimum_axle_station_meters << ",\n"
           << "  \"maximum_duration_seconds\": "
           << static_cast<double>(definition.maximum_observation_sample_count) *
                  kObservationPeriodSeconds
           << ",\n"
           << "  \"conditioner_identifier\": "
           << JsonString(conditioner.config().identifier) << ",\n"
           << "  \"observation_count\": " << summary.observation_count
           << ",\n"
           << "  \"control_event_count\": " << summary.control_event_count
           << ",\n"
           << "  \"simulated_duration_seconds\": "
           << summary.simulated_duration_seconds << ",\n"
           << "  \"final_minimum_axle_station_meters\": "
           << summary.final_minimum_axle_station_meters << ",\n"
           << "  \"solver_advance_wall_seconds\": "
           << summary.solver_advance_wall_seconds << "\n"
           << "}\n";
    CloseChecked(&output, path);
}

}  // namespace

IrwSingleCurveGuidanceSimpackRealtimeRunSummary
RunIrwSingleCurveGuidanceSimpackRealtime(
    const IrwSingleCurveGuidanceSimpackRealtimeDefinition& definition,
    const std::filesystem::path& simpack_installation_path,
    const std::filesystem::path& model_path,
    const std::filesystem::path& torque_conditioner_configuration_path,
    const std::filesystem::path& output_directory) {
    if (definition.experiment_name.empty() ||
        definition.active_track_name.empty() ||
        definition.control_profile_identity.empty() ||
        !std::isfinite(definition.base_speed_meters_per_second) ||
        !(definition.base_speed_meters_per_second > 0.0) ||
        !std::isfinite(definition.curve_radius_meters) ||
        !(definition.curve_radius_meters > 0.0) ||
        !std::isfinite(definition.terminal_minimum_axle_station_meters) ||
        !(definition.terminal_minimum_axle_station_meters > 0.0) ||
        definition.maximum_observation_sample_count == 0 ||
        definition.maximum_observation_sample_count %
                kObservationSamplesPerControlPeriod !=
            0 ||
        !definition.operating_point_evaluator ||
        !SameBits(definition.recurrence_config.sample_period_seconds,
                  kControlPeriodSeconds)) {
        Reject("the run definition is incomplete or invalid");
    }
    ValidateTemporaryModel(model_path, definition);
    if (!std::filesystem::is_directory(simpack_installation_path) ||
        !std::filesystem::is_regular_file(model_path) ||
        !std::filesystem::is_regular_file(
            torque_conditioner_configuration_path)) {
        Reject("the SIMPACK installation, model or conditioner path is "
               "invalid");
    }
    AtomicOutputDirectory output(output_directory);
    auto conditioner =
        configuration::LoadWheelDriveTorqueCommandConditionerFromJsonFile(
            torque_conditioner_configuration_path);
    if (!SameBits(conditioner.config().sample_period_seconds,
                  kControlPeriodSeconds) ||
        conditioner.config().forward_wheel_angular_speed_sign != -1.0) {
        Reject("the torque conditioner does not match the frozen 10 ms, "
               "negative-forward SIMPACK contract");
    }

    SimpackRealtimeInstance instance({
        .installation_path = simpack_installation_path,
        .model_path = model_path,
        .log_path = output.working_path() / "simpack_realtime.log",
        .cpu_assignment = {},
        .communication_timeout_seconds = 5.0,
        .verbose_level = 0,
    });
    const AbiBinding binding = BindAbi(instance);
    control::IrwFullStateWheelSpeedGuidanceRecurrence recurrence(
        definition.recurrence_config);
    control::IrwFullStateWheelSpeedGuidanceControllerState controller_state;
    WheelDriveTorqueChannelValues conditioner_memory{};

    const std::filesystem::path observations_path =
        output.working_path() / "observations.tsv";
    const std::filesystem::path controls_path =
        output.working_path() / "control_events.tsv";
    auto observations = OpenOutput(observations_path);
    auto controls = OpenOutput(controls_path);
    WriteObservationHeader(&observations);
    WriteControlHeader(&controls);

    Observation observation = DecodeObservation(instance.ReadOutputs(), binding);
    auto operating_point =
        definition.operating_point_evaluator(observation.axle_stations_meters);
    auto transaction = ComputeIrwGuidanceControlTransaction(
        recurrence, conditioner, MechanicalInput(observation),
        operating_point, controller_state, conditioner_memory);
    WheelDriveTorqueChannelValues held_torques =
        transaction.conditioning_result.actual_wheel_torques_newton_metres;
    instance.SetInitialInputs(BindInputs(instance, binding, held_torques));
    observation = DecodeObservation(instance.ReadOutputs(), binding);

    IrwSingleCurveGuidanceSimpackRealtimeRunSummary summary;
    WriteObservation(&observations, 0, 0.0, observation, held_torques);
    WriteControlEvent(&controls, 0, "initialization", 0.0, observation,
                      operating_point, transaction);
    summary.observation_count = 1;
    summary.control_event_count = 1;
    controller_state = transaction.controller_result.next_state;
    conditioner_memory = transaction.conditioning_result
                             .next_drive_side_torque_memory_newton_metres;

    if (AllAxlesReachedTerminalStation(
            observation, definition.terminal_minimum_axle_station_meters)) {
        Reject("the initial SIMPACK state already satisfies the terminal "
               "station condition");
    }
    instance.Start();
    bool complete = false;
    for (std::uint64_t sample = 1;
         sample <= definition.maximum_observation_sample_count; ++sample) {
        instance.SetInputs(BindInputs(instance, binding, held_torques));
        const double target_time =
            static_cast<double>(sample) * kObservationPeriodSeconds;
        const Clock::time_point advance_begin = Clock::now();
        instance.Advance(target_time);
        summary.solver_advance_wall_seconds +=
            ElapsedSeconds(advance_begin, Clock::now());
        observation = DecodeObservation(instance.ReadOutputs(), binding);
        WriteObservation(&observations, sample, target_time, observation,
                         held_torques);
        ++summary.observation_count;
        summary.simulated_duration_seconds = target_time;

        if (sample % kObservationSamplesPerControlPeriod != 0) {
            continue;
        }
        complete = AllAxlesReachedTerminalStation(
            observation, definition.terminal_minimum_axle_station_meters);
        operating_point =
            definition.operating_point_evaluator(
                observation.axle_stations_meters);
        transaction = ComputeIrwGuidanceControlTransaction(
            recurrence, conditioner, MechanicalInput(observation),
            operating_point, controller_state, conditioner_memory);
        const std::uint64_t event_ordinal =
            sample / kObservationSamplesPerControlPeriod;
        WriteControlEvent(&controls, event_ordinal,
                          complete ? "terminal" : "periodic", target_time,
                          observation, operating_point, transaction);
        ++summary.control_event_count;
        held_torques =
            transaction.conditioning_result.actual_wheel_torques_newton_metres;
        controller_state = transaction.controller_result.next_state;
        conditioner_memory = transaction.conditioning_result
                                 .next_drive_side_torque_memory_newton_metres;
        if (complete) {
            break;
        }
    }
    if (!complete) {
        Reject("not all axles reached the terminal station before the "
               "configured safety cap");
    }
    summary.final_minimum_axle_station_meters =
        *std::ranges::min_element(observation.axle_stations_meters);

    CloseChecked(&observations, observations_path);
    CloseChecked(&controls, controls_path);
    WriteMetadata(output.working_path() / "metadata.json", model_path,
                  conditioner, definition, summary);
    auto complete_marker = OpenOutput(output.working_path() / "COMPLETE");
    complete_marker << summary.observation_count << " observations\n"
                    << summary.control_event_count << " control events\n";
    CloseChecked(&complete_marker, output.working_path() / "COMPLETE");
    output.Publish();
    return summary;
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
