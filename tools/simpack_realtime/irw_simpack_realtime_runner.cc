#include "irw_simpack_realtime_runner.h"

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
#include <limits>
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

#include "simpack_realtime_instance.h"

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/configuration/load_irw_longitudinal_cruise_controller.h"
#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"
#include "orvd/control/sampled_longitudinal_cruise_controller.h"

namespace orvd::simpack_realtime {
namespace {

using Clock = std::chrono::steady_clock;
using actuation::WheelDriveTorqueChannelValues;

constexpr std::size_t kWheelCount = 8;
constexpr std::size_t kAxleCount = 4;
constexpr std::size_t kMaximumContactPatchCount = 8;
constexpr double kFrozenSamplePeriodSeconds = 0.01;
constexpr double kFailureDataFlushIntervalSeconds = 1.0;
constexpr int kSimpackFixedStepIntegratorType = 5;

constexpr std::array<std::string_view, kWheelCount> kSimpackWheelNames{
    "axle01_left", "axle01_right", "axle02_left", "axle02_right",
    "axle03_left", "axle03_right", "axle04_left", "axle04_right"};
constexpr std::array<std::string_view, kWheelCount> kInputNames{
    "$S_IRWBogie_Front.$UI_UA_Simat",
    "$S_IRWBogie_Front.$UI_UB_Simat",
    "$S_IRWBogie_Front.$UI_UC_Simat",
    "$S_IRWBogie_Front.$UI_UD_Simat",
    "$S_IRWBogie_Rear.$UI_UA_Simat",
    "$S_IRWBogie_Rear.$UI_UB_Simat",
    "$S_IRWBogie_Rear.$UI_UC_Simat",
    "$S_IRWBogie_Rear.$UI_UD_Simat"};

struct SignalColumn final {
    std::string heading;
    std::string output_name;
};

struct BoundSignalColumn final {
    std::string heading;
    std::size_t output_index{};
};

struct ContactPatchForces final {
    double longitudinal_newtons{};
    double lateral_newtons{};
    double normal_newtons{};
};

struct WheelSpeedObservation final {
    WheelDriveTorqueChannelValues raw_simpack_rates_radians_per_second{};
    WheelDriveTorqueChannelValues
        forward_circumferential_speeds_meters_per_second{};
    double common_forward_circumferential_speed_meters_per_second{};
};

struct DecodedObservation final {
    std::vector<double> body_signals;
    std::array<double, kAxleCount> axle_track_stations_meters{};
    std::array<double, kAxleCount> axle_lateral_displacements_meters{};
    std::array<double, kAxleCount> axle_yaw_angles_radians{};
    WheelSpeedObservation wheel_speeds;
    WheelDriveTorqueChannelValues q_vertical_newtons{};
    std::array<std::array<ContactPatchForces, kMaximumContactPatchCount>,
               kWheelCount>
        patches{};
    WheelDriveTorqueChannelValues total_normal_newtons{};
    WheelDriveTorqueChannelValues total_longitudinal_newtons{};
    WheelDriveTorqueChannelValues total_lateral_newtons{};
    std::array<std::size_t, kWheelCount> contact_patch_counts{};
    double last_axle_station_meters{};
};

struct EventCandidate final {
    WheelSpeedObservation wheel_speeds;
    control::SampledLongitudinalCruiseControllerState controller_state_before;
    WheelDriveTorqueChannelValues conditioner_memory_before_newton_metres{};
    control::SampledLongitudinalCruiseControllerResult controller_result;
    std::array<double, kAxleCount> differential_torques_newton_metres{};
    WheelDriveTorqueChannelValues requested_torques_newton_metres{};
    actuation::WheelDriveTorqueConditioningResult conditioning_result;
};

struct AbiBinding final {
    std::array<std::size_t, kWheelCount> input_indices{};
    std::array<std::size_t, kWheelCount> wheel_rate_output_indices{};
    std::array<std::size_t, kWheelCount> q_output_indices{};
    std::array<std::array<std::array<std::size_t, 3>,
                          kMaximumContactPatchCount>,
               kWheelCount>
        contact_output_indices{};
    std::vector<BoundSignalColumn> body_signal_columns;
    std::array<std::size_t, kAxleCount> axle_station_output_indices{};
    std::array<std::size_t, kAxleCount> axle_lateral_output_indices{};
    std::array<std::size_t, kAxleCount> axle_yaw_output_indices{};
    std::size_t last_axle_station_output_index{};
};

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("IRW SIMPACK Realtime cruise: " + detail);
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

[[nodiscard]] double ElapsedSeconds(Clock::time_point begin,
                                    Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
}

[[nodiscard]] std::filesystem::path CanonicalExistingFile(
    const std::filesystem::path& path, std::string_view description) {
    std::error_code error;
    const std::filesystem::path result = std::filesystem::canonical(path, error);
    if (error || !std::filesystem::is_regular_file(result)) {
        Reject(std::string(description) + " is not an existing regular file: '" +
               path.string() + "'");
    }
    return result;
}

[[nodiscard]] std::string_view TrimAsciiWhitespace(std::string_view value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

[[nodiscard]] std::string RequireSpckAssignment(
    const std::filesystem::path& model_path, std::string_view property) {
    std::ifstream input(model_path);
    if (!input) {
        Reject("could not inspect SIMPACK model '" + model_path.string() +
               "'");
    }
    std::string result;
    std::size_t count{};
    std::string line;
    while (std::getline(input, line)) {
        const std::string_view view = TrimAsciiWhitespace(line);
        if (!view.starts_with(property) ||
            (view.size() > property.size() &&
             view[property.size()] != ' ' &&
             view[property.size()] != '\t' &&
             view[property.size()] != '(')) {
            continue;
        }
        const std::size_t equals = view.find('=');
        if (equals == std::string_view::npos) {
            Reject("malformed SIMPACK assignment for '" +
                   std::string(property) + "'");
        }
        const std::size_t comment = view.find('!', equals + 1);
        result = std::string(TrimAsciiWhitespace(view.substr(
            equals + 1, comment == std::string_view::npos
                            ? std::string_view::npos
                            : comment - equals - 1)));
        ++count;
    }
    if (!input.eof()) {
        Reject("could not finish reading SIMPACK model '" +
               model_path.string() + "'");
    }
    if (count != 1) {
        Reject("SIMPACK model must contain exactly one '" +
               std::string(property) + "' assignment");
    }
    return result;
}

[[nodiscard]] int ParseSpckIntegerAssignment(
    const std::filesystem::path& model_path, std::string_view property) {
    const std::string text = RequireSpckAssignment(model_path, property);
    int result{};
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), result);
    if (error != std::errc{} || end != text.data() + text.size()) {
        Reject("SIMPACK assignment '" + std::string(property) +
               "' is not an integer");
    }
    return result;
}

[[nodiscard]] double ParseSpckScalarAssignment(
    const std::filesystem::path& model_path, std::string_view property) {
    const std::string text = RequireSpckAssignment(model_path, property);
    double result{};
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), result,
        std::chars_format::general);
    if (error != std::errc{} || end != text.data() + text.size() ||
        !std::isfinite(result)) {
        Reject("SIMPACK assignment '" + std::string(property) +
               "' is not a finite scalar");
    }
    return result;
}

void RequireRealtimeExecutionContract(
    const std::filesystem::path& model_path) {
    if (ParseSpckIntegerAssignment(model_path, "slv.integ.type") !=
        kSimpackFixedStepIntegratorType) {
        Reject("SIMPACK model must select its fixed-step integrator "
               "(slv.integ.type=5)");
    }
    if (ParseSpckIntegerAssignment(model_path, "slv.rt.enable") != 1) {
        Reject("SIMPACK model must enable Realtime execution");
    }
    if (ParseSpckIntegerAssignment(model_path, "slv.rt.log.rate") != 0) {
        Reject("SIMPACK model must disable its internal Realtime logger");
    }
    const double communication_step =
        ParseSpckScalarAssignment(model_path, "slv.rt.stepsize");
    if (!SameBits(communication_step, kFrozenSamplePeriodSeconds)) {
        Reject("SIMPACK Realtime communication step must equal the frozen "
               "0.01-second control period");
    }
    if (!(ParseSpckScalarAssignment(model_path, "slv.integ.fix.h") > 0.0)) {
        Reject("SIMPACK fixed integrator step must be positive");
    }
}

[[nodiscard]] std::filesystem::path PartialPathFor(
    const std::filesystem::path& final_path) {
    return final_path.parent_path() /
           (final_path.filename().string() + ".partial");
}

[[nodiscard]] std::filesystem::path CreatePartialDirectory(
    const std::filesystem::path& final_path) {
    if (final_path.empty() || final_path.filename().empty()) {
        Reject("output_directory must name a final directory");
    }
    const std::filesystem::path parent = final_path.has_parent_path()
                                             ? final_path.parent_path()
                                             : std::filesystem::path(".");
    if (!std::filesystem::is_directory(parent)) {
        Reject("output parent is not an existing directory: '" +
               parent.string() + "'");
    }
    const std::filesystem::path partial = PartialPathFor(final_path);
    if (std::filesystem::exists(final_path) ||
        std::filesystem::exists(partial)) {
        Reject("output final or partial path already exists");
    }
    if (!std::filesystem::create_directory(partial)) {
        Reject("could not create partial output directory '" +
               partial.string() + "'");
    }
    return partial;
}

[[nodiscard]] std::ofstream OpenOutput(const std::filesystem::path& path) {
    std::ofstream output(path);
    if (!output) {
        Reject("could not open output file '" + path.string() + "'");
    }
    output << std::setprecision(17);
    return output;
}

void CloseChecked(std::ofstream* output,
                  const std::filesystem::path& path) {
    output->flush();
    if (!*output) {
        Reject("could not flush output file '" + path.string() + "'");
    }
    output->close();
    if (!*output) {
        Reject("could not close output file '" + path.string() + "'");
    }
}

[[nodiscard]] std::string JsonString(std::string_view input) {
    std::ostringstream result;
    result << '"';
    for (const unsigned char character : input) {
        switch (character) {
            case '"':
                result << "\\\"";
                break;
            case '\\':
                result << "\\\\";
                break;
            case '\b':
                result << "\\b";
                break;
            case '\f':
                result << "\\f";
                break;
            case '\n':
                result << "\\n";
                break;
            case '\r':
                result << "\\r";
                break;
            case '\t':
                result << "\\t";
                break;
            default:
                if (character < 0x20) {
                    result << "\\u00" << std::hex << std::setw(2)
                           << std::setfill('0')
                           << static_cast<unsigned int>(character)
                           << std::dec << std::setfill(' ');
                } else {
                    result << static_cast<char>(character);
                }
                break;
        }
    }
    result << '"';
    return result.str();
}

[[nodiscard]] std::string PatchOutputName(std::string_view wheel,
                                          std::size_t patch,
                                          std::string_view quantity) {
    std::ostringstream name;
    name << "$Y_contact_" << wheel << "_patch" << std::setw(2)
         << std::setfill('0') << patch + 1 << '_' << quantity;
    return name.str();
}

[[nodiscard]] std::vector<SignalColumn> BodySignalColumns() {
    std::vector<SignalColumn> result{
        {"carbody.track_station_meters", "$Y_carbody_s"},
        {"carbody.vertical_meters", "$Y_carbody_z"},
        {"carbody.pitch_radians", "$Y_carbody_pitch"},
        {"frame_front.track_station_meters", "$Y_frame_front_s"},
        {"frame_front.vertical_meters", "$Y_frame_front_z"},
        {"frame_front.pitch_radians", "$Y_frame_front_pitch"},
        {"frame_rear.track_station_meters", "$Y_frame_rear_s"},
        {"frame_rear.vertical_meters", "$Y_frame_rear_z"},
        {"frame_rear.pitch_radians", "$Y_frame_rear_pitch"},
    };
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        std::ostringstream source;
        source << "$Y_axle" << std::setw(2) << std::setfill('0') << axle + 1;
        result.push_back(
            {"axlebridge_" +
                 std::string(kIrwSimpackRealtimeAxleIdentifiers[axle]) +
                 ".track_station_meters",
             source.str() + "_s"});
        result.push_back(
            {"axlebridge_" +
                 std::string(kIrwSimpackRealtimeAxleIdentifiers[axle]) +
                 ".lateral_meters",
             source.str() + "_y"});
        result.push_back(
            {"axlebridge_" +
                 std::string(kIrwSimpackRealtimeAxleIdentifiers[axle]) +
                 ".vertical_meters",
             source.str() + "_z"});
        result.push_back(
            {"axlebridge_" +
                 std::string(kIrwSimpackRealtimeAxleIdentifiers[axle]) +
                 ".yaw_radians",
             source.str() + "_yaw"});
        result.push_back(
            {"axlebridge_" +
                 std::string(kIrwSimpackRealtimeAxleIdentifiers[axle]) +
                 ".pitch_radians",
             source.str() + "_pitch"});
    }
    return result;
}

[[nodiscard]] std::vector<std::string> ExpectedOutputNames() {
    std::vector<std::string> result{
        "$Y_SpeedDiff_FrontA", "$Y_SpeedDiff_FrontB",
        "$Y_SpeedDiff_FrontC", "$Y_SpeedDiff_FrontD",
        "$Y_SpeedDiff_RearA", "$Y_SpeedDiff_RearB",
        "$Y_SpeedDiff_RearC", "$Y_SpeedDiff_RearD"};
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        std::ostringstream prefix;
        prefix << "$Y_axle" << std::setw(2) << std::setfill('0') << axle + 1;
        result.push_back(prefix.str() + "_y");
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        std::ostringstream prefix;
        prefix << "$Y_axle" << std::setw(2) << std::setfill('0') << axle + 1;
        result.push_back(prefix.str() + "_yaw");
    }
    for (const std::string_view wheel : kSimpackWheelNames) {
        result.push_back("$Y_" + std::string(wheel) + "_w");
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        std::ostringstream name;
        name << "$Y_axle" << std::setw(2) << std::setfill('0') << axle + 1
             << "_s";
        result.push_back(name.str());
    }
    for (const std::string_view wheel : kSimpackWheelNames) {
        result.push_back("$Y_QVertical_" + std::string(wheel));
    }
    for (const SignalColumn& signal : BodySignalColumns()) {
        if (std::find(result.begin(), result.end(), signal.output_name) ==
            result.end()) {
            result.push_back(signal.output_name);
        }
    }
    for (const std::string_view wheel : kSimpackWheelNames) {
        for (std::size_t patch = 0; patch < kMaximumContactPatchCount;
             ++patch) {
            result.push_back(PatchOutputName(wheel, patch, "tx"));
            result.push_back(PatchOutputName(wheel, patch, "ty"));
            result.push_back(PatchOutputName(wheel, patch, "normal"));
        }
    }
    return result;
}

void RequireExactNames(const std::vector<std::string>& actual,
                       const std::vector<std::string>& expected,
                       std::string_view kind) {
    const std::unordered_set<std::string> actual_set(actual.begin(),
                                                      actual.end());
    const std::unordered_set<std::string> expected_set(expected.begin(),
                                                        expected.end());
    if (actual.size() != expected.size() || actual_set != expected_set) {
        for (const std::string& name : expected) {
            if (!actual_set.contains(name)) {
                Reject(std::string(kind) + " ABI is missing '" + name + "'");
            }
        }
        for (const std::string& name : actual) {
            if (!expected_set.contains(name)) {
                Reject(std::string(kind) + " ABI has unexpected '" + name +
                       "'");
            }
        }
        Reject(std::string(kind) + " ABI has a mismatched element count");
    }
}

[[nodiscard]] std::unordered_map<std::string, std::size_t> IndexNames(
    const std::vector<std::string>& names) {
    std::unordered_map<std::string, std::size_t> result;
    result.reserve(names.size());
    for (std::size_t index = 0; index < names.size(); ++index) {
        result.emplace(names[index], index);
    }
    return result;
}

[[nodiscard]] std::size_t RequireIndex(
    const std::unordered_map<std::string, std::size_t>& indices,
    const std::string& name) {
    const auto iterator = indices.find(name);
    if (iterator == indices.end()) {
        Reject("the bound ABI does not contain '" + name + "'");
    }
    return iterator->second;
}

[[nodiscard]] AbiBinding BindAbi(const SimpackRealtimeInstance& instance) {
    std::vector<std::string> expected_inputs;
    expected_inputs.reserve(kInputNames.size());
    for (const std::string_view name : kInputNames) {
        expected_inputs.emplace_back(name);
    }
    RequireExactNames(instance.input_names(), expected_inputs, "u-Input");
    const std::vector<std::string> expected_outputs = ExpectedOutputNames();
    RequireExactNames(instance.output_names(), expected_outputs, "y-Output");

    const auto input_indices = IndexNames(instance.input_names());
    const auto output_indices = IndexNames(instance.output_names());
    AbiBinding result;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        result.input_indices[wheel] =
            RequireIndex(input_indices, std::string(kInputNames[wheel]));
        result.wheel_rate_output_indices[wheel] = RequireIndex(
            output_indices,
            "$Y_" + std::string(kSimpackWheelNames[wheel]) + "_w");
        result.q_output_indices[wheel] = RequireIndex(
            output_indices,
            "$Y_QVertical_" + std::string(kSimpackWheelNames[wheel]));
        for (std::size_t patch = 0; patch < kMaximumContactPatchCount;
             ++patch) {
            result.contact_output_indices[wheel][patch][0] = RequireIndex(
                output_indices,
                PatchOutputName(kSimpackWheelNames[wheel], patch, "tx"));
            result.contact_output_indices[wheel][patch][1] = RequireIndex(
                output_indices,
                PatchOutputName(kSimpackWheelNames[wheel], patch, "ty"));
            result.contact_output_indices[wheel][patch][2] = RequireIndex(
                output_indices,
                PatchOutputName(kSimpackWheelNames[wheel], patch, "normal"));
        }
    }
    for (const SignalColumn& signal : BodySignalColumns()) {
        result.body_signal_columns.push_back(
            {signal.heading, RequireIndex(output_indices, signal.output_name)});
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        std::ostringstream prefix;
        prefix << "$Y_axle" << std::setw(2) << std::setfill('0') << axle + 1;
        result.axle_station_output_indices[axle] =
            RequireIndex(output_indices, prefix.str() + "_s");
        result.axle_lateral_output_indices[axle] =
            RequireIndex(output_indices, prefix.str() + "_y");
        result.axle_yaw_output_indices[axle] =
            RequireIndex(output_indices, prefix.str() + "_yaw");
    }
    result.last_axle_station_output_index =
        RequireIndex(output_indices, "$Y_axle04_s");
    return result;
}

[[nodiscard]] WheelSpeedObservation ObserveWheelSpeeds(
    std::span<const double> outputs, const AbiBinding& binding,
    const configuration::IrwLongitudinalCruiseControllerAsset& asset) {
    WheelSpeedObservation result;
    double sum{};
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        const double simpack_rate = outputs[binding.wheel_rate_output_indices[wheel]];
        result.raw_simpack_rates_radians_per_second[wheel] = simpack_rate;
        const double orvd_raw_rate = -simpack_rate;
        const double forward_speed =
            asset.forward_joint_rate_signs[wheel] * orvd_raw_rate *
            asset.nominal_rolling_radius_meters;
        result.forward_circumferential_speeds_meters_per_second[wheel] =
            forward_speed;
        sum += forward_speed;
    }
    result.common_forward_circumferential_speed_meters_per_second =
        sum / static_cast<double>(kWheelCount);
    return result;
}

[[nodiscard]] DecodedObservation DecodeObservation(
    std::span<const double> outputs, const AbiBinding& binding,
    const configuration::IrwLongitudinalCruiseControllerAsset& asset) {
    DecodedObservation result;
    result.body_signals.reserve(binding.body_signal_columns.size());
    for (const BoundSignalColumn& signal : binding.body_signal_columns) {
        result.body_signals.push_back(outputs[signal.output_index]);
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        // The Type-25 outputs are already the source axle-bridge quantities
        // used by the accepted SIMPACK--ORVD response comparison.  Do not
        // infer or introduce a second sign conversion here.
        result.axle_track_stations_meters[axle] =
            outputs[binding.axle_station_output_indices[axle]];
        result.axle_lateral_displacements_meters[axle] =
            outputs[binding.axle_lateral_output_indices[axle]];
        result.axle_yaw_angles_radians[axle] =
            outputs[binding.axle_yaw_output_indices[axle]];
    }
    result.wheel_speeds = ObserveWheelSpeeds(outputs, binding, asset);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        result.q_vertical_newtons[wheel] =
            -outputs[binding.q_output_indices[wheel]];
        for (std::size_t patch = 0; patch < kMaximumContactPatchCount;
             ++patch) {
            const auto& indices = binding.contact_output_indices[wheel][patch];
            ContactPatchForces& forces = result.patches[wheel][patch];
            // The frozen Type-78 ABI already exposes wheel-side Tx/Ty.  Its
            // raw normal output alone needs negation to become compressive N.
            forces.longitudinal_newtons = outputs[indices[0]];
            forces.lateral_newtons = outputs[indices[1]];
            forces.normal_newtons = -outputs[indices[2]];
            if (forces.longitudinal_newtons != 0.0 ||
                forces.lateral_newtons != 0.0 ||
                forces.normal_newtons != 0.0) {
                ++result.contact_patch_counts[wheel];
            }
            result.total_longitudinal_newtons[wheel] +=
                forces.longitudinal_newtons;
            result.total_lateral_newtons[wheel] += forces.lateral_newtons;
            result.total_normal_newtons[wheel] += forces.normal_newtons;
        }
    }
    result.last_axle_station_meters =
        outputs[binding.last_axle_station_output_index];
    return result;
}

[[nodiscard]] EventCandidate ComputeEvent(
    std::uint64_t ordinal, double time_seconds,
    const DecodedObservation& observation,
    const control::SampledLongitudinalCruiseController& controller,
    const actuation::WheelDriveTorqueCommandConditioner& conditioner,
    const control::SampledLongitudinalCruiseControllerState& controller_state,
    const WheelDriveTorqueChannelValues& conditioner_memory,
    const WheelDriveTorqueChannelValues& previous_actual_wheel_torques,
    const IrwSimpackRealtimeDifferentialTorqueCallback&
        differential_torque_callback) {
    EventCandidate result;
    result.wheel_speeds = observation.wheel_speeds;
    result.controller_state_before = controller_state;
    result.conditioner_memory_before_newton_metres = conditioner_memory;
    result.controller_result = controller.Step(
        observation.wheel_speeds
            .common_forward_circumferential_speed_meters_per_second,
        controller_state);
    if (differential_torque_callback) {
        const IrwSimpackRealtimeControlObservation control_observation{
            ordinal,
            time_seconds,
            observation.axle_track_stations_meters,
            observation.axle_lateral_displacements_meters,
            observation.axle_yaw_angles_radians,
            observation.wheel_speeds.raw_simpack_rates_radians_per_second,
            previous_actual_wheel_torques};
        result.differential_torques_newton_metres =
            differential_torque_callback(control_observation);
    }
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        const double differential =
            result.differential_torques_newton_metres[axle];
        if (!std::isfinite(differential)) {
            Reject("differential-torque callback returned a non-finite value");
        }
        result.requested_torques_newton_metres[2 * axle] =
            result.controller_result.requested_common_wheel_torque_newton_metres +
            differential;
        result.requested_torques_newton_metres[2 * axle + 1] =
            result.controller_result.requested_common_wheel_torque_newton_metres -
            differential;
    }
    result.conditioning_result = conditioner.Step(
        result.requested_torques_newton_metres,
        observation.wheel_speeds.raw_simpack_rates_radians_per_second,
        conditioner_memory);
    return result;
}

[[nodiscard]] std::vector<double> BindInputs(
    const SimpackRealtimeInstance& instance, const AbiBinding& binding,
    const WheelDriveTorqueChannelValues& torques) {
    std::vector<double> result(instance.input_names().size());
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        // The frozen Type-93 actuator's From/To marker orientation makes its
        // input scalar share the accepted ORVD wheel-torque sign even though
        // the observed SIMPACK wheel-joint rate is negative forward. A direct
        // positive/negative closed-vehicle check, not a rate-sign inference,
        // establishes this adapter contract.
        result[binding.input_indices[wheel]] = torques[wheel];
    }
    return result;
}

void WriteObservationHeader(std::ofstream* output,
                            const AbiBinding& binding) {
    *output << "control_grid_ordinal\ttime_seconds"
            << "\tcommon_forward_wheel_circumferential_speed_meters_per_second";
    for (const BoundSignalColumn& signal : binding.body_signal_columns) {
        *output << '\t' << signal.heading;
    }
    for (const std::string_view wheel :
         kIrwSimpackRealtimeWheelIdentifiers) {
        *output << '\t' << "wheel_" << wheel
                << ".raw_simpack_rate_radians_per_second"
                << '\t' << "wheel_" << wheel
                << ".forward_circumferential_speed_meters_per_second"
                << '\t' << "wheel_" << wheel << ".contact_patch_count"
                << '\t' << "wheel_" << wheel << ".q_vertical_newtons"
                << '\t' << "wheel_" << wheel << ".normal_force_newtons"
                << '\t' << "wheel_" << wheel
                << ".longitudinal_force_newtons"
                << '\t' << "wheel_" << wheel << ".lateral_force_newtons"
                << '\t' << "wheel_" << wheel
                << ".held_orvd_wheel_torque_newton_metres"
                << '\t' << "wheel_" << wheel
                << ".applied_simpack_input_torque_newton_metres";
    }
    *output << '\n';
}

void WriteObservation(std::ofstream* output,
                      std::uint64_t control_grid_ordinal,
                      double time_seconds,
                      const DecodedObservation& observation,
                      const WheelDriveTorqueChannelValues& held_torques) {
    *output
        << control_grid_ordinal << '\t' << time_seconds << '\t'
        << observation.wheel_speeds
               .common_forward_circumferential_speed_meters_per_second;
    for (const double value : observation.body_signals) {
        *output << '\t' << value;
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output
            << '\t'
            << observation.wheel_speeds
                   .raw_simpack_rates_radians_per_second[wheel]
            << '\t'
            << observation.wheel_speeds
                   .forward_circumferential_speeds_meters_per_second[wheel]
            << '\t' << observation.contact_patch_counts[wheel] << '\t'
            << observation.q_vertical_newtons[wheel] << '\t'
            << observation.total_normal_newtons[wheel] << '\t'
            << observation.total_longitudinal_newtons[wheel] << '\t'
            << observation.total_lateral_newtons[wheel] << '\t'
            << held_torques[wheel] << '\t' << held_torques[wheel];
    }
    *output << '\n';
}

void WritePatchHeader(std::ofstream* output) {
    *output << "control_grid_ordinal\ttime_seconds\tinterface_name"
            << "\tpatch_ordinal"
            << "\tnormal_force_newtons\tlongitudinal_force_newtons"
            << "\tlateral_force_newtons\n";
}

std::size_t WritePatches(std::ofstream* output,
                         std::uint64_t control_grid_ordinal,
                         double time_seconds,
                         const DecodedObservation& observation) {
    std::size_t count{};
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        for (std::size_t patch = 0; patch < kMaximumContactPatchCount;
             ++patch) {
            const ContactPatchForces& value = observation.patches[wheel][patch];
            if (value.longitudinal_newtons == 0.0 &&
                value.lateral_newtons == 0.0 &&
                value.normal_newtons == 0.0) {
                continue;
            }
            *output << control_grid_ordinal << '\t' << time_seconds << '\t'
                    << "wheel_" << kIrwSimpackRealtimeWheelIdentifiers[wheel]
                    << '\t' << patch << '\t'
                    << value.normal_newtons << '\t'
                    << value.longitudinal_newtons << '\t'
                    << value.lateral_newtons << '\n';
            ++count;
        }
    }
    return count;
}

void WriteControlHeader(std::ofstream* output) {
    *output << "event_ordinal\tevent_kind\tevent_time_seconds"
            << "\tcommon_forward_wheel_circumferential_speed_meters_per_second"
            << "\tspeed_error_meters_per_second"
            << "\tcontroller_integral_before_meters"
            << "\tcontroller_filtered_output_before_newton_metres"
            << "\trequested_common_wheel_torque_newton_metres"
            << "\tcontroller_integral_after_meters"
            << "\tcontroller_filtered_output_after_newton_metres";
    for (const std::string_view axle :
         kIrwSimpackRealtimeAxleIdentifiers) {
        *output << '\t' << "axlebridge_" << axle
                << ".requested_differential_torque_newton_metres";
    }
    for (const std::string_view wheel :
         kIrwSimpackRealtimeWheelIdentifiers) {
        *output << '\t' << "wheel_" << wheel
                << ".actual_orvd_wheel_torque_newton_metres"
                << '\t' << "wheel_" << wheel
                << ".applied_simpack_input_torque_newton_metres"
                << '\t' << "wheel_" << wheel << ".dynamic_limit_newton_metres"
                << '\t' << "wheel_" << wheel << ".limit_flags"
                << '\t' << "wheel_" << wheel
                << ".conditioner_memory_before_newton_metres"
                << '\t' << "wheel_" << wheel
                << ".conditioner_memory_after_newton_metres";
    }
    *output << '\n';
}

void WriteControlEvent(std::ofstream* output, std::uint64_t event_ordinal,
                       std::string_view event_kind, double event_time_seconds,
                       const EventCandidate& event) {
    *output
        << event_ordinal << '\t' << event_kind << '\t' << event_time_seconds
        << '\t'
        << event.wheel_speeds
               .common_forward_circumferential_speed_meters_per_second
        << '\t' << event.controller_result.speed_error_meters_per_second
        << '\t' << event.controller_state_before.speed_pi.integral << '\t'
        << event.controller_state_before.speed_pi.filtered_output << '\t'
        << event.controller_result.requested_common_wheel_torque_newton_metres
        << '\t' << event.controller_result.next_state.speed_pi.integral << '\t'
        << event.controller_result.next_state.speed_pi.filtered_output;
    for (const double differential :
         event.differential_torques_newton_metres) {
        *output << '\t' << differential;
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        *output
            << '\t'
            << event.conditioning_result.actual_wheel_torques_newton_metres[wheel]
            << '\t'
            << event.conditioning_result.actual_wheel_torques_newton_metres[wheel]
            << '\t'
            << event.conditioning_result
                   .wheel_dynamic_torque_limits_newton_metres[wheel]
            << '\t'
            << static_cast<std::uint16_t>(
                   event.conditioning_result.limit_flags[wheel])
            << '\t' << event.conditioner_memory_before_newton_metres[wheel]
            << '\t'
            << event.conditioning_result
                   .next_drive_side_torque_memory_newton_metres[wheel];
    }
    *output << '\n';
}

void WriteMetadata(
    const std::filesystem::path& path,
    const IrwSimpackRealtimeRunConfiguration& run_configuration,
    const SimpackRealtimeInstance& instance,
    const configuration::IrwLongitudinalCruiseControllerAsset& controller,
    const actuation::WheelDriveTorqueCommandConditioner& conditioner,
    const IrwSimpackRealtimeRunSummary& summary) {
    std::ofstream output = OpenOutput(path);
    output << "{\n"
           << "  \"complete\": true,\n"
           << "  \"model_path\": "
           << JsonString(instance.model_path().string()) << ",\n"
           << "  \"controller_identifier\": "
           << JsonString(controller.controller.config().identifier) << ",\n"
           << "  \"conditioner_identifier\": "
           << JsonString(conditioner.config().identifier) << ",\n"
           << "  \"last_axle_stop_station_meters\": "
           << run_configuration.last_axle_stop_station_meters << ",\n"
           << "  \"maximum_simulation_time_seconds\": "
           << run_configuration.maximum_simulation_time_seconds << ",\n"
           << "  \"control_sample_period_seconds\": "
           << kFrozenSamplePeriodSeconds << ",\n"
           << "  \"simpack_fixed_integrator_step_seconds\": "
           << ParseSpckScalarAssignment(instance.model_path(),
                                        "slv.integ.fix.h")
           << ",\n"
           << "  \"simpack_communication_timeout_seconds\": "
           << run_configuration.communication_timeout_seconds << ",\n"
           << "  \"simpack_cpu_assignment\": "
           << JsonString(run_configuration.cpu_assignment) << ",\n"
           << "  \"observation_decimation\": "
           << run_configuration.observation_decimation << ",\n"
           << "  \"differential_torque_callback_configured\": "
           << (run_configuration.differential_torque_callback ? "true" : "false")
           << ",\n"
           << "  \"nominal_observation_period_seconds\": "
           << static_cast<double>(run_configuration.observation_decimation) *
                  kFrozenSamplePeriodSeconds
           << ",\n"
           << "  \"input_count\": " << instance.input_names().size()
           << ",\n"
           << "  \"output_count\": " << instance.output_names().size()
           << ",\n"
           << "  \"maximum_contact_patch_count_per_wheel\": "
           << kMaximumContactPatchCount << ",\n"
           << "  \"observation_count\": " << summary.observation_count
           << ",\n"
           << "  \"control_event_count\": " << summary.control_event_count
           << ",\n"
           << "  \"contact_patch_row_count\": "
           << summary.contact_patch_row_count << ",\n"
           << "  \"final_simulation_time_seconds\": "
           << summary.final_simulation_time_seconds << ",\n"
           << "  \"final_last_axle_station_meters\": "
           << summary.final_last_axle_station_meters << ",\n"
           << "  \"solver_advance_wall_time_seconds\": "
           << summary.solver_advance_wall_time_seconds << ",\n"
           << "  \"realtime_loop_wall_time_seconds\": "
           << summary.realtime_loop_wall_time_seconds
           << "\n"
           << "}\n";
    CloseChecked(&output, path);
}

void WriteFailure(const std::filesystem::path& partial,
                  std::string_view detail) noexcept {
    try {
        std::ofstream output(partial / "failure.txt", std::ios::app);
        output << detail << '\n';
    } catch (...) {
    }
}

void ValidateControllerContract(
    const configuration::IrwLongitudinalCruiseControllerAsset& asset,
    const actuation::WheelDriveTorqueCommandConditioner& conditioner) {
    if (!SameBits(asset.controller.config().sample_period_seconds,
                  kFrozenSamplePeriodSeconds) ||
        !SameBits(conditioner.config().sample_period_seconds,
                  kFrozenSamplePeriodSeconds)) {
        Reject("controller and conditioner must use the frozen 0.01-second "
               "sample period");
    }
    if (!std::isfinite(asset.nominal_rolling_radius_meters) ||
        !(asset.nominal_rolling_radius_meters > 0.0)) {
        Reject("nominal rolling radius must be finite and positive");
    }
    for (const double sign : asset.forward_joint_rate_signs) {
        if (sign != -1.0 && sign != 1.0) {
            Reject("forward joint-rate signs must be -1 or +1");
        }
    }
    if (conditioner.config().forward_wheel_angular_speed_sign != -1.0) {
        Reject("conditioner must use the frozen negative-forward SIMPACK "
               "wheel-speed convention");
    }
}

[[nodiscard]] IrwSimpackRealtimeRunSummary Execute(
    const IrwSimpackRealtimeRunConfiguration& run_configuration,
    const std::filesystem::path& model_path,
    const std::filesystem::path& controller_path,
    const std::filesystem::path& conditioner_path,
    const std::filesystem::path& partial) {
    auto controller_asset =
        configuration::LoadIrwLongitudinalCruiseControllerFromJsonFile(
            controller_path);
    auto conditioner =
        configuration::LoadWheelDriveTorqueCommandConditionerFromJsonFile(
            conditioner_path);
    ValidateControllerContract(controller_asset, conditioner);

    const std::filesystem::path observations_path =
        partial / "observations.tsv";
    const std::filesystem::path patches_path = partial / "contact_patches.tsv";
    const std::filesystem::path controls_path = partial / "control_events.tsv";
    std::ofstream observations = OpenOutput(observations_path);
    std::ofstream patches = OpenOutput(patches_path);
    std::ofstream controls = OpenOutput(controls_path);

    SimpackRealtimeInstance instance({
        run_configuration.simpack_installation_path,
        model_path,
        partial / "simpack_realtime.log",
        run_configuration.cpu_assignment,
        run_configuration.communication_timeout_seconds,
        run_configuration.simpack_verbose_level,
    });
    const AbiBinding binding = BindAbi(instance);
    WriteObservationHeader(&observations, binding);
    WritePatchHeader(&patches);
    WriteControlHeader(&controls);

    control::SampledLongitudinalCruiseControllerState controller_state;
    WheelDriveTorqueChannelValues conditioner_memory{};
    WheelDriveTorqueChannelValues previous_actual_torques{};
    std::vector<double> outputs = instance.ReadOutputs();
    DecodedObservation decoded =
        DecodeObservation(outputs, binding, controller_asset);
    EventCandidate event = ComputeEvent(
        0, 0.0, decoded, controller_asset.controller, conditioner,
        controller_state, conditioner_memory, previous_actual_torques,
        run_configuration.differential_torque_callback);
    WheelDriveTorqueChannelValues held_torques =
        event.conditioning_result.actual_wheel_torques_newton_metres;
    const std::vector<double> initial_inputs =
        BindInputs(instance, binding, held_torques);
    instance.SetInitialInputs(initial_inputs);
    outputs = instance.ReadOutputs();
    decoded = DecodeObservation(outputs, binding, controller_asset);

    IrwSimpackRealtimeRunSummary summary;
    WriteControlEvent(&controls, 0, "initialization", 0.0, event);
    ++summary.control_event_count;
    WriteObservation(&observations, 0, 0.0, decoded, held_torques);
    summary.contact_patch_row_count +=
        WritePatches(&patches, 0, 0.0, decoded);
    ++summary.observation_count;
    summary.final_last_axle_station_meters =
        decoded.last_axle_station_meters;

    controller_state = event.controller_result.next_state;
    conditioner_memory =
        event.conditioning_result.next_drive_side_torque_memory_newton_metres;

    const double sample_period =
        controller_asset.controller.config().sample_period_seconds;
    const double maximum_event_count_value =
        std::floor(run_configuration.maximum_simulation_time_seconds /
                   sample_period);
    if (maximum_event_count_value < 1.0) {
        Reject("maximum simulation time is shorter than one control period");
    }
    if (maximum_event_count_value >=
        static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
        Reject("maximum simulation time exceeds the control-grid range");
    }
    const auto maximum_event_count =
        static_cast<std::uint64_t>(maximum_event_count_value);
    const std::uint64_t flush_interval = std::max<std::uint64_t>(
        1, static_cast<std::uint64_t>(
               std::llround(kFailureDataFlushIntervalSeconds / sample_period)));

    const Clock::time_point begin = Clock::now();
    bool complete =
        decoded.last_axle_station_meters >=
        run_configuration.last_axle_stop_station_meters;
    if (!complete) {
        instance.Start();
    }
    for (std::uint64_t ordinal = 1;
         !complete && ordinal <= maximum_event_count; ++ordinal) {
        const std::vector<double> inputs =
            BindInputs(instance, binding, held_torques);
        instance.SetInputs(inputs);
        const double target_time =
            static_cast<double>(ordinal) * sample_period;
        const Clock::time_point advance_begin = Clock::now();
        instance.Advance(target_time);
        summary.solver_advance_wall_time_seconds +=
            ElapsedSeconds(advance_begin, Clock::now());
        outputs = instance.ReadOutputs();
        decoded = DecodeObservation(outputs, binding, controller_asset);
        summary.final_simulation_time_seconds = target_time;
        summary.final_last_axle_station_meters =
            decoded.last_axle_station_meters;
        complete = decoded.last_axle_station_meters >=
                   run_configuration.last_axle_stop_station_meters;
        const bool at_simulation_time_cap = ordinal == maximum_event_count;
        if (ordinal % run_configuration.observation_decimation == 0 ||
            complete || at_simulation_time_cap) {
            WriteObservation(&observations, ordinal, target_time, decoded,
                             held_torques);
            summary.contact_patch_row_count +=
                WritePatches(&patches, ordinal, target_time, decoded);
            ++summary.observation_count;
        }
        event = ComputeEvent(
            ordinal, target_time, decoded, controller_asset.controller,
            conditioner, controller_state, conditioner_memory, held_torques,
            run_configuration.differential_torque_callback);
        WriteControlEvent(&controls, ordinal, "periodic", target_time, event);
        ++summary.control_event_count;
        if (!complete && !at_simulation_time_cap) {
            held_torques =
                event.conditioning_result.actual_wheel_torques_newton_metres;
            controller_state = event.controller_result.next_state;
            conditioner_memory = event.conditioning_result
                                     .next_drive_side_torque_memory_newton_metres;
        }
        if (ordinal % flush_interval == 0) {
            observations.flush();
            patches.flush();
            controls.flush();
            if (!observations || !patches || !controls) {
                Reject("streaming output flush failed");
            }
        }
    }
    summary.realtime_loop_wall_time_seconds =
        ElapsedSeconds(begin, Clock::now());
    if (!complete) {
        Reject("last axle did not reach the required stop station before "
               "the simulation-time cap");
    }

    CloseChecked(&observations, observations_path);
    CloseChecked(&patches, patches_path);
    CloseChecked(&controls, controls_path);
    WriteMetadata(partial / "metadata.json", run_configuration, instance,
                  controller_asset, conditioner, summary);
    return summary;
}

}  // namespace

IrwSimpackRealtimeRunSummary RunIrwSimpackRealtimeCruise(
    const IrwSimpackRealtimeRunConfiguration& configuration) {
    if (!std::filesystem::is_directory(
            configuration.simpack_installation_path)) {
        Reject("simpack_installation_path is not an existing directory");
    }
    if (!std::isfinite(configuration.last_axle_stop_station_meters)) {
        Reject("last_axle_stop_station_meters must be finite");
    }
    if (!std::isfinite(configuration.maximum_simulation_time_seconds) ||
        !(configuration.maximum_simulation_time_seconds > 0.0)) {
        Reject("maximum_simulation_time_seconds must be finite and positive");
    }
    if (configuration.observation_decimation == 0) {
        Reject("observation_decimation must be positive");
    }
    const std::filesystem::path model =
        CanonicalExistingFile(configuration.model_path, "model_path");
    RequireRealtimeExecutionContract(model);
    const std::filesystem::path controller = CanonicalExistingFile(
        configuration.controller_configuration_path,
        "controller_configuration_path");
    const std::filesystem::path conditioner = CanonicalExistingFile(
        configuration.torque_conditioner_configuration_path,
        "torque_conditioner_configuration_path");
    const std::filesystem::path partial =
        CreatePartialDirectory(configuration.output_directory);
    try {
        IrwSimpackRealtimeRunSummary summary = Execute(
            configuration, model, controller, conditioner, partial);
        std::filesystem::rename(partial, configuration.output_directory);
        return summary;
    } catch (const std::exception& error) {
        WriteFailure(partial, error.what());
        throw;
    } catch (...) {
        WriteFailure(partial, "unknown non-standard exception");
        throw;
    }
}

}  // namespace orvd::simpack_realtime
