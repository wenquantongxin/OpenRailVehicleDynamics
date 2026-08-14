#include "simpack_realtime_instance.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <spck_rt.h>

namespace orvd::simpack_realtime {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("SIMPACK Realtime direct call: " + detail);
}

int PrintToFile(void* context, int level, const char* format,
                va_list arguments) {
    auto* file = static_cast<std::FILE*>(context);
    if (file == nullptr || format == nullptr) {
        return 0;
    }
    const char* label = "INFO";
    if (level == SPCK_RT_LOG_WARN) {
        label = "WARN";
    } else if (level == SPCK_RT_LOG_ERROR) {
        label = "ERROR";
    }
    flockfile(file);
    const int prefix_count = std::fprintf(file, "[%s] ", label);
    const int message_count = std::vfprintf(file, format, arguments);
    std::fflush(file);
    funlockfile(file);
    return prefix_count < 0 || message_count < 0 ? -1
                                                 : prefix_count + message_count;
}

void RequireFiniteValues(std::span<const double> values,
                         const std::string& description) {
    for (const double value : values) {
        if (!std::isfinite(value)) {
            Reject(description + " contains a non-finite value");
        }
    }
}

}  // namespace

struct SimpackRealtimeInstance::Implementation final {
    explicit Implementation(
        SimpackRealtimeInstanceConfiguration input_configuration)
        : configuration(std::move(input_configuration)) {}

    ~Implementation() {
        if (instance != nullptr) {
            SpckRtFinish(&instance);
        }
        if (log_file != nullptr) {
            std::fclose(log_file);
        }
    }

    SimpackRealtimeInstanceConfiguration configuration;
    std::filesystem::path canonical_model_path;
    std::FILE* log_file{nullptr};
    SpckRtInstance* instance{nullptr};
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<double> initial_input_values;
    bool started{false};
    double last_target_time_seconds{0.0};
};

SimpackRealtimeInstance::SimpackRealtimeInstance(
    SimpackRealtimeInstanceConfiguration configuration)
    : implementation_(
          std::make_unique<Implementation>(std::move(configuration))) {
    auto& value = *implementation_;
    if (!std::filesystem::is_directory(value.configuration.installation_path)) {
        Reject("installation_path is not an existing directory: '" +
               value.configuration.installation_path.string() + "'");
    }
    std::error_code error;
    value.canonical_model_path =
        std::filesystem::canonical(value.configuration.model_path, error);
    if (error || !std::filesystem::is_regular_file(value.canonical_model_path)) {
        Reject("model_path is not an existing regular file: '" +
               value.configuration.model_path.string() + "'");
    }
    if (value.configuration.log_path.empty() ||
        !std::filesystem::is_directory(
            value.configuration.log_path.parent_path())) {
        Reject("the log path must have an existing parent directory");
    }
    if (!std::isfinite(value.configuration.communication_timeout_seconds) ||
        !(value.configuration.communication_timeout_seconds > 0.0)) {
        Reject("communication_timeout_seconds must be finite and positive");
    }
    if (value.configuration.verbose_level < 0 ||
        value.configuration.verbose_level > 5) {
        Reject("verbose_level must be between zero and five");
    }

    value.log_file = std::fopen(value.configuration.log_path.c_str(), "w");
    if (value.log_file == nullptr) {
        Reject("could not open log file '" +
               value.configuration.log_path.string() + "'");
    }
    const std::string installation =
        value.configuration.installation_path.string();
    const std::string model = value.canonical_model_path.string();
    const char* cpus = value.configuration.cpu_assignment.empty()
                           ? nullptr
                           : value.configuration.cpu_assignment.c_str();
    const int init_error = SpckRtInitPM(
        0, installation.c_str(), model.c_str(), nullptr, cpus, 0,
        value.configuration.verbose_level, &PrintToFile, value.log_file,
        nullptr, value.configuration.communication_timeout_seconds,
        &value.instance);
    if (init_error != 0 || value.instance == nullptr) {
        Reject("SpckRtInitPM failed with code " +
               std::to_string(init_error));
    }

    int input_count{};
    int output_count{};
    SpckRtGetUYDim(value.instance, &input_count, &output_count);
    if (input_count <= 0 || output_count <= 0) {
        Reject("the loaded model exposes a non-positive input/output count");
    }
    value.input_names.reserve(static_cast<std::size_t>(input_count));
    value.initial_input_values.reserve(
        static_cast<std::size_t>(input_count));
    std::unordered_set<std::string> unique_names;
    for (int index = 0; index < input_count; ++index) {
        const char* name = SpckRtGetUInputName(value.instance, index);
        if (name == nullptr || *name == '\0') {
            Reject("u-Input " + std::to_string(index) + " has no name");
        }
        if (!unique_names.emplace(name).second) {
            Reject("duplicate u-Input name '" + std::string(name) + "'");
        }
        if (SpckRtGetUInputID(value.instance, index) != index) {
            Reject("u-Input IDs are not the contiguous external order");
        }
        value.input_names.emplace_back(name);
        value.initial_input_values.push_back(
            SpckRtGetUInputInitialValue(value.instance, index));
    }
    RequireFiniteValues(value.initial_input_values,
                        "initial u-Input values");

    unique_names.clear();
    value.output_names.reserve(static_cast<std::size_t>(output_count));
    for (int index = 0; index < output_count; ++index) {
        const char* name = SpckRtGetYOutputName(value.instance, index);
        if (name == nullptr || *name == '\0') {
            Reject("y-Output " + std::to_string(index) + " has no name");
        }
        if (!unique_names.emplace(name).second) {
            Reject("duplicate y-Output name '" + std::string(name) + "'");
        }
        value.output_names.emplace_back(name);
    }
}

SimpackRealtimeInstance::~SimpackRealtimeInstance() = default;

const std::filesystem::path& SimpackRealtimeInstance::model_path()
    const noexcept {
    return implementation_->canonical_model_path;
}

const std::vector<std::string>& SimpackRealtimeInstance::input_names()
    const noexcept {
    return implementation_->input_names;
}

const std::vector<std::string>& SimpackRealtimeInstance::output_names()
    const noexcept {
    return implementation_->output_names;
}

const std::vector<double>& SimpackRealtimeInstance::initial_input_values()
    const noexcept {
    return implementation_->initial_input_values;
}

std::vector<double> SimpackRealtimeInstance::ReadOutputs() const {
    std::vector<double> outputs(implementation_->output_names.size());
    SpckRtGetY(implementation_->instance, outputs.data());
    RequireFiniteValues(outputs, "y-Output vector");
    return outputs;
}

void SimpackRealtimeInstance::SetInitialInputs(
    std::span<const double> values) {
    if (implementation_->started) {
        Reject("initial inputs cannot be changed after Start");
    }
    if (values.size() != implementation_->input_names.size()) {
        Reject("initial u-Input vector has the wrong size");
    }
    RequireFiniteValues(values, "initial u-Input vector");
    if (SpckRtSetInitialU(implementation_->instance, values.data()) != 0) {
        Reject("SpckRtSetInitialU failed");
    }
}

void SimpackRealtimeInstance::Start() {
    if (implementation_->started) {
        Reject("Start was called more than once");
    }
    if (SpckRtStart(implementation_->instance) != 0) {
        Reject("SpckRtStart failed");
    }
    implementation_->started = true;
}

void SimpackRealtimeInstance::SetInputs(std::span<const double> values) {
    if (!implementation_->started) {
        Reject("SetInputs requires a started instance");
    }
    if (values.size() != implementation_->input_names.size()) {
        Reject("u-Input vector has the wrong size");
    }
    RequireFiniteValues(values, "u-Input vector");
    SpckRtSetU(implementation_->instance, values.data());
}

void SimpackRealtimeInstance::Advance(double target_time_seconds) {
    if (!implementation_->started) {
        Reject("Advance requires a started instance");
    }
    if (!std::isfinite(target_time_seconds) ||
        !(target_time_seconds >
          implementation_->last_target_time_seconds)) {
        Reject("advance targets must be finite and strictly increasing");
    }
    const int result =
        SpckRtAdvance(implementation_->instance, target_time_seconds);
    if (result != 0) {
        Reject("SpckRtAdvance(" + std::to_string(target_time_seconds) +
               ") failed with code " + std::to_string(result));
    }
    implementation_->last_target_time_seconds = target_time_seconds;
}

}  // namespace orvd::simpack_realtime
