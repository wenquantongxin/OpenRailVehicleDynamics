#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace orvd::dynamics_qualification::internal {

struct QualificationCliOptions final {
    // Includes argv[0], preserving the existing positional CLI layouts.
    std::vector<char*> positional_arguments;
    std::optional<std::filesystem::path> integration_config_path;
    bool publish_scene_record{};
};

// This parses only the shared named options. Each executable retains its own
// positional paths, duration validation and usage message.
inline QualificationCliOptions ParseQualificationCliOptions(
    int argc, char** argv, bool allow_scene_record = false) {
    QualificationCliOptions result;
    result.positional_arguments.reserve(static_cast<std::size_t>(argc));
    result.positional_arguments.push_back(argv[0]);
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument(argv[index]);
        if (allow_scene_record && argument == "--scene-record") {
            result.publish_scene_record = true;
        } else if (argument == "--integration-config") {
            if (result.integration_config_path.has_value()) {
                throw std::invalid_argument("--integration-config may be specified only once");
            }
            if (index + 1 == argc || std::string_view(argv[index + 1]).empty() ||
                std::string_view(argv[index + 1]).starts_with("--")) {
                throw std::invalid_argument("--integration-config requires one non-empty path");
            }
            result.integration_config_path = argv[++index];
        } else if (argument.starts_with("--")) {
            throw std::invalid_argument("unknown option: " + std::string(argument));
        } else {
            result.positional_arguments.push_back(argv[index]);
        }
    }
    return result;
}

}  // namespace orvd::dynamics_qualification::internal
