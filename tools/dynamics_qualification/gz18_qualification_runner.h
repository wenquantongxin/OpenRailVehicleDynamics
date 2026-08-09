#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace orvd::dynamics_qualification {

// One internal GZ18 qualification recipe. This type belongs to the migration
// tool, is not installed and is not a public vehicle-simulation contract.
struct Gz18QualificationRunConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::string track_irregularity_identifier;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
    std::int64_t sample_period_nanoseconds{};
};

struct Gz18QualificationRunSummary final {
    std::size_t sample_count{};
    double advance_wall_seconds{};
    double observation_wall_seconds{};
    double data_and_metadata_write_wall_seconds{};
    bool used_before_track_definition_interval{false};
    bool used_after_track_definition_interval{false};
};

// Runs one qualification in a private assembled system. The input files are
// read once; neither a mutable runtime context nor a final state is accepted or
// returned. The destination must not already exist. A successful call publishes
// one complete directory with a same-parent rename; a failure publishes no
// destination and destroys the private simulation state.
[[nodiscard]] Gz18QualificationRunSummary RunGz18Qualification(
    const Gz18QualificationRunConfiguration& configuration);

}  // namespace orvd::dynamics_qualification
