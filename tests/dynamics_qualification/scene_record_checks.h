#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "orvd/scene_record/scene_record_reader.h"

namespace scene_record_checks {

inline void CheckExport(const std::filesystem::path& plain,
                        const std::filesystem::path& recorded,
                        std::int64_t duration_ns, std::int64_t period_ns,
                        bool independent_wheels) {
    const auto require = [](bool condition, const char* message) {
        if (!condition) throw std::runtime_error(message);
    };
    const auto read = [](const std::filesystem::path& path) {
        std::ifstream input(path, std::ios::binary);
        if (!input) throw std::runtime_error("cannot read " + path.string());
        return std::string(std::istreambuf_iterator<char>(input), {});
    };
    require(!std::filesystem::exists(plain / "scene_record"),
            "scene export must be opt-in");
    for (const char* file : {"continuous_states.tsv", "observations.tsv",
                             "contact_patches.tsv"}) {
        require(read(plain / file) == read(recorded / file),
                "scene export changed a physical output");
    }
    const auto record = orvd::scene_record::ReadSceneRecord(recorded / "scene_record");
    const auto& description = record.description;
    const auto count = static_cast<std::size_t>(duration_ns / period_ns + 1);
    require(record.frames.size() == count &&
                description.nominal_sample_period_seconds ==
                    static_cast<double>(period_ns) * 1e-9,
            "scene sample count or nominal period differs from the clock");
    require(record.assets.track.has_value() &&
                !record.assets.track->stations_meters.empty() &&
                !record.assets.visual_definition_json.empty(),
            "scene export omitted track or visual assets");
    require(description.topology.wheel_placements.size() == 8 &&
                description.scalar_definitions.size() == 45,
            "scene export omitted wheel placements or physical readouts");
    std::set<std::string> bodies;
    for (const auto& body : description.topology.bodies) bodies.insert(body.name);
    for (const auto& field : description.scalar_definitions) {
        for (const char* suffix : {".track_station_meters", ".lateral_meters", ".yaw_radians"}) {
            if (field.name.ends_with(suffix)) {
                require(bodies.contains(field.name.substr(0, field.name.rfind('.'))),
                        "track-coordinate scalar is not named after its body");
                require(field.unit == (std::string(suffix) == ".yaw_radians" ? "rad" : "m"),
                        "track-coordinate scalar has the wrong unit");
            }
        }
    }
    const auto split = [](const std::string& line) {
        std::vector<std::string> fields;
        std::istringstream input(line);
        for (std::string field; std::getline(input, field, '\t');) fields.push_back(field);
        return fields;
    };
    std::istringstream observations(read(recorded / "observations.tsv"));
    std::string line;
    std::getline(observations, line);
    const auto columns = split(line);
    for (std::size_t sample = 0; sample < count; ++sample) {
        require(static_cast<bool>(std::getline(observations, line)), "missing observation row");
        const auto row = split(line);
        const auto& frame = record.frames[sample];
        using orvd::scene_observation::SamplePhase;
        const auto phase = sample == 0 ? SamplePhase::kInitialAcceptedState :
            sample + 1 == count ? SamplePhase::kAcceptedEndpoint : SamplePhase::kDenseIntermediateSample;
        require(frame.identity.sample_index == static_cast<std::int64_t>(sample) &&
                    frame.identity.time_nanoseconds == static_cast<std::int64_t>(sample) * period_ns &&
                    frame.identity.time_seconds == std::stod(row.at(2)) && frame.identity.phase == phase,
                "scene frame lost its sample identity");
        require(frame.bodies.size() == bodies.size() &&
                    frame.wheel_spin_angles_radians.size() == (independent_wheels ? 8U : 0U),
                "scene body or wheel-angle layout is wrong");
        for (std::size_t index = 0; index < description.scalar_definitions.size(); ++index) {
            const auto column = std::find(columns.begin(), columns.end(), description.scalar_definitions[index].name);
            require(column != columns.end(), "scene scalar is absent from observations");
            require(frame.scalars.values.at(index) == std::stod(row.at(column - columns.begin())) &&
                        frame.scalars.statuses.at(index) == static_cast<std::uint8_t>(orvd::scene_observation::ScalarStatus::kValid),
                    "scene scalar differs from the recorded physical observation");
        }
    }
}

}  // namespace scene_record_checks
