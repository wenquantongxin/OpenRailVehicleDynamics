#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

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
    const auto& track = *record.assets.track;
    const auto quaternion = [](const auto& wxyz) {
        return Eigen::Quaterniond(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
    };
    const auto body_slot = [&](const std::string& name) {
        const auto found = std::find_if(description.topology.bodies.begin(),
            description.topology.bodies.end(), [&](const auto& body) { return body.name == name; });
        require(found != description.topology.bodies.end(), "heading body missing from topology");
        return static_cast<std::size_t>(found - description.topology.bodies.begin());
    };
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
            const auto& name = description.scalar_definitions[index].name;
            if (!name.ends_with(".yaw_radians")) continue;
            const auto body = name.substr(0, name.rfind('.'));
            const auto station_field = std::find_if(description.scalar_definitions.begin(),
                description.scalar_definitions.end(), [&](const auto& field) {
                    return field.name == body + ".track_station_meters";
                });
            require(station_field != description.scalar_definitions.end(), "heading station missing");
            const double station = frame.scalars.values.at(station_field - description.scalar_definitions.begin());
            const auto upper = std::upper_bound(track.stations_meters.begin(), track.stations_meters.end(), station);
            const auto next = std::clamp<std::size_t>(upper - track.stations_meters.begin(), 1, track.stations_meters.size() - 1);
            const auto previous = next - 1;
            const double fraction = std::clamp((station - track.stations_meters[previous]) /
                (track.stations_meters[next] - track.stations_meters[previous]), 0.0, 1.0);
            const auto track_rotation = quaternion(track.rotation_inertial_from_track_wxyz[previous]).slerp(
                fraction, quaternion(track.rotation_inertial_from_track_wxyz[next])).toRotationMatrix();
            const auto wheel = std::find_if(description.topology.wheel_placements.begin(),
                description.topology.wheel_placements.end(), [&](const auto& placement) {
                    return placement.carrier_body_name == body;
                });
            Eigen::Vector3d direction;
            double expected;
            if (wheel == description.topology.wheel_placements.end()) {
                direction = track_rotation.transpose() *
                    quaternion(frame.bodies[body_slot(body)].orientation_wxyz).toRotationMatrix().col(0);
                expected = std::atan2(direction.y(), direction.x());
            } else {
                direction = track_rotation.transpose() *
                    quaternion(frame.bodies[body_slot(wheel->wheel_body_name)].orientation_wxyz).toRotationMatrix() *
                    wheel->spin_axis_in_wheel_body_frame;
                expected = std::atan(-direction.x() / direction.y());
            }
            require(std::abs(frame.scalars.values[index] - expected) < 1e-10,
                    "recorded heading disagrees with the body or wheel geometry");
        }
    }
}

}  // namespace scene_record_checks
