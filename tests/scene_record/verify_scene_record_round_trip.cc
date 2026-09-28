#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "orvd/scene_record/scene_record_reader.h"
#include "orvd/scene_record/scene_record_writer.h"

// A record written from stated values and read back must return the same
// values, statuses and identities, including sample identities that differ
// from the frame row numbers; a record that was not closed must be refused as
// incomplete; the writer must refuse an existing directory, a placement that
// names a body the topology does not list, a wrong body count and a non-valid
// scalar carrying a value.

namespace {

using namespace orvd;

[[noreturn]] void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

scene_observation::SceneTopology MakeTopology() {
    scene_observation::SceneTopology topology;
    topology.bodies = {{"carrier", true}, {"wheel", false}};
    scene_observation::SceneWheelPlacement wheel;
    wheel.interface_name = "wheel_r";
    wheel.wheel_body_name = "wheel";
    wheel.carrier_body_name = "carrier";
    wheel.side = wheel_rail_contact::WheelSide::kRight;
    wheel.datum_in_wheel_body_frame_meters = Eigen::Vector3d(0.0, -0.7465, 0.0);
    wheel.spin_axis_in_wheel_body_frame = Eigen::Vector3d(0.0, -1.0, 0.0);
    wheel.nominal_rolling_radius_meters = 0.43;
    topology.wheel_placements.push_back(wheel);
    return topology;
}

std::vector<scene_observation::ScalarDefinition> MakeDefinitions() {
    return {{"a", "m", "world", "length", "stated", "instantaneous"},
            {"b", "N", "world", "force", "stated", "instantaneous"},
            {"c", "1", "none", "count", "stated", "instantaneous"}};
}

// Sample identities come from the run program and need not equal the frame
// row numbers.
constexpr std::int64_t kSampleIdentities[] = {42, 105, 901, 1300};

scene_observation::SceneFrame MakeFrame(int index) {
    scene_observation::SceneFrame frame;
    frame.identity.time_seconds = 0.01 * index;
    frame.identity.time_nanoseconds = 10000000LL * index;
    frame.identity.sample_index = kSampleIdentities[index];
    frame.identity.phase =
        index == 0 ? scene_observation::SamplePhase::kInitialAcceptedState
        : index == 2 ? scene_observation::SamplePhase::kAcceptedEndpoint
                     : scene_observation::SamplePhase::kDenseIntermediateSample;
    for (int body = 0; body < 2; ++body) {
        scene_observation::BodyState state;
        state.position_meters = {1.0 * index, -2.0 * body, 0.5};
        state.orientation_wxyz = {0.8, 0.0, 0.6, 0.0};
        state.linear_velocity_meters_per_second = {16.7, 0.0, 0.0};
        state.angular_velocity_radians_per_second = {0.0, -38.8 * body, 0.0};
        frame.bodies.push_back(state);
    }
    frame.wheel_spin_angles_radians = {7.25 * index};
    frame.scalars.values = {1.5 + index, 0.0, 0.0};
    frame.scalars.statuses = {1, 0, 2};
    return frame;
}

scene_observation::TrackSampleTable MakeTrack() {
    scene_observation::TrackSampleTable table;
    table.stations_meters = {-5.0, 5.0};
    table.centerline_in_inertial_meters = {{-5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
    table.rotation_inertial_from_track_wxyz = {{1.0, 0.0, 0.0, 0.0},
                                               {1.0, 0.0, 0.0, 0.0}};
    table.curvature_radians_per_meter = {0.0, 0.0};
    table.superelevation_meters = {0.0, 0.0};
    table.left_rail_datum_in_inertial_meters = {{-5.0, -0.75, 0.0},
                                                {5.0, -0.75, 0.0}};
    table.right_rail_datum_in_inertial_meters = {{-5.0, 0.75, 0.0},
                                                 {5.0, 0.75, 0.0}};
    return table;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr,
                     "usage: verify_scene_record_round_trip FIXTURE_DIRECTORY\n");
        return 2;
    }
    try {
        const std::filesystem::path fixtures = argv[1];
        std::filesystem::remove_all(fixtures);
        std::filesystem::create_directories(fixtures);
        const std::filesystem::path record_directory = fixtures / "record";
        const std::string visual_json =
            "{\"vehicle_name\": \"stated\", \"parts\": []}\n";
        {
            scene_record::SceneRecordWriter writer(record_directory,
                                                   MakeTopology(),
                                                   MakeDefinitions(),
                                                   visual_json, true);
            for (int index = 0; index < 3; ++index) {
                writer.WriteFrame(MakeFrame(index));
            }
            writer.WriteTrackSampleTable(MakeTrack());
            bool incomplete_refused = false;
            try {
                (void)scene_record::ReadSceneRecord(record_directory);
            } catch (const std::runtime_error&) {
                incomplete_refused = true;
            }
            if (!incomplete_refused) {
                Fail("a record without scene.json was accepted");
            }
            scene_observation::SceneFrame wrong_bodies = MakeFrame(3);
            wrong_bodies.bodies.pop_back();
            bool body_count_refused = false;
            try {
                writer.WriteFrame(wrong_bodies);
            } catch (const std::runtime_error&) {
                body_count_refused = true;
            }
            scene_observation::SceneFrame nonzero_placeholder = MakeFrame(3);
            nonzero_placeholder.scalars.values[2] = 4.0;
            bool placeholder_refused = false;
            try {
                writer.WriteFrame(nonzero_placeholder);
            } catch (const std::runtime_error&) {
                placeholder_refused = true;
            }
            scene_observation::SceneFrame missing_spin = MakeFrame(3);
            missing_spin.wheel_spin_angles_radians.clear();
            bool spin_count_refused = false;
            try {
                writer.WriteFrame(missing_spin);
            } catch (const std::runtime_error&) {
                spin_count_refused = true;
            }
            scene_observation::SceneFrame huge_identity = MakeFrame(3);
            huge_identity.identity.time_nanoseconds = 9007199254740993LL;
            bool identity_refused = false;
            try {
                writer.WriteFrame(huge_identity);
            } catch (const std::runtime_error&) {
                identity_refused = true;
            }
            if (!body_count_refused || !placeholder_refused ||
                !spin_count_refused || !identity_refused ||
                writer.frame_count() != 3) {
                Fail("the writer accepted a wrong body count, a non-valid "
                     "scalar carrying a value, a wrong spin-angle count or an "
                     "integer identity beyond the exact range");
            }
            writer.Close();
        }
        bool existing_refused = false;
        try {
            scene_record::SceneRecordWriter again(record_directory,
                                                  MakeTopology(),
                                                  MakeDefinitions(), "", true);
        } catch (const std::runtime_error&) {
            existing_refused = true;
        }
        // A record without spin angles is a different, equally valid layout.
        {
            const std::filesystem::path plain_directory = fixtures / "plain";
            scene_record::SceneRecordWriter plain(plain_directory,
                                                  MakeTopology(),
                                                  MakeDefinitions(), "", false);
            scene_observation::SceneFrame frame = MakeFrame(0);
            frame.wheel_spin_angles_radians.clear();
            plain.WriteFrame(frame);
            plain.Close();
            const auto plain_record = scene_record::ReadSceneRecord(plain_directory);
            if (plain_record.frames.size() != 1 ||
                !plain_record.frames[0].wheel_spin_angles_radians.empty() ||
                plain_record.visual_definition_json != "") {
                Fail("a record without spin angles did not round-trip");
            }
        }
        if (!existing_refused) {
            Fail("the writer overwrote an existing directory");
        }
        // A placement whose carrier body the topology does not list is
        // refused before anything is written.
        {
            auto unlisted = MakeTopology();
            unlisted.wheel_placements[0].carrier_body_name = "missing";
            bool refused = false;
            try {
                scene_record::SceneRecordWriter bad(fixtures / "unlisted",
                                                    std::move(unlisted),
                                                    MakeDefinitions(), "", false);
            } catch (const std::runtime_error&) {
                refused = true;
            }
            if (!refused || std::filesystem::exists(fixtures / "unlisted")) {
                Fail("a placement naming an unlisted carrier body was accepted");
            }
        }

        const auto record = scene_record::ReadSceneRecord(record_directory);
        if (record.topology.bodies.size() != 2 ||
            record.topology.bodies[0].name != "carrier" ||
            !record.topology.bodies[0].moves_freely_in_world ||
            record.topology.bodies[1].moves_freely_in_world ||
            record.topology.wheel_placements.size() != 1 ||
            record.topology.wheel_placements[0].wheel_body_name != "wheel" ||
            record.topology.wheel_placements[0].carrier_body_name != "carrier" ||
            record.topology.wheel_placements[0].datum_in_wheel_body_frame_meters !=
                Eigen::Vector3d(0.0, -0.7465, 0.0) ||
            record.topology.wheel_placements[0].nominal_rolling_radius_meters !=
                0.43) {
            Fail("the topology did not round-trip");
        }
        if (record.scalar_definitions.size() != 3 ||
            record.scalar_definitions[1].name != "b" ||
            record.scalar_definitions[1].unit != "N" ||
            record.visual_definition_json != visual_json) {
            Fail("scalar definitions or the visual definition did not "
                 "round-trip");
        }
        if (!record.track.has_value() ||
            record.track->stations_meters != std::vector<double>{-5.0, 5.0} ||
            record.track->right_rail_datum_in_inertial_meters[1] !=
                std::array<double, 3>{5.0, 0.75, 0.0}) {
            Fail("the track table did not round-trip");
        }
        if (record.frames.size() != 3) {
            Fail("the frame count did not round-trip");
        }
        for (int index = 0; index < 3; ++index) {
            const auto expected = MakeFrame(index);
            const auto& actual = record.frames[static_cast<std::size_t>(index)];
            if (actual.identity.sample_index != kSampleIdentities[index]) {
                Fail("the sample identity was replaced by the row number");
            }
            if (actual.identity.time_seconds != expected.identity.time_seconds ||
                actual.identity.time_nanoseconds !=
                    expected.identity.time_nanoseconds ||
                actual.identity.sample_index != expected.identity.sample_index ||
                actual.identity.phase != expected.identity.phase) {
                Fail("frame identity did not round-trip");
            }
            for (std::size_t body = 0; body < 2; ++body) {
                if (actual.bodies[body].position_meters !=
                        expected.bodies[body].position_meters ||
                    actual.bodies[body].orientation_wxyz !=
                        expected.bodies[body].orientation_wxyz ||
                    actual.bodies[body].linear_velocity_meters_per_second !=
                        expected.bodies[body].linear_velocity_meters_per_second ||
                    actual.bodies[body].angular_velocity_radians_per_second !=
                        expected.bodies[body]
                            .angular_velocity_radians_per_second) {
                    Fail("a body state did not round-trip bit for bit");
                }
            }
            if (actual.scalars.values != expected.scalars.values ||
                actual.scalars.statuses != expected.scalars.statuses ||
                actual.wheel_spin_angles_radians !=
                    expected.wheel_spin_angles_radians) {
                Fail("scalar values, statuses or spin angles did not round-trip");
            }
        }
        std::puts("scene record round trip, incomplete-record refusal and "
                  "writer rejections passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
