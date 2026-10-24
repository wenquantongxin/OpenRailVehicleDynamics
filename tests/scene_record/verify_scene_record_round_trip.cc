#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
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
    wheel.spin_joint_name = "wheel_spin";
    wheel.spin_angle_scalar = scene_observation::WheelSpinScalarBinding{"a", -1.0};
    topology.wheel_placements.push_back(wheel);
    return topology;
}

std::vector<scene_observation::ScalarDefinition> MakeDefinitions() {
    return {{"a", "rad", "wheel", "spin", "stated", "instantaneous"},
            {"b", "N", "world", "force", "stated", "instantaneous"},
            {"c", "1", "none", "count", "stated", "instantaneous"}};
}

scene_observation::SceneDescription MakeDescription() {
    scene_observation::SceneDescription description;
    description.topology = MakeTopology();
    description.scalar_definitions = MakeDefinitions();
    description.nominal_sample_period_seconds = 0.01;
    return description;
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
    table.left_rail_datum_in_track = {-0.75, 0.0};
    table.right_rail_datum_in_track = {0.75, 0.0};
    return table;
}

template <typename Function>
void ExpectRefused(Function action, const std::string& detail) {
    try {
        action();
    } catch (const std::exception&) {
        return;
    }
    Fail(detail);
}

void VerifyAssets(const std::filesystem::path& fixtures,
                  const scene_record::SceneAssets& recorded_assets) {
    const auto bundle_directory = fixtures / "bundle";
    scene_record::WriteSceneBundle(bundle_directory, recorded_assets);
    if (std::filesystem::exists(bundle_directory / "frames.f64le") ||
        std::filesystem::exists(bundle_directory / "scalar_statuses.u8") ||
        std::filesystem::exists(bundle_directory / "scene.json.pending")) {
        Fail("a bundle contains frame tables or an unpublished scene document");
    }
    const auto bundle = scene_record::ReadSceneBundle(bundle_directory);
    const auto bundle_assets =
        scene_record::ReadSceneAssets(bundle_directory / "scene.json");
    if (!bundle.track || !bundle_assets.track ||
        bundle.world_frame != recorded_assets.world_frame ||
        bundle.visual_definition_json != recorded_assets.visual_definition_json ||
        bundle.track->stations_meters != recorded_assets.track->stations_meters ||
        bundle_assets.track->left_rail_datum_in_track.lateral_meters != -0.75 ||
        bundle_assets.track->right_rail_datum_in_track.lateral_meters != 0.75) {
        Fail("bundle assets did not round-trip");
    }
    ExpectRefused([&] {
        scene_record::WriteSceneBundle(bundle_directory, recorded_assets);
    }, "a bundle overwrote an existing directory");
    ExpectRefused([&] {
        (void)scene_record::ReadSceneBundle(fixtures / "record");
    }, "ReadSceneBundle accepted a scene record");
    ExpectRefused([&] {
        (void)scene_record::ReadSceneRecord(bundle_directory);
    }, "ReadSceneRecord accepted a bundle");

    // Static asset extraction must not open either binary table.
    const auto assets_only_directory = fixtures / "record_assets_only";
    std::filesystem::copy(fixtures / "record", assets_only_directory,
                          std::filesystem::copy_options::recursive);
    std::filesystem::remove(assets_only_directory / "frames.f64le");
    std::filesystem::remove(assets_only_directory / "scalar_statuses.u8");
    const auto assets_only =
        scene_record::ReadSceneAssets(assets_only_directory / "scene.json");
    if (!assets_only.track ||
        assets_only.visual_definition_json != recorded_assets.visual_definition_json) {
        Fail("static asset extraction depended on frame tables");
    }
    ExpectRefused([&] {
        (void)scene_record::ReadSceneRecord(assets_only_directory);
    }, "a record without its frame tables was accepted for playback");

    scene_record::SceneAssets selected;
    const auto inherited = scene_record::ComposeSceneAssets(recorded_assets, selected);
    if (!inherited.track ||
        inherited.visual_definition_json != recorded_assets.visual_definition_json) {
        Fail("missing selected assets did not retain the base assets");
    }
    selected.visual_definition_json = "{ \"vehicle_name\": \"selected\" }\n";
    const auto visual_override =
        scene_record::ComposeSceneAssets(recorded_assets, selected);
    if (!visual_override.track ||
        visual_override.visual_definition_json != selected.visual_definition_json) {
        Fail("selected visual definition did not override just the visual asset");
    }
    selected.visual_definition_json.clear();
    selected.track = MakeTrack();
    selected.track->superelevation_meters = {0.01, 0.02};
    const auto track_override =
        scene_record::ComposeSceneAssets(recorded_assets, selected);
    if (track_override.track->superelevation_meters != std::vector<double>{0.01, 0.02} ||
        track_override.visual_definition_json != recorded_assets.visual_definition_json) {
        Fail("selected track did not override just the track asset");
    }
    selected.visual_definition_json = "{\"vehicle_name\":\"both\"}";
    const auto both_override =
        scene_record::ComposeSceneAssets(recorded_assets, selected);
    if (both_override.track->superelevation_meters != std::vector<double>{0.01, 0.02} ||
        both_override.visual_definition_json != selected.visual_definition_json) {
        Fail("selected track and visual definition did not override both assets");
    }
    selected.world_frame = "another world";
    ExpectRefused([&] {
        (void)scene_record::ComposeSceneAssets(recorded_assets, selected);
    }, "assets from different world frames were composed");
    ExpectRefused([&] {
        scene_record::SceneRecordWriter writer(
            fixtures / "different_world", MakeDescription(), selected, false);
    }, "record description and assets with different frames were accepted");
    if (std::filesystem::exists(fixtures / "different_world")) {
        Fail("invalid record inputs created a directory");
    }

    const auto empty_directory = fixtures / "empty_bundle";
    scene_record::WriteSceneBundle(empty_directory, {});
    const auto empty = scene_record::ReadSceneBundle(empty_directory);
    if (empty.track || !empty.visual_definition_json.empty()) {
        Fail("absent bundle assets did not round-trip");
    }
    auto invalid = recorded_assets;
    invalid.track->left_rail_datum_in_track.vertical_meters =
        std::numeric_limits<double>::quiet_NaN();
    ExpectRefused([&] {
        scene_record::WriteSceneBundle(fixtures / "invalid_datum", invalid);
    }, "a nonfinite rail datum was accepted");
    invalid = recorded_assets;
    invalid.track->centerline_in_inertial_meters.pop_back();
    ExpectRefused([&] {
        scene_record::WriteSceneBundle(fixtures / "invalid_track", invalid);
    }, "track columns of different lengths were accepted");
    invalid = recorded_assets;
    invalid.track->stations_meters = {5.0, -5.0};
    ExpectRefused([&] {
        scene_record::WriteSceneBundle(fixtures / "unordered_track", invalid);
    }, "track stations that are not increasing were accepted");
    invalid = recorded_assets;
    invalid.visual_definition_json = "[]";
    ExpectRefused([&] {
        scene_record::WriteSceneBundle(fixtures / "invalid_visual", invalid);
    }, "a non-object visual definition was accepted");
    if (std::filesystem::exists(fixtures / "invalid_datum") ||
        std::filesystem::exists(fixtures / "invalid_track") ||
        std::filesystem::exists(fixtures / "invalid_visual") ||
        std::filesystem::exists(fixtures / "unordered_track")) {
        Fail("invalid bundle inputs created a directory");
    }
    const auto legacy_directory = fixtures / "legacy";
    std::filesystem::create_directory(legacy_directory);
    std::ofstream(legacy_directory / "scene.json") <<
        "{\"record\":\"orvd.scene_record\"}";
    ExpectRefused([&] {
        (void)scene_record::ReadSceneAssets(legacy_directory / "scene.json");
    }, "legacy record assets were accepted");
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
                                                   MakeDescription(),
                                                   {.visual_definition_json = visual_json},
                                                   true);
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
                                                  MakeDescription(), {}, true);
        } catch (const std::runtime_error&) {
            existing_refused = true;
        }
        // A record without spin angles is a different, equally valid layout.
        {
            const std::filesystem::path plain_directory = fixtures / "plain";
            scene_record::SceneRecordWriter plain(plain_directory,
                                                  MakeDescription(), {}, false);
            scene_observation::SceneFrame frame = MakeFrame(0);
            frame.wheel_spin_angles_radians.clear();
            plain.WriteFrame(frame);
            plain.Close();
            const auto plain_record = scene_record::ReadSceneRecord(plain_directory);
            if (plain_record.frames.size() != 1 ||
                !plain_record.frames[0].wheel_spin_angles_radians.empty() ||
                plain_record.assets.visual_definition_json != "") {
                Fail("a record without spin angles did not round-trip");
            }
        }
        if (!existing_refused) {
            Fail("the writer overwrote an existing directory");
        }
        // A placement whose carrier body the topology does not list is
        // refused before anything is written.
        {
            auto unlisted = MakeDescription();
            unlisted.topology.wheel_placements[0].carrier_body_name = "missing";
            bool refused = false;
            try {
                scene_record::SceneRecordWriter bad(fixtures / "unlisted",
                                                    std::move(unlisted),
                                                    {}, false);
            } catch (const std::runtime_error&) {
                refused = true;
            }
            if (!refused || std::filesystem::exists(fixtures / "unlisted")) {
                Fail("a placement naming an unlisted carrier body was accepted");
            }
        }

        const auto record = scene_record::ReadSceneRecord(record_directory);
        if (record.description.topology.bodies.size() != 2 ||
            record.description.topology.bodies[0].name != "carrier" ||
            !record.description.topology.bodies[0].moves_freely_in_world ||
            record.description.topology.bodies[1].moves_freely_in_world ||
            record.description.topology.wheel_placements.size() != 1 ||
            record.description.topology.wheel_placements[0].wheel_body_name != "wheel" ||
            record.description.topology.wheel_placements[0].carrier_body_name != "carrier" ||
            record.description.topology.wheel_placements[0].datum_in_wheel_body_frame_meters !=
                Eigen::Vector3d(0.0, -0.7465, 0.0) ||
            record.description.topology.wheel_placements[0].nominal_rolling_radius_meters !=
                0.43 ||
            record.description.topology.wheel_placements[0].spin_joint_name !=
                std::optional<std::string>("wheel_spin") ||
            !record.description.topology.wheel_placements[0].spin_angle_scalar ||
            record.description.topology.wheel_placements[0].spin_angle_scalar->name != "a" ||
            record.description.topology.wheel_placements[0].spin_angle_scalar->sign != -1.0 ||
            record.description.nominal_sample_period_seconds != std::optional<double>(0.01)) {
            Fail("the topology did not round-trip");
        }
        if (record.description.scalar_definitions.size() != 3 ||
            record.description.scalar_definitions[1].name != "b" ||
            record.description.scalar_definitions[1].unit != "N" ||
            record.assets.visual_definition_json != visual_json) {
            Fail("scalar definitions or the visual definition did not "
                 "round-trip");
        }
        if (!record.assets.track.has_value() ||
            record.assets.track->stations_meters != std::vector<double>{-5.0, 5.0} ||
            record.assets.track->right_rail_datum_in_inertial_meters[1] !=
                std::array<double, 3>{5.0, 0.75, 0.0} ||
            record.assets.track->left_rail_datum_in_track.lateral_meters != -0.75 ||
            record.assets.track->right_rail_datum_in_track.lateral_meters != 0.75) {
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
        VerifyAssets(fixtures, record.assets);
        std::puts("scene record and bundle round trip, asset composition, "
                  "incomplete-record refusal and "
                  "writer rejections passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
