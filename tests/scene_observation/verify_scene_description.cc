#include <cstddef>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include "orvd/scene_observation/scene_description.h"

namespace {

using orvd::scene_observation::DecodeSceneDescription;
using orvd::scene_observation::EncodeSceneDescription;
using orvd::scene_observation::SceneDescription;
using orvd::scene_observation::SceneWheelPlacement;
using orvd::scene_observation::WheelSpinScalarBinding;

[[noreturn]] void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

SceneDescription MakeDescription() {
    SceneDescription description;
    description.topology.bodies = {{"carrier", true}, {"right_wheel", false}};
    description.scalar_definitions = {
        {"right_angle", "rad", "wheel profile", "spin angle",
         "revolute joint position", "unwrapped angle at sample time"},
        {"speed", "m/s", "world", "speed", "body velocity norm",
         "at sample time"}};
    SceneWheelPlacement right;
    right.interface_name = "right_contact";
    right.wheel_body_name = "right_wheel";
    right.carrier_body_name = "carrier";
    right.side = orvd::wheel_rail_contact::WheelSide::kRight;
    right.datum_in_wheel_body_frame_meters = Eigen::Vector3d(0.1, -0.75, 0.2);
    right.spin_axis_in_wheel_body_frame = -Eigen::Vector3d::UnitY();
    right.nominal_rolling_radius_meters = 0.43;
    right.spin_joint_name = "right_revolute_joint";
    right.spin_angle_scalar = WheelSpinScalarBinding{"right_angle", -1.0};
    description.topology.wheel_placements.push_back(right);
    SceneWheelPlacement left = right;
    left.interface_name = "left_contact";
    left.wheel_body_name = "carrier";
    left.side = orvd::wheel_rail_contact::WheelSide::kLeft;
    left.datum_in_wheel_body_frame_meters.y() = 0.75;
    left.spin_joint_name.reset();
    left.spin_angle_scalar.reset();
    description.topology.wheel_placements.push_back(left);
    description.nominal_sample_period_seconds = 0.005;
    return description;
}

void CheckEqual(const SceneDescription& expected,
                const SceneDescription& actual) {
    if (actual.world_frame != expected.world_frame ||
        actual.nominal_sample_period_seconds !=
            expected.nominal_sample_period_seconds ||
        actual.topology.bodies.size() != expected.topology.bodies.size() ||
        actual.topology.wheel_placements.size() !=
            expected.topology.wheel_placements.size() ||
        actual.scalar_definitions.size() != expected.scalar_definitions.size()) {
        Fail("scene description identity or collection sizes changed");
    }
    for (std::size_t index = 0; index < expected.topology.bodies.size(); ++index) {
        const auto& before = expected.topology.bodies[index];
        const auto& after = actual.topology.bodies[index];
        if (before.name != after.name ||
            before.moves_freely_in_world != after.moves_freely_in_world) {
            Fail("body identity or order changed");
        }
    }
    for (std::size_t index = 0;
         index < expected.topology.wheel_placements.size(); ++index) {
        const auto& before = expected.topology.wheel_placements[index];
        const auto& after = actual.topology.wheel_placements[index];
        if (before.interface_name != after.interface_name ||
            before.wheel_body_name != after.wheel_body_name ||
            before.carrier_body_name != after.carrier_body_name ||
            before.side != after.side ||
            !before.datum_in_wheel_body_frame_meters.isApprox(
                after.datum_in_wheel_body_frame_meters, 1e-15) ||
            !before.spin_axis_in_wheel_body_frame.isApprox(
                after.spin_axis_in_wheel_body_frame, 1e-15) ||
            before.nominal_rolling_radius_meters !=
                after.nominal_rolling_radius_meters ||
            before.spin_joint_name != after.spin_joint_name ||
            before.spin_angle_scalar.has_value() !=
                after.spin_angle_scalar.has_value()) {
            Fail("wheel placement, spin joint or order changed");
        }
        if (before.spin_angle_scalar.has_value() &&
            (before.spin_angle_scalar->name != after.spin_angle_scalar->name ||
             before.spin_angle_scalar->sign != after.spin_angle_scalar->sign)) {
            Fail("wheel spin scalar binding changed");
        }
    }
    for (std::size_t index = 0; index < expected.scalar_definitions.size();
         ++index) {
        const auto& before = expected.scalar_definitions[index];
        const auto& after = actual.scalar_definitions[index];
        if (before.name != after.name || before.unit != after.unit ||
            before.reference_frame != after.reference_frame ||
            before.quantity != after.quantity ||
            before.method != after.method ||
            before.sample_semantics != after.sample_semantics) {
            Fail("scalar definition or order changed");
        }
    }
}

template <typename Operation>
void ExpectRejected(Operation operation, std::string_view reason) {
    try {
        operation();
    } catch (const std::invalid_argument&) {
        return;
    }
    Fail("accepted invalid scene description: " + std::string(reason));
}

std::string Replace(std::string encoded, std::string_view from,
                    std::string_view to) {
    const auto position = encoded.find(from);
    if (position == std::string::npos) {
        Fail("test mutation did not find '" + std::string(from) + "'");
    }
    encoded.replace(position, from.size(), to);
    return encoded;
}

void VerifyRoundTrip() {
    auto description = MakeDescription();
    const std::string encoded = EncodeSceneDescription(description);
    CheckEqual(description, DecodeSceneDescription(encoded));
    if (description.world_frame !=
        "ORVD model world frame; a track scenario's world frame is the track "
        "inertial frame I: +x along increasing station at the line origin, "
        "+y to the right, +z downward") {
        Fail("the default world-frame description is incomplete");
    }
    if (encoded.find("unit quaternion [w, x, y, z] mapping body coordinates "
                     "to world coordinates; a wheel body's orientation "
                     "includes its spin") == std::string::npos) {
        Fail("the orientation convention is incomplete");
    }
    description.nominal_sample_period_seconds.reset();
    description.world_frame = "test model world frame";
    description.topology.wheel_placements[0].spin_angle_scalar->sign = 1.0;
    CheckEqual(description,
               DecodeSceneDescription(EncodeSceneDescription(description)));
    description.topology.wheel_placements.clear();
    description.scalar_definitions.clear();
    CheckEqual(description,
               DecodeSceneDescription(EncodeSceneDescription(description)));

    // This is a trusted internal description: field order and unknown fields
    // do not matter, and a repeated singleton keeps its last value.
    CheckEqual(MakeDescription(), DecodeSceneDescription(
        "{\"producer_note\":{},\"world_frame\":\"ignored earlier value\"," +
        encoded.substr(1)));
}

void VerifyInvalidDescriptions() {
    const auto reject = [](auto change, std::string_view reason) {
        auto description = MakeDescription();
        change(description);
        ExpectRejected([&] { (void)EncodeSceneDescription(description); }, reason);
    };
    reject([](auto& description) {
        description.topology.wheel_placements[1].side = description.topology.wheel_placements[0].side;
    }, "duplicate carrier side");
    reject([](auto& description) { description.world_frame.clear(); },
           "empty world frame");
    reject([](auto& description) { description.topology.bodies.clear(); },
           "empty topology");
    reject([](auto& description) { description.topology.bodies[0].name.clear(); },
           "empty body name");
    reject([](auto& description) {
        description.topology.bodies[1].name = "carrier";
    }, "duplicate body name");
    reject([](auto& description) {
        description.scalar_definitions[1].name = "right_angle";
    }, "duplicate scalar name");
    reject([](auto& description) {
        description.scalar_definitions[0].name.clear();
    }, "empty scalar name");
    reject([](auto& description) {
        description.topology.wheel_placements[1].interface_name =
            "right_contact";
    }, "duplicate wheel interface name");
    reject([](auto& description) {
        description.topology.wheel_placements[0].interface_name.clear();
    }, "empty wheel interface name");
    reject([](auto& description) {
        description.topology.wheel_placements[0].wheel_body_name = "absent";
    }, "unknown wheel body");
    reject([](auto& description) {
        description.topology.wheel_placements[0].carrier_body_name = "absent";
    }, "unknown carrier body");
    reject([](auto& description) {
        description.topology.wheel_placements[0].spin_joint_name = "";
    }, "empty spin joint");
    reject([](auto& description) {
        description.topology.wheel_placements[0].spin_angle_scalar->name =
            "absent";
    }, "unknown spin scalar");
    reject([](auto& description) {
        description.scalar_definitions[0].unit = "degree";
    }, "spin scalar not in radians");
    for (double sign : {0.0, 0.5, 2.0,
                        std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
        reject([sign](auto& description) {
            description.topology.wheel_placements[0].spin_angle_scalar->sign =
                sign;
        }, "spin sign other than +1 or -1");
    }
    for (double period : {0.0, -0.001,
                          std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()}) {
        reject([period](auto& description) {
            description.nominal_sample_period_seconds = period;
        }, "nonpositive or non-finite sample period");
    }
    reject([](auto& description) {
        description.topology.wheel_placements[0]
            .datum_in_wheel_body_frame_meters.x() =
            std::numeric_limits<double>::quiet_NaN();
    }, "non-finite wheel placement");
    reject([](auto& description) {
        description.topology.wheel_placements[0]
            .spin_axis_in_wheel_body_frame *= 2.0;
    }, "non-unit spin axis");
    for (double radius : {0.0, -0.1}) {
        reject([radius](auto& description) {
            description.topology.wheel_placements[0]
                .nominal_rolling_radius_meters = radius;
        }, "nonpositive rolling radius");
    }
}

void VerifyInvalidJson() {
    const std::string encoded = EncodeSceneDescription(MakeDescription());
    const auto reject = [&encoded](std::string_view from, std::string_view to,
                                   std::string_view reason) {
        const auto changed = Replace(encoded, from, to);
        ExpectRejected([&] { (void)DecodeSceneDescription(changed); }, reason);
    };
    ExpectRejected([] { (void)DecodeSceneDescription("{"); }, "malformed JSON");
    ExpectRejected([] { (void)DecodeSceneDescription("[]"); },
                   "non-object description");
    reject("\"length_unit\":\"meter\"", "\"length_unit\":\"mm\"",
           "wrong length unit");
    reject("\"time_unit\":\"second\"", "\"time_unit\":\"ms\"",
           "wrong time unit");
    reject("\"angle_unit\":\"radian\"", "\"angle_unit\":\"degree\"",
           "wrong angle unit");
    reject("unit quaternion [w, x, y, z]", "unit quaternion [x, y, z, w]",
           "wrong quaternion ordering");
    reject("\"0\":\"not_ready\"", "\"0\":\"valid\"",
           "wrong scalar status code");
    reject("\"0\":\"not_ready\"", "\"0\":\"not_ready\",\"3\":\"other\"",
           "extra scalar status code");
    reject("\"2\":\"accepted_endpoint\"", "\"2\":\"unknown\"",
           "wrong phase code");
    reject("\"2\":\"accepted_endpoint\"",
           "\"2\":\"accepted_endpoint\",\"3\":\"other\"", "extra phase code");
    reject("\"nominal_sample_period_seconds\":0.005,", "",
           "missing nominal period field");
    reject("\"nominal_sample_period_seconds\":0.005",
           "\"nominal_sample_period_seconds\":0", "zero nominal period");
    reject("\"nominal_sample_period_seconds\":0.005",
           "\"nominal_sample_period_seconds\":1e309", "non-finite nominal period");
    reject("\"nominal_sample_period_seconds\":0.005",
           "\"nominal_sample_period_seconds\":\"0.005\"",
           "string nominal period");
    reject("\"moves_freely_in_world\":true", "\"moves_freely_in_world\":1",
           "non-boolean body motion flag");
    reject("\"side\":\"right\"", "\"side\":\"middle\"", "unknown wheel side");
    reject("\"spin_joint_name\":\"right_revolute_joint\",", "",
           "missing spin joint field");
    reject("\"spin_joint_name\":\"right_revolute_joint\"",
           "\"spin_joint_name\":3", "non-string spin joint");
    reject("\"spin_angle_scalar\":null,", "", "missing spin scalar field");
    reject("\"sign\":-1.0", "\"sign\":0", "zero spin sign");
    reject("\"name\":\"right_angle\",\"sign\":-1.0",
           "\"name\":\"absent\",\"sign\":-1.0", "unknown spin scalar");
    reject("\"unit\":\"rad\"", "\"unit\":\"degree\"", "wrong spin scalar unit");
    reject("\"wheel_body_name\":\"right_wheel\"",
           "\"wheel_body_name\":\"absent\"", "unknown wheel body reference");
    reject("\"carrier_body_name\":\"carrier\"",
           "\"carrier_body_name\":\"absent\"", "unknown carrier body reference");
    reject("\"spin_axis_in_wheel_body_frame\":[",
           "\"spin_axis_in_wheel_body_frame\":[0,", "wrong spin axis dimension");
    reject("\"name\":\"speed\"", "\"name\":\"right_angle\"",
           "duplicate scalar definition name");
    reject("\"interface_name\":\"left_contact\"",
           "\"interface_name\":\"right_contact\"",
           "duplicate wheel interface name");
    reject("\"name\":\"right_wheel\"", "\"name\":\"carrier\"",
           "duplicate body name");
}

}  // namespace

int main() {
    try {
        VerifyRoundTrip();
        VerifyInvalidDescriptions();
        VerifyInvalidJson();
        std::puts("scene description round trip, references and scalar bindings passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
