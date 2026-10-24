#include "orvd/scene_observation/scene_description.h"

#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "orvd/scene_observation/scene_frame.h"

namespace orvd::scene_observation {
namespace {

template <typename Code>
std::string CodeKey(Code code) {
    return std::to_string(static_cast<unsigned>(code));
}

const nlohmann::json& ScalarStatusCodes() {
    static const nlohmann::json codes = {
        {CodeKey(ScalarStatus::kNotReady), "not_ready"},
        {CodeKey(ScalarStatus::kValid), "valid"},
        {CodeKey(ScalarStatus::kPlaceholder), "placeholder"}};
    return codes;
}

const nlohmann::json& PhaseCodes() {
    static const nlohmann::json codes = {
        {CodeKey(SamplePhase::kInitialAcceptedState), "initial_accepted_state"},
        {CodeKey(SamplePhase::kDenseIntermediateSample), "dense_intermediate_sample"},
        {CodeKey(SamplePhase::kAcceptedEndpoint), "accepted_endpoint"}};
    return codes;
}

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("scene description: " + detail);
}

void AddUniqueName(std::set<std::string>& names, const std::string& name,
                   const char* kind) {
    if (name.empty()) {
        Reject(std::string(kind) + " name is empty");
    }
    if (!names.insert(name).second) {
        Reject(std::string(kind) + " name '" + name + "' is repeated");
    }
}

void ValidateDescription(const SceneDescription& description) {
    if (description.world_frame.empty()) {
        Reject("world_frame is empty");
    }
    if (description.nominal_sample_period_seconds.has_value() &&
        (!std::isfinite(*description.nominal_sample_period_seconds) ||
         *description.nominal_sample_period_seconds <= 0.0)) {
        Reject("nominal_sample_period_seconds must be finite and positive");
    }
    if (description.topology.bodies.empty()) {
        Reject("the topology names no rigid body");
    }
    std::set<std::string> body_names;
    for (const auto& body : description.topology.bodies) {
        AddUniqueName(body_names, body.name, "body");
    }
    std::set<std::string> scalar_names;
    std::map<std::string, const ScalarDefinition*> scalar_definitions;
    for (const auto& scalar : description.scalar_definitions) {
        AddUniqueName(scalar_names, scalar.name, "scalar");
        scalar_definitions.emplace(scalar.name, &scalar);
    }
    std::set<std::string> interface_names;
    std::set<std::pair<std::string, wheel_rail_contact::WheelSide>> carrier_sides;
    for (const auto& wheel : description.topology.wheel_placements) {
        AddUniqueName(interface_names, wheel.interface_name, "wheel interface");
        if (!body_names.contains(wheel.wheel_body_name)) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' names an unknown wheel body '" + wheel.wheel_body_name +
                   "'");
        }
        if (!body_names.contains(wheel.carrier_body_name)) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' names an unknown carrier body '" +
                   wheel.carrier_body_name + "'");
        }
        if (wheel.side != wheel_rail_contact::WheelSide::kRight &&
            wheel.side != wheel_rail_contact::WheelSide::kLeft) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' has an unknown wheel side");
        }
        if (!carrier_sides.emplace(wheel.carrier_body_name, wheel.side).second) {
            Reject("a carrier has more than one wheel placement on the same side");
        }
        if (!wheel.datum_in_wheel_body_frame_meters.allFinite() ||
            !wheel.spin_axis_in_wheel_body_frame.allFinite() ||
            !std::isfinite(wheel.nominal_rolling_radius_meters)) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' has non-finite placement geometry");
        }
        if (std::abs(wheel.spin_axis_in_wheel_body_frame.squaredNorm() - 1.0) >
            1e-8) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' spin axis must be a unit vector");
        }
        if (wheel.nominal_rolling_radius_meters <= 0.0) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' rolling radius must be positive");
        }
        if (wheel.spin_joint_name.has_value() &&
            wheel.spin_joint_name->empty()) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' has an empty spin joint name");
        }
        if (!wheel.spin_angle_scalar.has_value()) {
            continue;
        }
        const auto& binding = *wheel.spin_angle_scalar;
        const auto scalar = scalar_definitions.find(binding.name);
        if (scalar == scalar_definitions.end()) {
            Reject("wheel interface '" + wheel.interface_name +
                   "' names an unknown spin scalar '" + binding.name + "'");
        }
        if (scalar->second->unit != "rad") {
            Reject("spin scalar '" + binding.name + "' must have unit 'rad'");
        }
        if (binding.sign != 1.0 && binding.sign != -1.0) {
            Reject("spin scalar '" + binding.name + "' sign must be +1 or -1");
        }
    }
}

nlohmann::json Vector3Json(const Eigen::Vector3d& value) {
    return nlohmann::json::array({value.x(), value.y(), value.z()});
}

Eigen::Vector3d ReadVector3(const nlohmann::json& value) {
    if (!value.is_array() || value.size() != 3) {
        Reject("a wheel placement vector must have exactly three entries");
    }
    return Eigen::Vector3d(value[0].get<double>(), value[1].get<double>(),
                           value[2].get<double>());
}

const nlohmann::json& ReadArray(const nlohmann::json& object,
                                const char* name) {
    const auto& value = object.at(name);
    if (!value.is_array()) {
        Reject(std::string(name) + " must be an array");
    }
    return value;
}

void RequireString(const nlohmann::json& object, const char* name,
                   const char* expected) {
    if (object.at(name).get<std::string>() != expected) {
        Reject(std::string(name) + " must be '" + expected + "'");
    }
}

wheel_rail_contact::WheelSide ReadSide(const nlohmann::json& value) {
    const auto name = value.get<std::string>();
    if (name == "right") {
        return wheel_rail_contact::WheelSide::kRight;
    }
    if (name == "left") {
        return wheel_rail_contact::WheelSide::kLeft;
    }
    Reject("unknown wheel side '" + name + "'");
}

}  // namespace

std::string EncodeSceneDescription(const SceneDescription& description) {
    ValidateDescription(description);
    nlohmann::json bodies = nlohmann::json::array();
    for (const auto& body : description.topology.bodies) {
        bodies.push_back({{"name", body.name},
                          {"moves_freely_in_world", body.moves_freely_in_world}});
    }
    nlohmann::json wheels = nlohmann::json::array();
    for (const auto& wheel : description.topology.wheel_placements) {
        wheels.push_back(
            {{"interface_name", wheel.interface_name},
             {"wheel_body_name", wheel.wheel_body_name},
             {"carrier_body_name", wheel.carrier_body_name},
             {"side", wheel.side == wheel_rail_contact::WheelSide::kRight
                          ? "right"
                          : "left"},
             {"datum_in_wheel_body_frame_meters",
              Vector3Json(wheel.datum_in_wheel_body_frame_meters)},
             {"spin_axis_in_wheel_body_frame",
              Vector3Json(wheel.spin_axis_in_wheel_body_frame)},
             {"nominal_rolling_radius_meters",
              wheel.nominal_rolling_radius_meters},
             {"spin_joint_name", wheel.spin_joint_name.has_value()
                                     ? nlohmann::json(*wheel.spin_joint_name)
                                     : nlohmann::json(nullptr)},
             {"spin_angle_scalar",
              wheel.spin_angle_scalar.has_value()
                  ? nlohmann::json{{"name", wheel.spin_angle_scalar->name},
                                   {"sign", wheel.spin_angle_scalar->sign}}
                  : nlohmann::json(nullptr)}});
    }
    nlohmann::json scalars = nlohmann::json::array();
    for (const auto& scalar : description.scalar_definitions) {
        scalars.push_back({{"name", scalar.name},
                           {"unit", scalar.unit},
                           {"reference_frame", scalar.reference_frame},
                           {"quantity", scalar.quantity},
                           {"method", scalar.method},
                           {"sample_semantics", scalar.sample_semantics}});
    }
    const nlohmann::json encoded = {
        {"world_frame", description.world_frame},
        {"length_unit", kSceneLengthUnit},
        {"time_unit", kSceneTimeUnit},
        {"angle_unit", kSceneAngleUnit},
        {"orientation", kSceneOrientation},
        {"bodies", std::move(bodies)},
        {"wheel_placements", std::move(wheels)},
        {"scalars", std::move(scalars)},
        {"scalar_status_codes", ScalarStatusCodes()},
        {"phase_codes", PhaseCodes()},
        {"nominal_sample_period_seconds",
         description.nominal_sample_period_seconds.has_value()
             ? nlohmann::json(*description.nominal_sample_period_seconds)
             : nlohmann::json(nullptr)}};
    try {
        return encoded.dump();
    } catch (const nlohmann::json::exception& error) {
        Reject(error.what());
    }
}

SceneDescription DecodeSceneDescription(std::string_view json) {
    try {
        const auto encoded = nlohmann::json::parse(json);
        RequireString(encoded, "length_unit", kSceneLengthUnit);
        RequireString(encoded, "time_unit", kSceneTimeUnit);
        RequireString(encoded, "angle_unit", kSceneAngleUnit);
        RequireString(encoded, "orientation", kSceneOrientation);
        const auto& statuses = encoded.at("scalar_status_codes");
        if (statuses != ScalarStatusCodes()) {
            Reject("scalar_status_codes disagrees with the public scene codes");
        }
        const auto& phases = encoded.at("phase_codes");
        if (phases != PhaseCodes()) {
            Reject("phase_codes disagrees with the public scene codes");
        }

        SceneDescription description;
        description.world_frame = encoded.at("world_frame").get<std::string>();
        const auto& period = encoded.at("nominal_sample_period_seconds");
        if (!period.is_null()) {
            description.nominal_sample_period_seconds = period.get<double>();
        }
        for (const auto& body : ReadArray(encoded, "bodies")) {
            description.topology.bodies.push_back(SceneBody{
                body.at("name").get<std::string>(),
                body.at("moves_freely_in_world").get<bool>()});
        }
        for (const auto& wheel : ReadArray(encoded, "wheel_placements")) {
            SceneWheelPlacement placement;
            placement.interface_name =
                wheel.at("interface_name").get<std::string>();
            placement.wheel_body_name =
                wheel.at("wheel_body_name").get<std::string>();
            placement.carrier_body_name =
                wheel.at("carrier_body_name").get<std::string>();
            placement.side = ReadSide(wheel.at("side"));
            placement.datum_in_wheel_body_frame_meters =
                ReadVector3(wheel.at("datum_in_wheel_body_frame_meters"));
            placement.spin_axis_in_wheel_body_frame =
                ReadVector3(wheel.at("spin_axis_in_wheel_body_frame"));
            placement.nominal_rolling_radius_meters =
                wheel.at("nominal_rolling_radius_meters").get<double>();
            const auto& joint = wheel.at("spin_joint_name");
            if (!joint.is_null()) {
                placement.spin_joint_name = joint.get<std::string>();
            }
            const auto& binding = wheel.at("spin_angle_scalar");
            if (!binding.is_null()) {
                placement.spin_angle_scalar = WheelSpinScalarBinding{
                    binding.at("name").get<std::string>(),
                    binding.at("sign").get<double>()};
            }
            description.topology.wheel_placements.push_back(
                std::move(placement));
        }
        for (const auto& scalar : ReadArray(encoded, "scalars")) {
            description.scalar_definitions.push_back(ScalarDefinition{
                scalar.at("name").get<std::string>(),
                scalar.at("unit").get<std::string>(),
                scalar.at("reference_frame").get<std::string>(),
                scalar.at("quantity").get<std::string>(),
                scalar.at("method").get<std::string>(),
                scalar.at("sample_semantics").get<std::string>()});
        }
        ValidateDescription(description);
        return description;
    } catch (const nlohmann::json::exception& error) {
        Reject(error.what());
    }
}

}  // namespace orvd::scene_observation
