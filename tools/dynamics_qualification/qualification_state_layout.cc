#include "qualification_state_layout.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace orvd::dynamics_qualification {
namespace {
using Json = nlohmann::json;

[[noreturn]] void Invalid(const std::string& message) {
    throw std::invalid_argument("qualification state layout: " + message);
}

void Positive(double value, std::string_view field) {
    if (!std::isfinite(value) || value <= 0.0) {
        Invalid(std::string(field) + " must be positive and finite");
    }
}

void Mark(std::vector<bool>& covered, int start, int size) {
    if (start < 0 || size < 0 || static_cast<std::size_t>(start) > covered.size() ||
        static_cast<std::size_t>(size) > covered.size() - static_cast<std::size_t>(start)) {
        Invalid("coordinate binding range is outside the state");
    }
    for (int i = start; i < start + size; ++i) {
        if (covered[static_cast<std::size_t>(i)]) Invalid("overlapping coordinate binding ranges");
        covered[static_cast<std::size_t>(i)] = true;
    }
}

void RequireComplete(const std::vector<bool>& covered) {
    for (bool present : covered) {
        if (!present) Invalid("coordinate binding does not cover the complete state");
    }
}

}  // namespace

Json BuildQualificationStateLayout(const configuration::AssembledVehicleSystem& assembled,
                      const Eigen::Ref<const Eigen::VectorXd>& initial) {
    const auto& system = assembled.system();
    const auto& model = assembled.model();
    const auto& binding = assembled.binding();
    if (initial.size() != system.continuous_state_size() || !initial.allFinite()) {
        Invalid("initial physical state has invalid size or values");
    }
    const auto qr = system.generalized_positions_state_range();
    const auto vr = system.generalized_velocities_state_range();
    const auto zr = system.series_spring_damper_force_state_range();
    const int nq = qr.size();
    const int nz = zr.size();
    if (nq <= 0 || qr.start() != 0 || vr.start() != nq ||
        zr.start() != vr.start() + vr.size() ||
        zr.start() + nz != initial.size()) {
        Invalid("unexpected physical state layout");
    }
    auto context = model.CreateDefaultContext();
    model.SetGeneralizedPositions(context.get(), initial.segment(qr.start(), nq));
    std::vector<bool> q_covered(static_cast<std::size_t>(nq), false);
    std::vector<bool> v_covered(static_cast<std::size_t>(vr.size()), false);
    std::vector<bool> z_covered(static_cast<std::size_t>(nz), false);
    Json blocks = Json::array();
    const auto coordinate_block = [&](const std::string& owner, const char* family,
                                      const char* joint_kind,
                                      int start, int size,
                                      int velocity_start, int velocity_size,
                                      double multiplier, const char* unit) {
        Mark(q_covered, start, size);
        Mark(v_covered, velocity_start, velocity_size);
        blocks.push_back({{"owner", owner}, {"family", family}, {"joint_kind", joint_kind},
                          {"coordinate_start", start}, {"size", size},
                          {"physical_velocity_start", vr.start() + velocity_start},
                          {"physical_velocity_size", velocity_size},
                          {"position_unit", unit},
                          {"coordinate_velocity_unit", std::string(unit) + "/s"},
                          {"coordinate_acceleration_unit", std::string(unit) + "/s^2"},
                          {"scale_multiplier", multiplier}});
    };
    Json norms = Json::array();
    for (const auto& body : binding.free_body_station_offsets) {
        const auto handle = model.GetRigidBodyByName(body.body_name);
        if (!model.IsFreeBody(handle)) Invalid("binding body is not free: " + body.body_name);
        const auto range = model.GetFreeBodyPositionRange(handle);
        const auto velocity = model.GetFreeBodyVelocityRange(handle);
        if (range.size() != 7) Invalid("free-body position range must have seven entries");
        if (velocity.size() != 6) Invalid("free-body velocity range must have six entries");
        const double rho = initial.segment(range.start(), 4).norm();
        Positive(rho, "initial quaternion norm");
        coordinate_block(body.body_name, "quaternion", "free_quaternion", range.start(), 4,
                         velocity.start(), 3, rho, "stored_quaternion");
        blocks.back()["quaternion_order"] = "wxyz";
        blocks.back()["rotation_convention"] = "R_WB";
        blocks.back()["physical_velocity_kind"] = "angular_velocity";
        blocks.back()["physical_velocity_expression_frame"] = "world";
        coordinate_block(body.body_name, "translation", "free_translation", range.start() + 4, 3,
                         velocity.start() + 3, 3, 1.0, "m");
        blocks.back()["position_expression_frame"] = "world";
        blocks.back()["physical_velocity_kind"] = "linear_velocity";
        blocks.back()["physical_velocity_expression_frame"] = "world";
        norms.push_back({{"body_name", body.body_name}, {"coordinate_start", range.start()},
                         {"initial_norm", rho}});
    }
    const auto angle_joint = [&](const std::string& name, int size, const char* joint_kind) {
        const auto joint = model.GetJointByName(name);
        const auto range = model.GetJointPositionRange(joint);
        const auto velocity = model.GetJointVelocityRange(joint);
        if (range.size() != size) Invalid("unexpected angle joint range: " + name);
        if (velocity.size() != size) Invalid("unexpected angle joint velocity range: " + name);
        coordinate_block(name, "angle", joint_kind, range.start(), size,
                         velocity.start(), velocity.size(), 1.0, "rad");
        if (size == 3) {
            blocks.back()["angle_order"] = "roll_pitch_yaw";
            blocks.back()["rotation_convention"] = "R_FM=Rz(yaw)*Ry(pitch)*Rx(roll)";
            blocks.back()["physical_velocity_kind"] = "angular_velocity";
            blocks.back()["physical_velocity_expression_frame"] = "parent_joint_frame";
        } else {
            blocks.back()["angle_convention"] = "child_relative_to_parent_about_declared_joint_axis";
            blocks.back()["physical_velocity_kind"] = "joint_angular_rate";
            blocks.back()["physical_velocity_expression_frame"] = "joint_axis_in_parent_frame";
        }
    };
    for (const auto& name : binding.revolute_joint_names) angle_joint(name, 1, "revolute");
    for (const auto& name : binding.ball_rpy_joint_names) angle_joint(name, 3, "ball_rpy");
    for (const auto& name : binding.series_spring_viscous_damper_names) {
        const auto range = system.series_spring_damper_force_state_range(
            system.GetSeriesSpringViscousDamperIndexByName(name));
        if (range.size() != 1) Invalid("series-force range must have one entry");
        const int start = range.start() - zr.start();
        Mark(z_covered, start, 1);
        blocks.push_back({{"owner", name}, {"family", "force"}, {"joint_kind", "series_force"},
                          {"internal_state_start", start}, {"physical_state_start", range.start()},
                          {"size", 1}, {"unit", "N"}});
    }
    RequireComplete(q_covered);
    RequireComplete(v_covered);
    RequireComplete(z_covered);
    return {{"physical_positions", {{"start", qr.start()}, {"size", nq}}},
            {"physical_velocities", {{"start", vr.start()}, {"size", vr.size()}}},
            {"internal_state", {{"start", zr.start()}, {"size", nz}}},
            {"coordinate_blocks", std::move(blocks)},
            {"initial_quaternion_norms", std::move(norms)}};
}

}  // namespace orvd::dynamics_qualification
