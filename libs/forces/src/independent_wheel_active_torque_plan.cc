#include "orvd/forces/independent_wheel_active_torque_plan.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "body_wrench_pair.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"

namespace orvd::forces {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("independent-wheel active torque plan: " +
                                detail);
}

}  // namespace

IndependentWheelActiveTorquePlan::IndependentWheelActiveTorquePlan(
    const multibody_model::MultibodyModel& model,
    std::vector<IndependentWheelActiveTorqueCoupleDefinition> definitions)
    : model_(&model) {
    if (!model.is_finalized()) {
        throw std::logic_error(
            "IndependentWheelActiveTorquePlan requires a finalized "
            "multibody model");
    }
    if (definitions.empty()) {
        Reject("at least one named torque channel is required");
    }

    std::unordered_set<std::string> channel_names;
    std::unordered_set<std::string> wheel_body_names;
    channel_names.reserve(definitions.size());
    wheel_body_names.reserve(definitions.size());
    bindings_.reserve(definitions.size());
    for (auto& definition : definitions) {
        if (definition.channel_name.empty() ||
            definition.axis_provider_body_name.empty() ||
            definition.wheel_body_name.empty() ||
            definition.reaction_frame_body_name.empty()) {
            Reject("a channel or one of its three body names is empty");
        }
        if (!channel_names.insert(definition.channel_name).second) {
            Reject("more than one channel is named '" +
                   definition.channel_name + "'");
        }
        if (!wheel_body_names.insert(definition.wheel_body_name).second) {
            Reject("more than one channel applies positive torque to body '" +
                   definition.wheel_body_name + "'");
        }
        if (definition.axis_provider_body_name == definition.wheel_body_name ||
            definition.axis_provider_body_name ==
                definition.reaction_frame_body_name ||
            definition.wheel_body_name ==
                definition.reaction_frame_body_name) {
            Reject("channel '" + definition.channel_name +
                   "' does not name three distinct bodies");
        }

        const auto axis_provider =
            model.GetRigidBodyByName(definition.axis_provider_body_name);
        const auto wheel = model.GetRigidBodyByName(definition.wheel_body_name);
        const auto reaction =
            model.GetRigidBodyByName(definition.reaction_frame_body_name);
        bindings_.push_back(Binding{
            std::move(definition.channel_name),
            std::move(definition.axis_provider_body_name),
            std::move(definition.wheel_body_name),
            std::move(definition.reaction_frame_body_name), axis_provider,
            wheel, reaction});
    }
}

const IndependentWheelActiveTorquePlan::Binding&
IndependentWheelActiveTorquePlan::binding(int index) const {
    if (index < 0 || index >= channel_count()) {
        Reject("the requested channel index is out of range");
    }
    return bindings_[static_cast<std::size_t>(index)];
}

std::string_view IndependentWheelActiveTorquePlan::channel_name(
    int index) const {
    return binding(index).channel_name;
}

std::string_view IndependentWheelActiveTorquePlan::axis_provider_body_name(
    int index) const {
    return binding(index).axis_provider_body_name;
}

std::string_view IndependentWheelActiveTorquePlan::wheel_body_name(
    int index) const {
    return binding(index).wheel_body_name;
}

std::string_view IndependentWheelActiveTorquePlan::reaction_frame_body_name(
    int index) const {
    return binding(index).reaction_frame_body_name;
}

void IndependentWheelActiveTorquePlan::CalcAppliedForces(
    const multibody_model::MultibodyEvaluationContext& context,
    std::span<const double> held_wheel_torques_newton_metres,
    std::span<multibody_model::AppliedBodyWrench> body_wrenches) const {
    if (held_wheel_torques_newton_metres.size() != bindings_.size()) {
        Reject("the held-torque span has " +
               std::to_string(held_wheel_torques_newton_metres.size()) +
               " entries, but this plan has " +
               std::to_string(bindings_.size()) +
               " channels; nothing was written");
    }
    if (body_wrenches.size() != 2 * bindings_.size()) {
        Reject("the body-wrench span has " +
               std::to_string(body_wrenches.size()) +
               " entries, but this plan requires " +
               std::to_string(2 * bindings_.size()) +
               "; nothing was written");
    }
    for (double torque : held_wheel_torques_newton_metres) {
        if (!std::isfinite(torque)) {
            Reject("a held wheel torque is not finite; nothing was written");
        }
    }

    // This first query validates the context before any caller-owned output is
    // touched. All other body handles were resolved from the same frozen model.
    (void)model_->CalcPoseInWorld(context,
                                  bindings_.front().axis_provider_body);

    for (std::size_t ordinal = 0; ordinal < bindings_.size(); ++ordinal) {
        const Binding& current = bindings_[ordinal];
        const Eigen::Vector3d axis_in_world =
            model_->CalcPoseInWorld(context, current.axis_provider_body)
                .rotation() *
            Eigen::Vector3d::UnitY();
        const Eigen::Vector3d moment_on_wheel =
            held_wheel_torques_newton_metres[ordinal] * axis_in_world;
        internal::EmitCoupleWrenchPair(
            *model_,
            multibody_model::BodyFixedPoint{current.wheel_body,
                                             Eigen::Vector3d::Zero()},
            multibody_model::BodyFixedPoint{current.reaction_frame_body,
                                             Eigen::Vector3d::Zero()},
            moment_on_wheel, body_wrenches.subspan(2 * ordinal, 2));
    }
}

}  // namespace orvd::forces
