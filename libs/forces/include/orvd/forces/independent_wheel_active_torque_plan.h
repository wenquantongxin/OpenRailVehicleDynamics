#pragma once

/// @file
/// Named pure-torque couples for independently rotating wheels.

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model.h"

namespace orvd::forces {

/// One held wheel-torque channel.
///
/// The axle bridge supplies only the torque direction: its body-frame +Y axis.
/// Positive torque acts on the independent wheel and the equal reaction acts on
/// the named bogie frame.  The axle bridge itself receives no reaction wrench.
struct IndependentWheelActiveTorqueCoupleDefinition {
    std::string channel_name;
    std::string axis_provider_body_name;
    std::string wheel_body_name;
    std::string reaction_frame_body_name;
};

/// A frozen, model-bound set of independently rotating wheel torque couples.
///
/// Construction resolves every body name.  Evaluation performs no name lookup,
/// allocation, limiting or control update: it converts the already-conditioned
/// held torque of each channel into one wheel wrench and one frame wrench.
class IndependentWheelActiveTorquePlan {
   public:
    IndependentWheelActiveTorquePlan(
        const multibody_model::MultibodyModel& model,
        std::vector<IndependentWheelActiveTorqueCoupleDefinition> definitions);
    IndependentWheelActiveTorquePlan(
        multibody_model::MultibodyModel&&,
        std::vector<IndependentWheelActiveTorqueCoupleDefinition>) = delete;
    IndependentWheelActiveTorquePlan(
        const multibody_model::MultibodyModel&&,
        std::vector<IndependentWheelActiveTorqueCoupleDefinition>) = delete;

    IndependentWheelActiveTorquePlan(
        const IndependentWheelActiveTorquePlan&) = delete;
    IndependentWheelActiveTorquePlan& operator=(
        const IndependentWheelActiveTorquePlan&) = delete;
    IndependentWheelActiveTorquePlan(IndependentWheelActiveTorquePlan&&) =
        delete;
    IndependentWheelActiveTorquePlan& operator=(
        IndependentWheelActiveTorquePlan&&) = delete;

    [[nodiscard]] const multibody_model::MultibodyModel& model() const {
        return *model_;
    }
    [[nodiscard]] int channel_count() const {
        return static_cast<int>(bindings_.size());
    }
    [[nodiscard]] int body_wrench_count() const { return 2 * channel_count(); }

    /// Setup-time identity accessors. Evaluation remains ordinal-only.
    [[nodiscard]] std::string_view channel_name(int index) const;
    [[nodiscard]] std::string_view axis_provider_body_name(int index) const;
    [[nodiscard]] std::string_view wheel_body_name(int index) const;
    [[nodiscard]] std::string_view reaction_frame_body_name(int index) const;

    /// Writes two pure-moment body wrenches per channel.
    ///
    /// Every size and finiteness check precedes every output write.  A positive
    /// scalar produces +tau about the provider body's world-expressed +Y axis
    /// on the wheel and -tau about that same axis on the reaction frame.
    void CalcAppliedForces(
        const multibody_model::MultibodyEvaluationContext& context,
        std::span<const double> held_wheel_torques_newton_metres,
        std::span<multibody_model::AppliedBodyWrench> body_wrenches) const;

   private:
    struct Binding {
        std::string channel_name;
        std::string axis_provider_body_name;
        std::string wheel_body_name;
        std::string reaction_frame_body_name;
        multibody_model::RigidBodyHandle axis_provider_body;
        multibody_model::RigidBodyHandle wheel_body;
        multibody_model::RigidBodyHandle reaction_frame_body;
    };

    [[nodiscard]] const Binding& binding(int index) const;

    const multibody_model::MultibodyModel* model_;
    std::vector<Binding> bindings_;
};

}  // namespace orvd::forces
