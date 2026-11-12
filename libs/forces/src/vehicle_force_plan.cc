#include "orvd/forces/vehicle_force_plan.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "body_wrench_pair.h"
#include "orvd/multibody_model/multibody_frame_spatial_velocity.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"

namespace orvd::forces {
namespace {

using multibody_model::AppliedBodyWrench;
using multibody_model::BodyFixedPoint;
using multibody_model::FrameHandle;
using multibody_model::MultibodyEvaluationContext;
using multibody_model::MultibodyModel;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("vehicle force plan: " + detail);
}

int AxisIndex(ForceElementAxis axis) {
    switch (axis) {
        case ForceElementAxis::kLongitudinal:
            return 0;
        case ForceElementAxis::kLateral:
            return 1;
        case ForceElementAxis::kVertical:
            return 2;
    }
    Reject("an element states an axis this library does not know");
}

// The relative motion every family reads: where the opposite frame's origin sits
// relative to the reference frame's origin, how the opposite frame is turned
// relative to it, and how fast both are changing — all expressed in the
// reference frame, because that is the frame the constitutive constants are
// stated in.
struct RelativeMotion {
    BodyFixedPoint reference_point;
    BodyFixedPoint opposite_point;
    Eigen::Vector3d displacement_in_reference_frame_meters;
    Eigen::Matrix3d rotation_of_opposite_in_reference;
    Eigen::Vector3d velocity_in_reference_frame_meters_per_second;
    Eigen::Vector3d angular_velocity_in_reference_frame_radians_per_second;
    // The same displacement in the world, needed for the support moment, which
    // is a statement about where the two application points are.
    Eigen::Vector3d displacement_in_world_meters;
    Eigen::Matrix3d rotation_of_reference_in_world;
};

RelativeMotion CalcRelativeMotion(const MultibodyModel& model,
                                  const MultibodyEvaluationContext& context,
                                  const ForceElementEnd& reference,
                                  const ForceElementEnd& opposite) {
    const auto reference_pose =
        model.CalcPoseInWorld(context, reference.frame);
    const auto opposite_pose = model.CalcPoseInWorld(context, opposite.frame);
    const auto relative_velocity =
        model.CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
            context, opposite.frame, reference.frame, reference.frame);

    RelativeMotion motion;
    motion.reference_point =
        model.CalcFrameOriginAsBodyFixedPoint(context, reference.frame);
    motion.opposite_point =
        model.CalcFrameOriginAsBodyFixedPoint(context, opposite.frame);
    motion.rotation_of_reference_in_world = reference_pose.rotation();
    motion.displacement_in_world_meters =
        opposite_pose.translation() - reference_pose.translation();
    motion.displacement_in_reference_frame_meters =
        reference_pose.rotation().transpose() *
        motion.displacement_in_world_meters;
    motion.rotation_of_opposite_in_reference =
        reference_pose.rotation().transpose() * opposite_pose.rotation();
    motion.velocity_in_reference_frame_meters_per_second =
        relative_velocity.translational_velocity_at_frame_origin_meters_per_second();
    motion.angular_velocity_in_reference_frame_radians_per_second =
        relative_velocity.angular_velocity_radians_per_second();
    return motion;
}

// The one place the full wrench of a two-point element is written.
//
// The forces act at the two frame origins, which are body-fixed points, and are
// equal and opposite. That pair alone is not balanced: a constitutive force that
// is not parallel to the line joining the two points leaves a residual couple
// (p_reference - p_opposite) x force. The reference end therefore also carries
//
//     support moment = (p_opposite - p_reference) x force
//
// which cancels it exactly, so the pair contributes no net force and no net
// moment about any point. Moving both forces to a midpoint would also balance,
// and would be wrong for a different reason: the two bodies would then receive
// forces at points that are not the ones the element attaches to, and the
// endpoint virtual power would no longer equal the constitutive power.
void EmitTranslationalWrenchPair(
    const MultibodyModel& model, const BodyFixedPoint& reference,
    const BodyFixedPoint& opposite,
    const Eigen::Vector3d& force_on_reference_in_world,
    const Eigen::Vector3d& displacement_in_world,
    std::span<AppliedBodyWrench> destination) {
    const FrameHandle world = model.world_frame();
    destination[0] = AppliedBodyWrench{
        reference.body, reference.position_in_body_frame_meters, world,
        displacement_in_world.cross(force_on_reference_in_world),
        force_on_reference_in_world};
    destination[1] = AppliedBodyWrench{
        opposite.body, opposite.position_in_body_frame_meters, world,
        Eigen::Vector3d::Zero(), -force_on_reference_in_world};
}

void RequireDistinctLiveEnds(const MultibodyModel& model,
                             const MultibodyEvaluationContext& context,
                             const ForceElementEnd& reference,
                             const ForceElementEnd& opposite,
                             const std::string& what) {
    if (reference.frame == opposite.frame) {
        Reject(what + " names one frame as both of its ends");
    }
    const BodyFixedPoint reference_point =
        model.CalcFrameOriginAsBodyFixedPoint(context, reference.frame);
    const BodyFixedPoint opposite_point =
        model.CalcFrameOriginAsBodyFixedPoint(context, opposite.frame);
    if (reference_point.body == opposite_point.body) {
        Reject(what +
               " has both ends on one body, so it could exert no relative "
               "force");
    }
}

void RequireFinite(const Eigen::Vector3d& value, const std::string& what) {
    if (!value.allFinite()) {
        Reject(what + " is not finite");
    }
}

void RequirePassive(const Eigen::Vector3d& value, const std::string& what) {
    RequireFinite(value, what);
    if ((value.array() < 0.0).any()) {
        Reject(what + " has a negative component");
    }
}

void RequireUniqueName(const std::string& name, const std::string& what,
                       std::unordered_set<std::string>* names) {
    if (name.empty()) {
        Reject(what + " needs a non-empty name");
    }
    if (!names->insert(name).second) {
        Reject("more than one force element is named '" + name + "'");
    }
}

std::string ElementDescription(std::string_view family, std::size_t index,
                               const std::string& name) {
    if (name.empty()) {
        return std::string(family) + " at index " + std::to_string(index);
    }
    return std::string(family) + " '" + name + "'";
}

void ValidateSaturatedDamperCurve(
    const SaturatedPiecewiseLinearDamper& element, const std::string& what) {
    if (element.curve.size() < 2) {
        Reject(what + " needs at least two curve points");
    }
    if (element.curve.front().relative_velocity_meters_per_second != 0.0 ||
        element.curve.front().force_newtons != 0.0) {
        Reject(what +
               " must start at the origin; the curve states the non-negative "
               "half and the other half is its odd reflection");
    }
    for (std::size_t point = 0; point < element.curve.size(); ++point) {
        const auto& current = element.curve[point];
        if (!std::isfinite(current.relative_velocity_meters_per_second) ||
            !std::isfinite(current.force_newtons)) {
            Reject(what + " has a curve point that is not finite");
        }
        if (current.force_newtons < 0.0) {
            Reject(what +
                   " has a negative force on its non-negative velocity half, "
                   "which would create rather than dissipate energy");
        }
        if (point == 0) {
            continue;
        }
        const auto& previous = element.curve[point - 1];
        if (!(current.relative_velocity_meters_per_second >
              previous.relative_velocity_meters_per_second)) {
            Reject(what +
                   " has curve points that do not ascend strictly in "
                   "velocity");
        }
    }
}

double EvaluateValidatedSaturatedDamperCurve(
    const SaturatedPiecewiseLinearDamper& damper,
    double relative_velocity_meters_per_second) {
    const double sign = relative_velocity_meters_per_second < 0.0 ? -1.0 : 1.0;
    const double speed = std::abs(relative_velocity_meters_per_second);
    const auto& curve = damper.curve;
    if (speed >= curve.back().relative_velocity_meters_per_second) {
        return sign * curve.back().force_newtons;
    }
    for (std::size_t point = 1; point < curve.size(); ++point) {
        const auto& upper = curve[point];
        if (speed <= upper.relative_velocity_meters_per_second) {
            const auto& lower = curve[point - 1];
            const double span = upper.relative_velocity_meters_per_second -
                                lower.relative_velocity_meters_per_second;
            const double fraction =
                (speed - lower.relative_velocity_meters_per_second) / span;
            return sign * (lower.force_newtons +
                           fraction * (upper.force_newtons -
                                       lower.force_newtons));
        }
    }
    return sign * curve.back().force_newtons;
}

double InverseBilinearDamping(
    const PointToPointSeriesSpringBilinearDamper& element, double force) {
    const double magnitude = std::abs(force);
    if (magnitude <= element.knee_force_newtons) {
        return force / element.damping_below_knee_newton_seconds_per_meter;
    }
    return std::copysign(
        element.knee_force_newtons /
                element.damping_below_knee_newton_seconds_per_meter +
            (magnitude - element.knee_force_newtons) /
                element.damping_above_knee_newton_seconds_per_meter,
        force);
}

struct PointToPointMotion {
    double length_meters;
    double length_rate_meters_per_second;
};

PointToPointMotion CalcPointToPointMotion(const RelativeMotion& motion, std::string_view name) {
    const double length = motion.displacement_in_world_meters.stableNorm();
    if (!(length > 0.0) || !std::isfinite(length)) {
        throw std::runtime_error("point-to-point element '" + std::string(name) +
                                 "' has zero or non-finite attachment separation");
    }
    const double rate = (motion.displacement_in_reference_frame_meters / length)
                            .dot(motion.velocity_in_reference_frame_meters_per_second);
    return {length, rate};
}

PointToPointLinearSpringDamperObservation EvaluateLinearPointToPoint(
    const PointToPointLinearSpringDamper& element, const RelativeMotion& motion) {
    const auto [length, rate] = CalcPointToPointMotion(motion, element.name);
    const double spring =
        element.stiffness_newtons_per_meter * (length - element.nominal_length_meters);
    const double damping = element.damping_newton_seconds_per_meter * rate;
    return {length, rate, spring, damping, spring + damping};
}

PointToPointHystereticSpringObservation EvaluateHystereticPointToPoint(
    const PointToPointHystereticSpring& element, const RelativeMotion& motion) {
    const auto [length, rate] = CalcPointToPointMotion(motion, element.name);
    const double spring =
        element.stiffness_newtons_per_meter * (length - element.nominal_length_meters);
    const double hysteresis =
        element.hysteresis_ratio * std::abs(spring) *
        std::clamp(rate / element.hysteresis_transition_speed_meters_per_second, -1.0, 1.0);
    return {length, rate, spring, hysteresis, spring + hysteresis};
}

}  // namespace

VehicleForcePlan::VehicleForcePlan(
    const MultibodyModel& model, VehicleForceElementCollection elements)
    : model_(&model), elements_(std::move(elements)) {
    if (!model.is_finalized()) {
        throw std::logic_error(
            "VehicleForcePlan requires a finalized multibody model");
    }
    const auto validation_context = model.CreateDefaultContext();
    std::unordered_set<std::string> names;
    for (std::size_t index = 0;
         index < elements_.translational_spring_dampers.size(); ++index) {
        const auto& element = elements_.translational_spring_dampers[index];
        const std::string what = ElementDescription(
            "translational spring-damper", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        RequirePassive(element.stiffness_newtons_per_meter,
                       what + " stiffness");
        RequirePassive(element.damping_newton_seconds_per_meter,
                       what + " damping");
    }
    for (std::size_t index = 0;
         index < elements_.roll_spring_damper_couples.size(); ++index) {
        const auto& element = elements_.roll_spring_damper_couples[index];
        const std::string what = ElementDescription(
            "roll spring-damper couple", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        if (!std::isfinite(element.stiffness_newton_meters_per_radian) ||
            element.stiffness_newton_meters_per_radian < 0.0 ||
            !std::isfinite(element.damping_newton_meter_seconds_per_radian) ||
            element.damping_newton_meter_seconds_per_radian < 0.0) {
            Reject(what +
                   " needs finite non-negative stiffness and damping");
        }
    }
    for (std::size_t index = 0;
         index < elements_.series_spring_viscous_dampers.size(); ++index) {
        const auto& element = elements_.series_spring_viscous_dampers[index];
        const std::string what = ElementDescription(
            "series spring-viscous damper", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        (void)AxisIndex(element.axis);
        // Either constant at zero leaves the constitutive law without a time
        // constant: the element would be algebraic, and belongs to a different
        // family and to no state component.
        if (!(element.series_stiffness_newtons_per_meter > 0.0) ||
            !std::isfinite(element.series_stiffness_newtons_per_meter) ||
            !(element.series_damping_newton_seconds_per_meter > 0.0) ||
            !std::isfinite(element.series_damping_newton_seconds_per_meter)) {
            Reject(what +
                   " needs a finite positive series stiffness and damping; with "
                   "either at zero it is an algebraic element and carries no "
                   "state");
        }
        if (!std::isfinite(element.series_stiffness_newtons_per_meter /
                           element.series_damping_newton_seconds_per_meter)) {
            Reject(what +
                   " has stiffness divided by damping outside the finite "
                   "binary64 range");
        }
    }
    for (std::size_t index = 0;
         index < elements_.point_to_point_series_spring_bilinear_dampers.size();
         ++index) {
        const auto& element =
            elements_.point_to_point_series_spring_bilinear_dampers[index];
        const std::string what = ElementDescription(
            "point-to-point series spring-bilinear damper", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        const double stiffness = element.series_stiffness_newtons_per_meter;
        const double inner_damping =
            element.damping_below_knee_newton_seconds_per_meter;
        const double knee_force = element.knee_force_newtons;
        const double outer_damping =
            element.damping_above_knee_newton_seconds_per_meter;
        for (const double value :
             {stiffness, inner_damping, knee_force, outer_damping}) {
            if (!(value > 0.0) || !std::isfinite(value)) {
                Reject(what + " needs four finite positive constants");
            }
        }
        if (!std::isfinite(stiffness / inner_damping) ||
            !std::isfinite(stiffness / outer_damping) ||
            !std::isfinite(knee_force / inner_damping)) {
            Reject(what +
                   " has a stiffness-to-damping ratio or knee velocity outside "
                   "the finite binary64 range");
        }
    }
    for (std::size_t index = 0; index < elements_.point_to_point_linear_spring_dampers.size();
         ++index) {
        const auto& element = elements_.point_to_point_linear_spring_dampers[index];
        const std::string what =
            ElementDescription("point-to-point linear spring-damper", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        if (!(element.nominal_length_meters > 0.0) ||
            !std::isfinite(element.nominal_length_meters) ||
            !std::isfinite(element.stiffness_newtons_per_meter) ||
            element.stiffness_newtons_per_meter < 0.0 ||
            !std::isfinite(element.damping_newton_seconds_per_meter) ||
            element.damping_newton_seconds_per_meter < 0.0) {
            Reject(what +
                   " needs finite positive nominal length and finite "
                   "non-negative stiffness and damping");
        }
    }
    for (std::size_t index = 0; index < elements_.point_to_point_hysteretic_springs.size();
         ++index) {
        const auto& element = elements_.point_to_point_hysteretic_springs[index];
        const std::string what =
            ElementDescription("point-to-point hysteretic spring", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        if (!(element.nominal_length_meters > 0.0) ||
            !std::isfinite(element.nominal_length_meters) ||
            !std::isfinite(element.stiffness_newtons_per_meter) ||
            element.stiffness_newtons_per_meter < 0.0 || !std::isfinite(element.hysteresis_ratio) ||
            element.hysteresis_ratio < 0.0 || element.hysteresis_ratio > 1.0 ||
            !(element.hysteresis_transition_speed_meters_per_second > 0.0) ||
            !std::isfinite(element.hysteresis_transition_speed_meters_per_second)) {
            Reject(what +
                   " needs finite positive nominal length and transition speed, "
                   "non-negative stiffness and hysteresis ratio in [0, 1]");
        }
    }
    for (std::size_t index = 0;
         index < elements_.saturated_piecewise_linear_dampers.size();
         ++index) {
        const auto& element =
            elements_.saturated_piecewise_linear_dampers[index];
        const std::string what = ElementDescription(
            "saturated piecewise linear damper", index, element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context, element.reference_end,
                                element.opposite_end, what);
        (void)AxisIndex(element.axis);
        ValidateSaturatedDamperCurve(element, what);
    }
    for (std::size_t index = 0;
         index < elements_.reference_frame_linear_bushings.size();
         ++index) {
        const auto& element =
            elements_.reference_frame_linear_bushings[index];
        const std::string what = ElementDescription(
            "reference-frame linear bushing", index,
            element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context,
                                element.reference_end, element.opposite_end,
                                what);
        RequirePassive(
            element.rotational_damping_newton_meter_seconds_per_radian,
            what + " rotational damping");
        RequirePassive(element.translational_stiffness_newtons_per_meter,
                       what + " translational stiffness");
        RequirePassive(element.translational_damping_newton_seconds_per_meter,
                       what + " translational damping");
    }
}

int VehicleForcePlan::FindTranslationalSpringDamperOrdinal(
    std::string_view name) const {
    for (std::size_t index = 0;
         index < elements_.translational_spring_dampers.size(); ++index) {
        if (elements_.translational_spring_dampers[index].name == name) {
            return static_cast<int>(index);
        }
    }
    Reject("no translational spring-damper is named '" + std::string(name) +
           "'");
}

int VehicleForcePlan::FindSeriesSpringViscousDamperOrdinal(
    std::string_view name) const {
    for (std::size_t index = 0;
         index < elements_.series_spring_viscous_dampers.size(); ++index) {
        if (elements_.series_spring_viscous_dampers[index].name == name) {
            return static_cast<int>(index);
        }
    }
    Reject("no series spring-viscous damper is named '" + std::string(name) +
           "'");
}

int VehicleForcePlan::FindPointToPointSeriesSpringBilinearDamperOrdinal(
    std::string_view name) const {
    for (std::size_t index = 0;
         index < elements_.point_to_point_series_spring_bilinear_dampers.size();
         ++index) {
        if (elements_.point_to_point_series_spring_bilinear_dampers[index].name ==
            name) {
            return static_cast<int>(index);
        }
    }
    Reject("no point-to-point series spring-bilinear damper is named '" +
           std::string(name) + "'");
}

double VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(
    const SaturatedPiecewiseLinearDamper& damper,
    double relative_velocity_meters_per_second) {
    const std::string what = ElementDescription(
        "saturated piecewise linear damper", 0, damper.name);
    ValidateSaturatedDamperCurve(damper, what);
    if (!std::isfinite(relative_velocity_meters_per_second)) {
        Reject(what + " was queried at a non-finite relative velocity");
    }
    return EvaluateValidatedSaturatedDamperCurve(
        damper, relative_velocity_meters_per_second);
}

std::string_view VehicleForcePlan::point_to_point_linear_spring_damper_name(int ordinal) const {
    if (ordinal < 0 || ordinal >= point_to_point_linear_spring_damper_count()) {
        throw std::out_of_range("point-to-point linear spring-damper ordinal");
    }
    return elements_.point_to_point_linear_spring_dampers[static_cast<std::size_t>(ordinal)].name;
}

void VehicleForcePlan::CalcPointToPointLinearSpringDamperObservations(
    const MultibodyEvaluationContext& context,
    std::span<PointToPointLinearSpringDamperObservation> observations) const {
    if (observations.size() != elements_.point_to_point_linear_spring_dampers.size()) {
        Reject("the point-to-point linear observation output has the wrong length");
    }
    for (std::size_t index = 0; index < observations.size(); ++index) {
        const auto& element = elements_.point_to_point_linear_spring_dampers[index];
        const RelativeMotion motion =
            CalcRelativeMotion(*model_, context, element.reference_end, element.opposite_end);
        observations[index] = EvaluateLinearPointToPoint(element, motion);
    }
}

std::string_view VehicleForcePlan::point_to_point_hysteretic_spring_name(int ordinal) const {
    if (ordinal < 0 || ordinal >= point_to_point_hysteretic_spring_count()) {
        throw std::out_of_range("point-to-point hysteretic spring ordinal");
    }
    return elements_.point_to_point_hysteretic_springs[static_cast<std::size_t>(ordinal)].name;
}

void VehicleForcePlan::CalcPointToPointHystereticSpringObservations(
    const MultibodyEvaluationContext& context,
    std::span<PointToPointHystereticSpringObservation> observations) const {
    if (observations.size() != elements_.point_to_point_hysteretic_springs.size()) {
        Reject("the point-to-point hysteretic observation output has the wrong length");
    }
    for (std::size_t index = 0; index < observations.size(); ++index) {
        const auto& element = elements_.point_to_point_hysteretic_springs[index];
        const RelativeMotion motion =
            CalcRelativeMotion(*model_, context, element.reference_end, element.opposite_end);
        observations[index] = EvaluateHystereticPointToPoint(element, motion);
    }
}

void VehicleForcePlan::CalcAppliedForces(
    const MultibodyEvaluationContext& context,
    const Eigen::Ref<const Eigen::VectorXd>& series_forces,
    const Eigen::Ref<const Eigen::VectorXd>& nominal_forces,
    std::span<AppliedBodyWrench> body_wrenches,
    Eigen::Ref<Eigen::VectorXd> series_force_derivatives) const {
    if (series_forces.size() != series_spring_damper_force_state_count()) {
        Reject("the series force state has the wrong length; nothing was "
               "written");
    }
    if (nominal_forces.size() != nominal_force_component_count()) {
        Reject("the nominal force vector has the wrong length; nothing was "
               "written");
    }
    if (static_cast<int>(body_wrenches.size()) != body_wrench_count()) {
        Reject("the wrench output has the wrong length; nothing was written");
    }
    if (series_force_derivatives.size() !=
        series_spring_damper_force_state_count()) {
        Reject("the state derivative output has the wrong length; nothing was "
               "written");
    }

    std::size_t slot = 0;
    for (std::size_t index = 0;
         index < elements_.translational_spring_dampers.size(); ++index) {
        const auto& element = elements_.translational_spring_dampers[index];
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end, element.opposite_end);
        // The opposite end moving away from the reference end in the reference
        // frame's own axes stretches the element, and the force on the
        // reference end follows it: the element pulls its reference end toward
        // the opposite one.
        const Eigen::Vector3d force_in_reference =
            element.stiffness_newtons_per_meter.cwiseProduct(
                motion.displacement_in_reference_frame_meters) +
            element.damping_newton_seconds_per_meter.cwiseProduct(
                motion.velocity_in_reference_frame_meters_per_second) +
            nominal_forces.segment<3>(3 * static_cast<Eigen::Index>(index));
        EmitTranslationalWrenchPair(
            *model_, motion.reference_point, motion.opposite_point,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (const auto& element : elements_.roll_spring_damper_couples) {
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end, element.opposite_end);
        // The linear roll coordinate is the (2,1) entry of the relative
        // rotation in Eigen's zero-based indexing — the third row, second
        // column — which is sin(roll) for a pure roll and equals the roll angle
        // to first order.
        const double roll = motion.rotation_of_opposite_in_reference(2, 1);
        const double roll_rate =
            motion.angular_velocity_in_reference_frame_radians_per_second.x();
        const double moment =
            element.stiffness_newton_meters_per_radian * roll +
            element.damping_newton_meter_seconds_per_radian * roll_rate;
        // A moment about the reference frame's own first axis, taken to the
        // world. It is already a moment in physical space; no attitude map is
        // involved.
        const Eigen::Vector3d moment_in_world =
            motion.rotation_of_reference_in_world * Eigen::Vector3d(moment, 0.0, 0.0);
        internal::EmitCoupleWrenchPair(
            *model_, motion.reference_point, motion.opposite_point,
            moment_in_world, body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (std::size_t index = 0;
         index < elements_.series_spring_viscous_dampers.size(); ++index) {
        const auto& element = elements_.series_spring_viscous_dampers[index];
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end, element.opposite_end);
        const int axis = AxisIndex(element.axis);
        const double relative_velocity =
            motion.velocity_in_reference_frame_meters_per_second[axis];
        const double force = series_forces[static_cast<Eigen::Index>(index)];
        // dF/dt = K v - (K/C) F. The state is the force itself, because the
        // spring and the damper share it while their deflections differ.
        series_force_derivatives[static_cast<Eigen::Index>(index)] =
            element.series_stiffness_newtons_per_meter * relative_velocity -
            (element.series_stiffness_newtons_per_meter /
             element.series_damping_newton_seconds_per_meter) *
                force;
        Eigen::Vector3d force_in_reference = Eigen::Vector3d::Zero();
        force_in_reference[axis] = force;
        EmitTranslationalWrenchPair(
            *model_, motion.reference_point, motion.opposite_point,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (std::size_t index = 0;
         index < elements_.point_to_point_series_spring_bilinear_dampers.size();
         ++index) {
        const auto& element =
            elements_.point_to_point_series_spring_bilinear_dampers[index];
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end, element.opposite_end);
        const double length = motion.displacement_in_world_meters.stableNorm();
        if (!(length > 0.0) || !std::isfinite(length)) {
            throw std::runtime_error(
                "point-to-point series spring-bilinear damper '" +
                element.name + "' has zero or non-finite attachment separation");
        }
        const Eigen::Vector3d direction_in_world =
            motion.displacement_in_world_meters / length;
        const double length_rate =
            (motion.displacement_in_reference_frame_meters / length)
                .dot(motion.velocity_in_reference_frame_meters_per_second);
        const Eigen::Index state_index =
            static_cast<Eigen::Index>(series_spring_viscous_damper_count()) +
            static_cast<Eigen::Index>(index);
        const double force = series_forces[state_index];
        series_force_derivatives[state_index] =
            element.series_stiffness_newtons_per_meter *
            (length_rate - InverseBilinearDamping(element, force));
        const Eigen::Vector3d force_in_world = force * direction_in_world;
        const FrameHandle world = model_->world_frame();
        body_wrenches[slot] = AppliedBodyWrench{
            motion.reference_point.body,
            motion.reference_point.position_in_body_frame_meters, world,
            Eigen::Vector3d::Zero(), force_in_world};
        body_wrenches[slot + 1] = AppliedBodyWrench{
            motion.opposite_point.body,
            motion.opposite_point.position_in_body_frame_meters, world,
            Eigen::Vector3d::Zero(), -force_in_world};
        slot += 2;
    }

    for (const auto& element : elements_.point_to_point_linear_spring_dampers) {
        const RelativeMotion motion =
            CalcRelativeMotion(*model_, context, element.reference_end, element.opposite_end);
        const auto observation = EvaluateLinearPointToPoint(element, motion);
        const Eigen::Vector3d force_in_world =
            observation.force_newtons *
            (motion.displacement_in_world_meters / observation.length_meters);
        const FrameHandle world = model_->world_frame();
        body_wrenches[slot] = AppliedBodyWrench{
            motion.reference_point.body, motion.reference_point.position_in_body_frame_meters,
            world, Eigen::Vector3d::Zero(), force_in_world};
        body_wrenches[slot + 1] = AppliedBodyWrench{
            motion.opposite_point.body, motion.opposite_point.position_in_body_frame_meters, world,
            Eigen::Vector3d::Zero(), -force_in_world};
        slot += 2;
    }

    for (const auto& element : elements_.point_to_point_hysteretic_springs) {
        const RelativeMotion motion =
            CalcRelativeMotion(*model_, context, element.reference_end, element.opposite_end);
        const auto observation = EvaluateHystereticPointToPoint(element, motion);
        const Eigen::Vector3d force_in_world =
            observation.force_newtons *
            (motion.displacement_in_world_meters / observation.length_meters);
        const FrameHandle world = model_->world_frame();
        body_wrenches[slot] = AppliedBodyWrench{
            motion.reference_point.body, motion.reference_point.position_in_body_frame_meters,
            world, Eigen::Vector3d::Zero(), force_in_world};
        body_wrenches[slot + 1] = AppliedBodyWrench{
            motion.opposite_point.body, motion.opposite_point.position_in_body_frame_meters, world,
            Eigen::Vector3d::Zero(), -force_in_world};
        slot += 2;
    }

    for (const auto& element :
         elements_.saturated_piecewise_linear_dampers) {
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end, element.opposite_end);
        const int axis = AxisIndex(element.axis);
        Eigen::Vector3d force_in_reference = Eigen::Vector3d::Zero();
        force_in_reference[axis] = EvaluateValidatedSaturatedDamperCurve(
            element, motion.velocity_in_reference_frame_meters_per_second[axis]);
        EmitTranslationalWrenchPair(
            *model_, motion.reference_point, motion.opposite_point,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (const auto& element : elements_.reference_frame_linear_bushings) {
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end, element.opposite_end);
        const Eigen::Vector3d force_in_reference =
            element.translational_stiffness_newtons_per_meter.cwiseProduct(
                motion.displacement_in_reference_frame_meters) +
            element.translational_damping_newton_seconds_per_meter.cwiseProduct(
                motion.velocity_in_reference_frame_meters_per_second);
        const Eigen::Vector3d moment_in_world =
            motion.rotation_of_reference_in_world *
            element.rotational_damping_newton_meter_seconds_per_radian
                .cwiseProduct(
                    motion.angular_velocity_in_reference_frame_radians_per_second);
        EmitTranslationalWrenchPair(
            *model_, motion.reference_point, motion.opposite_point,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        body_wrenches[slot].torque_about_point_newton_metres += moment_in_world;
        body_wrenches[slot + 1].torque_about_point_newton_metres -= moment_in_world;
        slot += 2;
    }
}

}  // namespace orvd::forces
