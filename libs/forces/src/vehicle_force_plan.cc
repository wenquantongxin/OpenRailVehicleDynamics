#include "orvd/forces/vehicle_force_plan.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include "orvd/multibody_model/multibody_frame_spatial_velocity.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"

namespace orvd::forces {
namespace {

using multibody_model::AppliedBodyWrench;
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
                                  FrameHandle reference, FrameHandle opposite) {
    const auto reference_pose = model.CalcPoseInWorld(context, reference);
    const auto opposite_pose = model.CalcPoseInWorld(context, opposite);
    const auto relative_velocity =
        model.CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
            context, opposite, reference, reference);

    RelativeMotion motion;
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
    const MultibodyModel& model, const ForceElementEnd& reference,
    const ForceElementEnd& opposite,
    const Eigen::Vector3d& force_on_reference_in_world,
    const Eigen::Vector3d& displacement_in_world,
    std::span<AppliedBodyWrench> destination) {
    const FrameHandle world = model.world_frame();
    destination[0] = AppliedBodyWrench{
        reference.body, reference.point_in_body_frame_meters, world,
        displacement_in_world.cross(force_on_reference_in_world),
        force_on_reference_in_world};
    destination[1] = AppliedBodyWrench{
        opposite.body, opposite.point_in_body_frame_meters, world,
        Eigen::Vector3d::Zero(), -force_on_reference_in_world};
}

// A pure couple: equal and opposite moments, no force, and therefore no
// dependence on where the two application points are.
void EmitCoupleWrenchPair(const MultibodyModel& model,
                          const ForceElementEnd& reference,
                          const ForceElementEnd& opposite,
                          const Eigen::Vector3d& moment_on_reference_in_world,
                          std::span<AppliedBodyWrench> destination) {
    const FrameHandle world = model.world_frame();
    destination[0] = AppliedBodyWrench{
        reference.body, reference.point_in_body_frame_meters, world,
        moment_on_reference_in_world, Eigen::Vector3d::Zero()};
    destination[1] = AppliedBodyWrench{
        opposite.body, opposite.point_in_body_frame_meters, world,
        -moment_on_reference_in_world, Eigen::Vector3d::Zero()};
}

void RequireDistinctLiveEnds(const MultibodyModel& model,
                             const ForceElementEnd& reference,
                             const ForceElementEnd& opposite,
                             const std::string& what) {
    if (reference.frame == opposite.frame) {
        Reject(what + " names one frame as both of its ends");
    }
    if (reference.body == opposite.body) {
        Reject(what +
               " has both ends on one body, so it could exert no relative "
               "force");
    }
    // Asking the model where each frame is validates every handle against it.
    const auto context = model.CreateDefaultContext();
    (void)model.CalcPoseInWorld(*context, reference.frame);
    (void)model.CalcPoseInWorld(*context, opposite.frame);
    (void)model.CalcPoseInWorld(*context, reference.body);
    (void)model.CalcPoseInWorld(*context, opposite.body);
    if (!reference.point_in_body_frame_meters.allFinite() ||
        !opposite.point_in_body_frame_meters.allFinite()) {
        Reject(what + " has an application point that is not finite");
    }
}

void RequireFinite(const Eigen::Vector3d& value, const std::string& what) {
    if (!value.allFinite()) {
        Reject(what + " is not finite");
    }
}

}  // namespace

VehicleForcePlan::VehicleForcePlan(
    const MultibodyModel& model,
    std::vector<TranslationalSpringDamper> translational_spring_dampers,
    std::vector<RollSpringDamperCouple> roll_spring_damper_couples,
    std::vector<SeriesSpringViscousDamper> series_spring_viscous_dampers,
    std::vector<SaturatedPiecewiseLinearDamper>
        saturated_piecewise_linear_dampers)
    : model_(&model),
      translational_(std::move(translational_spring_dampers)),
      roll_(std::move(roll_spring_damper_couples)),
      series_(std::move(series_spring_viscous_dampers)),
      clipped_(std::move(saturated_piecewise_linear_dampers)) {
    if (!model.is_finalized()) {
        throw std::logic_error(
            "VehicleForcePlan requires a finalized multibody model");
    }
    for (std::size_t index = 0; index < translational_.size(); ++index) {
        const auto& element = translational_[index];
        const std::string what =
            "translational spring-damper " + std::to_string(index);
        RequireDistinctLiveEnds(model, element.reference_end,
                                element.opposite_end, what);
        RequireFinite(element.stiffness_newtons_per_meter, what + "'s stiffness");
        RequireFinite(element.damping_newton_seconds_per_meter,
                      what + "'s damping");
    }
    for (std::size_t index = 0; index < roll_.size(); ++index) {
        const auto& element = roll_[index];
        const std::string what =
            "roll spring-damper couple " + std::to_string(index);
        RequireDistinctLiveEnds(model, element.reference_end,
                                element.opposite_end, what);
        if (!std::isfinite(element.stiffness_newton_meters_per_radian) ||
            !std::isfinite(element.damping_newton_meter_seconds_per_radian)) {
            Reject(what + " has a stiffness or damping that is not finite");
        }
    }
    for (std::size_t index = 0; index < series_.size(); ++index) {
        const auto& element = series_[index];
        const std::string what =
            "series spring-viscous damper " + std::to_string(index);
        RequireDistinctLiveEnds(model, element.reference_end,
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
    }
    for (std::size_t index = 0; index < clipped_.size(); ++index) {
        const auto& element = clipped_[index];
        const std::string what =
            "saturated piecewise linear damper " + std::to_string(index);
        RequireDistinctLiveEnds(model, element.reference_end,
                                element.opposite_end, what);
        (void)AxisIndex(element.axis);
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
            if (point > 0 &&
                !(current.relative_velocity_meters_per_second >
                  element.curve[point - 1].relative_velocity_meters_per_second)) {
                Reject(what +
                       " has curve points that do not ascend strictly in "
                       "velocity");
            }
        }
    }
}

double VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(
    const SaturatedPiecewiseLinearDamper& damper,
    double relative_velocity_meters_per_second) {
    // The curve states the non-negative half; the other half is its odd
    // reflection, so the sign is taken out and put back.
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
    for (std::size_t index = 0; index < translational_.size(); ++index) {
        const auto& element = translational_[index];
        const RelativeMotion motion =
            CalcRelativeMotion(*model_, context, element.reference_end.frame,
                               element.opposite_end.frame);
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
            *model_, element.reference_end, element.opposite_end,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (const auto& element : roll_) {
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end.frame, element.opposite_end.frame);
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
        EmitCoupleWrenchPair(*model_, element.reference_end,
                             element.opposite_end, moment_in_world,
                             body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (std::size_t index = 0; index < series_.size(); ++index) {
        const auto& element = series_[index];
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end.frame, element.opposite_end.frame);
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
            *model_, element.reference_end, element.opposite_end,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        slot += 2;
    }

    for (const auto& element : clipped_) {
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.reference_end.frame, element.opposite_end.frame);
        const int axis = AxisIndex(element.axis);
        Eigen::Vector3d force_in_reference = Eigen::Vector3d::Zero();
        force_in_reference[axis] = SaturatedPiecewiseLinearDamperForce(
            element, motion.velocity_in_reference_frame_meters_per_second[axis]);
        EmitTranslationalWrenchPair(
            *model_, element.reference_end, element.opposite_end,
            motion.rotation_of_reference_in_world * force_in_reference,
            motion.displacement_in_world_meters, body_wrenches.subspan(slot, 2));
        slot += 2;
    }
}

}  // namespace orvd::forces
