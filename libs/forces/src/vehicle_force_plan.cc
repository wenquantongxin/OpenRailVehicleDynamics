#include "orvd/forces/vehicle_force_plan.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <unordered_set>

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
    Eigen::Vector3d reference_origin_in_world_meters;
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
    motion.reference_origin_in_world_meters = reference_pose.translation();
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

// Returns the canonical quaternion used by the frozen WRL path. Eigen already
// constructs a unit quaternion from the model's rotation matrix; selecting the
// non-negative scalar representative removes the q/-q ambiguity before taking
// a half angle.
Eigen::Quaterniond CanonicalQuaternion(const Eigen::Matrix3d& rotation) {
    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();
    if (quaternion.w() < 0.0) {
        quaternion.coeffs() *= -1.0;
    }
    return quaternion;
}

Eigen::Matrix3d CalcHalfAngleRotation(const Eigen::Matrix3d& rotation_a_c) {
    const Eigen::Quaterniond quaternion_a_c =
        CanonicalQuaternion(rotation_a_c);
    const double scalar =
        std::sqrt(0.5 * (quaternion_a_c.w() + 1.0));
    const double vector_scale = 1.0 / (2.0 * scalar);
    const Eigen::Quaterniond quaternion_a_b(
        scalar, quaternion_a_c.x() * vector_scale,
        quaternion_a_c.y() * vector_scale,
        quaternion_a_c.z() * vector_scale);
    return quaternion_a_b.toRotationMatrix();
}

struct SpaceXyzRollPitchYawKinematics {
    Eigen::Vector3d angles_radians;
    Eigen::Matrix3d rates_from_parent_angular_velocity;
};

// The same canonical space-XYZ extraction and parent-expressed angular-rate
// map used by the frozen WRL bushing. Keeping this small implementation here
// avoids exposing or linking a second Drake mathematical API through the
// first-party force module.
SpaceXyzRollPitchYawKinematics CalcSpaceXyzRollPitchYawKinematics(
    const Eigen::Matrix3d& rotation_a_c) {
    const Eigen::Quaterniond quaternion = CanonicalQuaternion(rotation_a_c);
    const double r22 = rotation_a_c(2, 2);
    const double r21 = rotation_a_c(2, 1);
    const double r10 = rotation_a_c(1, 0);
    const double r00 = rotation_a_c(0, 0);
    const double positive_cosine =
        std::sqrt((r22 * r22 + r21 * r21 + r10 * r10 + r00 * r00) /
                  2.0);
    const double pitch = std::atan2(-rotation_a_c(2, 0), positive_cosine);

    const double y_a = quaternion.x() + quaternion.z();
    const double x_a = quaternion.w() - quaternion.y();
    const double y_b = quaternion.z() - quaternion.x();
    const double x_b = quaternion.w() + quaternion.y();
    const double epsilon = std::numeric_limits<double>::epsilon();
    const double z_a = std::abs(y_a) <= epsilon && std::abs(x_a) <= epsilon
                           ? 0.0
                           : std::atan2(y_a, x_a);
    const double z_b = std::abs(y_b) <= epsilon && std::abs(x_b) <= epsilon
                           ? 0.0
                           : std::atan2(y_b, x_b);
    double roll = z_a - z_b;
    double yaw = z_a + z_b;
    if (roll > std::numbers::pi) {
        roll -= 2.0 * std::numbers::pi;
    } else if (roll < -std::numbers::pi) {
        roll += 2.0 * std::numbers::pi;
    }
    if (yaw > std::numbers::pi) {
        yaw -= 2.0 * std::numbers::pi;
    } else if (yaw < -std::numbers::pi) {
        yaw += 2.0 * std::numbers::pi;
    }

    const double cosine_pitch = std::cos(pitch);
    constexpr double kGimbalLockToleranceCosPitch = 0.008;
    if (std::abs(cosine_pitch) < kGimbalLockToleranceCosPitch) {
        throw std::runtime_error(
            "half-angle midpoint roll-pitch-yaw bushing: pitch is within "
            "the frozen 0.008 cosine threshold of the space-XYZ "
            "roll-pitch-yaw singularity");
    }
    const double sine_pitch = std::sin(pitch);
    const double sine_yaw = std::sin(yaw);
    const double cosine_yaw = std::cos(yaw);
    const double inverse_cosine_pitch = 1.0 / cosine_pitch;

    SpaceXyzRollPitchYawKinematics result;
    result.angles_radians = Eigen::Vector3d(roll, pitch, yaw);
    result.rates_from_parent_angular_velocity <<
        cosine_yaw * inverse_cosine_pitch,
        sine_yaw * inverse_cosine_pitch, 0.0, -sine_yaw, cosine_yaw, 0.0,
        cosine_yaw * inverse_cosine_pitch * sine_pitch,
        sine_yaw * inverse_cosine_pitch * sine_pitch, 1.0;
    return result;
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

// A pure couple: equal and opposite moments, no force, and therefore no
// dependence on where the two application points are.
void EmitCoupleWrenchPair(const MultibodyModel& model,
                          const BodyFixedPoint& reference,
                          const BodyFixedPoint& opposite,
                          const Eigen::Vector3d& moment_on_reference_in_world,
                          std::span<AppliedBodyWrench> destination) {
    const FrameHandle world = model.world_frame();
    destination[0] = AppliedBodyWrench{
        reference.body, reference.position_in_body_frame_meters, world,
        moment_on_reference_in_world, Eigen::Vector3d::Zero()};
    destination[1] = AppliedBodyWrench{
        opposite.body, opposite.position_in_body_frame_meters, world,
        -moment_on_reference_in_world, Eigen::Vector3d::Zero()};
}

// The WRL half-angle bushing applies both resultants at one instantaneous
// world-space midpoint. That midpoint is converted separately into a point on
// each body; it is not either frame origin and carries no translational-family
// support moment.
void EmitMidpointBushingWrenchPair(
    const MultibodyModel& model, const MultibodyEvaluationContext& context,
    const RelativeMotion& motion,
    const Eigen::Vector3d& force_on_c_in_world,
    const Eigen::Vector3d& torque_on_c_in_world,
    std::span<AppliedBodyWrench> destination) {
    const Eigen::Vector3d midpoint_in_world =
        motion.reference_origin_in_world_meters +
        0.5 * motion.displacement_in_world_meters;
    const auto pose_world_body_a =
        model.CalcPoseInWorld(context, motion.reference_point.body);
    const auto pose_world_body_c =
        model.CalcPoseInWorld(context, motion.opposite_point.body);
    const Eigen::Vector3d midpoint_in_body_a =
        pose_world_body_a.rotation().transpose() *
        (midpoint_in_world - pose_world_body_a.translation());
    const Eigen::Vector3d midpoint_in_body_c =
        pose_world_body_c.rotation().transpose() *
        (midpoint_in_world - pose_world_body_c.translation());
    const FrameHandle world = model.world_frame();
    destination[0] = AppliedBodyWrench{
        motion.reference_point.body, midpoint_in_body_a, world,
        -torque_on_c_in_world, -force_on_c_in_world};
    destination[1] = AppliedBodyWrench{
        motion.opposite_point.body, midpoint_in_body_c, world,
        torque_on_c_in_world, force_on_c_in_world};
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
         index < elements_.half_angle_midpoint_roll_pitch_yaw_bushings.size();
         ++index) {
        const auto& element =
            elements_.half_angle_midpoint_roll_pitch_yaw_bushings[index];
        const std::string what = ElementDescription(
            "half-angle midpoint roll-pitch-yaw bushing", index,
            element.name);
        RequireUniqueName(element.name, what, &names);
        RequireDistinctLiveEnds(model, *validation_context,
                                element.frame_a_end, element.frame_c_end,
                                what);
        RequirePassive(
            element.rotational_stiffness_newton_meters_per_radian,
            what + " rotational stiffness");
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
        EmitCoupleWrenchPair(*model_, motion.reference_point,
                             motion.opposite_point, moment_in_world,
                             body_wrenches.subspan(slot, 2));
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

    for (const auto& element :
         elements_.half_angle_midpoint_roll_pitch_yaw_bushings) {
        const RelativeMotion motion = CalcRelativeMotion(
            *model_, context, element.frame_a_end, element.frame_c_end);
        const Eigen::Matrix3d rotation_a_b =
            CalcHalfAngleRotation(motion.rotation_of_opposite_in_reference);
        const Eigen::Matrix3d rotation_b_a = rotation_a_b.transpose();
        const Eigen::Vector3d displacement_in_b =
            rotation_b_a * motion.displacement_in_reference_frame_meters;
        const Eigen::Vector3d displacement_rate_in_b =
            rotation_b_a *
            (motion.velocity_in_reference_frame_meters_per_second -
             0.5 *
                 motion.angular_velocity_in_reference_frame_radians_per_second
                     .cross(motion.displacement_in_reference_frame_meters));
        const Eigen::Vector3d force_on_c_in_b =
            -(element.translational_stiffness_newtons_per_meter.cwiseProduct(
                  displacement_in_b) +
              element.translational_damping_newton_seconds_per_meter
                  .cwiseProduct(displacement_rate_in_b));
        const Eigen::Vector3d force_on_c_in_world =
            motion.rotation_of_reference_in_world * rotation_a_b *
            force_on_c_in_b;

        const SpaceXyzRollPitchYawKinematics rpy =
            CalcSpaceXyzRollPitchYawKinematics(
                motion.rotation_of_opposite_in_reference);
        const Eigen::Vector3d rpy_rates =
            rpy.rates_from_parent_angular_velocity *
            motion.angular_velocity_in_reference_frame_radians_per_second;
        const Eigen::Vector3d generalized_torque_on_c =
            -(element.rotational_stiffness_newton_meters_per_radian
                  .cwiseProduct(rpy.angles_radians) +
              element.rotational_damping_newton_meter_seconds_per_radian
                  .cwiseProduct(rpy_rates));
        const Eigen::Vector3d torque_on_c_in_a =
            rpy.rates_from_parent_angular_velocity.transpose() *
            generalized_torque_on_c;
        const Eigen::Vector3d torque_on_c_in_world =
            motion.rotation_of_reference_in_world * torque_on_c_in_a;
        EmitMidpointBushingWrenchPair(
            *model_, context, motion, force_on_c_in_world,
            torque_on_c_in_world, body_wrenches.subspan(slot, 2));
        slot += 2;
    }
}

}  // namespace orvd::forces
