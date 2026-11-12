#include "orvd/configuration/assemble_vehicle_multibody_model.h"

#include <cmath>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "orvd/multibody_runtime/multibody_physical_parameters.h"
#include "orvd/track_geometry/track_inertial_frame.h"
#include "vehicle_definition_inertia.h"
#include "vehicle_instance_assembly.h"

namespace orvd::configuration {
namespace {

using multibody_model::FrameHandle;
using multibody_model::MultibodyModel;
using multibody_model::RigidBodyHandle;

// The unit inertia about the body origin that the multibody layer wants, from
// the inertia about the centre of mass that the record states.
//
//   I_Bo = I_Bcm + m * (|c|^2 * 1 - c * c^T),   G_Bo = I_Bo / m
//
// with c the offset of the centre of mass from the body origin. The shift term
// is even in c, so inverting this map cannot reveal a sign error in the offset:
// the mass matrix coupling block, which is odd in c, is what shows whether this
// conversion carried the sign through. Neither can say whether the record's own
// sign is the one the source model states — that comparison is against the
// source, not against anything the record and the model can check between them.
multibody_runtime::RigidBodyInertiaParameters UnitInertiaAboutBodyOrigin(
    const VehicleRigidBodyDefinition& body) {
    if (!std::isfinite(body.mass_kilograms) ||
        !(body.mass_kilograms > 0.0)) {
        throw std::invalid_argument("rigid body '" + body.name +
                                    "' must have finite positive mass");
    }
    internal::ThrowIfSingularFreeBodyCenterOfMassInertia(
        body, "rigid body '" + body.name + "'");
    const Eigen::Vector3d& c = body.center_of_mass_in_body_frame_meters;
    const Eigen::Vector3d& moments =
        body.inertia_moments_about_center_of_mass_kilogram_square_meters;
    const Eigen::Vector3d& products =
        body.inertia_products_about_center_of_mass_kilogram_square_meters;

    Eigen::Matrix3d inertia_about_center_of_mass;
    inertia_about_center_of_mass << moments.x(), products.x(), products.y(),
        products.x(), moments.y(), products.z(),
        products.y(), products.z(), moments.z();

    const Eigen::Matrix3d shift =
        c.squaredNorm() * Eigen::Matrix3d::Identity() - c * c.transpose();
    const Eigen::Matrix3d unit_inertia_about_body_origin =
        inertia_about_center_of_mass / body.mass_kilograms + shift;

    multibody_runtime::RigidBodyInertiaParameters parameters;
    parameters.mass_kilograms = body.mass_kilograms;
    parameters.center_of_mass_in_body_frame = c;
    parameters.unit_inertia_moments =
        unit_inertia_about_body_origin.diagonal();
    parameters.unit_inertia_products =
        Eigen::Vector3d(unit_inertia_about_body_origin(0, 1),
                        unit_inertia_about_body_origin(0, 2),
                        unit_inertia_about_body_origin(1, 2));
    return parameters;
}

FrameHandle ResolveFrame(
    const std::unordered_map<std::string, FrameHandle>& frames,
    const std::string& name, const std::string& what) {
    const auto found = frames.find(name);
    if (found == frames.end()) {
        throw std::invalid_argument("the " + what + " names frame '" + name +
                                    "', which the vehicle description does not "
                                    "declare");
    }
    return found->second;
}

forces::ForceElementAxis ParseAxis(const std::string& axis,
                                   const std::string& what) {
    if (axis == "longitudinal") {
        return forces::ForceElementAxis::kLongitudinal;
    }
    if (axis == "lateral") {
        return forces::ForceElementAxis::kLateral;
    }
    if (axis == "vertical") {
        return forces::ForceElementAxis::kVertical;
    }
    throw std::invalid_argument(what + " states an axis '" + axis +
                                "' this library does not know");
}

// Where a force element's end attaches, resolved once by name.  The finalized
// model remains authoritative for the frame's carrier body and body-fixed
// origin; an edited source record cannot tear those apart afterwards.
struct EndResolver {
    const MultibodyModel* model;

    forces::ForceElementEnd operator()(const std::string& frame_name,
                                       const std::string& what) const {
        try {
            return forces::ForceElementEnd{model->GetFrameByName(frame_name)};
        } catch (const std::invalid_argument&) {
            throw std::invalid_argument(what + " names frame '" + frame_name +
                                        "', which the assembled model does not "
                                        "contain");
        }
    }
};

}  // namespace

void internal::AppendPointToPointLinearSpringDampers(
    std::span<const PointToPointLinearSpringDamperDefinition> definitions,
    std::string_view name_prefix, const MultibodyModel& model,
    forces::VehicleForceElementCollection& elements) {
    const EndResolver resolve{&model};
    for (const auto& element : definitions) {
        const auto name = QualifyVehicleName(name_prefix, element.name);
        const std::string what = "point-to-point linear spring-damper '" + name + "'";
        elements.point_to_point_linear_spring_dampers.push_back(
            {name,
             resolve(QualifyVehicleName(name_prefix, element.reference_frame_name),
                     what + " reference end"),
             resolve(QualifyVehicleName(name_prefix, element.opposite_frame_name),
                     what + " opposite end"),
             element.nominal_length_meters, element.stiffness_newtons_per_meter,
             element.damping_newton_seconds_per_meter});
    }
}

void internal::AppendPointToPointHystereticSprings(
    std::span<const PointToPointHystereticSpringDefinition> definitions,
    std::string_view name_prefix, const MultibodyModel& model,
    forces::VehicleForceElementCollection& elements) {
    const EndResolver resolve{&model};
    for (const auto& element : definitions) {
        const auto name = QualifyVehicleName(name_prefix, element.name);
        const std::string what = "point-to-point hysteretic spring '" + name + "'";
        elements.point_to_point_hysteretic_springs.push_back(
            {name,
             resolve(QualifyVehicleName(name_prefix, element.reference_frame_name),
                     what + " reference end"),
             resolve(QualifyVehicleName(name_prefix, element.opposite_frame_name),
                     what + " opposite end"),
             element.nominal_length_meters, element.stiffness_newtons_per_meter,
             element.hysteresis_ratio, element.hysteresis_transition_speed_meters_per_second});
    }
}

void internal::AppendVehicleForceElements(const VehicleDefinition& vehicle,
                                          std::string_view instance_name,
                                          const MultibodyModel& model,
                                          forces::VehicleForceElementCollection& elements) {
    const auto qualified = [&](const std::string& name) {
        return QualifyVehicleName(instance_name, name);
    };
    const EndResolver resolve{&model};

    std::vector<forces::TranslationalSpringDamper> translational;
    translational.reserve(vehicle.translational_spring_dampers.size());
    for (const auto& element : vehicle.translational_spring_dampers) {
        const std::string what = "translational spring-damper '" + element.name + "'";
        translational.push_back(forces::TranslationalSpringDamper{
            qualified(element.name),
            resolve(qualified(element.reference_frame_name), what + " reference end"),
            resolve(qualified(element.opposite_frame_name), what + " opposite end"),
            element.stiffness_newtons_per_meter, element.damping_newton_seconds_per_meter});
    }

    std::vector<forces::RollSpringDamperCouple> roll;
    roll.reserve(vehicle.roll_spring_damper_couples.size());
    for (const auto& element : vehicle.roll_spring_damper_couples) {
        const std::string what = "roll spring-damper couple '" + element.name + "'";
        roll.push_back(forces::RollSpringDamperCouple{
            qualified(element.name),
            resolve(qualified(element.reference_frame_name), what + " reference end"),
            resolve(qualified(element.opposite_frame_name), what + " opposite end"),
            element.stiffness_newton_meters_per_radian,
            element.damping_newton_meter_seconds_per_radian});
    }

    std::vector<forces::SeriesSpringViscousDamper> series;
    series.reserve(vehicle.series_spring_viscous_dampers.size());
    for (const auto& element : vehicle.series_spring_viscous_dampers) {
        const std::string what = "series spring-viscous damper '" + element.name + "'";
        series.push_back(forces::SeriesSpringViscousDamper{
            qualified(element.name),
            resolve(qualified(element.reference_frame_name), what + " reference end"),
            resolve(qualified(element.opposite_frame_name), what + " opposite end"),
            ParseAxis(element.axis, what), element.series_stiffness_newtons_per_meter,
            element.series_damping_newton_seconds_per_meter});
    }

    std::vector<forces::PointToPointSeriesSpringBilinearDamper> point_to_point;
    point_to_point.reserve(
        vehicle.point_to_point_series_spring_bilinear_dampers.size());
    for (const auto& element :
         vehicle.point_to_point_series_spring_bilinear_dampers) {
        const std::string what =
            "point-to-point series spring-bilinear damper '" + element.name + "'";
        point_to_point.push_back(forces::PointToPointSeriesSpringBilinearDamper{
            qualified(element.name),
            resolve(qualified(element.reference_frame_name), what + " reference end"),
            resolve(qualified(element.opposite_frame_name), what + " opposite end"),
            element.series_stiffness_newtons_per_meter,
            element.damping_below_knee_newton_seconds_per_meter, element.knee_force_newtons,
            element.damping_above_knee_newton_seconds_per_meter});
    }

    std::vector<forces::SaturatedPiecewiseLinearDamper> clipped;
    clipped.reserve(vehicle.saturated_piecewise_linear_dampers.size());
    for (const auto& element : vehicle.saturated_piecewise_linear_dampers) {
        const std::string what =
            "saturated piecewise linear damper '" + element.name + "'";
        std::vector<forces::SaturatedPiecewiseLinearDamperPoint> curve;
        curve.reserve(element.curve.size());
        for (const auto& point : element.curve) {
            curve.push_back(forces::SaturatedPiecewiseLinearDamperPoint{
                point.relative_velocity_meters_per_second, point.force_newtons});
        }
        clipped.push_back(forces::SaturatedPiecewiseLinearDamper{
            qualified(element.name),
            resolve(qualified(element.reference_frame_name), what + " reference end"),
            resolve(qualified(element.opposite_frame_name), what + " opposite end"),
            ParseAxis(element.axis, what), std::move(curve)});
    }

    std::vector<forces::ReferenceFrameLinearBushing> bushings;
    bushings.reserve(vehicle.reference_frame_linear_bushings.size());
    for (const auto& element : vehicle.reference_frame_linear_bushings) {
        const std::string what = "reference-frame linear bushing '" +
                                 element.name + "'";
        bushings.push_back(forces::ReferenceFrameLinearBushing{
            qualified(element.name),
            resolve(qualified(element.reference_frame_name), what + " reference end"),
            resolve(qualified(element.opposite_frame_name), what + " opposite end"),
            element.translational_stiffness_newtons_per_meter,
            element.translational_damping_newton_seconds_per_meter,
            element.rotational_damping_newton_meter_seconds_per_radian});
    }

    const auto append = [](auto& into, auto& from) {
        into.insert(into.end(), std::make_move_iterator(from.begin()),
                    std::make_move_iterator(from.end()));
    };
    append(elements.translational_spring_dampers, translational);
    append(elements.roll_spring_damper_couples, roll);
    append(elements.series_spring_viscous_dampers, series);
    append(elements.point_to_point_series_spring_bilinear_dampers, point_to_point);
    append(elements.saturated_piecewise_linear_dampers, clipped);
    append(elements.reference_frame_linear_bushings, bushings);
    internal::AppendPointToPointLinearSpringDampers(vehicle.point_to_point_linear_spring_dampers,
                                                    instance_name, model, elements);
    internal::AppendPointToPointHystereticSprings(vehicle.point_to_point_hysteretic_springs,
                                                  instance_name, model, elements);
}

std::unique_ptr<forces::VehicleForcePlan> BuildVehicleForcePlan(const VehicleDefinition& vehicle,
                                                                const MultibodyModel& model) {
    forces::VehicleForceElementCollection elements;
    internal::AppendVehicleForceElements(vehicle, "", model, elements);
    return std::make_unique<forces::VehicleForcePlan>(model, std::move(elements));
}

void internal::AppendVehicleMultibodyModel(const VehicleDefinition& vehicle,
                                           std::string_view instance_name, MultibodyModel& model) {
    const auto qualified = [&](const std::string& name) {
        return QualifyVehicleName(instance_name, name);
    };
    if (vehicle.vehicle_name.empty()) {
        throw std::invalid_argument(
            "a vehicle description must have a non-empty vehicle name");
    }

    // One name resolves to one frame whether it belongs to a body or to a frame
    // fixed on one: the multibody layer keeps bodies and frames in a single
    // namespace, so the two can never collide.
    // Name lookups on the model itself only answer after finalization, and by
    // then every element is already added, so the assembler keeps its own map
    // of the handles it just created.
    std::unordered_map<std::string, RigidBodyHandle> bodies;
    std::unordered_map<std::string, FrameHandle> frames;
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        const RigidBodyHandle handle =
            model.AddRigidBody(qualified(body.name), UnitInertiaAboutBodyOrigin(body));
        bodies.emplace(body.name, handle);
        frames.emplace(body.name, model.body_frame(handle));
    }

    for (const VehicleFixedFrameDefinition& frame : vehicle.fixed_frames) {
        multibody_runtime::FixedFramePoseParameters pose;
        pose.R_PF = frame.rotation_in_body_frame;
        pose.p_PoFo_P = frame.position_in_body_frame_meters;
        const auto carrier = bodies.find(frame.body_name);
        if (carrier == bodies.end()) {
            throw std::invalid_argument(
                "fixed frame '" + frame.name + "' names body '" +
                frame.body_name +
                "', which the vehicle description does not declare");
        }
        frames.emplace(frame.name,
                       model.AddFixedFrame(qualified(frame.name), carrier->second, pose));
    }

    for (const VehicleWeldJointDefinition& joint : vehicle.weld_joints) {
        model.AddWeldJoint(
            qualified(joint.name),
            ResolveFrame(frames, joint.parent_frame_name, "weld joint '" + joint.name + "' parent"),
            ResolveFrame(frames, joint.child_frame_name, "weld joint '" + joint.name + "' child"));
    }

    for (const VehicleRevoluteJointDefinition& joint : vehicle.revolute_joints) {
        model.AddRevoluteJoint(qualified(joint.name),
                               ResolveFrame(frames, joint.parent_frame_name,
                                            "revolute joint '" + joint.name + "' parent"),
                               ResolveFrame(frames, joint.child_frame_name,
                                            "revolute joint '" + joint.name + "' child"),
                               joint.axis_in_parent_frame,
                               joint.damping_newton_metre_seconds_per_radian);
    }

    for (const VehicleBallRpyJointDefinition& joint :
         vehicle.ball_rpy_joints) {
        model.AddBallRpyJoint(qualified(joint.name),
                              ResolveFrame(frames, joint.parent_frame_name,
                                           "Ball-RPY joint '" + joint.name + "' parent"),
                              ResolveFrame(frames, joint.child_frame_name,
                                           "Ball-RPY joint '" + joint.name + "' child"),
                              joint.default_roll_pitch_yaw_angles_radians);
    }

    // Last, so that a description whose joints are wrong fails on the joint
    // rather than on a free-body declaration that would merely be the first
    // statement to notice the tree already closed.
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        if (body.moves_freely_in_world) {
            model.DeclareFreeBody(bodies.at(body.name));
        }
    }
}

std::unique_ptr<MultibodyModel> AssembleVehicleMultibodyModel(
    const VehicleDefinition& vehicle,
    double gravitational_acceleration_magnitude_meters_per_second_squared) {
    auto model = std::make_unique<MultibodyModel>();
    model->SetGravityVector(track_geometry::GravitationalAccelerationInInertial(
        gravitational_acceleration_magnitude_meters_per_second_squared));
    internal::AppendVehicleMultibodyModel(vehicle, "", *model);
    model->Finalize();
    return model;
}

}  // namespace orvd::configuration
