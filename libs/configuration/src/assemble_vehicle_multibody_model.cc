#include "orvd/configuration/assemble_vehicle_multibody_model.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "orvd/multibody_runtime/multibody_physical_parameters.h"
#include "orvd/track_geometry/track_inertial_frame.h"

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

std::unique_ptr<forces::VehicleForcePlan> BuildVehicleForcePlan(
    const VehicleDefinition& vehicle, const MultibodyModel& model) {
    const EndResolver resolve{&model};

    std::vector<forces::TranslationalSpringDamper> translational;
    translational.reserve(vehicle.translational_spring_dampers.size());
    for (const auto& element : vehicle.translational_spring_dampers) {
        const std::string what = "translational spring-damper '" + element.name + "'";
        translational.push_back(forces::TranslationalSpringDamper{
            element.name,
            resolve(element.reference_frame_name, what + " reference end"),
            resolve(element.opposite_frame_name, what + " opposite end"),
            element.stiffness_newtons_per_meter,
            element.damping_newton_seconds_per_meter});
    }

    std::vector<forces::RollSpringDamperCouple> roll;
    roll.reserve(vehicle.roll_spring_damper_couples.size());
    for (const auto& element : vehicle.roll_spring_damper_couples) {
        const std::string what = "roll spring-damper couple '" + element.name + "'";
        roll.push_back(forces::RollSpringDamperCouple{
            element.name,
            resolve(element.reference_frame_name, what + " reference end"),
            resolve(element.opposite_frame_name, what + " opposite end"),
            element.stiffness_newton_meters_per_radian,
            element.damping_newton_meter_seconds_per_radian});
    }

    std::vector<forces::SeriesSpringViscousDamper> series;
    series.reserve(vehicle.series_spring_viscous_dampers.size());
    for (const auto& element : vehicle.series_spring_viscous_dampers) {
        const std::string what = "series spring-viscous damper '" + element.name + "'";
        series.push_back(forces::SeriesSpringViscousDamper{
            element.name,
            resolve(element.reference_frame_name, what + " reference end"),
            resolve(element.opposite_frame_name, what + " opposite end"),
            ParseAxis(element.axis, what),
            element.series_stiffness_newtons_per_meter,
            element.series_damping_newton_seconds_per_meter});
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
            element.name,
            resolve(element.reference_frame_name, what + " reference end"),
            resolve(element.opposite_frame_name, what + " opposite end"),
            ParseAxis(element.axis, what), std::move(curve)});
    }

    return std::make_unique<forces::VehicleForcePlan>(
        model, std::move(translational), std::move(roll), std::move(series),
        std::move(clipped));
}

std::unique_ptr<MultibodyModel> AssembleVehicleMultibodyModel(
    const VehicleDefinition& vehicle,
    double gravitational_acceleration_magnitude_meters_per_second_squared) {
    if (vehicle.vehicle_name.empty()) {
        throw std::invalid_argument(
            "a vehicle description must have a non-empty vehicle name");
    }
    auto model = std::make_unique<MultibodyModel>();
    model->SetGravityVector(track_geometry::GravitationalAccelerationInInertial(
        gravitational_acceleration_magnitude_meters_per_second_squared));

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
            model->AddRigidBody(body.name, UnitInertiaAboutBodyOrigin(body));
        bodies.emplace(body.name, handle);
        frames.emplace(body.name, model->body_frame(handle));
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
                       model->AddFixedFrame(frame.name, carrier->second, pose));
    }

    for (const VehicleWeldJointDefinition& joint : vehicle.weld_joints) {
        model->AddWeldJoint(
            joint.name,
            ResolveFrame(frames, joint.parent_frame_name,
                         "weld joint '" + joint.name + "' parent"),
            ResolveFrame(frames, joint.child_frame_name,
                         "weld joint '" + joint.name + "' child"));
    }

    for (const VehicleRevoluteJointDefinition& joint : vehicle.revolute_joints) {
        model->AddRevoluteJoint(
            joint.name,
            ResolveFrame(frames, joint.parent_frame_name,
                         "revolute joint '" + joint.name + "' parent"),
            ResolveFrame(frames, joint.child_frame_name,
                         "revolute joint '" + joint.name + "' child"),
            joint.axis_in_parent_frame,
            joint.damping_newton_metre_seconds_per_radian);
    }

    // Last, so that a description whose joints are wrong fails on the joint
    // rather than on a free-body declaration that would merely be the first
    // statement to notice the tree already closed.
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        if (body.moves_freely_in_world) {
            model->DeclareFreeBody(bodies.at(body.name));
        }
    }

    model->Finalize();
    return model;
}

}  // namespace orvd::configuration
