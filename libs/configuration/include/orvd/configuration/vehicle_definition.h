#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// The mechanical description of one rail vehicle: the rigid bodies, the frames
// fixed to them, and the relations that hold them together.
//
// This is a description, not a model. It says what exists and where; it does not
// say how the assembled model stores coordinates, which body the rigid tree will
// treat as inboard, or what the vehicle is standing on. A vehicle assembler
// turns one of these into a multibody model; nothing else is allowed to.
//
// Every vector field names the frame it is expressed in. That frame is always
// one the record itself defines, never an ambient convention: a position in a
// body frame means the body this record declared, and an axis in a parent frame
// means the frame this record named as the joint's parent.
//
// The inertia convention here is the single authoritative one: mass, the offset
// of the centre of mass from the body origin, and the inertia about the centre
// of mass. The multibody layer wants a unit inertia about the body origin
// instead; converting is the assembler's job, and the converted numbers are
// never written back into a record. Storing both would make it possible for a
// record to disagree with itself.
//
// This is deliberately a mutable value record. A C++ research workflow may
// construct one directly or load JSON and then edit it before assembly. The
// assembler reads that call-time value into a new model and retains no reference
// to it, so later edits do not alter a model that has already been finalized.
// JSON strictness describes only the file-to-record boundary; it is not a ban on
// explicit C++ modelling.

namespace orvd::configuration {

// One rigid body of the vehicle.
//
// A placeholder body — unit mass, unit inertia, no extent — is a legitimate
// member: it carries frames that force elements attach to, and it contributes
// its mass to the vehicle. It is described the same way as any other body, so
// nothing downstream has to know which bodies are placeholders.
struct VehicleRigidBodyDefinition {
    std::string name;

    // Whether the body moves freely with respect to the world.
    //
    // Stated, not inferred from the absence of joints. A free body is a relation
    // to the world in the same sense that a joint is a relation between two
    // bodies, and a record that merely omitted every joint would be describing
    // something it did not mean.
    bool moves_freely_in_world{false};

    double mass_kilograms{0.0};

    Eigen::Vector3d center_of_mass_in_body_frame_meters{Eigen::Vector3d::Zero()};

    // Ixx, Iyy, Izz about the centre of mass, expressed in the body frame.
    Eigen::Vector3d inertia_moments_about_center_of_mass_kilogram_square_meters{
        Eigen::Vector3d::Zero()};

    // The (x,y), (x,z), and (y,z) entries of the symmetric inertia matrix about
    // the centre of mass, expressed in the body frame. With h = I*w, these are
    // the conventional negative products, for example Ixy = -integral(x*y dm).
    Eigen::Vector3d inertia_products_about_center_of_mass_kilogram_square_meters{
        Eigen::Vector3d::Zero()};
};

// A frame fixed to a body: an attachment point, a measurement point, or the
// mounting seat a weld needs because a weld itself carries no relative pose.
struct VehicleFixedFrameDefinition {
    std::string name;
    std::string body_name;
    Eigen::Vector3d position_in_body_frame_meters{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d rotation_in_body_frame{Eigen::Matrix3d::Identity()};
};

// Bodies and frames share one namespace, so a frame reference below is either
// the name of a fixed frame this record declared, or the name of a body — which
// means that body's own frame, at its origin. There is no third possibility and
// no ambiguity: a name belongs to exactly one of the two.

struct VehicleRevoluteJointDefinition {
    std::string name;
    std::string parent_frame_name;
    std::string child_frame_name;
    Eigen::Vector3d axis_in_parent_frame{Eigen::Vector3d::UnitY()};

    // Stated for every joint, including where it is zero. The multibody layer
    // demands a value, and a record that left it out would be handing that
    // decision to whichever default happened to be in force.
    double damping_newton_metre_seconds_per_radian{0.0};
};

struct VehicleWeldJointDefinition {
    std::string name;
    std::string parent_frame_name;
    std::string child_frame_name;
};

struct VehicleDefinition {
    std::string vehicle_name;
    std::vector<VehicleRigidBodyDefinition> rigid_bodies;
    std::vector<VehicleFixedFrameDefinition> fixed_frames;
    std::vector<VehicleRevoluteJointDefinition> revolute_joints;
    std::vector<VehicleWeldJointDefinition> weld_joints;
};

}  // namespace orvd::configuration
