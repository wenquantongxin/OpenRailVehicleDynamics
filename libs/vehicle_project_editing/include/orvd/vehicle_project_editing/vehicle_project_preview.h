#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"

// What a 3D view draws for one revision of a vehicle project.
//
// Everything is expressed in the drawing frame: the local track frame at the
// layout reference station of a straight, level line. Its origin is on the
// track centre line at that station, +x points forward along increasing track
// station, +y to the right of an observer facing forward and +z downward
// (track_geometry/track_inertial_frame.h), so a body above the track plane has
// a negative z. Lengths are in metres. A rotation named R_drawing_from_body
// takes body-frame components to drawing-frame components.
//
// Every pose is evaluated by the vehicle's assembled multibody model, so the
// drawing is what the solver side would see. A placed free body sits at
// x = its mechanical station offset, y = its placement's lateral offset and
// z = its placement's vertical offset, turned by its placement's rotation:
// the start-up assembly's composition on a straight, level line with the
// layout reference station at the origin. Every jointed body takes the joint
// positions the vehicle definition gives as defaults: zero for a revolute
// joint, the stated default angles for a Ball-RPY joint. There are no
// velocities, no track geometry and no start-up state.
//
// A free body without a placement has no pose. It, every body jointed to it
// directly or through other bodies, every fixed frame, joint and force element
// on one of those bodies, and every wheel turning with one of them are
// reported as not drawable rather than drawn at the origin.

namespace orvd::vehicle_project_editing {

struct RigidBodyDrawing {
    std::string body_name;
    Eigen::Matrix3d rotation_drawing_from_body{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d origin_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
};

struct FixedFrameDrawing {
    std::string frame_name;
    Eigen::Matrix3d rotation_drawing_from_frame{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d origin_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
};

// A revolute joint turns its child frame about an axis through the origin its
// two frames share.
struct RevoluteJointDrawing {
    std::string joint_name;
    Eigen::Vector3d location_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
    // A unit vector: the joint's axis in its parent frame, normalised as the
    // model uses it, expressed in the drawing frame.
    Eigen::Vector3d axis_in_drawing_frame{Eigen::Vector3d::UnitY()};
};

// A Ball-RPY joint, drawn at the point it turns about, or a weld, drawn at its
// location: in both, the origin the joint's two frames share.
struct JointLocationDrawing {
    std::string joint_name;
    Eigen::Vector3d location_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
};

// A force element, drawn between the origins of its two end frames.
struct ForceElementDrawing {
    vehicle_project::ElementReference element;
    Eigen::Vector3d reference_end_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
    Eigen::Vector3d opposite_end_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
};

// A wheel, drawn as its nominal rolling circle, for one contact interface of
// the project's contact record. `wheel_body_name` is the interface's wheel
// body, the rigid body the wheel turns with: the wheelset body of a rigid
// wheelset, the wheel's own body when it rotates independently.
//
// The datum is the centre of the nominal rolling circle, and the outward spin
// axis a unit vector along the spin axis pointing from the carrier's
// wheel-profile origin towards the wheel, so the two wheels of a carrier point
// apart. With R_BW the carrier's rotation_body_from_nonspinning_wheel_profile
// and d the signed lateral datum, the record's lateral datum magnitude for a
// right wheel and its negative for a left one, they are R_BW * (0, d, 0) and
// R_BW * (0, sign(d), 0) in the wheel body's frame, taken to the drawing frame
// through the pose the wheel body is drawn with.
struct WheelDrawing {
    std::string interface_name;
    std::string wheel_body_name;
    Eigen::Vector3d datum_in_drawing_frame_meters{Eigen::Vector3d::Zero()};
    Eigen::Vector3d outward_spin_axis_in_drawing_frame{Eigen::Vector3d::UnitY()};
    double nominal_rolling_radius_meters{0.0};
};

// Each list follows the order the vehicle definition declares its elements in;
// the force elements run through the definition's force-element families in
// declaration order, and the wheels follow the contact declaration's interface
// order. A project without a contact record has no wheels.
struct VehicleProjectPreview {
    // The revision of the editing session's project this preview was computed
    // from. A session's revision only increases, so a value below the session's
    // current revision marks a preview of an older project.
    std::uint64_t project_revision{0};
    std::vector<RigidBodyDrawing> rigid_bodies{};
    std::vector<FixedFrameDrawing> fixed_frames{};
    std::vector<RevoluteJointDrawing> revolute_joints{};
    std::vector<JointLocationDrawing> ball_rpy_joints{};
    std::vector<JointLocationDrawing> weld_joints{};
    std::vector<ForceElementDrawing> force_elements{};
    std::vector<WheelDrawing> wheels{};
    // Every body, fixed frame, joint and force element that has no pose to
    // draw, and the contact interface of every wheel whose wheel body has
    // none, in the order of the lists above.
    std::vector<vehicle_project::ElementReference> not_drawable_elements{};
};

}  // namespace orvd::vehicle_project_editing
