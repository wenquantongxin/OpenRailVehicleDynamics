#pragma once

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model_handles.h"

namespace orvd::multibody_model {

/// A spatial force on a rigid body at body-fixed point Q, expressed in E.
/// The torque is about Q; Q is both the application point and moment reference.
struct AppliedBodyWrench {
    RigidBodyHandle body;
    Eigen::Vector3d point_position_in_body_frame_meters;
    FrameHandle expressed_in_frame;
    Eigen::Vector3d torque_about_point_newton_metres;
    Eigen::Vector3d force_newtons;
};

struct AppliedRevoluteJointTorque {
    JointHandle joint;
    double torque_newton_metres{};
};

struct AppliedPrismaticJointForce {
    JointHandle joint;
    double force_newtons{};
};

}  // namespace orvd::multibody_model
