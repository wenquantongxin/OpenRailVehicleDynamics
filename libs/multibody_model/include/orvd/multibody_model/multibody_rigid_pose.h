#pragma once

/// @file
/// Where something is and how it is turned.
///
/// A pose is a rotation and a translation, and nothing else. Which two frames
/// it relates is the query's business and is said in the query's name: what
/// `CalcWorldPose(context, body)` returns is that body's frame in the world
/// frame, expressed in the world frame, and there is nowhere else for the
/// answer to be measured from. Putting the frames into the type instead would
/// make `WorldPose` a type that a relative-pose query could not return, and the
/// first such query would either need a second type or would have to lie.
///
/// The rotation comes out as a matrix rather than a quaternion. That is the
/// form the kinematics produce and the form a caller composes with; converting
/// here would be answering a question the caller did not ask, and converting
/// back is theirs to do.
///
/// A pose is produced, never supplied. Nothing in this layer takes one as an
/// input today, so there is no constructor for a caller — not because a caller
/// would misuse one, but because giving one now would freeze an input
/// validation policy before there is anything to validate it for.
///
/// The accessors are reference-qualified. A query returns a pose by value, so
/// `const auto& R = model.CalcPoseInWorld(context, body).rotation();` binds a
/// reference to a member of a temporary that dies at the end of that statement.
/// The `&&` overloads return by value, which makes that line copy rather than
/// dangle; the `const&` overloads keep the zero-copy read for a pose the caller
/// is holding.

#include <utility>

#include <Eigen/Dense>

namespace orvd::multibody_model {

class MultibodyModel;

class RigidPose {
   public:
    /// The rotation, as a 3×3 matrix whose columns are the posed frame's axes.
    [[nodiscard]] const Eigen::Matrix3d& rotation() const& { return rotation_; }
    [[nodiscard]] Eigen::Matrix3d rotation() && { return std::move(rotation_); }

    /// The position of the posed frame's origin.
    [[nodiscard]] const Eigen::Vector3d& translation() const& {
        return translation_;
    }
    [[nodiscard]] Eigen::Vector3d translation() && {
        return std::move(translation_);
    }

   private:
    friend class MultibodyModel;

    RigidPose(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
        : rotation_(rotation), translation_(translation) {}

    Eigen::Matrix3d rotation_;
    Eigen::Vector3d translation_;
};

}  // namespace orvd::multibody_model
