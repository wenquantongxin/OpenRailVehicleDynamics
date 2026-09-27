#include "orvd/scene_observation/body_state.h"

#include <Eigen/Geometry>

namespace orvd::scene_observation {
namespace {

std::array<double, 3> Triple(const Eigen::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}

}  // namespace

BodyStateSampler::BodyStateSampler(const multibody_model::MultibodyModel& model)
    : model_(&model) {
    bodies_.reserve(static_cast<std::size_t>(model.num_rigid_bodies()));
    names_.reserve(bodies_.capacity());
    for (int index = 0; index < model.num_rigid_bodies(); ++index) {
        const auto body = model.GetRigidBody(index);
        bodies_.push_back(body);
        names_.emplace_back(model.GetRigidBodyName(body));
    }
}

std::vector<BodyState> BodyStateSampler::Sample(
    const multibody_model::MultibodyEvaluationContext& context) const {
    std::vector<BodyState> states;
    states.reserve(bodies_.size());
    for (const auto body : bodies_) {
        const auto pose = model_->CalcPoseInWorld(context, body);
        const Eigen::Quaterniond orientation(pose.rotation());
        const auto velocity =
            model_->CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                context, body);
        states.push_back(BodyState{
            Triple(pose.translation()),
            {orientation.w(), orientation.x(), orientation.y(),
             orientation.z()},
            Triple(velocity
                       .translational_velocity_at_frame_origin_meters_per_second()),
            Triple(velocity.angular_velocity_radians_per_second())});
    }
    return states;
}

}  // namespace orvd::scene_observation
