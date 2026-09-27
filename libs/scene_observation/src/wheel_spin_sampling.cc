#include "orvd/scene_observation/wheel_spin_sampling.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "orvd/multibody_model/multibody_evaluation_context.h"

namespace orvd::scene_observation {

WheelSpinAngleSampler::WheelSpinAngleSampler(
    const multibody_model::MultibodyModel& model, const SceneTopology& topology)
    : model_(&model) {
    wheels_.reserve(topology.wheel_placements.size());
    available_ = !topology.wheel_placements.empty();
    for (const auto& placement : topology.wheel_placements) {
        if (!placement.spin_joint_name.has_value()) {
            available_ = false;
        }
    }
    if (!available_) {
        return;
    }
    // Probe each joint with a small positive displacement from the default
    // configuration. Only that wheel moves, so the wheel's own orientation
    // change is the joint rotation expressed in the wheel body frame.
    constexpr double kProbeRadians = 1.0e-3;
    auto context = model.CreateDefaultContext();
    const Eigen::VectorXd default_positions = context->generalized_positions();
    for (const auto& placement : topology.wheel_placements) {
        const auto joint = model.GetJointByName(*placement.spin_joint_name);
        const auto range = model.GetJointPositionRange(joint);
        if (range.size() != 1) {
            throw std::invalid_argument(
                "wheel spin sampler: joint '" + *placement.spin_joint_name +
                "' does not own exactly one generalized position");
        }
        const auto wheel = model.GetRigidBodyByName(placement.wheel_body_name);
        model.SetGeneralizedPositions(context.get(), default_positions);
        const Eigen::Matrix3d rotation_before =
            model.CalcPoseInWorld(*context, wheel).rotation();
        Eigen::VectorXd displaced = default_positions;
        displaced[range.start()] += kProbeRadians;
        model.SetGeneralizedPositions(context.get(), displaced);
        const Eigen::Matrix3d rotation_after =
            model.CalcPoseInWorld(*context, wheel).rotation();
        const Eigen::AngleAxisd increment(
            Eigen::Matrix3d(rotation_before.transpose() * rotation_after));
        const double alignment =
            increment.axis().dot(placement.spin_axis_in_wheel_body_frame);
        if (std::abs(increment.angle() - kProbeRadians) > 1.0e-9 ||
            std::abs(std::abs(alignment) - 1.0) > 1.0e-9) {
            throw std::invalid_argument(
                "wheel spin sampler: joint '" + *placement.spin_joint_name +
                "' does not turn wheel '" + placement.wheel_body_name +
                "' about its spin axis");
        }
        wheels_.push_back(WheelBinding{range.start(),
                                       alignment > 0.0 ? 1.0 : -1.0});
    }
}

void WheelSpinAngleSampler::Sample(
    const multibody_model::MultibodyEvaluationContext& context,
    std::span<double> angles_radians) const {
    if (!available_) {
        throw std::logic_error(
            "wheel spin sampler: not every wheel has a spin joint");
    }
    if (angles_radians.size() != wheels_.size()) {
        throw std::invalid_argument(
            "wheel spin sampler: the output span does not have one entry per "
            "wheel");
    }
    const Eigen::VectorXd& positions = context.generalized_positions();
    for (std::size_t index = 0; index < wheels_.size(); ++index) {
        angles_radians[index] =
            wheels_[index].sign * positions[wheels_[index].position_index];
    }
}

}  // namespace orvd::scene_observation
