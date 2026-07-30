#include "orvd/multibody_runtime/multibody_state_layout.h"

#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::multibody_runtime {
namespace {

void RequireNonNegative(int value, const char* what) {
    if (value < 0) {
        throw std::invalid_argument(std::string("multibody state layout: ") +
                                    what + " is " + std::to_string(value) +
                                    ", which cannot be negative");
    }
}

}  // namespace

MultibodyStateLayout::MultibodyStateLayout(
    MultibodyStateLayoutDescription description)
    : description_(std::move(description)) {
    RequireNonNegative(description_.generalized_position_count,
                       "the generalized position count");
    RequireNonNegative(description_.generalized_velocity_count,
                       "the generalized velocity count");
    RequireNonNegative(description_.rigid_body_count, "the rigid body count");
    RequireNonNegative(description_.fixed_frame_count, "the fixed frame count");
    RequireNonNegative(description_.joint_actuator_count,
                       "the joint actuator count");
    RequireNonNegative(description_.revolute_spring_count,
                       "the revolute spring count");
    RequireNonNegative(description_.linear_bushing_count,
                       "the linear bushing count");

    joint_damping_starts_.reserve(description_.joint_velocity_counts.size());
    for (std::size_t index = 0;
         index < description_.joint_velocity_counts.size(); ++index) {
        const int count = description_.joint_velocity_counts[index];
        if (count < 0) {
            throw std::invalid_argument(
                "multibody state layout: joint " + std::to_string(index) +
                " reports " + std::to_string(count) +
                " velocities, which cannot be negative");
        }
        joint_damping_starts_.push_back(total_joint_damping_count_);
        total_joint_damping_count_ += count;
    }
}

int MultibodyStateLayout::joint_velocity_count(int joint_index) const {
    if (joint_index < 0 || joint_index >= damped_joint_count()) {
        throw std::invalid_argument(
            "multibody state layout: joint index " +
            std::to_string(joint_index) + " is outside the " +
            std::to_string(damped_joint_count()) + " damped joints");
    }
    return description_.joint_velocity_counts[static_cast<std::size_t>(joint_index)];
}

int MultibodyStateLayout::joint_damping_start(int joint_index) const {
    if (joint_index < 0 || joint_index >= damped_joint_count()) {
        throw std::invalid_argument(
            "multibody state layout: joint index " +
            std::to_string(joint_index) + " is outside the " +
            std::to_string(damped_joint_count()) + " damped joints");
    }
    return joint_damping_starts_[static_cast<std::size_t>(joint_index)];
}

}  // namespace orvd::multibody_runtime
