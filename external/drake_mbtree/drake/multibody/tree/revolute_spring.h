#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context_fwd.h"

namespace drake {
namespace multibody {

/// This %ForceElement models a torsional spring attached to a RevoluteJoint
/// and applies a torque to that joint
/// <pre>
///   τ = -k⋅(θ - θ₀)
/// </pre>
/// where θ₀ is the nominal joint position. Note that joint damping exists
/// within the RevoluteJoint itself, and so is not included here.
///
/// The k (stiffness) and θ₀ (nominal angle) specified in the constructor
/// are kept as default values. These parameters are stored within the context
/// and can be accessed and set by context dependent getters/setters.
template <typename T>
class RevoluteSpring final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteSpring);

  /// Constructor for a spring attached to the given joint
  /// @param[in] nominal_angle
  ///   The nominal angle of the spring  θ₀, in radians, at which the spring
  ///   applies no moment.
  /// @param[in] stiffness
  ///   The stiffness k of the spring in N⋅m/rad.
  /// @throws std::exception if `stiffness` is negative.
  RevoluteSpring(const RevoluteJoint<T>& joint, double nominal_angle,
                 double stiffness);

  ~RevoluteSpring() override;

  /// Returns the joint associated with this spring.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  const RevoluteJoint<T>& joint() const;

  /// Returns the default spring reference angle θ₀ in radians.
  double default_nominal_angle() const { return nominal_angle_; }

  /// Returns the default stiffness constant k in N⋅m/rad.
  double default_stiffness() const { return stiffness_; }

  /// Returns the state dependent nominal angle θ₀.
  /// @retval returns the nominal angle θ₀ in radians.
  const T& GetNominalAngle(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const {
    this->ValidateStateInstance(state);
    return state.revolute_spring_parameters(spring_parameter_slot_)
        .nominal_angle_radians;
  }

  /// Sets the value of the nominal angle θ₀ for this force element.
  ///
  /// The stiffness is read and written back unchanged: the two live in one
  /// record, and the store takes whole records.
  void SetNominalAngle(orvd::multibody_runtime::MultibodyStateInstance* state,
                       const T& nominal_angle) const {
    DRAKE_THROW_UNLESS(state != nullptr);
    this->ValidateStateInstance(*state);
    orvd::multibody_runtime::RevoluteSpringParameters parameters =
        state->revolute_spring_parameters(spring_parameter_slot_);
    parameters.nominal_angle_radians = nominal_angle;
    state->set_revolute_spring_parameters(spring_parameter_slot_, parameters);
  }

  /// Returns the state dependent stiffness coefficient k.
  /// @retval returns the stiffness k in N⋅m/rad.
  const T& GetStiffness(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const {
    this->ValidateStateInstance(state);
    return state.revolute_spring_parameters(spring_parameter_slot_).stiffness;
  }

  /// Sets the value of the linear stiffness coefficient k for this force
  /// element.
  /// @param[in] stiffness The stiffness value k with units N⋅m/rad.
  /// @throws std::exception if `stiffness` is negative. The store refuses it
  /// too; this check is kept so the refusal names the caller's argument.
  void SetStiffness(orvd::multibody_runtime::MultibodyStateInstance* state,
                    const T& stiffness) const {
    DRAKE_THROW_UNLESS(state != nullptr);
    this->ValidateStateInstance(*state);
    DRAKE_THROW_UNLESS(stiffness >= 0);
    orvd::multibody_runtime::RevoluteSpringParameters parameters =
        state->revolute_spring_parameters(spring_parameter_slot_);
    parameters.stiffness = stiffness;
    state->set_revolute_spring_parameters(spring_parameter_slot_, parameters);
  }

  T DoCalcPotentialEnergy(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  T DoCalcConservativePower(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  T DoCalcNonConservativePower(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

 protected:
  void DoCalcAndAddForceContribution(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<T>> DoShallowClone() const override;

 private:
  // Implementation for ForceElement::DoAssignForceElementParameterSlots().
  void DoAssignForceElementParameterSlots(
      orvd::rigid_multibody_tree::internal::MultibodyParameterSlotAllocator*
          allocator) final {
    spring_parameter_slot_ = allocator->AllocateRevoluteSpringSlot();
  }

  // Implementation for ForceElement::DoWriteDefaultForceElementParameters().
  void DoWriteDefaultForceElementParameters(
      orvd::multibody_runtime::MultibodyStateInstance* state) const final {
    orvd::multibody_runtime::RevoluteSpringParameters parameters;
    parameters.stiffness = stiffness_;
    parameters.nominal_angle_radians = nominal_angle_;
    state->set_revolute_spring_parameters(spring_parameter_slot_, parameters);
  }

  // Private constructor used by DoShallowClone(), which cannot use the public
  // constructor because it has no joint reference to hand it.
  RevoluteSpring(ModelInstanceIndex model_instance, JointIndex joint_index,
                 double nominal_angle, double stiffness);

  const JointIndex joint_index_;
  double nominal_angle_{};
  double stiffness_{};
  int spring_parameter_slot_{
      orvd::rigid_multibody_tree::internal::kUnassignedParameterSlot};
};

}  // namespace multibody
}  // namespace drake

extern template class drake::multibody::RevoluteSpring<double>;
