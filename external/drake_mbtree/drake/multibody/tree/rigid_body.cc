#include "drake/multibody/tree/rigid_body.h"

#include <memory>

#include "drake/multibody/tree/model_instance.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context.h"

namespace drake {
namespace multibody {

// RigidBodyFrame (a.k.a. LinkFrame) function definitions.

template <typename T>
RigidBodyFrame<T>::~RigidBodyFrame() = default;

template <typename T>
std::unique_ptr<Frame<T>> RigidBodyFrame<T>::DoShallowClone() const {
  // RigidBodyFrame's constructor cannot be called from std::make_unique since
  // it is private and therefore we use "new".
  return std::unique_ptr<RigidBodyFrame<T>>(
      new RigidBodyFrame<T>(this->body()));
}

// RigidBody function definitions.

template <typename T>
RigidBody<T>::~RigidBody() = default;

template <typename T>
ScopedName RigidBody<T>::scoped_name() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double>& M)
    : MultibodyElement<T>(default_model_instance()),
      name_(internal::DeprecateWhenEmptyName(body_name, "RigidBody")),
      link_frame_(*this),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        ModelInstanceIndex model_instance,
                        const SpatialInertia<double>& M)
    : MultibodyElement<T>(model_instance),
      name_(internal::DeprecateWhenEmptyName(body_name, "RigidBody")),
      link_frame_(*this),
      default_spatial_inertia_(M) {}

template <typename T>
void RigidBody<T>::SetCenterOfMassInBodyFrameNoModifyInertia(
    orvd::multibody_runtime::MultibodyStateInstance* state,
    const Vector3<T>& center_of_mass_position) const {
  DRAKE_THROW_UNLESS(state != nullptr);
  this->ValidateStateInstance(*state);
  DRAKE_THROW_UNLESS(
      inertia_parameter_slot_ !=
      orvd::rigid_multibody_tree::internal::kUnassignedParameterSlot);
  orvd::multibody_runtime::RigidBodyInertiaParameters parameters =
      state->rigid_body_inertia_parameters(inertia_parameter_slot_);
  parameters.center_of_mass_in_body_frame = center_of_mass_position;
  state->set_rigid_body_inertia_parameters(inertia_parameter_slot_, parameters);
}

template <typename T>
void RigidBody<T>::SetUnitInertiaAboutBodyOrigin(
    orvd::multibody_runtime::MultibodyStateInstance* state,
    const UnitInertia<T>& G_BBo_B) const {
  DRAKE_THROW_UNLESS(state != nullptr);
  this->ValidateStateInstance(*state);
  DRAKE_THROW_UNLESS(
      inertia_parameter_slot_ !=
      orvd::rigid_multibody_tree::internal::kUnassignedParameterSlot);
  orvd::multibody_runtime::RigidBodyInertiaParameters parameters =
      state->rigid_body_inertia_parameters(inertia_parameter_slot_);
  parameters.unit_inertia_moments = G_BBo_B.get_moments();
  parameters.unit_inertia_products = G_BBo_B.get_products();
  state->set_rigid_body_inertia_parameters(inertia_parameter_slot_, parameters);
}

template <typename T>
void RigidBody<T>::SetCenterOfMassInBodyFrameAndPreserveCentralInertia(
    orvd::multibody_runtime::MultibodyStateInstance* state,
    const Vector3<T>& center_of_mass_position) const {
  DRAKE_THROW_UNLESS(state != nullptr);
  this->ValidateStateInstance(*state);
  DRAKE_THROW_UNLESS(
      inertia_parameter_slot_ !=
      orvd::rigid_multibody_tree::internal::kUnassignedParameterSlot);

  // Get pi_BoBcm_B position from Bo to Bcm before Bcm changes location, and
  // Gi_BBo_B (B's initial unit inertia about Bo, before Bcm changes).
  orvd::multibody_runtime::RigidBodyInertiaParameters parameters =
      state->rigid_body_inertia_parameters(inertia_parameter_slot_);
  const SpatialInertia<T> Mi_BBo_B =
      orvd::rigid_multibody_tree::internal::ToSpatialInertia(parameters);
  const Vector3<T>& pi_BoBcm_B = Mi_BBo_B.get_com();
  const UnitInertia<T>& Gi_BBo_B = Mi_BBo_B.get_unit_inertia();

  // Calculate Gf_BBo_B (B's final unit inertia about Bo, after Bcm changes).
  const Vector3<T>& pf_BoBcm_B = center_of_mass_position;  // Alias for clarity.
  const RotationalInertia<T> I_BBo_B = Gi_BBo_B.ShiftToThenAwayFromCenterOfMass(
      /* mass = */ 1, pi_BoBcm_B, pf_BoBcm_B);
  const UnitInertia<T> Gf_BBo_B = UnitInertia<T>(I_BBo_B);
  // Note: One way to conceptualize this calculation is that B's origin Bo moves
  // from its initial location Boi to an intermediate location Bof and it only
  // returns to its initial location Boi when the new centre of mass is stored.
  // Hint: Drawing a picture can help speed making sense of this.

  // One write, not two. Written separately, the store would briefly hold the
  // new unit inertia against the old centre of mass — a combination that need
  // not be physically realisable, so the first write could be refused outright
  // — and the inertia version would advance twice for one change.
  parameters.unit_inertia_moments = Gf_BBo_B.get_moments();
  parameters.unit_inertia_products = Gf_BBo_B.get_products();
  parameters.center_of_mass_in_body_frame = pf_BoBcm_B;
  state->set_rigid_body_inertia_parameters(inertia_parameter_slot_, parameters);
}

template <typename T>
void RigidBody<T>::AddInForce(const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
                              const Vector3<T>& p_BP_E,
                              const SpatialForce<T>& F_Bp_E,
                              const Frame<T>& frame_E,
                              MultibodyForces<T>* forces) const {
  DRAKE_THROW_UNLESS(forces != nullptr);
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_THROW_UNLESS(
      forces->CheckHasRightSizeForModel(this->get_parent_tree()));
  const math::RotationMatrix<T> R_WE =
      frame_E.CalcRotationMatrixInWorld(context);
  const Vector3<T> p_PB_W = -(R_WE * p_BP_E);
  const SpatialForce<T> F_Bo_W = (R_WE * F_Bp_E).Shift(p_PB_W);
  AddInForceInWorld(F_Bo_W, forces);
}

template <typename T>
Vector3<T> RigidBody<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context) const {
  const RigidBody<T>& body_B = *this;
  const Frame<T>& frame_B = body_B.body_frame();

  // Form frame_B's spatial velocity in the world frame W, expressed in W.
  const SpatialVelocity<T>& V_WBo_W =
      body_B.EvalSpatialVelocityInWorld(context);

  // Form v_WBcm_W, Bcm's translational velocity in frame W, expressed in W.
  const Vector3<T> p_BoBcm_B =
      CalcCenterOfMassInBodyFrame(context.state());
  const math::RotationMatrix<T> R_WB =
      frame_B.CalcRotationMatrixInWorld(context);
  const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;
  const Vector3<T> v_WBcm_W = V_WBo_W.Shift(p_BoBcm_W).translational();
  return v_WBcm_W;
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::RigidBodyFrame<double>;

template class drake::multibody::RigidBody<double>;
