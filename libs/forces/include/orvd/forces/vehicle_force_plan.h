#pragma once

#include <span>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "orvd/forces/vehicle_force_elements.h"
#include "orvd/multibody_model/multibody_model.h"

// The frozen set of force elements one vehicle carries, and the one place they
// are evaluated.
//
// The plan is built once and never changes. It holds names for setup-time slot
// resolution, resolved frame handles and constitutive constants, but no
// reference to the record it was built from: a record that is edited afterwards
// has no way to reach a plan already compiled from it.
//
// The plan is also read-only at evaluation. Everything that varies — the state
// the series elements carry, the nominal force each translational element is
// preloaded with, and the wrenches produced — is passed in or out by the
// caller, so two contexts of one system can be evaluated without sharing a
// buffer. That is why this class has no scratch of its own.
//
// Each element is evaluated exactly once per call. Cross-family output order is
// deliberately not a public contract: the only consumer accumulates the whole
// typed plan, and startup lookup resolves names to typed slots rather than to a
// wrench-buffer position.

namespace orvd::system_assembly {
class SystemInstance;
}

namespace orvd::forces {

class VehicleForcePlan {
   public:
    // Builds the plan against `model`, which must be finalized and must outlive
    // the plan and every system compiled from it. Every frame handle must
    // belong to that model.
    //
    // Throws std::invalid_argument if an element name is empty or repeated, if
    // any handle is foreign or invalid, if an element's two ends are one frame
    // or one body, if an algebraic stiffness or damping is negative or not
    // finite, if a series element's stiffness or damping is not positive — a
    // series element with either at zero has no time constant and is not this
    // family — or if stiffness divided by damping is not finite. A clipped
    // damper curve must start at the origin, ascend strictly in velocity and
    // state finite non-negative force.
    // Throws std::invalid_argument if an endpoint is the world frame.
    // Throws std::logic_error if the model is not finalized.
    VehicleForcePlan(
        const multibody_model::MultibodyModel& model,
        std::vector<TranslationalSpringDamper> translational_spring_dampers,
        std::vector<RollSpringDamperCouple> roll_spring_damper_couples,
        std::vector<SeriesSpringViscousDamper> series_spring_viscous_dampers,
        std::vector<SaturatedPiecewiseLinearDamper>
            saturated_piecewise_linear_dampers);
    VehicleForcePlan(
        multibody_model::MultibodyModel&&,
        std::vector<TranslationalSpringDamper>,
        std::vector<RollSpringDamperCouple>,
        std::vector<SeriesSpringViscousDamper>,
        std::vector<SaturatedPiecewiseLinearDamper>) = delete;
    VehicleForcePlan(
        const multibody_model::MultibodyModel&&,
        std::vector<TranslationalSpringDamper>,
        std::vector<RollSpringDamperCouple>,
        std::vector<SeriesSpringViscousDamper>,
        std::vector<SaturatedPiecewiseLinearDamper>) = delete;

    VehicleForcePlan(const VehicleForcePlan&) = delete;
    VehicleForcePlan& operator=(const VehicleForcePlan&) = delete;
    VehicleForcePlan(VehicleForcePlan&&) = delete;
    VehicleForcePlan& operator=(VehicleForcePlan&&) = delete;

    [[nodiscard]] const multibody_model::MultibodyModel& model() const {
        return *model_;
    }

    [[nodiscard]] int translational_spring_damper_count() const {
        return static_cast<int>(translational_.size());
    }
    [[nodiscard]] int roll_spring_damper_couple_count() const {
        return static_cast<int>(roll_.size());
    }
    [[nodiscard]] int series_spring_viscous_damper_count() const {
        return static_cast<int>(series_.size());
    }
    [[nodiscard]] int saturated_piecewise_linear_damper_count() const {
        return static_cast<int>(clipped_.size());
    }

    /// One force component of continuous state per series element, and none for
    /// any other family. The number of state components is therefore a property
    /// of which constitutive families the vehicle has, not of its data.
    [[nodiscard]] int series_spring_damper_force_state_count() const {
        return static_cast<int>(series_.size());
    }

    /// A nominal force slot per translational element, three components each.
    [[nodiscard]] int nominal_force_component_count() const {
        return 3 * translational_spring_damper_count();
    }

    /// Two body wrenches per element, always both written.
    [[nodiscard]] int body_wrench_count() const {
        return 2 * (translational_spring_damper_count() +
                    roll_spring_damper_couple_count() +
                    series_spring_viscous_damper_count() +
                    saturated_piecewise_linear_damper_count());
    }

    /// Evaluates every element once against `context`.
    ///
    /// `series_forces` holds the current force state of each series element;
    /// `nominal_forces` holds three components per translational element, in the
    /// element's reference frame, acting on its reference end. Both are read,
    /// never written.
    ///
    /// `body_wrenches` receives `body_wrench_count()` entries and
    /// `series_force_derivatives` receives `series_spring_damper_force_state_count()`.
    ///
    /// Throws std::invalid_argument if the context is foreign or any span is
    /// the wrong length; nothing is written in that case.
    void CalcAppliedForces(
        const multibody_model::MultibodyEvaluationContext& context,
        const Eigen::Ref<const Eigen::VectorXd>& series_forces,
        const Eigen::Ref<const Eigen::VectorXd>& nominal_forces,
        std::span<multibody_model::AppliedBodyWrench> body_wrenches,
        Eigen::Ref<Eigen::VectorXd> series_force_derivatives) const;

    /// The force a saturated piecewise linear damper produces at one finite
    /// relative velocity. Exposed because it is the whole constitutive content
    /// of that family and a gate has to be able to sample it without an
    /// assembled vehicle around it.
    ///
    /// @throws std::invalid_argument if the raw damper does not satisfy the
    /// same curve contract enforced by the plan constructor, or if the query
    /// velocity is not finite.
    [[nodiscard]] static double SaturatedPiecewiseLinearDamperForce(
        const SaturatedPiecewiseLinearDamper& damper,
        double relative_velocity_meters_per_second);

   private:
    friend class system_assembly::SystemInstance;

    [[nodiscard]] int FindTranslationalSpringDamperOrdinal(
        std::string_view name) const;
    [[nodiscard]] int FindSeriesSpringViscousDamperOrdinal(
        std::string_view name) const;

    const multibody_model::MultibodyModel* model_;
    std::vector<TranslationalSpringDamper> translational_;
    std::vector<RollSpringDamperCouple> roll_;
    std::vector<SeriesSpringViscousDamper> series_;
    std::vector<SaturatedPiecewiseLinearDamper> clipped_;
};

}  // namespace orvd::forces
