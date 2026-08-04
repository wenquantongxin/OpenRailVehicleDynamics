#pragma once

#include <span>
#include <vector>

#include <Eigen/Core>

#include "orvd/forces/vehicle_force_elements.h"
#include "orvd/multibody_model/multibody_model.h"

// The frozen set of force elements one vehicle carries, and the one place they
// are evaluated.
//
// The plan is built once and never changes. It holds resolved handles and
// constitutive constants, not names and not a reference to the record it was
// built from: a record that is edited afterwards has no way to reach a plan
// already compiled from it.
//
// The plan is also read-only at evaluation. Everything that varies — the state
// the series elements carry, the nominal force each translational element is
// preloaded with, and the wrenches produced — is passed in or out by the
// caller, so two contexts of one system can be evaluated without sharing a
// buffer. That is why this class has no scratch of its own.
//
// Each element is evaluated exactly once per call, in the order the arrays hold
// them: translational, then roll, then series, then clipped. The order is fixed
// so that a wrench buffer index means the same thing on every call.

namespace orvd::forces {

class VehicleForcePlan {
   public:
    // Builds the plan against `model`, which must be finalized and must outlive
    // the plan. Every frame handle must belong to that model.
    //
    // Throws std::invalid_argument if any handle is foreign or invalid, if an
    // element's two ends are the same frame, if a stiffness or damping is not
    // finite, if a series element's stiffness or damping is not positive — a
    // series element with either at zero has no time constant and is not this
    // family — or if a clipped damper's curve is not a strictly ascending run of
    // finite points starting at the origin.
    // Throws std::logic_error if the model is not finalized.
    VehicleForcePlan(
        const multibody_model::MultibodyModel& model,
        std::vector<TranslationalSpringDamper> translational_spring_dampers,
        std::vector<RollSpringDamperCouple> roll_spring_damper_couples,
        std::vector<SeriesSpringViscousDamper> series_spring_viscous_dampers,
        std::vector<SaturatedPiecewiseLinearDamper>
            saturated_piecewise_linear_dampers);

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

    /// The force a saturated piecewise linear damper produces at one relative
    /// velocity. Exposed because it is the whole constitutive content of that
    /// family and a gate has to be able to sample it without an assembled
    /// vehicle around it.
    [[nodiscard]] static double SaturatedPiecewiseLinearDamperForce(
        const SaturatedPiecewiseLinearDamper& damper,
        double relative_velocity_meters_per_second);

   private:
    const multibody_model::MultibodyModel* model_;
    std::vector<TranslationalSpringDamper> translational_;
    std::vector<RollSpringDamperCouple> roll_;
    std::vector<SeriesSpringViscousDamper> series_;
    std::vector<SaturatedPiecewiseLinearDamper> clipped_;
};

}  // namespace orvd::forces
