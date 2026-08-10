#pragma once

/// @file
/// A frozen executable system and its uniquely owned runtime state.

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_applied_forces.h"

namespace orvd::multibody_model {
class ForwardDynamicsWorkspace;
class MultibodyEvaluationContext;
class MultibodyModel;
}

namespace orvd::forces {
class IndependentWheelActiveTorquePlan;
class VehicleForcePlan;
class WheelRailContactForcePlan;
class WheelRailContactForceWorkspace;
}

namespace orvd::system_assembly {

class CompiledSystemPlan;
class SystemAssemblyDescription;
class SystemInstance;

namespace internal {
using SystemIdentity = std::uint64_t;
}

/// The stable identity of the first multibody component in one system.
class MultibodyComponentIndex {
   public:
    [[nodiscard]] bool operator==(const MultibodyComponentIndex&) const =
        default;

   private:
    friend class SystemInstance;
    MultibodyComponentIndex(internal::SystemIdentity system_identity,
                            int ordinal)
        : system_identity_(system_identity), ordinal_(ordinal) {}

    internal::SystemIdentity system_identity_{0};
    int ordinal_{-1};
};

/// A translational spring-damper's context-local nominal-force slot.
///
/// Resolved once by name, then carried as a system-bound typed index.  It is
/// neither an array subscript nor interchangeable with another force family.
class TranslationalSpringDamperIndex {
   public:
    [[nodiscard]] bool operator==(
        const TranslationalSpringDamperIndex&) const = default;

   private:
    friend class SystemInstance;
    TranslationalSpringDamperIndex(internal::SystemIdentity system_identity,
                                   int ordinal)
        : system_identity_(system_identity), ordinal_(ordinal) {}

    internal::SystemIdentity system_identity_{0};
    int ordinal_{-1};
};

/// One series spring-viscous damper's force-state component.
class SeriesSpringViscousDamperIndex {
   public:
    [[nodiscard]] bool operator==(
        const SeriesSpringViscousDamperIndex&) const = default;

   private:
    friend class SystemInstance;
    SeriesSpringViscousDamperIndex(internal::SystemIdentity system_identity,
                                   int ordinal)
        : system_identity_(system_identity), ordinal_(ordinal) {}

    internal::SystemIdentity system_identity_{0};
    int ordinal_{-1};
};

/// One frozen block in the whole-system continuous vector.
class SystemContinuousStateRange {
   public:
    [[nodiscard]] int start() const { return start_; }
    [[nodiscard]] int size() const { return size_; }
    [[nodiscard]] bool operator==(const SystemContinuousStateRange&) const =
        default;

   private:
    friend class SystemInstance;
    SystemContinuousStateRange(int start, int size)
        : start_(start), size_(size) {}

    int start_{};
    int size_{};
};

/// The root runtime context for one `SystemInstance`.
///
/// It owns exactly one time value, one multibody evaluation context, the
/// force-element state that belongs to no body, and all admitted context-local
/// data including held active torques. Its owner decides whether a particular
/// instance is the publicly accepted context or an integrator trial context.
/// The multibody context in turn owns the G21 state store; this class does not
/// mirror q or v. The model-bound forward-dynamics workspace and the scratch
/// the plan needs to reach that facade are created beside it once and reused,
/// so that two contexts of one system can be evaluated without sharing a
/// buffer.
class SystemRuntimeContext {
   public:
    ~SystemRuntimeContext();

    SystemRuntimeContext(const SystemRuntimeContext&) = delete;
    SystemRuntimeContext& operator=(const SystemRuntimeContext&) = delete;
    SystemRuntimeContext(SystemRuntimeContext&&) = delete;
    SystemRuntimeContext& operator=(SystemRuntimeContext&&) = delete;

    [[nodiscard]] const Eigen::VectorXd& generalized_positions() const;
    [[nodiscard]] const Eigen::VectorXd& generalized_velocities() const;

    /// The force of each series spring-viscous damper, in newtons, acting on
    /// that element's reference end along its stated axis.  Empty when the
    /// system declares no such element.
    [[nodiscard]] const Eigen::VectorXd& series_spring_damper_forces() const {
        return series_spring_damper_forces_;
    }

    /// The nominal force each translational element is preloaded with, three
    /// components per element, in that element's reference frame and acting on
    /// its reference end.  Zero until a resolved start-up state states them.
    [[nodiscard]] const Eigen::VectorXd& nominal_forces() const {
        return nominal_forces_;
    }

    /// The already-conditioned wheel-side torques held between controller
    /// events.  This fixed-layout context-local input is not part of [q;v;z].
    [[nodiscard]] const Eigen::VectorXd&
    held_independent_wheel_active_torques_newton_metres() const {
        return held_independent_wheel_active_torques_newton_metres_;
    }

    [[nodiscard]] double time_seconds() const { return time_seconds_; }

    /// The local projection branch currently held by this context for each
    /// unique wheel-rail carrier. It is accepted history in a public context
    /// and candidate history in the integrator's trial context. Empty when this
    /// system has no contact force plan. The values are numerical history, not
    /// continuous physical state.
    [[nodiscard]] std::span<const double>
    wheel_rail_projection_station_hints_meters() const {
        return wheel_rail_projection_station_hints_meters_;
    }

   private:
    friend class SystemInstance;
    friend class CompiledSystemPlan;
    SystemRuntimeContext(internal::SystemIdentity issuer,
                         const multibody_model::MultibodyModel& model,
                         double initial_time_seconds,
                         int series_spring_damper_force_state_count,
                         int nominal_force_component_count,
                         int held_active_torque_count,
                         int body_wrench_count,
                         const forces::WheelRailContactForcePlan*
                             contact_force_plan);

    internal::SystemIdentity issuer_;
    double time_seconds_;
    std::unique_ptr<multibody_model::MultibodyEvaluationContext>
        multibody_context_;
    std::unique_ptr<multibody_model::ForwardDynamicsWorkspace>
        forward_dynamics_workspace_;
    Eigen::VectorXd series_spring_damper_forces_;
    /// Context-local, written by start-up assembly and read thereafter.  It is
    /// not part of the continuous state and nothing integrates it; the system
    /// bridge explicitly copies it into its trial context so RHS evaluations
    /// nevertheless see the accepted value.
    Eigen::VectorXd nominal_forces_;
    /// Context-local zero-order-held input. G73 only stores and applies these
    /// values; command conditioning and event updates belong to later goals.
    Eigen::VectorXd held_independent_wheel_active_torques_newton_metres_;
    /// The wrenches the force plan produces, sized once. It belongs to the
    /// context so that two contexts of one system never write one buffer.
    std::vector<multibody_model::AppliedBodyWrench> body_wrenches_;
    std::unique_ptr<forces::WheelRailContactForceWorkspace>
        contact_force_workspace_;
    std::vector<double> wheel_rail_projection_station_hints_meters_;
    Eigen::VectorXd series_force_derivatives_;
    /// Scratch for the multibody facade's own [qdot; vdot], which is shorter
    /// than this system's state.  It belongs to the context and not to the
    /// frozen plan, so two contexts never write the same buffer.
    Eigen::VectorXd multibody_state_time_derivatives_;
};

/// A short-lived, non-owning view of the executable multibody component.
///
/// The view intentionally does not forward the model's calculation API.  Its
/// three references are the existing model, its one authoritative runtime
/// state, and the preallocated call workspace.
class MultibodyComponentView {
   public:
    [[nodiscard]] const multibody_model::MultibodyModel& model() const;
    [[nodiscard]] multibody_model::MultibodyEvaluationContext& context() const;
    [[nodiscard]] multibody_model::ForwardDynamicsWorkspace&
    forward_dynamics_workspace() const;

   private:
    friend class SystemInstance;
    MultibodyComponentView(
        const multibody_model::MultibodyModel& model,
        multibody_model::MultibodyEvaluationContext& context,
        multibody_model::ForwardDynamicsWorkspace& forward_dynamics_workspace)
        : model_(&model),
          context_(&context),
          forward_dynamics_workspace_(&forward_dynamics_workspace) {}

    const multibody_model::MultibodyModel* model_;
    multibody_model::MultibodyEvaluationContext* context_;
    multibody_model::ForwardDynamicsWorkspace* forward_dynamics_workspace_;
};

/// The frozen executable form of a `SystemAssemblyDescription`.
class SystemInstance {
   public:
    explicit SystemInstance(const SystemAssemblyDescription& description);

    SystemInstance(const SystemInstance&) = delete;
    SystemInstance& operator=(const SystemInstance&) = delete;
    SystemInstance(SystemInstance&&) = delete;
    SystemInstance& operator=(SystemInstance&&) = delete;

    /// The vehicle's force elements, or null when the system carries none.
    [[nodiscard]] const forces::VehicleForcePlan* force_plan() const {
        return force_plan_;
    }

    [[nodiscard]] const forces::WheelRailContactForcePlan* contact_force_plan()
        const {
        return contact_force_plan_;
    }

    [[nodiscard]] const forces::IndependentWheelActiveTorquePlan*
    active_torque_plan() const {
        return active_torque_plan_;
    }

    [[nodiscard]] int vehicle_body_wrench_count() const {
        return vehicle_body_wrench_count_;
    }
    [[nodiscard]] int contact_body_wrench_count() const {
        return contact_body_wrench_count_;
    }
    [[nodiscard]] int active_torque_body_wrench_count() const {
        return active_torque_body_wrench_count_;
    }
    [[nodiscard]] int held_active_torque_count() const {
        return held_active_torque_count_;
    }

    [[nodiscard]] MultibodyComponentIndex multibody_component() const;
    [[nodiscard]] SystemContinuousStateRange generalized_positions_state_range()
        const;
    [[nodiscard]] SystemContinuousStateRange generalized_velocities_state_range()
        const;

    /// Where the series spring-viscous dampers' force state sits in the
    /// contiguous vector.  A size of zero says the system declared none.
    [[nodiscard]] SystemContinuousStateRange
    series_spring_damper_force_state_range() const;

    /// Resolves a force-element name once at setup time.  Evaluation remains
    /// entirely ordinal and performs no string lookup.
    [[nodiscard]] TranslationalSpringDamperIndex
    GetTranslationalSpringDamperIndexByName(std::string_view name) const;
    [[nodiscard]] SeriesSpringViscousDamperIndex
    GetSeriesSpringViscousDamperIndexByName(std::string_view name) const;

    /// The one-component state range belonging to a named series element.
    [[nodiscard]] SystemContinuousStateRange
    series_spring_damper_force_state_range(
        SeriesSpringViscousDamperIndex element) const;

    [[nodiscard]] int continuous_state_size() const;

    /// Creates the default physical state at an explicit finite time.
    [[nodiscard]] std::unique_ptr<SystemRuntimeContext>
    CreateDefaultRuntimeContext(double initial_time_seconds) const;

    /// Copies the frozen `[q; v; z]` continuous-state layout into `output`.
    /// `output` must already have `continuous_state_size()` entries.
    void CopyContinuousState(
        const SystemRuntimeContext& context,
        Eigen::Ref<Eigen::VectorXd> output) const;

    /// Replaces the frozen `[q; v; z]` state as one transaction.
    ///
    /// This is the contiguous-vector bridge used by ODE backends.  The q and v
    /// parts are mapped directly into the authoritative multibody state; no
    /// mirrored system state is kept.  The context's current time is retained.
    void SetContinuousState(
        SystemRuntimeContext& context,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state) const;

    /// Replaces the finite time and complete `[q; v; z]` state as one
    /// transaction.
    ///
    /// Every check precedes every write: the time and complete state are
    /// validated, including the force-state finiteness and the model's own
    /// conditions on q and v, which
    /// that facade validates before it writes anything of its own.  A refusal
    /// therefore leaves time, q, v and z all unchanged — including a refusal
    /// that only the force state earns, which is checked before q and v are
    /// touched.
    void SetTimeAndContinuousState(
        SystemRuntimeContext& context, double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state) const;

    /// Replaces time, `[q;v;z]`, and the wheel-rail projection history as one
    /// accepted transaction. Every check precedes every write.
    void SetTimeContinuousStateAndWheelRailProjectionHints(
        SystemRuntimeContext& context, double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        std::span<const double> projection_station_hints_meters) const;

    /// Reprojects every unique contact carrier at the state currently installed
    /// in `context`, using that context's latest accepted/candidate station as
    /// the local seed. All hints are replaced only after every carrier succeeds.
    /// No wheel contact kernel is evaluated.
    void UpdateWheelRailProjectionStationHints(
        SystemRuntimeContext& context) const;

    /// States the nominal force of one translational element in this context.
    ///
    /// A start-up assembly writes these once; they are read on every evaluation
    /// and never integrated.  Two contexts of one system hold their own.
    ///
    /// @throws std::invalid_argument if the context is foreign, the index is
    /// out of range, or the force is not finite; nothing is written.
    void SetNominalForce(
        SystemRuntimeContext& context,
        TranslationalSpringDamperIndex element,
        const Eigen::Vector3d&
            nominal_force_on_reference_end_in_reference_frame_newtons) const;

    /// Atomically replaces every named held independent-wheel active torque.
    ///
    /// The caller supplies the frozen plan order. Every entry is validated
    /// before any write; a foreign context, wrong count or non-finite value
    /// leaves the complete held vector unchanged.
    void SetHeldIndependentWheelActiveTorques(
        SystemRuntimeContext& context,
        std::span<const double> held_wheel_torques_newton_metres) const;

    /// Copies every currently admitted item of context-local data.
    ///
    /// These are joint damping, translational-element nominal forces and held
    /// independent-wheel active torques. Time, q, v, z, projection history and
    /// evaluation scratch are deliberately not copied. Both context identities
    /// are checked before any destination category is written.
    void CopyContextLocalData(const SystemRuntimeContext& source,
                              SystemRuntimeContext& destination) const;

    /// Resolves the stable component index to direct references.  No name or
    /// run-time type lookup occurs on this path.
    ///
    /// @throws std::invalid_argument if either argument belongs to another
    /// system instance.
    [[nodiscard]] MultibodyComponentView GetMultibodyComponentView(
        SystemRuntimeContext& context, MultibodyComponentIndex component) const;

   private:
    internal::SystemIdentity identity_;
    const multibody_model::MultibodyModel* multibody_model_;
    int generalized_position_count_{};
    int generalized_velocity_count_{};
    int series_spring_damper_force_state_count_{};
    int nominal_force_component_count_{};
    const forces::VehicleForcePlan* force_plan_{nullptr};
    const forces::WheelRailContactForcePlan* contact_force_plan_{nullptr};
    const forces::IndependentWheelActiveTorquePlan* active_torque_plan_{
        nullptr};
    int vehicle_body_wrench_count_{};
    int contact_body_wrench_count_{};
    int active_torque_body_wrench_count_{};
    int held_active_torque_count_{};
};

}  // namespace orvd::system_assembly
