#pragma once

/// @file
/// A frozen executable system and its uniquely owned runtime state.

#include <cstdint>
#include <memory>

#include <Eigen/Dense>

namespace orvd::multibody_model {
class ForwardDynamicsWorkspace;
class MultibodyEvaluationContext;
class MultibodyModel;
}

namespace orvd::system_assembly {

class SystemAssemblyDescription;
class SystemInstance;

namespace internal {
using SystemIdentity = std::uint64_t;
}

/// The stable identity of the first multibody component in one system.
class MultibodyComponentIndex {
   public:
    [[nodiscard]] bool is_valid() const { return system_identity_ != 0; }
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

/// One frozen block in the future whole-system continuous vector.
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
/// It owns exactly one multibody evaluation context.  That context in turn
/// owns the G21 state store; this class does not mirror q or v.  The model-bound
/// forward-dynamics workspace is created beside it once and reused.
class SystemRuntimeContext {
   public:
    ~SystemRuntimeContext();

    SystemRuntimeContext(const SystemRuntimeContext&) = delete;
    SystemRuntimeContext& operator=(const SystemRuntimeContext&) = delete;
    SystemRuntimeContext(SystemRuntimeContext&&) = delete;
    SystemRuntimeContext& operator=(SystemRuntimeContext&&) = delete;

    [[nodiscard]] const Eigen::VectorXd& generalized_positions() const;
    [[nodiscard]] const Eigen::VectorXd& generalized_velocities() const;

   private:
    friend class SystemInstance;
    SystemRuntimeContext(internal::SystemIdentity issuer,
                         const multibody_model::MultibodyModel& model);

    internal::SystemIdentity issuer_;
    std::unique_ptr<multibody_model::MultibodyEvaluationContext>
        multibody_context_;
    std::unique_ptr<multibody_model::ForwardDynamicsWorkspace>
        forward_dynamics_workspace_;
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

    [[nodiscard]] MultibodyComponentIndex multibody_component() const;
    [[nodiscard]] SystemContinuousStateRange generalized_positions_state_range()
        const;
    [[nodiscard]] SystemContinuousStateRange generalized_velocities_state_range()
        const;
    [[nodiscard]] int continuous_state_size() const;

    [[nodiscard]] std::unique_ptr<SystemRuntimeContext>
    CreateDefaultRuntimeContext() const;

    /// Copies the frozen `[q; v]` continuous-state layout into `output`.
    /// `output` must already have `continuous_state_size()` entries.
    void CopyContinuousState(
        const SystemRuntimeContext& context,
        Eigen::Ref<Eigen::VectorXd> output) const;

    /// Replaces the frozen `[q; v]` state as one transaction.
    ///
    /// This is the contiguous-vector bridge used by ODE backends.  The input
    /// is mapped directly into the authoritative multibody state; no mirrored
    /// system state is kept.
    void SetContinuousState(
        SystemRuntimeContext& context,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state) const;

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
};

}  // namespace orvd::system_assembly
