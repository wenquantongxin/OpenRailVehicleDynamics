#include "system_continuous_state_backend.h"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <omp.h>

// The parallel Jacobian path is product code. Refuse toolchains that accept an
// OpenMP-looking link configuration while discarding pragma semantics.
#ifndef _OPENMP
#error "OpenMP compile semantics are required: _OPENMP is not defined"
#endif

#include "orvd/integrators/cvode_continuous_state_advancer.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"
#include "orvd/track_geometry/track_station_projection.h"

#include "bdf_integration_access.h"
#include "dense_finite_difference_jacobian_provider.h"
#include "radau5_continuous_state_advancer.h"

namespace orvd::integrators::internal {
namespace {

constexpr int kMinimumParallelJacobianWorkerCount = 4;
constexpr int kIntermediateParallelJacobianWorkerCount = 8;
constexpr int kMaximumParallelJacobianWorkerCount = 16;

// Observes only the production bridge's formal recoverability decision. The
// count is kept behind the source-private system access seam so a real vehicle
// qualification can prove that a projection-window miss occurred without
// adding a test hook to either numerical backend or the installed API.
class RecoverableFailureObservingRhs final : public ContinuousStateRhs {
   public:
    explicit RecoverableFailureObservingRhs(SystemRhsBridge& bridge)
        : bridge_(&bridge) {}

    [[nodiscard]] int continuous_state_size() const override {
        return bridge_->continuous_state_size();
    }

    void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        Eigen::Ref<Eigen::VectorXd> state_time_derivatives) override {
        bridge_->CalcTimeDerivatives(time_seconds, continuous_state,
                                     state_time_derivatives);
    }

    [[nodiscard]] bool IsRecoverableFailure(
        const std::exception_ptr& failure) const noexcept override {
        const bool recoverable = bridge_->IsRecoverableFailure(failure);
        if (!recoverable) {
            return false;
        }
        try {
            std::rethrow_exception(failure);
        } catch (const track_geometry::TrackStationProjectionWindowMiss&) {
            if (recoverable_projection_window_miss_classification_count_ !=
                std::numeric_limits<std::uint64_t>::max()) {
                ++recoverable_projection_window_miss_classification_count_;
            }
        } catch (...) {
            // The bridge owns the recoverability policy. A future recoverable
            // exception remains recoverable without being mislabeled as a
            // projection-window miss by this narrow diagnostic.
        }
        return true;
    }

    [[nodiscard]] std::uint64_t
    recoverable_projection_window_miss_classification_count() const noexcept {
        return recoverable_projection_window_miss_classification_count_;
    }

   private:
    SystemRhsBridge* bridge_;
    mutable std::uint64_t
        recoverable_projection_window_miss_classification_count_{};
};

[[nodiscard]] bool UsesCvode(
    SystemContinuousStateIntegrationRecipe recipe) {
    switch (recipe) {
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf2:
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf5:
            return true;
        case SystemContinuousStateIntegrationRecipe::kRadau5:
            return false;
    }
    throw std::invalid_argument(
        "system integration backend: unsupported recipe");
}

[[nodiscard]] int ResolveParallelJacobianWorkerCount(
    const system_assembly::SystemInstance& system) {
    if (system.contact_force_plan() == nullptr || omp_get_dynamic() != 0) {
        return 0;
    }
    const int maximum_threads = omp_get_max_threads();
    if (maximum_threads >= kMaximumParallelJacobianWorkerCount) {
        return kMaximumParallelJacobianWorkerCount;
    }
    if (maximum_threads >= kIntermediateParallelJacobianWorkerCount) {
        return kIntermediateParallelJacobianWorkerCount;
    }
    if (maximum_threads >= kMinimumParallelJacobianWorkerCount) {
        return kMinimumParallelJacobianWorkerCount;
    }
    return 0;
}

class SystemDenseFiniteDifferenceJacobian final
    : public DenseFiniteDifferenceJacobianProvider {
   public:
    SystemDenseFiniteDifferenceJacobian(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& source_context,
        int worker_count,
        NoCallTimeAppliedForces no_call_time_applied_forces)
        : system_(&system),
          source_context_(&source_context),
          state_size_(system.continuous_state_size()),
          worker_count_(worker_count),
          failures_(static_cast<std::size_t>(state_size_)),
          attempted_(static_cast<std::size_t>(state_size_)) {
        if (worker_count_ != kMinimumParallelJacobianWorkerCount &&
            worker_count_ != kIntermediateParallelJacobianWorkerCount &&
            worker_count_ != kMaximumParallelJacobianWorkerCount) {
            throw std::invalid_argument(
                "system dense Jacobian: worker count must be four, eight, "
                "or sixteen");
        }

        Eigen::VectorXd initial_state(state_size_);
        system_->CopyContinuousState(*source_context_, initial_state);
        workers_.reserve(static_cast<std::size_t>(worker_count_));
        for (int ordinal = 0; ordinal < worker_count_; ++ordinal) {
            auto worker = std::make_unique<Worker>();
            worker->context = system_->CreateDefaultRuntimeContext(
                source_context_->time_seconds());
            system_->SetTimeContinuousStateAndWheelRailProjectionHints(
                *worker->context, source_context_->time_seconds(),
                initial_state,
                source_context_
                    ->wheel_rail_projection_station_hints_meters());
            worker->rhs = std::make_unique<SystemRhsBridge>(
                *system_, plan, *worker->context,
                no_call_time_applied_forces);
            worker->rhs->SynchronizeContextLocalDataFrom(*source_context_);
            worker->state.resize(state_size_);
            worker->derivatives.resize(state_size_);
            workers_.push_back(std::move(worker));
        }
    }

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return state_size_;
    }

    [[nodiscard]] int requested_worker_count() const noexcept override {
        return worker_count_;
    }

    [[nodiscard]] DenseFiniteDifferenceJacobianBatchResult
    CalcPerturbedDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        const Eigen::Ref<const Eigen::VectorXd>& increments,
        Eigen::MatrixXd& perturbed_derivatives) override {
        if (continuous_state.size() != state_size_ ||
            increments.size() != state_size_ ||
            perturbed_derivatives.rows() != state_size_ ||
            perturbed_derivatives.cols() != state_size_) {
            throw std::invalid_argument(
                "system dense Jacobian: callback storage has the wrong "
                "shape");
        }

        const auto projection_hints =
            source_context_
                ->wheel_rail_projection_station_hints_meters();
        for (const auto& worker : workers_) {
            system_->SetTimeContinuousStateAndWheelRailProjectionHints(
                *worker->context, time_seconds, continuous_state,
                projection_hints);
        }
        std::fill(failures_.begin(), failures_.end(), nullptr);
        std::fill(attempted_.begin(), attempted_.end(), 0U);

#pragma omp parallel num_threads(worker_count_)
        {
            Worker& worker = *workers_[static_cast<std::size_t>(
                omp_get_thread_num())];
#pragma omp for schedule(dynamic, 1)
            for (int column = 0; column < state_size_; ++column) {
                try {
                    worker.state = continuous_state;
                    worker.state[column] += increments[column];
                    system_assembly::internal::
                        SeedExactWheelRailContactEvaluationCaches(
                            *source_context_, *worker.context);
                    attempted_[static_cast<std::size_t>(column)] = 1U;
                    worker.rhs->CalcTimeDerivatives(
                        time_seconds, worker.state, worker.derivatives);
                    perturbed_derivatives.col(column) = worker.derivatives;
                } catch (...) {
                    failures_[static_cast<std::size_t>(column)] =
                        std::current_exception();
                }
            }
        }

        DenseFiniteDifferenceJacobianBatchResult result;
        for (int column = 0; column < state_size_; ++column) {
            result.attempted_right_hand_side_evaluation_count +=
                attempted_[static_cast<std::size_t>(column)] != 0U ? 1 : 0;
            if (result.lowest_column_failure == nullptr &&
                failures_[static_cast<std::size_t>(column)] != nullptr) {
                result.lowest_column_failure =
                    failures_[static_cast<std::size_t>(column)];
            }
        }
        return result;
    }

    [[nodiscard]] bool IsRecoverableFailure(
        const std::exception_ptr& failure) const noexcept override {
        return workers_.front()->rhs->IsRecoverableFailure(failure);
    }

    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& source_context) {
        for (const auto& worker : workers_) {
            worker->rhs->SynchronizeContextLocalDataFrom(source_context);
        }
    }

   private:
    struct Worker final {
        // Declared before rhs because the bridge borrows this context.
        std::unique_ptr<system_assembly::SystemRuntimeContext> context;
        std::unique_ptr<SystemRhsBridge> rhs;
        Eigen::VectorXd state;
        Eigen::VectorXd derivatives;
    };

    const system_assembly::SystemInstance* system_;
    system_assembly::SystemRuntimeContext* source_context_;
    int state_size_;
    int worker_count_;
    std::vector<std::unique_ptr<Worker>> workers_;
    std::vector<std::exception_ptr> failures_;
    std::vector<unsigned char> attempted_;
};

struct CvodeBdf2Runtime final {
    std::unique_ptr<CvodeContinuousStateAdvancer> advancer;
};

struct CvodeBdf5Runtime final {
    std::unique_ptr<CvodeContinuousStateAdvancer> advancer;
};

struct Radau5Runtime final {
    std::unique_ptr<Radau5ContinuousStateAdvancer> advancer;
};

using ConcreteRuntime =
    std::variant<CvodeBdf2Runtime, CvodeBdf5Runtime, Radau5Runtime>;

[[nodiscard]] ContinuousStateAdvancer& RuntimeAdvancer(
    ConcreteRuntime& runtime) {
    return std::visit(
        [](auto& concrete) -> ContinuousStateAdvancer& {
            return *concrete.advancer;
        },
        runtime);
}

[[nodiscard]] const ContinuousStateAdvancer& RuntimeAdvancer(
    const ConcreteRuntime& runtime) {
    return std::visit(
        [](const auto& concrete) -> const ContinuousStateAdvancer& {
            return *concrete.advancer;
        },
        runtime);
}

[[nodiscard]] SystemContinuousStateIntegrationRecipe RuntimeRecipe(
    const ConcreteRuntime& runtime) noexcept {
    return std::visit(
        [](const auto& concrete) noexcept {
            using Runtime = std::decay_t<decltype(concrete)>;
            if constexpr (std::is_same_v<Runtime, CvodeBdf2Runtime>) {
                return SystemContinuousStateIntegrationRecipe::kCvodeBdf2;
            } else if constexpr (std::is_same_v<Runtime,
                                                CvodeBdf5Runtime>) {
                return SystemContinuousStateIntegrationRecipe::kCvodeBdf5;
            } else {
                static_assert(std::is_same_v<Runtime, Radau5Runtime>);
                return SystemContinuousStateIntegrationRecipe::kRadau5;
            }
        },
        runtime);
}

}  // namespace

class SystemContinuousStateBackend::Implementation final {
   public:
    Implementation(
        SystemContinuousStateIntegrationRecipe recipe,
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& candidate_context,
        const system_assembly::SystemRuntimeContext& accepted_context,
        const Eigen::VectorXd& initial_continuous_state,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces)
        : rhs_bridge_(system, plan, candidate_context,
                      no_call_time_applied_forces),
          radau_rhs_observer_(rhs_bridge_) {
        rhs_bridge_.SynchronizeContextLocalDataFrom(accepted_context);
        if (UsesCvode(recipe)) {
            const int worker_count =
                ResolveParallelJacobianWorkerCount(system);
            if (worker_count != 0) {
                dense_jacobian_ =
                    std::make_unique<SystemDenseFiniteDifferenceJacobian>(
                        system, plan, candidate_context, worker_count,
                        no_call_time_applied_forces);
            }
        }
        runtime_.emplace(MakeRuntime(
            recipe, accepted_context.time_seconds(), initial_continuous_state,
            std::move(tolerances)));
    }

    [[nodiscard]] ContinuousStateAdvancer& advancer() {
        return RuntimeAdvancer(Runtime());
    }

    [[nodiscard]] const ContinuousStateAdvancer& advancer() const {
        return RuntimeAdvancer(Runtime());
    }

    [[nodiscard]] SystemContinuousStateIntegrationRecipe configured_recipe()
        const noexcept {
        return RuntimeRecipe(*runtime_);
    }

    [[nodiscard]] std::uint64_t
    recoverable_projection_window_miss_classification_count() const noexcept {
        return radau_rhs_observer_
            .recoverable_projection_window_miss_classification_count();
    }

    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted_context) {
        rhs_bridge_.SynchronizeContextLocalDataFrom(accepted_context);
        if (dense_jacobian_ != nullptr) {
            dense_jacobian_->SynchronizeContextLocalDataFrom(
                accepted_context);
        }
    }

    void NotifyAcceptedProjectionHistoryChange() {
        std::visit(
            [](auto& concrete) {
                using Runtime = std::decay_t<decltype(concrete)>;
                if constexpr (std::is_same_v<Runtime, Radau5Runtime>) {
                    concrete.advancer
                        ->InvalidateLinearizationAfterNumericalRhsHistoryChange();
                }
            },
            Runtime());
    }

   private:
    [[nodiscard]] ConcreteRuntime MakeRuntime(
        SystemContinuousStateIntegrationRecipe recipe,
        double initial_time_seconds,
        const Eigen::VectorXd& initial_continuous_state,
        ContinuousStateErrorTolerances tolerances) {
        switch (recipe) {
            case SystemContinuousStateIntegrationRecipe::kCvodeBdf2: {
                auto advancer =
                    std::make_unique<CvodeContinuousStateAdvancer>(
                        rhs_bridge_, initial_time_seconds,
                        initial_continuous_state,
                        std::move(tolerances));
                ConfigureAndVerifyCvode(*advancer, 2);
                return CvodeBdf2Runtime{std::move(advancer)};
            }
            case SystemContinuousStateIntegrationRecipe::kCvodeBdf5: {
                auto advancer = BdfIntegrationAccess::
                    MakeFifthOrderCvodeContinuousStateAdvancer(
                        rhs_bridge_, initial_time_seconds,
                        initial_continuous_state,
                        std::move(tolerances));
                ConfigureAndVerifyCvode(*advancer, 5);
                return CvodeBdf5Runtime{std::move(advancer)};
            }
            case SystemContinuousStateIntegrationRecipe::kRadau5:
                if (dense_jacobian_ != nullptr) {
                    throw std::logic_error(
                        "system integration backend: Radau5 cannot borrow "
                        "the CVODE Jacobian provider");
                }
                return Radau5Runtime{
                    std::make_unique<Radau5ContinuousStateAdvancer>(
                        radau_rhs_observer_, initial_time_seconds,
                        initial_continuous_state,
                        std::move(tolerances))};
        }
        throw std::invalid_argument(
            "system integration backend: unsupported recipe");
    }

    void ConfigureAndVerifyCvode(CvodeContinuousStateAdvancer& advancer,
                                 int expected_maximum_bdf_order) {
        if (dense_jacobian_ != nullptr) {
            DenseFiniteDifferenceJacobianRegistration::Attach(
                advancer, *dense_jacobian_);
        }
        if (BdfIntegrationAccess::ConfiguredMaximumBdfOrder(advancer) !=
            expected_maximum_bdf_order) {
            throw std::logic_error(
                "system integration backend: CVODE recipe identity "
                "mismatch");
        }
    }

    [[nodiscard]] ConcreteRuntime& Runtime() {
        if (!runtime_.has_value()) {
            throw std::logic_error(
                "system integration backend: runtime is not initialized");
        }
        return *runtime_;
    }

    [[nodiscard]] const ConcreteRuntime& Runtime() const {
        if (!runtime_.has_value()) {
            throw std::logic_error(
                "system integration backend: runtime is not initialized");
        }
        return *runtime_;
    }

    // Declaration order is intentional: the concrete advancer is destroyed
    // before the optional Jacobian provider, observing RHS and production
    // bridge it may borrow.
    SystemRhsBridge rhs_bridge_;
    RecoverableFailureObservingRhs radau_rhs_observer_;
    std::unique_ptr<SystemDenseFiniteDifferenceJacobian> dense_jacobian_;
    std::optional<ConcreteRuntime> runtime_;
};

SystemContinuousStateBackend::SystemContinuousStateBackend(
    SystemContinuousStateIntegrationRecipe recipe,
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& candidate_context,
    const system_assembly::SystemRuntimeContext& accepted_context,
    const Eigen::VectorXd& initial_continuous_state,
    ContinuousStateErrorTolerances tolerances,
    NoCallTimeAppliedForces no_call_time_applied_forces)
    : implementation_(std::make_unique<Implementation>(
          recipe, system, plan, candidate_context, accepted_context,
          initial_continuous_state, std::move(tolerances),
          no_call_time_applied_forces)) {}

SystemContinuousStateBackend::~SystemContinuousStateBackend() = default;

ContinuousStateAdvancer& SystemContinuousStateBackend::advancer() {
    return implementation_->advancer();
}

const ContinuousStateAdvancer& SystemContinuousStateBackend::advancer()
    const {
    return implementation_->advancer();
}

SystemContinuousStateIntegrationRecipe
SystemContinuousStateBackend::configured_recipe() const noexcept {
    return implementation_->configured_recipe();
}

std::uint64_t SystemContinuousStateBackend::
    recoverable_projection_window_miss_classification_count() const noexcept {
    return implementation_
        ->recoverable_projection_window_miss_classification_count();
}

void SystemContinuousStateBackend::SynchronizeContextLocalDataFrom(
    const system_assembly::SystemRuntimeContext& accepted_context) {
    implementation_->SynchronizeContextLocalDataFrom(accepted_context);
}

void SystemContinuousStateBackend::NotifyAcceptedProjectionHistoryChange() {
    implementation_->NotifyAcceptedProjectionHistoryChange();
}

}  // namespace orvd::integrators::internal
