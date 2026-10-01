#include "system_continuous_state_backend.h"

#include <algorithm>
#include <array>
#include <exception>
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

#include "cvode_continuous_state_advancer.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

#include "dense_finite_difference_jacobian_provider.h"
#include "radau5_continuous_state_advancer.h"
#include "newmark_continuous_state_advancer.h"
#include "zhai_continuous_state_advancer.h"
#include "system_coordinate_problem.h"

namespace orvd::integrators::internal {
namespace {

constexpr std::array<int, 5> kSupportedParallelJacobianWorkerCounts{
    4, 8, 12, 16, 32};

[[nodiscard]] constexpr int SelectParallelJacobianWorkerCount(
    int maximum_threads) noexcept {
    for (auto index = kSupportedParallelJacobianWorkerCounts.size(); index > 0;
         --index) {
        const int supported =
            kSupportedParallelJacobianWorkerCounts[index - 1];
        if (maximum_threads >= supported) {
            return supported;
        }
    }
    return 0;
}

static_assert(SelectParallelJacobianWorkerCount(3) == 0);
static_assert(SelectParallelJacobianWorkerCount(4) == 4);
static_assert(SelectParallelJacobianWorkerCount(7) == 4);
static_assert(SelectParallelJacobianWorkerCount(8) == 8);
static_assert(SelectParallelJacobianWorkerCount(11) == 8);
static_assert(SelectParallelJacobianWorkerCount(12) == 12);
static_assert(SelectParallelJacobianWorkerCount(15) == 12);
static_assert(SelectParallelJacobianWorkerCount(16) == 16);
static_assert(SelectParallelJacobianWorkerCount(31) == 16);
static_assert(SelectParallelJacobianWorkerCount(32) == 32);

[[nodiscard]] int ResolveParallelJacobianWorkerCount(
    const system_assembly::SystemInstance& system) {
    if (system.contact_force_plan() == nullptr || omp_get_dynamic() != 0) {
        return 0;
    }
    return SelectParallelJacobianWorkerCount(omp_get_max_threads());
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
        if (std::find(kSupportedParallelJacobianWorkerCounts.cbegin(),
                      kSupportedParallelJacobianWorkerCounts.cend(),
                      worker_count_) ==
            kSupportedParallelJacobianWorkerCounts.cend()) {
            throw std::invalid_argument(
                "system dense Jacobian: worker count must be four, eight, "
                "twelve, sixteen, or thirty-two");
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

template <class Configuration, int Order>
struct CvodeRuntime final {
    static constexpr std::string_view kIdentifier = Order == 2 ? "cvode_bdf2" : "cvode_bdf5";

    CvodeRuntime(const system_assembly::SystemInstance& system,
                 const system_assembly::CompiledSystemPlan& plan,
                 system_assembly::SystemRuntimeContext& candidate,
                 const system_assembly::SystemRuntimeContext& accepted,
                 const Eigen::VectorXd& initial, Configuration configuration,
                 NoCallTimeAppliedForces forces)
        : rhs(system, plan, candidate, forces) {
        rhs.SynchronizeContextLocalDataFrom(accepted);
        const int workers = ResolveParallelJacobianWorkerCount(system);
        if (workers != 0) {
            jacobian = std::make_unique<SystemDenseFiniteDifferenceJacobian>(
                system, plan, candidate, workers, forces);
        }
        advancer = std::make_unique<CvodeContinuousStateAdvancer>(
            rhs, accepted.time_seconds(), initial,
            std::move(configuration.tolerances),
            Order == 2 ? MaximumBdfOrder::kSecond : MaximumBdfOrder::kFifth);
        if (jacobian) {
            DenseFiniteDifferenceJacobianRegistration::Attach(*advancer, *jacobian);
        }
        if (advancer->configured_maximum_bdf_order() != Order) {
            throw std::logic_error("system integration backend: CVODE recipe identity mismatch");
        }
    }

    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted) {
        rhs.SynchronizeContextLocalDataFrom(accepted);
        if (jacobian) jacobian->SynchronizeContextLocalDataFrom(accepted);
    }
    void NotifyAcceptedProjectionHistoryChange() noexcept {}

    // Borrowers are destroyed first; the enclosing object never moves.
    SystemRhsBridge rhs;
    std::unique_ptr<SystemDenseFiniteDifferenceJacobian> jacobian;
    std::unique_ptr<CvodeContinuousStateAdvancer> advancer;
};
using CvodeBdf2Runtime = CvodeRuntime<CvodeBdf2Configuration, 2>;
using CvodeBdf5Runtime = CvodeRuntime<CvodeBdf5Configuration, 5>;

struct Radau5Runtime final {
    static constexpr std::string_view kIdentifier = "radau5";
    Radau5Runtime(const system_assembly::SystemInstance& system,
                  const system_assembly::CompiledSystemPlan& plan,
                  system_assembly::SystemRuntimeContext& candidate,
                  const system_assembly::SystemRuntimeContext& accepted,
                  const Eigen::VectorXd& initial, Radau5Configuration configuration,
                  NoCallTimeAppliedForces forces)
        : rhs(system, plan, candidate, forces) {
        rhs.SynchronizeContextLocalDataFrom(accepted);
        advancer = std::make_unique<Radau5ContinuousStateAdvancer>(
            rhs, accepted.time_seconds(), initial, std::move(configuration.tolerances));
    }
    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted) {
        rhs.SynchronizeContextLocalDataFrom(accepted);
    }
    void NotifyAcceptedProjectionHistoryChange() noexcept {
        advancer->InvalidateLinearizationAfterNumericalRhsHistoryChange();
    }
    SystemRhsBridge rhs;
    std::unique_ptr<Radau5ContinuousStateAdvancer> advancer;
};

template <class Configuration, class Advancer>
struct CoordinateRuntime final {
    static constexpr std::string_view kIdentifier =
        std::is_same_v<Configuration, NewmarkConfiguration> ? "newmark" : "zhai";
    CoordinateRuntime(const system_assembly::SystemInstance& system,
                      const system_assembly::CompiledSystemPlan& plan,
                      system_assembly::SystemRuntimeContext& candidate,
                      const system_assembly::SystemRuntimeContext& accepted,
                      const Eigen::VectorXd& initial, Configuration configuration,
                      NoCallTimeAppliedForces forces)
        : problem(system, plan, candidate, forces) {
        // The initial B/G evaluation must already see all accepted held inputs.
        problem.SynchronizeContextLocalDataFrom(accepted);
        advancer = std::make_unique<Advancer>(
            problem, accepted.time_seconds(), initial, std::move(configuration));
    }
    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted) {
        problem.SynchronizeContextLocalDataFrom(accepted);
    }
    void NotifyAcceptedProjectionHistoryChange() noexcept {
        // At the same accepted physical endpoint the returned projection is an
        // idempotent seed. B/G remain valid; do not restart the Zhai history.
        // External state/input/branch changes require explicit synchronization.
    }
    SystemCoordinateProblem problem;
    std::unique_ptr<Advancer> advancer;
};
using NewmarkRuntime = CoordinateRuntime<NewmarkConfiguration, NewmarkContinuousStateAdvancer>;
using ZhaiRuntime = CoordinateRuntime<ZhaiConfiguration, ZhaiContinuousStateAdvancer>;

using ConcreteRuntime = std::variant<std::unique_ptr<CvodeBdf2Runtime>,
    std::unique_ptr<CvodeBdf5Runtime>, std::unique_ptr<Radau5Runtime>,
    std::unique_ptr<NewmarkRuntime>, std::unique_ptr<ZhaiRuntime>>;

ConcreteRuntime MakeRuntime(
    SystemIntegrationMethodConfiguration method,
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& candidate,
    const system_assembly::SystemRuntimeContext& accepted,
    const Eigen::VectorXd& initial, NoCallTimeAppliedForces forces) {
    return std::visit([&](auto&& configuration) -> ConcreteRuntime {
        using Configuration = std::decay_t<decltype(configuration)>;
        using Runtime = std::conditional_t<std::is_same_v<Configuration, CvodeBdf2Configuration>,
            CvodeBdf2Runtime,
            std::conditional_t<std::is_same_v<Configuration, CvodeBdf5Configuration>,
                CvodeBdf5Runtime,
                std::conditional_t<std::is_same_v<Configuration, Radau5Configuration>,
                    Radau5Runtime,
                    std::conditional_t<std::is_same_v<Configuration, NewmarkConfiguration>,
                        NewmarkRuntime, ZhaiRuntime>>>>;
        static_assert(std::is_same_v<Configuration, CvodeBdf2Configuration> ||
                      std::is_same_v<Configuration, CvodeBdf5Configuration> ||
                      std::is_same_v<Configuration, Radau5Configuration> ||
                      std::is_same_v<Configuration, NewmarkConfiguration> ||
                      std::is_same_v<Configuration, ZhaiConfiguration>);
        return std::make_unique<Runtime>(system, plan, candidate, accepted, initial,
                                         std::move(configuration), forces);
    }, std::move(method));
}

}  // namespace

class SystemContinuousStateBackend::Implementation final {
   public:
    Implementation(SystemIntegrationMethodConfiguration configuration,
                   const system_assembly::SystemInstance& system,
                   const system_assembly::CompiledSystemPlan& plan,
                   system_assembly::SystemRuntimeContext& candidate,
                   const system_assembly::SystemRuntimeContext& accepted,
                   const Eigen::VectorXd& initial, NoCallTimeAppliedForces forces)
        : runtime_(MakeRuntime(std::move(configuration), system, plan, candidate,
                               accepted, initial, forces)) {}

    ContinuousStateAdvancer& advancer() {
        return std::visit([](auto& runtime) -> ContinuousStateAdvancer& {
            return *runtime->advancer;
        }, runtime_);
    }
    const ContinuousStateAdvancer& advancer() const {
        return std::visit([](const auto& runtime) -> const ContinuousStateAdvancer& {
            return *runtime->advancer;
        }, runtime_);
    }
    std::string_view method_identifier() const noexcept {
        return std::visit([](const auto& runtime) {
            return std::remove_reference_t<decltype(*runtime)>::kIdentifier;
        }, runtime_);
    }
    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted) {
        std::visit([&](auto& runtime) { runtime->SynchronizeContextLocalDataFrom(accepted); }, runtime_);
    }
    void NotifyAcceptedProjectionHistoryChange() {
        std::visit([](auto& runtime) { runtime->NotifyAcceptedProjectionHistoryChange(); }, runtime_);
    }

   private:
    ConcreteRuntime runtime_;
};

SystemContinuousStateBackend::SystemContinuousStateBackend(
    SystemIntegrationMethodConfiguration configuration,
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& candidate_context,
    const system_assembly::SystemRuntimeContext& accepted_context,
    const Eigen::VectorXd& initial_continuous_state,
    NoCallTimeAppliedForces no_call_time_applied_forces)
    : implementation_(std::make_unique<Implementation>(
          std::move(configuration), system, plan, candidate_context, accepted_context,
          initial_continuous_state, no_call_time_applied_forces)) {}

SystemContinuousStateBackend::~SystemContinuousStateBackend() = default;

ContinuousStateAdvancer& SystemContinuousStateBackend::advancer() {
    return implementation_->advancer();
}

const ContinuousStateAdvancer& SystemContinuousStateBackend::advancer()
    const {
    return implementation_->advancer();
}

std::string_view SystemContinuousStateBackend::method_identifier() const noexcept {
    return implementation_->method_identifier();
}

void SystemContinuousStateBackend::SynchronizeContextLocalDataFrom(
    const system_assembly::SystemRuntimeContext& accepted_context) {
    implementation_->SynchronizeContextLocalDataFrom(accepted_context);
}

void SystemContinuousStateBackend::NotifyAcceptedProjectionHistoryChange() {
    implementation_->NotifyAcceptedProjectionHistoryChange();
}

}  // namespace orvd::integrators::internal
