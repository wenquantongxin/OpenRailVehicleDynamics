#include "orvd/integrators/system_continuous_state_advancer.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <omp.h>

#include "orvd/integrators/cvode_continuous_state_advancer.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

#include "dense_finite_difference_jacobian_provider.h"
#include "integrator_limits.h"

namespace orvd::integrators {
namespace {

bool SameDoubleBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

constexpr int kMinimumParallelJacobianWorkerCount = 4;
constexpr int kMaximumParallelJacobianWorkerCount = 8;

int ResolveParallelJacobianWorkerCount(
    const system_assembly::SystemInstance& system) {
    if (system.contact_force_plan() == nullptr || omp_get_dynamic() != 0) {
        return 0;
    }
    const int maximum_threads = omp_get_max_threads();
    if (maximum_threads >= kMaximumParallelJacobianWorkerCount) {
        return kMaximumParallelJacobianWorkerCount;
    }
    if (maximum_threads >= kMinimumParallelJacobianWorkerCount) {
        return kMinimumParallelJacobianWorkerCount;
    }
    return 0;
}

class SystemDenseFiniteDifferenceJacobian final
    : public internal::DenseFiniteDifferenceJacobianProvider {
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
            worker_count_ != kMaximumParallelJacobianWorkerCount) {
            throw std::invalid_argument(
                "system dense Jacobian: worker count must be four or eight");
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

    [[nodiscard]] internal::DenseFiniteDifferenceJacobianBatchResult
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

        internal::DenseFiniteDifferenceJacobianBatchResult result;
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

}  // namespace

class SystemContinuousStateAdvancer::Implementation final {
   public:
    Implementation(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces)
        : system_(&system),
          accepted_context_(&accepted_context),
          candidate_context_(system.CreateDefaultRuntimeContext(
              accepted_context.time_seconds())),
          candidate_state_(system.continuous_state_size()),
          rhs_(system, plan, *candidate_context_,
               no_call_time_applied_forces) {
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        system_->SetTimeContinuousStateAndWheelRailProjectionHints(
            *candidate_context_, accepted_context_->time_seconds(),
            candidate_state_,
            accepted_context_->wheel_rail_projection_station_hints_meters());
        rhs_.SynchronizeContextLocalDataFrom(*accepted_context_);
        const int jacobian_worker_count =
            ResolveParallelJacobianWorkerCount(system);
        if (jacobian_worker_count != 0) {
            dense_jacobian_ =
                std::make_unique<SystemDenseFiniteDifferenceJacobian>(
                    system, plan, *candidate_context_, jacobian_worker_count,
                    no_call_time_applied_forces);
        }
        backend_ = std::make_unique<CvodeContinuousStateAdvancer>(
            rhs_, accepted_context_->time_seconds(), candidate_state_,
            std::move(tolerances));
        if (dense_jacobian_ != nullptr) {
            internal::DenseFiniteDifferenceJacobianRegistration::Attach(
                *backend_, *dense_jacobian_);
        }
    }

    void AdvanceTo(double target_time_seconds) {
        AdvanceToImpl(target_time_seconds, {}, nullptr);
    }

    Eigen::MatrixXd AdvanceToWithDenseStateSamples(
        double target_time_seconds,
        std::span<const double> sample_times_seconds) {
        Eigen::MatrixXd samples;
        AdvanceToImpl(target_time_seconds, sample_times_seconds, &samples);
        return samples;
    }

    [[nodiscard]] ContinuousStateIntegrationStatistics Statistics() const {
        return backend_->integration_statistics();
    }

    void AdvanceToImpl(double target_time_seconds,
                       std::span<const double> sample_times_seconds,
                       Eigen::MatrixXd* samples) {
        if (!std::isfinite(target_time_seconds)) {
            throw std::invalid_argument(
                "system continuous-state advancer: target time must be "
                "finite");
        }
        if (target_time_seconds < accepted_context_->time_seconds()) {
            throw std::invalid_argument(
                "system continuous-state advancer: target time precedes the "
                "accepted time");
        }
        if (samples != nullptr) {
            if (sample_times_seconds.empty()) {
                throw std::invalid_argument(
                    "system continuous-state advancer: dense sample times "
                    "must be non-empty");
            }
            for (std::size_t index = 0; index < sample_times_seconds.size();
                 ++index) {
                if (!std::isfinite(sample_times_seconds[index])) {
                    throw std::invalid_argument(
                        "system continuous-state advancer: a dense sample "
                        "time is not finite");
                }
                if (index != 0 &&
                    !(sample_times_seconds[index] >
                      sample_times_seconds[index - 1])) {
                    throw std::invalid_argument(
                        "system continuous-state advancer: dense sample "
                        "times must be strictly increasing");
                }
            }
            if (!SameDoubleBits(sample_times_seconds.front(),
                                accepted_context_->time_seconds())) {
                throw std::invalid_argument(
                    "system continuous-state advancer: the first dense "
                    "sample time must equal the accepted time");
            }
            if (!SameDoubleBits(sample_times_seconds.back(),
                                target_time_seconds)) {
                throw std::invalid_argument(
                    "system continuous-state advancer: the last dense "
                    "sample time must equal the target time");
            }
            if (target_time_seconds == accepted_context_->time_seconds() &&
                sample_times_seconds.size() != 1) {
                throw std::invalid_argument(
                    "system continuous-state advancer: a same-time dense "
                    "request must contain exactly one sample");
            }
        }
        if (requires_synchronization_) {
            throw std::logic_error(
                "system continuous-state advancer: synchronize from the "
                "accepted context before advancing");
        }
        if (target_time_seconds == accepted_context_->time_seconds()) {
            if (samples != nullptr) {
                samples->resize(system_->continuous_state_size(), 1);
                system_->CopyContinuousState(*accepted_context_,
                                             samples->col(0));
            }
            return;
        }

        std::size_t next_sample = 0;
        if (samples != nullptr) {
            samples->resize(
                system_->continuous_state_size(),
                static_cast<Eigen::Index>(sample_times_seconds.size()));
            system_->CopyContinuousState(*accepted_context_, samples->col(0));
            next_sample = 1;
        }

        try {
            bool reached_target = false;
            std::size_t successful_internal_step_count = 0;
            double expected_step_begin_time_seconds =
                accepted_context_->time_seconds();
            while (!reached_target) {
                if (successful_internal_step_count ==
                    internal::kMaximumInternalStepsPerPublicAdvance) {
                    throw std::runtime_error(
                        "system continuous-state advancer: one public "
                        "advance exceeded " +
                        std::to_string(
                            internal::kMaximumInternalStepsPerPublicAdvance) +
                        " successful internal steps");
                }
                const ContinuousStateInternalStep step =
                    backend_->AdvanceOneInternalStepToward(
                        target_time_seconds, candidate_state_);
                ++successful_internal_step_count;
                // The last RHS trial need not be at the accepted backend
                // endpoint. Continuity is checked against the preceding
                // backend endpoint, then the returned endpoint is installed
                // explicitly below.
                if (step.start_time_seconds !=
                        expected_step_begin_time_seconds ||
                    !(step.end_time_seconds > step.start_time_seconds)) {
                    throw std::runtime_error(
                        "system continuous-state advancer: the backend "
                        "returned a non-contiguous internal endpoint");
                }
                if (samples != nullptr) {
                    const auto interval = backend_->dense_output_interval();
                    if (!interval.has_value() ||
                        !(interval->end_time_seconds >
                          interval->start_time_seconds) ||
                        interval->end_time_seconds != step.end_time_seconds) {
                        throw std::runtime_error(
                            "system continuous-state advancer: the backend "
                            "did not expose a dense-output interval ending "
                            "at the successful internal endpoint");
                    }
                    // The last sample is copied from the accepted endpoint
                    // below. Every earlier requested time is read from the one
                    // successful interval that contains it.
                    while (next_sample + 1 < sample_times_seconds.size() &&
                           sample_times_seconds[next_sample] <=
                               step.end_time_seconds) {
                        if (sample_times_seconds[next_sample] <
                            interval->start_time_seconds) {
                            throw std::runtime_error(
                                "system continuous-state advancer: a dense "
                                "sample was not covered by the backend's "
                                "successful dense-output intervals");
                        }
                        backend_->CopyDenseState(
                            sample_times_seconds[next_sample],
                            samples->col(
                                static_cast<Eigen::Index>(next_sample)));
                        ++next_sample;
                    }
                }
                system_->SetTimeAndContinuousState(
                    *candidate_context_, step.end_time_seconds,
                    candidate_state_);
                system_->UpdateWheelRailProjectionStationHints(
                    *candidate_context_);
                expected_step_begin_time_seconds = step.end_time_seconds;
                reached_target = step.reached_stop;
            }
            if (samples != nullptr &&
                next_sample + 1 != sample_times_seconds.size()) {
                throw std::runtime_error(
                    "system continuous-state advancer: not every dense sample "
                    "time was covered before the target endpoint");
            }
            if (samples != nullptr) {
                samples->col(samples->cols() - 1) = candidate_state_;
            }
            system_->SetTimeContinuousStateAndWheelRailProjectionHints(
                *accepted_context_, target_time_seconds, candidate_state_,
                candidate_context_
                    ->wheel_rail_projection_station_hints_meters());
        } catch (...) {
            requires_synchronization_ = true;
            throw;
        }
    }

    void SynchronizeAfterAcceptedContextChange() {
        requires_synchronization_ = true;
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        system_->SetTimeContinuousStateAndWheelRailProjectionHints(
            *candidate_context_, accepted_context_->time_seconds(),
            candidate_state_,
            accepted_context_->wheel_rail_projection_station_hints_meters());
        rhs_.SynchronizeContextLocalDataFrom(*accepted_context_);
        if (dense_jacobian_ != nullptr) {
            dense_jacobian_->SynchronizeContextLocalDataFrom(
                *accepted_context_);
        }
        backend_->ReinitializeAfterExternalChange(
            accepted_context_->time_seconds(), candidate_state_);
        requires_synchronization_ = false;
    }

   private:
    const system_assembly::SystemInstance* system_;
    system_assembly::SystemRuntimeContext* accepted_context_;
    // Declared before rhs_ because the RHS borrows this context.
    std::unique_ptr<system_assembly::SystemRuntimeContext> candidate_context_;
    Eigen::VectorXd candidate_state_;
    SystemRhsBridge rhs_;
    // Declared before backend_ because CVODE borrows this provider.
    std::unique_ptr<SystemDenseFiniteDifferenceJacobian> dense_jacobian_;
    // Declared after every borrowed object so CVODE is destroyed first.
    std::unique_ptr<CvodeContinuousStateAdvancer> backend_;
    bool requires_synchronization_{false};
};

SystemContinuousStateAdvancer::SystemContinuousStateAdvancer(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& accepted_context,
    ContinuousStateErrorTolerances tolerances,
    NoCallTimeAppliedForces no_call_time_applied_forces)
    : implementation_(std::make_unique<Implementation>(
          system, plan, accepted_context, std::move(tolerances),
          no_call_time_applied_forces)) {}

SystemContinuousStateAdvancer::~SystemContinuousStateAdvancer() = default;

void SystemContinuousStateAdvancer::AdvanceTo(double target_time_seconds) {
    implementation_->AdvanceTo(target_time_seconds);
}

Eigen::MatrixXd
SystemContinuousStateAdvancer::AdvanceToWithDenseStateSamples(
    double target_time_seconds,
    std::span<const double> sample_times_seconds) {
    return implementation_->AdvanceToWithDenseStateSamples(
        target_time_seconds, sample_times_seconds);
}

ContinuousStateIntegrationStatistics
SystemContinuousStateAdvancer::integration_statistics() const {
    return implementation_->Statistics();
}

void SystemContinuousStateAdvancer::SynchronizeAfterAcceptedContextChange() {
    implementation_->SynchronizeAfterAcceptedContextChange();
}

}  // namespace orvd::integrators
