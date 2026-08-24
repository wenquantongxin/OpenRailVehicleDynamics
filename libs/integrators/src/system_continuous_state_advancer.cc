#include "orvd/integrators/system_continuous_state_advancer.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

#include "integrator_limits.h"
#include "system_continuous_state_backend.h"
#include "system_continuous_state_integration_access.h"

namespace orvd::integrators {
namespace {

bool SameDoubleBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

}  // namespace

class SystemContinuousStateAdvancer::Implementation final {
   public:
    Implementation(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces,
        internal::SystemContinuousStateIntegrationRecipe integration_recipe)
        : system_(&system),
          accepted_context_(&accepted_context),
          candidate_context_(system.CreateDefaultRuntimeContext(
              accepted_context.time_seconds())),
          candidate_state_(system.continuous_state_size()) {
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        system_->SetTimeContinuousStateAndWheelRailProjectionHints(
            *candidate_context_, accepted_context_->time_seconds(),
            candidate_state_,
            accepted_context_->wheel_rail_projection_station_hints_meters());
        backend_ =
            std::make_unique<internal::SystemContinuousStateBackend>(
                integration_recipe, system, plan, *candidate_context_,
                *accepted_context_, candidate_state_, std::move(tolerances),
                no_call_time_applied_forces);
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
        return Backend().integration_statistics();
    }

    [[nodiscard]] internal::SystemContinuousStateIntegrationRecipe
    ConfiguredRecipe() const noexcept {
        return backend_->configured_recipe();
    }

    [[nodiscard]] std::uint64_t
    RecoverableProjectionWindowMissClassificationCount() const noexcept {
        return backend_
            ->recoverable_projection_window_miss_classification_count();
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
                    Backend().AdvanceOneInternalStepToward(
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
                if (step.end_time_seconds > target_time_seconds ||
                    step.reached_stop !=
                        (step.end_time_seconds == target_time_seconds)) {
                    throw std::runtime_error(
                        "system continuous-state advancer: the backend "
                        "returned endpoint metadata inconsistent with the "
                        "requested stop");
                }
                if (samples != nullptr) {
                    const auto interval = Backend().dense_output_interval();
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
                        Backend().CopyDenseState(
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
                if (!candidate_context_
                         ->wheel_rail_projection_station_hints_meters()
                         .empty()) {
                    BackendBundle()
                        .NotifyAcceptedProjectionHistoryChange();
                }
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
        backend_->SynchronizeContextLocalDataFrom(*accepted_context_);
        Backend().ReinitializeAfterExternalChange(
            accepted_context_->time_seconds(), candidate_state_);
        requires_synchronization_ = false;
    }

   private:
    [[nodiscard]] internal::SystemContinuousStateBackend& BackendBundle() {
        if (backend_ == nullptr) {
            throw std::logic_error(
                "system continuous-state advancer: backend bundle is not "
                "initialized");
        }
        return *backend_;
    }

    [[nodiscard]] const internal::SystemContinuousStateBackend& BackendBundle()
        const {
        if (backend_ == nullptr) {
            throw std::logic_error(
                "system continuous-state advancer: backend bundle is not "
                "initialized");
        }
        return *backend_;
    }

    [[nodiscard]] ContinuousStateAdvancer& Backend() {
        return BackendBundle().advancer();
    }

    [[nodiscard]] const ContinuousStateAdvancer& Backend() const {
        return BackendBundle().advancer();
    }

    const system_assembly::SystemInstance* system_;
    system_assembly::SystemRuntimeContext* accepted_context_;
    // Declared before backend_ because its RHS borrows this context.
    std::unique_ptr<system_assembly::SystemRuntimeContext> candidate_context_;
    Eigen::VectorXd candidate_state_;
    std::unique_ptr<internal::SystemContinuousStateBackend> backend_;
    bool requires_synchronization_{false};
};

SystemContinuousStateAdvancer::SystemContinuousStateAdvancer(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& accepted_context,
    ContinuousStateErrorTolerances tolerances,
    NoCallTimeAppliedForces no_call_time_applied_forces)
    : SystemContinuousStateAdvancer(
          system, plan, accepted_context, std::move(tolerances),
          no_call_time_applied_forces,
          internal::SystemContinuousStateIntegrationRecipe::kCvodeBdf2) {}

SystemContinuousStateAdvancer::SystemContinuousStateAdvancer(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& accepted_context,
    ContinuousStateErrorTolerances tolerances,
    NoCallTimeAppliedForces no_call_time_applied_forces,
    internal::SystemContinuousStateIntegrationRecipe integration_recipe)
    : implementation_(std::make_unique<Implementation>(
          system, plan, accepted_context, std::move(tolerances),
          no_call_time_applied_forces, integration_recipe)) {}

SystemContinuousStateAdvancer::~SystemContinuousStateAdvancer() = default;

std::unique_ptr<SystemContinuousStateAdvancer>
internal::SystemContinuousStateIntegrationAccess::Make(
    SystemContinuousStateIntegrationRecipe recipe,
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& accepted_context,
    ContinuousStateErrorTolerances tolerances,
    NoCallTimeAppliedForces no_call_time_applied_forces) {
    return std::unique_ptr<SystemContinuousStateAdvancer>(
        new SystemContinuousStateAdvancer(
            system, plan, accepted_context, std::move(tolerances),
            no_call_time_applied_forces, recipe));
}

internal::SystemContinuousStateIntegrationRecipe
internal::SystemContinuousStateIntegrationAccess::ConfiguredRecipe(
    const SystemContinuousStateAdvancer& advancer) {
    return advancer.implementation_->ConfiguredRecipe();
}

std::uint64_t internal::SystemContinuousStateIntegrationAccess::
    RecoverableProjectionWindowMissClassificationCount(
        const SystemContinuousStateAdvancer& advancer) {
    return advancer.implementation_
        ->RecoverableProjectionWindowMissClassificationCount();
}

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
