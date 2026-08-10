#include "orvd/integrators/system_continuous_state_advancer.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/integrators/cvode_continuous_state_advancer.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

#include "integrator_limits.h"

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
        rhs_.SynchronizeContextParametersFrom(*accepted_context_);
        backend_ = std::make_unique<CvodeContinuousStateAdvancer>(
            rhs_, accepted_context_->time_seconds(), candidate_state_,
            std::move(tolerances));
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
        rhs_.SynchronizeContextParametersFrom(*accepted_context_);
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
    // Declared after rhs_ so it is destroyed first; CVODE borrows the RHS.
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
