#include "qualification_integration_run.h"

#include <cstdio>
#include <exception>
#include <string>
#include <utility>

namespace orvd::dynamics_qualification {

QualificationIntegrationRun::QualificationIntegrationRun(
    const std::filesystem::path& output_directory, nlohmann::json metadata,
    integrators::SystemIntegrationConfiguration config,
    const configuration::AssembledVehicleSystem& assembled,
    system_assembly::SystemRuntimeContext& accepted, double terminal)
    : output_directory_(output_directory), configuration_metadata_(std::move(metadata)),
      accepted_(accepted), requested_terminal_time_seconds_(terminal) {
    try {
        {
            ScopedQualificationRunTimer timer(timings_.backend_construction_wall_seconds);
            advancer_ = std::make_unique<integrators::SystemContinuousStateAdvancer>(
                assembled.system(), assembled.compiled_plan(), accepted,
                std::move(config), integrators::NoCallTimeAppliedForces{});
        }
        ledger_.BeginEpoch(advancer_->integration_statistics());
        if (configuration_metadata_.at("integrator_recipe_identifier") !=
            advancer_->method_identifier()) {
            throw std::logic_error("constructed backend differs from resolved integration configuration");
        }
    } catch (...) {
        RecordFailure("construction", terminal, std::current_exception());
        throw;
    }
}

const integrators::SystemContinuousStateAdvancer& QualificationIntegrationRun::advancer() const {
    return *advancer_;
}

void QualificationIntegrationRun::Capture() {
    ledger_.Capture(advancer_->integration_statistics());
}

Eigen::MatrixXd QualificationIntegrationRun::Advance(
    double stop, std::span<const double> sample_times) {
    try {
        Eigen::MatrixXd states;
        {
            ScopedQualificationRunTimer timer(timings_.advance_wall_seconds);
            states = advancer_->AdvanceToWithDenseStateSamples(stop, sample_times);
        }
        Capture();
        return states;
    } catch (...) {
        RecordFailure("advance", stop, std::current_exception());
        throw;
    }
}

void QualificationIntegrationRun::Synchronize() {
    try {
        Capture();
        {
            ScopedQualificationRunTimer timer(timings_.synchronization_wall_seconds);
            advancer_->SynchronizeAfterAcceptedContextChange();
        }
        ledger_.BeginEpoch(advancer_->integration_statistics());
    } catch (...) {
        RecordFailure("synchronization", accepted_.time_seconds(), std::current_exception());
        throw;
    }
}

void QualificationIntegrationRun::RecordFailure(
    const char* phase, double target, std::exception_ptr error) noexcept {
    try {
        if (advancer_) {
            try { Capture(); }
            catch (const std::exception& e) { ledger_.MarkStatisticsUnavailable(e.what()); }
            catch (...) { ledger_.MarkStatisticsUnavailable("statistics retrieval failed"); }
        } else {
            ledger_.MarkStatisticsUnavailable("backend construction did not return an object");
        }
        nlohmann::json result{
            {"completed", false}, {"failure_stage", phase},
            {"requested_terminal_time_seconds", requested_terminal_time_seconds_},
            {"requested_stop_time_seconds", target},
            {"last_public_accepted_time_seconds", accepted_.time_seconds()},
            {"configuration", configuration_metadata_},
            {"work", ledger_.ToJson()}, {"numerical_timings", timings_.ToJson()},
            {"numerical_failure_reason", nullptr}, {"backend_code", nullptr}};
        const auto failure_details = QualificationFailureToJson(error);
        result["failure"] = failure_details;
        result["original_message"] = failure_details.at("message");
        result["numerical_failure_reason"] = failure_details.at("reason");
        result["backend_code"] = failure_details.at("backend_code");
        if (const auto failure = TryPublishFailureResult(output_directory_, result)) {
            std::fprintf(stderr, "could not publish failure result: %s\n", failure->c_str());
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "could not record numerical failure: %s\n", e.what());
    } catch (...) {
        std::fprintf(stderr, "could not record numerical failure\n");
    }
}

void AddQualificationBudgetEstimate(nlohmann::json& metadata,
    std::optional<std::uint64_t> step, std::uint64_t interval,
    std::uint64_t count, std::size_t budget) {
    metadata["scheduled_stop_interval_nanoseconds"] = interval;
    metadata["scheduled_stop_interval_count"] = count;
    metadata["fixed_step_budget_estimate"] = nullptr;
    if (step) {
        const auto steps = interval / *step + static_cast<std::uint64_t>(interval % *step != 0);
        metadata["fixed_step_budget_estimate"] = {
            {"steps_per_public_advance", steps}, {"exceeds_declared_budget", steps > budget},
            {"role", "estimate_only_runtime_budget_failure_is_authoritative"}};
    }
}

}  // namespace orvd::dynamics_qualification
