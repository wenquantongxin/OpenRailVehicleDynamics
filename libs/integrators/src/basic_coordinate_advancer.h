#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "coordinate_step_time.h"
#include "coordinate_numerical_failure.h"
#include "system_coordinate_problem.h"

namespace orvd::integrators::internal {

// Shared transaction/time/output adaptation, not a shared numerical method.
// The real Core owns its own formulas, nonlinear solve and method history.
template <typename Core, typename Configuration>
class BasicCoordinateAdvancerImplementation {
   public:
    BasicCoordinateAdvancerImplementation(SystemCoordinateProblem& problem,
                                          double initial_time,
                                          const Eigen::VectorXd& initial_physical,
                                          Configuration configuration,
                                          const char* method_name,
                                          std::function<Configuration(const CoordinateState&)> prepare = {})
        : problem_(problem),
          physical_size_(problem.physical_state_size()),
          nominal_h_(configuration.step_size_seconds),
          method_name_(method_name),
          coordinate_(problem.MakeCoordinateState(initial_time, initial_physical)),
          prepare_configuration_(std::move(prepare)),
          core_(problem, prepare_configuration_ ? prepare_configuration_(coordinate_) :
                                                  std::move(configuration), coordinate_),
          public_state_(initial_physical),
          candidate_physical_(physical_size_),
          dense_start_state_(physical_size_),
          reference_q_(coordinate_.q),
          public_time_(initial_time),
          grid_anchor_(initial_time) {}

    [[nodiscard]] int continuous_state_size() const { return physical_size_; }
    [[nodiscard]] double current_time_seconds() const { return public_time_; }

    [[nodiscard]] ContinuousStateIntegrationStatistics integration_statistics() const {
        auto result = core_.integration_statistics();
        // Publication may still fail after a numerical core endpoint succeeded
        // (for example during physical export). Work remains visible, while
        // successful_internal_step_count counts this adapter's published steps.
        result.successful_internal_step_count = successful_steps_;
        return result;
    }

    [[nodiscard]] CoordinateIntegrationDiagnostics diagnostics() const {
        return core_.diagnostics();
    }

    [[nodiscard]] const Configuration& core_configuration() const
        requires requires(const Core& core) { core.configuration(); }
    {
        return core_.configuration();
    }

    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const {
        ValidateOutput(output);
        output = public_state_;
    }

    [[nodiscard]] ContinuousStateInternalStep Advance(
        double stop, Eigen::Ref<Eigen::VectorXd> output) {
        if (!std::isfinite(stop) || stop < public_time_) {
            throw std::invalid_argument(std::string(method_name_) +
                                        ": stop must be finite and not precede the current time");
        }
        ValidateOutput(output);
        if (requires_reinitialization_) {
            throw std::logic_error(std::string(method_name_) +
                                    ": reinitialization is required after failure");
        }
        if (stop == public_time_) {
            output = public_state_;
            return {public_time_, public_time_, true};
        }

        requires_reinitialization_ = true;
        dense_interval_.reset();
        try {
            const double start = public_time_;
            const StepChoice step = ChooseStep(stop);
            core_.AdvanceOneStep(step.h, step.end);
            coordinate_.time_seconds = core_.current_time_seconds();
            core_.CopyCurrentState(coordinate_.q, coordinate_.s, coordinate_.z);
            problem_.CopyPhysicalState(coordinate_, candidate_physical_);

            // Every fallible callback has completed. Fixed-size owned storage
            // is committed before caller output is touched.
            dense_start_state_ = public_state_;
            public_state_ = candidate_physical_;
            public_time_ = step.end;
            dense_interval_ = ContinuousStateDenseOutputInterval{start, step.end};
            if (step.restart_grid) {
                grid_anchor_ = step.end;
                grid_index_ = 0;
            } else {
                ++grid_index_;
            }
            ++successful_steps_;
            requires_reinitialization_ = false;
            output = public_state_;
            return {start, step.end, step.end == stop};
        } catch (const CoordinateIntegrationFailure& failure) {
            RethrowCoordinateNumericalFailure(failure);
        }
    }

    void Reinitialize(double time, const Eigen::Ref<const Eigen::VectorXd>& physical) {
        // Geometry/input refusals are pre-entry caller errors. Neither the
        // physical endpoint nor an existing dense interval is invalidated.
        CoordinateState initial = problem_.MakeCoordinateState(time, physical);
        problem_.ValidateInitialState(time, initial.q, initial.s, initial.z);
        std::optional<Configuration> prepared;
        if (prepare_configuration_) prepared = prepare_configuration_(initial);
        requires_reinitialization_ = true;
        dense_interval_.reset();
        if constexpr (requires(Core& core, Configuration config) { core.Reinitialize(initial, config); }) {
            if (prepared) {
                core_.Reinitialize(initial, std::move(*prepared));
            } else {
                core_.Reinitialize(initial);
            }
        } else {
            core_.Reinitialize(initial);
        }
        coordinate_ = initial;
        reference_q_ = initial.q;
        public_state_ = physical;
        public_time_ = time;
        grid_anchor_ = time;
        grid_index_ = 0;
        successful_steps_ = 0;
        requires_reinitialization_ = false;
    }

    [[nodiscard]] std::optional<ContinuousStateDenseOutputInterval> dense_output_interval() const {
        return dense_interval_;
    }

    void CopyDenseState(double time, Eigen::Ref<Eigen::VectorXd> output) const {
        if (!std::isfinite(time)) {
            throw std::invalid_argument(std::string(method_name_) + ": dense time must be finite");
        }
        ValidateOutput(output);
        if (!dense_interval_.has_value()) {
            throw std::logic_error(std::string(method_name_) + ": dense output is unavailable");
        }
        if (time < dense_interval_->start_time_seconds || time > dense_interval_->end_time_seconds) {
            throw std::invalid_argument(std::string(method_name_) + ": dense time is outside the current interval");
        }
        if (time == dense_interval_->start_time_seconds) {
            output = dense_start_state_;
        } else if (time == dense_interval_->end_time_seconds) {
            output = public_state_;
        } else {
            const double fraction = (time - dense_interval_->start_time_seconds) /
                                    (dense_interval_->end_time_seconds - dense_interval_->start_time_seconds);
            problem_.CopyLinearlyInterpolatedPhysicalState(
                reference_q_, dense_start_state_, public_state_, fraction, output);
        }
    }

   private:
    struct StepChoice {
        double h;
        double end;
        bool restart_grid;
    };

    [[noreturn]] void TimeUnderflow() const {
        throw CoordinateIntegrationFailure(
            CoordinateIntegrationFailure::Reason::kStepSizeUnderflow,
            std::string(method_name_) + ": no compatible representable forward step");
    }

    [[nodiscard]] StepChoice ChooseStep(double stop) const {
        if (grid_index_ == std::numeric_limits<std::uint64_t>::max()) TimeUnderflow();
        const double next = std::fma(static_cast<double>(grid_index_ + 1), nominal_h_, grid_anchor_);
        if (CoordinateStopCanUseNominalStep(public_time_, nominal_h_, next, stop)) {
            return {nominal_h_, stop, true};
        }
        // Do not subtract distant finite times merely to decide whether a
        // nominal step fits: that difference can overflow unnecessarily.
        if (stop < next) {
            const double h = stop - public_time_;
            if (h > nominal_h_ || !CoordinateEndpointTimeIsCompatible(public_time_, h, stop)) {
                TimeUnderflow();
            }
            return {h, stop, true};
        }
        if (!CoordinateEndpointTimeIsCompatible(public_time_, nominal_h_, next)) TimeUnderflow();
        return {nominal_h_, next, false};
    }

    void ValidateOutput(const Eigen::Ref<Eigen::VectorXd>& output) const {
        if (output.size() != physical_size_) {
            throw std::invalid_argument(std::string(method_name_) + ": physical output has the wrong size");
        }
    }

    SystemCoordinateProblem& problem_;
    const int physical_size_;
    const double nominal_h_;
    const char* const method_name_;
    CoordinateState coordinate_;
    const std::function<Configuration(const CoordinateState&)> prepare_configuration_;
    Core core_;
    Eigen::VectorXd public_state_;
    Eigen::VectorXd candidate_physical_;
    Eigen::VectorXd dense_start_state_;
    Eigen::VectorXd reference_q_;
    double public_time_;
    double grid_anchor_;
    std::uint64_t grid_index_{};
    std::uint64_t successful_steps_{};
    bool requires_reinitialization_{};
    std::optional<ContinuousStateDenseOutputInterval> dense_interval_;
};

}  // namespace orvd::integrators::internal
