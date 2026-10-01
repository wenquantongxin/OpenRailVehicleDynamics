#include "newmark_continuous_state_advancer.h"

#include <utility>

#include "basic_coordinate_advancer.h"
#include "newmark_core.h"
#include "newmark_coordinate_layout.h"

namespace orvd::integrators::internal {

class NewmarkContinuousStateAdvancer::Implementation final
    : public BasicCoordinateAdvancerImplementation<NewmarkCore, NewmarkCoreConfiguration> {
   public:
    using Base = BasicCoordinateAdvancerImplementation<NewmarkCore, NewmarkCoreConfiguration>;
    Implementation(SystemCoordinateProblem& problem, double time,
                   const Eigen::VectorXd& state, NewmarkConfiguration configuration)
        : Base(problem, time, state,
               NewmarkCoreConfiguration{.step_size_seconds = configuration.nominal_step_size_seconds,
                                        .nonlinear_solver = {}},
               "Newmark continuous-state advancer",
               [layout = NewmarkCoordinateLayout(problem.model(), problem.internal_state_size()),
                configuration = std::move(configuration)](const CoordinateState& initial) {
                    return layout.Expand(configuration, initial.q);
               }) {}
};

NewmarkContinuousStateAdvancer::NewmarkContinuousStateAdvancer(
    SystemCoordinateProblem& problem, double time, const Eigen::VectorXd& state,
    NewmarkConfiguration configuration)
    : implementation_(std::make_unique<Implementation>(problem, time, state,
                                                       std::move(configuration))) {}
NewmarkContinuousStateAdvancer::~NewmarkContinuousStateAdvancer() = default;

int NewmarkContinuousStateAdvancer::continuous_state_size() const {
    return implementation_->continuous_state_size();
}
double NewmarkContinuousStateAdvancer::current_time_seconds() const {
    return implementation_->current_time_seconds();
}
ContinuousStateIntegrationStatistics NewmarkContinuousStateAdvancer::integration_statistics() const {
    return implementation_->integration_statistics();
}
CoordinateIntegrationDiagnostics NewmarkContinuousStateAdvancer::diagnostics() const {
    return implementation_->diagnostics();
}
const NewmarkCoreConfiguration& NewmarkContinuousStateAdvancer::expanded_configuration() const {
    return implementation_->core_configuration();
}
void NewmarkContinuousStateAdvancer::CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const {
    implementation_->CopyCurrentState(output);
}
ContinuousStateInternalStep NewmarkContinuousStateAdvancer::AdvanceOneInternalStepToward(
    double stop, Eigen::Ref<Eigen::VectorXd> output) {
    return implementation_->Advance(stop, output);
}
void NewmarkContinuousStateAdvancer::ReinitializeAfterExternalChange(
    double time, const Eigen::Ref<const Eigen::VectorXd>& state) {
    implementation_->Reinitialize(time, state);
}
std::optional<ContinuousStateDenseOutputInterval>
NewmarkContinuousStateAdvancer::dense_output_interval() const {
    return implementation_->dense_output_interval();
}
void NewmarkContinuousStateAdvancer::CopyDenseState(double time, Eigen::Ref<Eigen::VectorXd> output) const {
    implementation_->CopyDenseState(time, output);
}

}  // namespace orvd::integrators::internal
