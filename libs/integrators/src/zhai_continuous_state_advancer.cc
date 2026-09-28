#include "zhai_continuous_state_advancer.h"

#include <utility>

#include "basic_coordinate_advancer.h"
#include "zhai_core.h"

namespace orvd::integrators::internal {

class ZhaiContinuousStateAdvancer::Implementation final
    : public BasicCoordinateAdvancerImplementation<ZhaiCore, ZhaiConfiguration> {
   public:
    using Base = BasicCoordinateAdvancerImplementation<ZhaiCore, ZhaiConfiguration>;
    Implementation(SystemCoordinateProblem& problem, double time,
                   const Eigen::VectorXd& state, ZhaiConfiguration configuration)
        : Base(problem, time, state, std::move(configuration), "Zhai continuous-state advancer") {}
};

ZhaiContinuousStateAdvancer::ZhaiContinuousStateAdvancer(
    SystemCoordinateProblem& problem, double time, const Eigen::VectorXd& state,
    ZhaiConfiguration configuration)
    : implementation_(std::make_unique<Implementation>(problem, time, state,
                                                       std::move(configuration))) {}
ZhaiContinuousStateAdvancer::~ZhaiContinuousStateAdvancer() = default;

int ZhaiContinuousStateAdvancer::continuous_state_size() const {
    return implementation_->continuous_state_size();
}
double ZhaiContinuousStateAdvancer::current_time_seconds() const {
    return implementation_->current_time_seconds();
}
ContinuousStateIntegrationStatistics ZhaiContinuousStateAdvancer::integration_statistics() const {
    return implementation_->integration_statistics();
}
CoordinateIntegrationDiagnostics ZhaiContinuousStateAdvancer::diagnostics() const {
    return implementation_->diagnostics();
}
void ZhaiContinuousStateAdvancer::CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const {
    implementation_->CopyCurrentState(output);
}
ContinuousStateInternalStep ZhaiContinuousStateAdvancer::AdvanceOneInternalStepToward(
    double stop, Eigen::Ref<Eigen::VectorXd> output) {
    return implementation_->Advance(stop, output);
}
void ZhaiContinuousStateAdvancer::ReinitializeAfterExternalChange(
    double time, const Eigen::Ref<const Eigen::VectorXd>& state) {
    implementation_->Reinitialize(time, state);
}
std::optional<ContinuousStateDenseOutputInterval>
ZhaiContinuousStateAdvancer::dense_output_interval() const {
    return implementation_->dense_output_interval();
}
void ZhaiContinuousStateAdvancer::CopyDenseState(double time, Eigen::Ref<Eigen::VectorXd> output) const {
    implementation_->CopyDenseState(time, output);
}

}  // namespace orvd::integrators::internal
