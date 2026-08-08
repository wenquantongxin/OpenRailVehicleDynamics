#include "orvd/integrators/cvode_continuous_state_advancer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <cvode/cvode.h>
#include <cvode/cvode_ls.h>
#include <nvector/nvector_serial.h>
#include <sundials/sundials_context.h>
#include <sundials/sundials_types.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sunmatrix/sunmatrix_dense.h>

#include "integrator_limits.h"

namespace orvd::integrators {
namespace {

static_assert(std::is_same_v<sunrealtype, double>);
static_assert(std::is_same_v<sunindextype, std::int32_t>);

constexpr int kMaximumBdfOrder = 2;
constexpr long int kJacobianEvaluationFrequency = 51;
constexpr long int kLinearSetupFrequency = 20;

[[noreturn]] void ThrowSundialsFailure(const char* operation, int flag) {
    throw std::runtime_error(
        std::string("CVODE continuous-state advancer: ") + operation +
        " failed with SUNDIALS flag " + std::to_string(flag));
}

void RequireCvodeSuccess(int flag, const char* operation) {
    if (flag != CV_SUCCESS) {
        ThrowSundialsFailure(operation, flag);
    }
}

struct CvodeResources final {
    SUNContext context{nullptr};
    void* memory{nullptr};
    N_Vector state{nullptr};
    N_Vector absolute_tolerances{nullptr};
    N_Vector dense_state{nullptr};
    SUNMatrix matrix{nullptr};
    SUNLinearSolver linear_solver{nullptr};

    CvodeResources() = default;
    CvodeResources(const CvodeResources&) = delete;
    CvodeResources& operator=(const CvodeResources&) = delete;

    ~CvodeResources() {
        if (memory != nullptr) {
            CVodeFree(&memory);
        }
        if (linear_solver != nullptr) {
            SUNLinSolFree(linear_solver);
        }
        if (matrix != nullptr) {
            SUNMatDestroy(matrix);
        }
        if (absolute_tolerances != nullptr) {
            N_VDestroy(absolute_tolerances);
        }
        if (dense_state != nullptr) {
            N_VDestroy(dense_state);
        }
        if (state != nullptr) {
            N_VDestroy(state);
        }
        if (context != nullptr) {
            SUNContext_Free(&context);
        }
    }
};

void CopyEigenToNVector(const Eigen::Ref<const Eigen::VectorXd>& source,
                        N_Vector destination) {
    std::copy_n(source.data(), source.size(),
                N_VGetArrayPointer(destination));
}

void CopyNVectorToEigen(N_Vector source,
                        Eigen::Ref<Eigen::VectorXd> destination) {
    std::copy_n(N_VGetArrayPointer(source), destination.size(),
                destination.data());
}

}  // namespace

class CvodeContinuousStateAdvancer::Implementation final {
   public:
    Implementation(ContinuousStateRhs& rhs,
                   double initial_time_seconds,
                   Eigen::VectorXd initial_continuous_state,
                   ContinuousStateErrorTolerances tolerances)
        : rhs_(&rhs),
          state_size_(rhs.continuous_state_size()),
          public_time_seconds_(initial_time_seconds),
          public_state_(std::move(initial_continuous_state)) {
        ValidateConstruction(tolerances);
        InitializeResources(tolerances);
    }

    [[nodiscard]] int state_size() const { return state_size_; }
    [[nodiscard]] double public_time_seconds() const {
        return public_time_seconds_;
    }

    void CopyPublicState(Eigen::Ref<Eigen::VectorXd> output) const {
        if (output.size() != state_size_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: current-state output has "
                "the wrong size; nothing was written");
        }
        output = public_state_;
    }

    [[nodiscard]] ContinuousStateInternalStep AdvanceOneInternalStepToward(
        double stop_time_seconds,
        Eigen::Ref<Eigen::VectorXd> endpoint_continuous_state) {
        if (!std::isfinite(stop_time_seconds)) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: stop time must be finite");
        }
        if (stop_time_seconds < public_time_seconds_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: stop time precedes the "
                "current successful endpoint");
        }
        if (endpoint_continuous_state.size() != state_size_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: endpoint output has the "
                "wrong size; nothing was written");
        }
        if (needs_reinitialization_) {
            throw std::logic_error(
                "CVODE continuous-state advancer: a prior backend failure "
                "requires reinitialization before advancing");
        }
        if (stop_time_seconds == public_time_seconds_) {
            endpoint_continuous_state = public_state_;
            return ContinuousStateInternalStep{
                public_time_seconds_, public_time_seconds_, true};
        }

        const double step_begin_time_seconds = public_time_seconds_;
        dense_interval_.reset();
        rhs_exception_ = nullptr;
        const int stop_flag =
            CVodeSetStopTime(resources_.memory, stop_time_seconds);
        if (stop_flag != CV_SUCCESS) {
            MarkBackendFailure();
            ThrowSundialsFailure("CVodeSetStopTime", stop_flag);
        }

        sunrealtype reached_time{};
        const int advance_flag = CVode(resources_.memory, stop_time_seconds,
                                       resources_.state, &reached_time,
                                       CV_ONE_STEP);
        if (advance_flag != CV_SUCCESS && advance_flag != CV_TSTOP_RETURN) {
            const std::exception_ptr exception = rhs_exception_;
            MarkBackendFailure();
            if (exception != nullptr) {
                std::rethrow_exception(exception);
            }
            ThrowSundialsFailure("CVode", advance_flag);
        }
        // A recoverable RHS refusal may have preceded this successful step.
        // Do not let that rejected trial escape after CVODE has reduced its
        // step and found an admissible endpoint.
        rhs_exception_ = nullptr;
        sunrealtype backend_time{};
        sunrealtype last_step{};
        const int time_flag =
            CVodeGetCurrentTime(resources_.memory, &backend_time);
        if (time_flag != CV_SUCCESS) {
            MarkBackendFailure();
            ThrowSundialsFailure("CVodeGetCurrentTime", time_flag);
        }
        const int step_flag = CVodeGetLastStep(resources_.memory, &last_step);
        if (step_flag != CV_SUCCESS) {
            MarkBackendFailure();
            ThrowSundialsFailure("CVodeGetLastStep", step_flag);
        }
        if (!std::isfinite(backend_time) || !std::isfinite(last_step) ||
            backend_time != reached_time ||
            !(backend_time > step_begin_time_seconds) ||
            backend_time > stop_time_seconds || last_step <= 0.0) {
            MarkBackendFailure();
            throw std::runtime_error(
                "CVODE continuous-state advancer: invalid successful-step "
                "metadata for dense output");
        }

        public_state_ = Eigen::Map<const Eigen::VectorXd>(
            N_VGetArrayPointer(resources_.state), state_size_);
        public_time_seconds_ = backend_time;
        endpoint_continuous_state = public_state_;
        dense_interval_ = ContinuousStateDenseOutputInterval{
            backend_time - last_step, backend_time};
        return ContinuousStateInternalStep{
            step_begin_time_seconds, backend_time,
            backend_time == stop_time_seconds};
    }

    void Reinitialize(
        double committed_time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) {
        if (!std::isfinite(committed_time_seconds)) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: committed time must be "
                "finite");
        }
        if (committed_continuous_state.size() != state_size_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: committed state has the "
                "wrong size");
        }
        if (!committed_continuous_state.allFinite()) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: committed state must be "
                "finite");
        }

        dense_interval_.reset();
        rhs_exception_ = nullptr;
        CopyEigenToNVector(committed_continuous_state, resources_.state);
        const int flag = CVodeReInit(resources_.memory, committed_time_seconds,
                                     resources_.state);
        if (flag != CV_SUCCESS) {
            MarkBackendFailure();
            ThrowSundialsFailure("CVodeReInit", flag);
        }

        public_state_ = committed_continuous_state;
        public_time_seconds_ = committed_time_seconds;
        needs_reinitialization_ = false;
    }

    [[nodiscard]] std::optional<ContinuousStateDenseOutputInterval>
    dense_interval() const {
        return dense_interval_;
    }

    void CopyDenseState(double time_seconds,
                        Eigen::Ref<Eigen::VectorXd> output) const {
        if (!std::isfinite(time_seconds)) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: dense-output time must be "
                "finite");
        }
        if (output.size() != state_size_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: dense-state output has the "
                "wrong size; nothing was written");
        }
        if (!dense_interval_.has_value()) {
            throw std::logic_error(
                "CVODE continuous-state advancer: dense output is unavailable");
        }
        if (time_seconds < dense_interval_->start_time_seconds ||
            time_seconds > dense_interval_->end_time_seconds) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: requested time is outside "
                "the current dense-output interval");
        }

        const int flag = CVodeGetDky(resources_.memory, time_seconds, 0,
                                     resources_.dense_state);
        if (flag != CV_SUCCESS) {
            ThrowSundialsFailure("CVodeGetDky", flag);
        }
        CopyNVectorToEigen(resources_.dense_state, output);
    }

   private:
    static int EvaluateRhs(sunrealtype time_seconds, N_Vector state,
                           N_Vector state_time_derivatives,
                           void* user_data) noexcept {
        auto& self = *static_cast<Implementation*>(user_data);
        try {
            const Eigen::Map<const Eigen::VectorXd> input(
                N_VGetArrayPointer(state), self.state_size_);
            Eigen::Map<Eigen::VectorXd> output(
                N_VGetArrayPointer(state_time_derivatives), self.state_size_);
            self.rhs_->CalcTimeDerivatives(time_seconds, input, output);
            self.rhs_exception_ = nullptr;
            return 0;
        } catch (...) {
            self.rhs_exception_ = std::current_exception();
            return self.rhs_->IsRecoverableFailure(self.rhs_exception_) ? 1
                                                                        : -1;
        }
    }

    void ValidateConstruction(
        const ContinuousStateErrorTolerances& tolerances) const {
        if (state_size_ <= 0) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: RHS state size must be "
                "positive");
        }
        if (!std::isfinite(public_time_seconds_)) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: initial time must be finite");
        }
        if (public_state_.size() != state_size_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: initial state size does not "
                "match the RHS");
        }
        if (!public_state_.allFinite()) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: initial state must be "
                "finite");
        }
        if (tolerances.component_absolute_tolerances().size() != state_size_) {
            throw std::invalid_argument(
                "CVODE continuous-state advancer: absolute-tolerance size "
                "does not match the RHS");
        }
    }

    void InitializeResources(
        const ContinuousStateErrorTolerances& tolerances) {
        const int context_flag =
            SUNContext_Create(SUN_COMM_NULL, &resources_.context);
        if (context_flag != 0) {
            throw std::runtime_error(
                "CVODE continuous-state advancer: SUNContext_Create failed "
                "with flag " + std::to_string(context_flag));
        }
        resources_.state =
            N_VNew_Serial(static_cast<sunindextype>(state_size_),
                          resources_.context);
        resources_.absolute_tolerances =
            N_VNew_Serial(static_cast<sunindextype>(state_size_),
                          resources_.context);
        resources_.dense_state =
            N_VNew_Serial(static_cast<sunindextype>(state_size_),
                          resources_.context);
        if (resources_.state == nullptr ||
            resources_.absolute_tolerances == nullptr ||
            resources_.dense_state == nullptr) {
            throw std::runtime_error(
                "CVODE continuous-state advancer: N_VNew_Serial failed");
        }
        CopyEigenToNVector(public_state_, resources_.state);
        CopyEigenToNVector(tolerances.component_absolute_tolerances(),
                           resources_.absolute_tolerances);

        resources_.memory = CVodeCreate(CV_BDF, resources_.context);
        if (resources_.memory == nullptr) {
            throw std::runtime_error(
                "CVODE continuous-state advancer: CVodeCreate failed");
        }
        RequireCvodeSuccess(
            CVodeInit(resources_.memory, &Implementation::EvaluateRhs,
                      public_time_seconds_, resources_.state),
            "CVodeInit");
        RequireCvodeSuccess(CVodeSetUserData(resources_.memory, this),
                            "CVodeSetUserData");
        RequireCvodeSuccess(
            CVodeSVtolerances(resources_.memory, tolerances.relative_tolerance(),
                              resources_.absolute_tolerances),
            "CVodeSVtolerances");

        resources_.matrix = SUNDenseMatrix(
            static_cast<sunindextype>(state_size_),
            static_cast<sunindextype>(state_size_), resources_.context);
        if (resources_.matrix == nullptr) {
            throw std::runtime_error(
                "CVODE continuous-state advancer: dense matrix allocation "
                "failed");
        }
        resources_.linear_solver = SUNLinSol_Dense(
            resources_.state, resources_.matrix, resources_.context);
        if (resources_.linear_solver == nullptr) {
            throw std::runtime_error(
                "CVODE continuous-state advancer: dense linear solver "
                "allocation failed");
        }
        RequireCvodeSuccess(
            CVodeSetLinearSolver(resources_.memory, resources_.linear_solver,
                                 resources_.matrix),
            "CVodeSetLinearSolver");
        RequireCvodeSuccess(
            CVodeSetMaxOrd(resources_.memory, kMaximumBdfOrder),
            "CVodeSetMaxOrd");
        RequireCvodeSuccess(
            CVodeSetLSetupFrequency(resources_.memory, kLinearSetupFrequency),
            "CVodeSetLSetupFrequency");
        RequireCvodeSuccess(
            CVodeSetJacEvalFrequency(resources_.memory,
                                     kJacobianEvaluationFrequency),
            "CVodeSetJacEvalFrequency");
        RequireCvodeSuccess(
            CVodeSetMaxNumSteps(resources_.memory,
                               static_cast<long int>(
                                   internal::
                                       kMaximumInternalStepsPerPublicAdvance)),
            "CVodeSetMaxNumSteps");
        RequireCvodeSuccess(
            CVodeSetStabLimDet(resources_.memory, SUNFALSE),
            "CVodeSetStabLimDet");
    }

    void MarkBackendFailure() {
        dense_interval_.reset();
        needs_reinitialization_ = true;
    }

    ContinuousStateRhs* rhs_;
    int state_size_;
    double public_time_seconds_;
    Eigen::VectorXd public_state_;
    CvodeResources resources_;
    std::optional<ContinuousStateDenseOutputInterval> dense_interval_;
    std::exception_ptr rhs_exception_;
    bool needs_reinitialization_{false};
};

CvodeContinuousStateAdvancer::CvodeContinuousStateAdvancer(
    ContinuousStateRhs& rhs,
    double initial_time_seconds,
    Eigen::VectorXd initial_continuous_state,
    ContinuousStateErrorTolerances tolerances)
    : implementation_(std::make_unique<Implementation>(
          rhs, initial_time_seconds, std::move(initial_continuous_state),
          std::move(tolerances))) {}

CvodeContinuousStateAdvancer::~CvodeContinuousStateAdvancer() = default;

int CvodeContinuousStateAdvancer::continuous_state_size() const {
    return implementation_->state_size();
}

double CvodeContinuousStateAdvancer::current_time_seconds() const {
    return implementation_->public_time_seconds();
}

void CvodeContinuousStateAdvancer::CopyCurrentState(
    Eigen::Ref<Eigen::VectorXd> continuous_state) const {
    implementation_->CopyPublicState(continuous_state);
}

ContinuousStateInternalStep
CvodeContinuousStateAdvancer::AdvanceOneInternalStepToward(
    double stop_time_seconds,
    Eigen::Ref<Eigen::VectorXd> endpoint_continuous_state) {
    return implementation_->AdvanceOneInternalStepToward(
        stop_time_seconds, endpoint_continuous_state);
}

void CvodeContinuousStateAdvancer::ReinitializeAfterExternalChange(
    double committed_time_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) {
    implementation_->Reinitialize(committed_time_seconds,
                                  committed_continuous_state);
}

std::optional<ContinuousStateDenseOutputInterval>
CvodeContinuousStateAdvancer::dense_output_interval() const {
    return implementation_->dense_interval();
}

void CvodeContinuousStateAdvancer::CopyDenseState(
    double time_seconds,
    Eigen::Ref<Eigen::VectorXd> continuous_state) const {
    implementation_->CopyDenseState(time_seconds, continuous_state);
}

}  // namespace orvd::integrators
