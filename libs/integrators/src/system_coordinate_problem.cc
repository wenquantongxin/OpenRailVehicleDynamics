#include "system_coordinate_problem.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

namespace orvd::integrators::internal {
namespace {

[[noreturn]] void Reject(const char* detail) {
    throw std::invalid_argument(std::string("system coordinate problem: ") + detail);
}

void RequireFiniteMapping(const Eigen::VectorXd& result) {
    if (!result.allFinite()) {
        throw CoordinateIntegrationFailure(
            CoordinateIntegrationFailure::Reason::kNonFiniteEvaluation,
            "system coordinate problem: non-finite mapped result");
    }
}

bool Overlap(const Eigen::Ref<Eigen::VectorXd>& first,
             const Eigen::Ref<Eigen::VectorXd>& second) {
    if (first.size() == 0 || second.size() == 0) return false;
    const std::less<const double*> less;
    return less(first.data(), second.data() + second.size()) &&
           less(second.data(), first.data() + first.size());
}

const multibody_model::MultibodyModel& GetComponentModel(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& context) {
    const auto component = system.GetMultibodyComponentView(context, plan.derivative_component());
    // The view borrows the system's model; it does not own this reference.
    return component.model();
}

}  // namespace

class SystemCoordinateProblem::Implementation final {
   public:
    Implementation(const system_assembly::SystemInstance& system,
                   const system_assembly::CompiledSystemPlan& plan,
                   system_assembly::SystemRuntimeContext& trial_context,
                   NoCallTimeAppliedForces forces)
        : system_(system),
          plan_(plan),
          trial_context_(trial_context),
          rhs_(system, plan, trial_context, forces),
          model_(GetComponentModel(system, plan, trial_context)),
          q_range_(system.generalized_positions_state_range()),
          v_range_(system.generalized_velocities_state_range()),
          z_range_(system.series_spring_damper_force_state_range()),
          nq_(model_.num_generalized_positions()),
          nv_(model_.num_generalized_velocities()),
          nz_(z_range_.size()) {
        // The current compiled graph is exactly one multibody plus its series
        // force states. A future additional state family must not be silently
        // interpreted as z by this bridge.
        if (nq_ <= 0 || q_range_.start() != 0 || q_range_.size() != nq_ ||
            v_range_.start() != nq_ || v_range_.size() != nv_ ||
            z_range_.start() != nq_ + nv_ || nz_ < 0 ||
            z_range_.start() + nz_ != system_.continuous_state_size()) {
            Reject("unsupported coordinate state layout");
        }
        for (int i = 0; i < model_.num_rigid_bodies(); ++i) {
            const auto body = model_.GetRigidBody(i);
            if (model_.IsFreeBody(body)) {
                const auto range = model_.GetFreeBodyPositionRange(body);
                if (range.size() != 7 || range.start() < 0 ||
                    range.start() > nq_ - 7) {
                    Reject("unsupported free-body coordinate layout");
                }
                quaternion_starts_.push_back(range.start());
            }
        }
        geometry_context_ = model_.CreateDefaultContext();
        q_work_.resize(nq_);
        s_work_.resize(nq_);
        v_work_.resize(nv_);
        mapped_s_.resize(nq_);
        physical_work_.resize(system_.continuous_state_size());
        derivatives_.resize(system_.continuous_state_size());
        a_work_.resize(nv_);
        b_work_.resize(nq_);
    }

    void ValidateCoordinates(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                             const Eigen::Ref<const Eigen::VectorXd>& s,
                             const Eigen::Ref<const Eigen::VectorXd>& z) const {
        if (!std::isfinite(time) || q.size() != nq_ || s.size() != nq_ ||
            z.size() != nz_ || !q.allFinite() || !s.allFinite() || !z.allFinite()) {
            Reject("coordinate state has invalid size or non-finite data");
        }
    }

    void RecoverVelocity(const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& s) const {
        q_work_ = q;
        s_work_ = s;
        model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);
        model_.MapGeneralizedPositionDerivativesToVelocities(
            *geometry_context_, s_work_, &v_work_);
        RequireFiniteMapping(v_work_);
    }

    void MapVelocity() const {
        // In particular, this checks the existing Ball-RPY singular domain;
        // Nplus by itself has no division by cos(pitch) and cannot do so.
        model_.MapGeneralizedVelocitiesToPositionDerivatives(
            *geometry_context_, v_work_, &mapped_s_);
        RequireFiniteMapping(mapped_s_);
    }

    CoordinateState MakeCoordinateState(
        double time, const Eigen::Ref<const Eigen::VectorXd>& physical) const {
        if (!std::isfinite(time) || physical.size() != system_.continuous_state_size() ||
            !physical.allFinite()) {
            Reject("physical state has invalid size or non-finite data");
        }
        q_work_ = physical.segment(q_range_.start(), nq_);
        v_work_ = physical.segment(v_range_.start(), nv_);
        model_.SetGeneralizedState(geometry_context_.get(), q_work_, v_work_);
        MapVelocity();
        return {time, q_work_, mapped_s_, physical.segment(z_range_.start(), nz_)};
    }

    void CopyPhysicalState(const CoordinateState& coordinate,
                           Eigen::Ref<Eigen::VectorXd> physical) const {
        if (physical.size() != system_.continuous_state_size()) {
            Reject("physical state output has the wrong size");
        }
        ValidateCoordinates(coordinate.time_seconds, coordinate.q, coordinate.s, coordinate.z);
        RecoverVelocity(coordinate.q, coordinate.s);
        MapVelocity();
        PackPhysicalState(coordinate.z);
        physical = physical_work_;
    }

    void CopyLinearlyInterpolatedPhysicalState(
        const Eigen::Ref<const Eigen::VectorXd>& reference_q,
        const Eigen::Ref<const Eigen::VectorXd>& start_physical,
        const Eigen::Ref<const Eigen::VectorXd>& end_physical, double fraction,
        Eigen::Ref<Eigen::VectorXd> physical) const {
        const int physical_size = system_.continuous_state_size();
        if (reference_q.size() != nq_ || start_physical.size() != physical_size ||
            end_physical.size() != physical_size || physical.size() != physical_size ||
            !reference_q.allFinite() || !start_physical.allFinite() ||
            !end_physical.allFinite() || !std::isfinite(fraction) ||
            fraction < 0.0 || fraction > 1.0) {
            Reject("physical interpolation has invalid size, values or fraction");
        }
        // Preserve the model's quaternion storage domain without asking N or
        // Nplus to interpret a purely observational RPY sample as an ODE state.
        q_work_ = reference_q;
        model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);
        q_work_ = start_physical.head(nq_);
        model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);
        q_work_ = end_physical.head(nq_);
        model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);

        if (fraction == 0.0) {
            physical_work_ = start_physical;
        } else if (fraction == 1.0) {
            physical_work_ = end_physical;
        } else {
            for (int i = 0; i != physical_size; ++i) {
                physical_work_[i] = std::lerp(start_physical[i], end_physical[i], fraction);
            }
            for (const int start : quaternion_starts_) {
                const Eigen::Vector4d first = start_physical.segment<4>(start).normalized();
                Eigen::Vector4d second = end_physical.segment<4>(start).normalized();
                if (first.dot(second) < 0.0) second = -second;
                Eigen::Vector4d blended;
                for (int i = 0; i != 4; ++i) {
                    blended[i] = std::lerp(first[i], second[i], fraction);
                }
                physical_work_.segment<4>(start) =
                    blended.normalized() * reference_q.segment<4>(start).norm();
            }
            RequireFiniteMapping(physical_work_);
            q_work_ = physical_work_.head(nq_);
            model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);
        }
        // Stage the full result before publication, including exact endpoints,
        // so overlapping input/output spans cannot partially overwrite inputs.
        physical = physical_work_;
    }

    void PackPhysicalState(const Eigen::Ref<const Eigen::VectorXd>& z) const {
        physical_work_.segment(q_range_.start(), nq_) = q_work_;
        physical_work_.segment(v_range_.start(), nv_) = v_work_;
        physical_work_.segment(z_range_.start(), nz_) = z;
    }

    void ValidateInitial(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& s,
                         const Eigen::Ref<const Eigen::VectorXd>& z) const {
        ValidateCoordinates(time, q, s, z);
        RecoverVelocity(q, s);
        MapVelocity();
        for (const int start : quaternion_starts_) {
            const Eigen::Vector4d unit_q = q.segment<4>(start) / q.segment<4>(start).norm();
            const auto rate = s.segment<4>(start);
            const double radial_rate = unit_q.dot(rate);
            const double scale = std::max(1.0, rate.stableNorm());
            if (!std::isfinite(radial_rate) ||
                std::abs(radial_rate) > 64.0 * std::numeric_limits<double>::epsilon() * scale) {
                Reject("initial quaternion coordinate rate is not tangent");
            }
        }
    }

    void Evaluate(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& s,
                  const Eigen::Ref<const Eigen::VectorXd>& z,
                  Eigen::Ref<Eigen::VectorXd> b, Eigen::Ref<Eigen::VectorXd> g) {
        if (b.size() != nq_ || g.size() != nz_ || Overlap(b, g)) {
            Reject("derivative outputs have the wrong size or overlap");
        }
        ValidateCoordinates(time, q, s, z);
        RecoverVelocity(q, s);
        PackPhysicalState(z);
        rhs_.CalcTimeDerivatives(time, physical_work_, derivatives_);
        RequireFiniteMapping(derivatives_);
        a_work_ = derivatives_.segment(v_range_.start(), nv_);
        const auto component = system_.GetMultibodyComponentView(
            trial_context_, plan_.derivative_component());
        model_.MapGeneralizedVelocityDerivativesToPositionSecondDerivatives(
            component.context(), a_work_, &b_work_);
        b = b_work_;
        g = derivatives_.segment(z_range_.start(), nz_);
    }

    bool Project(const Eigen::Ref<const Eigen::VectorXd>& reference,
                 Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> s) {
        if (reference.size() != nq_ || q.size() != nq_ || s.size() != nq_ ||
            !reference.allFinite() || !q.allFinite() || !s.allFinite() || Overlap(q, s)) {
            Reject("projection inputs have invalid size, values or overlap");
        }
        // Validate the supplied reference with the model's existing quaternion
        // domain gate. It is scratch, never an initialization history update.
        q_work_ = reference;
        model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);
        RecoverVelocity(q, s);
        for (const int start : quaternion_starts_) {
            const double norm = q_work_.segment<4>(start).norm();
            const double reference_norm = reference.segment<4>(start).norm();
            const Eigen::Vector4d unit_q = q_work_.segment<4>(start) / norm;
            q_work_.segment<4>(start) = unit_q * reference_norm;
        }
        model_.SetGeneralizedPositions(geometry_context_.get(), q_work_);
        MapVelocity();
        bool changed = false;
        for (const int start : quaternion_starts_) {
            const bool block_changed =
                (q.segment<4>(start).array() != q_work_.segment<4>(start).array()).any() ||
                (s.segment<4>(start).array() != mapped_s_.segment<4>(start).array()).any();
            changed = changed || block_changed;
        }
        // Both candidate arrays change only after every mapping has succeeded.
        // Do not round-trip unaffected RPY, translational or scalar rates.
        for (const int start : quaternion_starts_) {
            q.segment<4>(start) = q_work_.segment<4>(start);
            s.segment<4>(start) = mapped_s_.segment<4>(start);
        }
        return changed;
    }

    const system_assembly::SystemInstance& system_;
    const system_assembly::CompiledSystemPlan& plan_;
    system_assembly::SystemRuntimeContext& trial_context_;
    SystemRhsBridge rhs_;
    const multibody_model::MultibodyModel& model_;
    const system_assembly::SystemContinuousStateRange q_range_;
    const system_assembly::SystemContinuousStateRange v_range_;
    const system_assembly::SystemContinuousStateRange z_range_;
    const int nq_;
    const int nv_;
    const int nz_;
    std::vector<int> quaternion_starts_;
    std::unique_ptr<multibody_model::MultibodyEvaluationContext> geometry_context_;
    mutable Eigen::VectorXd q_work_;
    mutable Eigen::VectorXd s_work_;
    mutable Eigen::VectorXd v_work_;
    mutable Eigen::VectorXd mapped_s_;
    mutable Eigen::VectorXd physical_work_;
    Eigen::VectorXd derivatives_;
    Eigen::VectorXd a_work_;
    Eigen::VectorXd b_work_;
};

SystemCoordinateProblem::SystemCoordinateProblem(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& trial_context, NoCallTimeAppliedForces forces)
    : implementation_(std::make_unique<Implementation>(system, plan, trial_context, forces)) {}

SystemCoordinateProblem::~SystemCoordinateProblem() = default;

int SystemCoordinateProblem::coordinate_size() const { return implementation_->nq_; }

int SystemCoordinateProblem::internal_state_size() const { return implementation_->nz_; }

int SystemCoordinateProblem::physical_state_size() const {
    return implementation_->system_.continuous_state_size();
}

CoordinateState SystemCoordinateProblem::MakeCoordinateState(
    double time_seconds, const Eigen::Ref<const Eigen::VectorXd>& physical_state) const {
    return implementation_->MakeCoordinateState(time_seconds, physical_state);
}

void SystemCoordinateProblem::CopyPhysicalState(
    const CoordinateState& coordinate_state, Eigen::Ref<Eigen::VectorXd> physical_state) const {
    implementation_->CopyPhysicalState(coordinate_state, physical_state);
}

void SystemCoordinateProblem::CopyLinearlyInterpolatedPhysicalState(
    const Eigen::Ref<const Eigen::VectorXd>& reference_q,
    const Eigen::Ref<const Eigen::VectorXd>& start_physical,
    const Eigen::Ref<const Eigen::VectorXd>& end_physical, double fraction,
    Eigen::Ref<Eigen::VectorXd> physical_state) const {
    implementation_->CopyLinearlyInterpolatedPhysicalState(
        reference_q, start_physical, end_physical, fraction, physical_state);
}

void SystemCoordinateProblem::SynchronizeContextLocalDataFrom(
    const system_assembly::SystemRuntimeContext& source_context) {
    implementation_->rhs_.SynchronizeContextLocalDataFrom(source_context);
}

void SystemCoordinateProblem::ValidateInitialState(
    double time, const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& s, const Eigen::Ref<const Eigen::VectorXd>& z) const {
    implementation_->ValidateInitial(time, q, s, z);
}

void SystemCoordinateProblem::Evaluate(
    double time, const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& s, const Eigen::Ref<const Eigen::VectorXd>& z,
    Eigen::Ref<Eigen::VectorXd> b, Eigen::Ref<Eigen::VectorXd> g) {
    implementation_->Evaluate(time, q, s, z, b, g);
}

bool SystemCoordinateProblem::ProjectEndpoint(
    const Eigen::Ref<const Eigen::VectorXd>& reference, Eigen::Ref<Eigen::VectorXd> q,
    Eigen::Ref<Eigen::VectorXd> s) {
    return implementation_->Project(reference, q, s);
}

}  // namespace orvd::integrators::internal
