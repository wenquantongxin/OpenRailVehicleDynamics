#pragma once

#include "coordinate_second_order_problem.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orvd::integrators::internal::testing {

// These small problems have analytic references.  Their Evaluate methods use
// the numerical q/s/z arguments; no accepted velocity is replaced by a reference.
class HarmonicOscillatorProblem final : public CoordinateSecondOrderProblem {
   public:
    explicit HarmonicOscillatorProblem(double frequency = 1.0)
        : frequency_(frequency) {}

    int coordinate_size() const noexcept override { return 1; }
    int internal_state_size() const noexcept override { return 0; }

    void Evaluate(double, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd>) override {
        b[0] = -frequency_ * frequency_ * q[0];
    }

    CoordinateState InitialState() const { return ExactState(0.0); }
    CoordinateState ExactState(double time) const {
        CoordinateState state;
        state.time_seconds = time;
        state.q = Eigen::VectorXd::Constant(1, std::cos(frequency_ * time));
        state.s = Eigen::VectorXd::Constant(
            1, -frequency_ * std::sin(frequency_ * time));
        state.z.resize(0);
        return state;
    }

   private:
    double frequency_;
};

class NonlinearScalarProblem final : public CoordinateSecondOrderProblem {
   public:
    int coordinate_size() const noexcept override { return 1; }
    int internal_state_size() const noexcept override { return 0; }

    void Evaluate(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd>) override {
        const double reference_q = std::cos(time);
        // q'' + q^3 = -cos(t) + cos(t)^3, q(0)=1, q'(0)=0.
        b[0] = -q[0] * q[0] * q[0] - reference_q +
               reference_q * reference_q * reference_q;
    }

    CoordinateState InitialState() const { return ExactState(0.0); }
    CoordinateState ExactState(double time) const {
        CoordinateState state;
        state.time_seconds = time;
        state.q = Eigen::VectorXd::Constant(1, std::cos(time));
        state.s = Eigen::VectorXd::Constant(1, -std::sin(time));
        state.z.resize(0);
        return state;
    }
};

class MaxwellCoupledProblem final : public CoordinateSecondOrderProblem {
   public:
    explicit MaxwellCoupledProblem(double relaxation_rate = 3.0,
                                   double series_stiffness = 2.0,
                                   double initial_force = 0.7)
        : relaxation_rate_(relaxation_rate),
          series_stiffness_(series_stiffness),
          initial_force_(initial_force) {}

    int coordinate_size() const noexcept override { return 1; }
    int internal_state_size() const noexcept override { return 1; }

    void Evaluate(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& s,
                  const Eigen::Ref<const Eigen::VectorXd>& z,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd> g) override {
        // A genuine mechanical/Maxwell feedback pair with manufactured load;
        // only the applied load is prescribed, not the evolved force or velocity.
        const double applied_load =
            std::cos(time) - 0.3 * std::sin(time) + ExactForce(time);
        b[0] = -2.0 * q[0] - 0.3 * s[0] - z[0] + applied_load;
        g[0] = series_stiffness_ * s[0] - relaxation_rate_ * z[0];
    }

    CoordinateState InitialState() const { return ExactState(0.0); }
    CoordinateState ExactState(double time) const {
        CoordinateState state;
        state.time_seconds = time;
        state.q = Eigen::VectorXd::Constant(1, std::cos(time));
        state.s = Eigen::VectorXd::Constant(1, -std::sin(time));
        state.z = Eigen::VectorXd::Constant(1, ExactForce(time));
        return state;
    }

   private:
    double ExactForce(double time) const {
        const double coefficient =
            series_stiffness_ / (1.0 + relaxation_rate_ * relaxation_rate_);
        return coefficient * (std::cos(time) - relaxation_rate_ * std::sin(time)) +
               (initial_force_ - coefficient) *
                   std::exp(-relaxation_rate_ * time);
    }

    double relaxation_rate_;
    double series_stiffness_;
    double initial_force_;
};

namespace fixture_detail {

inline Eigen::Matrix3d RpyRotation(const Eigen::Vector3d& q) {
    return (Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

inline Eigen::Matrix3d RpyNplus(const Eigen::Vector3d& q) {
    const double cp = std::cos(q[1]);
    const double sp = std::sin(q[1]);
    const double cy = std::cos(q[2]);
    const double sy = std::sin(q[2]);
    Eigen::Matrix3d matrix;
    matrix << cy * cp, -sy, 0.0,
              sy * cp,  cy, 0.0,
                   -sp, 0.0, 1.0;
    return matrix;
}

inline Eigen::Vector3d RpyVelocityToRate(const Eigen::Vector3d& q,
                                        const Eigen::Vector3d& omega) {
    const double cp = std::cos(q[1]);
    if (std::abs(cp) < 1e-3) {
        throw std::domain_error("RPY fixture: pitch outside the rate-map domain");
    }
    const double roll_rate =
        (std::cos(q[2]) * omega[0] + std::sin(q[2]) * omega[1]) / cp;
    return {roll_rate,
            -std::sin(q[2]) * omega[0] + std::cos(q[2]) * omega[1],
            std::sin(q[1]) * roll_rate + omega[2]};
}

inline Eigen::Vector3d RpyAccelerationBias(const Eigen::Vector3d& q,
                                         const Eigen::Vector3d& s) {
    // d/dt(Nplus(q)) * s, with q'=s and angular velocity expressed in the parent.
    const double cp = std::cos(q[1]);
    const double sp = std::sin(q[1]);
    const double cy = std::cos(q[2]);
    const double sy = std::sin(q[2]);
    return {(-sy * s[2] * cp - cy * sp * s[1]) * s[0] - cy * s[2] * s[1],
            (cy * s[2] * cp - sy * sp * s[1]) * s[0] - sy * s[2] * s[1],
            -cp * s[1] * s[0]};
}

struct PrescribedRotation {
    Eigen::Vector3d rpy;
    Eigen::Vector3d rates;
    Eigen::Vector3d second_derivatives;
    Eigen::Vector3d omega;
    Eigen::Vector3d alpha;
};

inline PrescribedRotation RotationReference(double time) {
    PrescribedRotation result;
    result.rpy = {0.25 * std::sin(1.3 * time) + 0.1 * time,
                  0.2 * std::cos(0.7 * time),
                  0.3 * std::sin(0.9 * time) + 0.15 * time};
    result.rates = {0.325 * std::cos(1.3 * time) + 0.1,
                   -0.14 * std::sin(0.7 * time),
                   0.27 * std::cos(0.9 * time) + 0.15};
    result.second_derivatives = {-0.4225 * std::sin(1.3 * time),
                                -0.098 * std::cos(0.7 * time),
                                -0.243 * std::sin(0.9 * time)};
    result.omega = RpyNplus(result.rpy) * result.rates;
    result.alpha = RpyNplus(result.rpy) * result.second_derivatives +
                   RpyAccelerationBias(result.rpy, result.rates);
    return result;
}

inline Eigen::Vector4d QuaternionRate(const Eigen::Vector4d& q,
                                     const Eigen::Vector3d& omega) {
    Eigen::Vector4d result;
    result[0] = -0.5 * omega.dot(q.tail<3>());
    result.tail<3>() = 0.5 * (q[0] * omega + omega.cross(q.tail<3>()));
    return result;
}

inline Eigen::Vector3d QuaternionPhysicalVelocity(const Eigen::Vector4d& q,
                                                 const Eigen::Vector4d& s) {
    const double norm_squared = q.squaredNorm();
    if (!(norm_squared > 0.0) || !std::isfinite(norm_squared)) {
        throw std::domain_error("quaternion fixture: invalid quaternion norm");
    }
    return (2.0 / norm_squared) *
           (q[0] * s.tail<3>() - s[0] * q.tail<3>() + q.tail<3>().cross(s.tail<3>()));
}

}  // namespace fixture_detail

class RpyRotationProblem final : public CoordinateSecondOrderProblem {
   public:
    explicit RpyRotationProblem(bool omit_convective_term = false)
        : omit_convective_term_(omit_convective_term) {}

    int coordinate_size() const noexcept override { return 3; }
    int internal_state_size() const noexcept override { return 0; }

    void Evaluate(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& s,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd>) override {
        Eigen::Vector3d physical_acceleration = fixture_detail::RotationReference(time).alpha;
        if (!omit_convective_term_) {
            physical_acceleration -= fixture_detail::RpyAccelerationBias(q, s);
        }
        b = fixture_detail::RpyVelocityToRate(q, physical_acceleration);
    }

    CoordinateState InitialState() const { return ExactState(0.0); }
    CoordinateState ExactState(double time) const {
        const auto reference = fixture_detail::RotationReference(time);
        CoordinateState state;
        state.time_seconds = time;
        state.q = reference.rpy;
        state.s = reference.rates;
        state.z.resize(0);
        return state;
    }

    Eigen::Matrix3d Rotation(const Eigen::VectorXd& q) const {
        return fixture_detail::RpyRotation(q);
    }
    Eigen::Vector3d PhysicalAngularVelocity(const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& s) const {
        return fixture_detail::RpyNplus(q) * s;
    }

   private:
    bool omit_convective_term_;
};

class QuaternionRotationProblem final : public CoordinateSecondOrderProblem {
   public:
    explicit QuaternionRotationProblem(double reference_norm = 1.7)
        : reference_norm_(reference_norm) {}

    int coordinate_size() const noexcept override { return 4; }
    int internal_state_size() const noexcept override { return 0; }

    void ValidateInitialState(double, const Eigen::Ref<const Eigen::VectorXd>& q,
                              const Eigen::Ref<const Eigen::VectorXd>& s,
                              const Eigen::Ref<const Eigen::VectorXd>&) const override {
        (void)fixture_detail::QuaternionPhysicalVelocity(q, s);
        const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() *
                                 q.norm() * std::max(1.0, s.norm());
        if (std::abs(q.dot(s)) > tolerance) {
            throw std::invalid_argument("quaternion fixture: initial rate is not tangent");
        }
    }

    void Evaluate(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& s,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd>) override {
        const Eigen::Vector3d omega = fixture_detail::QuaternionPhysicalVelocity(q, s);
        const Eigen::Vector3d alpha = fixture_detail::RotationReference(time).alpha;
        b = fixture_detail::QuaternionRate(q, alpha) - 0.25 * omega.squaredNorm() * q;
    }

    bool ProjectEndpoint(const Eigen::Ref<const Eigen::VectorXd>& q_reference,
                         Eigen::Ref<Eigen::VectorXd> q,
                         Eigen::Ref<Eigen::VectorXd> s) override {
        const Eigen::Vector3d omega = fixture_detail::QuaternionPhysicalVelocity(q, s);
        const Eigen::Vector4d projected_q = q_reference.norm() * q / q.norm();
        const Eigen::Vector4d projected_s = fixture_detail::QuaternionRate(projected_q, omega);
        const bool changed = (q.array() != projected_q.array()).any() ||
                             (s.array() != projected_s.array()).any();
        q = projected_q;
        s = projected_s;
        return changed;
    }

    CoordinateState InitialState() const { return ExactState(0.0); }
    CoordinateState ExactState(double time) const {
        const auto reference = fixture_detail::RotationReference(time);
        const Eigen::Quaterniond rotation(fixture_detail::RpyRotation(reference.rpy));
        CoordinateState state;
        state.time_seconds = time;
        state.q = reference_norm_ * Eigen::Vector4d(rotation.w(), rotation.x(),
                                                   rotation.y(), rotation.z());
        state.s = fixture_detail::QuaternionRate(state.q, reference.omega);
        state.z.resize(0);
        return state;
    }

    Eigen::Matrix3d Rotation(const Eigen::VectorXd& q) const {
        return Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
    }
    Eigen::Vector3d PhysicalAngularVelocity(const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& s) const {
        return fixture_detail::QuaternionPhysicalVelocity(q, s);
    }

   private:
    double reference_norm_;
};

inline double RotationAngleError(const Eigen::Matrix3d& actual,
                                 const Eigen::Matrix3d& expected) {
    return Eigen::AngleAxisd(expected.transpose() * actual).angle();
}

}  // namespace orvd::integrators::internal::testing
