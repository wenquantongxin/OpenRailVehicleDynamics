#include "newmark_coordinate_layout.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace orvd::integrators::internal {
namespace {

void RequirePositive(double value, const std::string& field) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Newmark configuration: " + field +
                                    " must be positive and finite");
    }
}

void ValidateFamily(const CoordinateNewtonScales& scales, const char* family) {
    const std::string prefix = std::string("nonlinear_solver.") + family + ".";
    RequirePositive(scales.position_correction, prefix + "position_correction");
    RequirePositive(scales.velocity_correction, prefix + "velocity_correction");
    RequirePositive(scales.acceleration_residual, prefix + "acceleration_residual");
    RequirePositive(scales.acceleration_reference, prefix + "acceleration_reference");
}

}  // namespace

void ValidateNewmarkConfiguration(const NewmarkConfiguration& configuration) {
    RequirePositive(configuration.nominal_step_size_seconds, "nominal_step_size_seconds");
    const auto& solver = configuration.nonlinear_solver;
    if (solver.maximum_iterations < 1) {
        throw std::invalid_argument("Newmark configuration: nonlinear_solver.maximum_iterations must be positive");
    }
    ValidateFamily(solver.translation, "translation");
    ValidateFamily(solver.angle, "angle");
    ValidateFamily(solver.quaternion, "quaternion");
    RequirePositive(solver.force.correction, "nonlinear_solver.force.correction");
    RequirePositive(solver.force.residual, "nonlinear_solver.force.residual");
    RequirePositive(solver.force.reference, "nonlinear_solver.force.reference");
}

NewmarkCoordinateLayout::NewmarkCoordinateLayout(
    const multibody_model::MultibodyModel& model, int series_force_state_size)
    : nq_(model.num_generalized_positions()), nz_(series_force_state_size) {
    if (nq_ <= 0 || nz_ < 0 || nq_ > std::numeric_limits<int>::max() - nz_) {
        throw std::invalid_argument("Newmark coordinate layout: invalid coordinate or series-force size");
    }
    std::vector<bool> covered(nq_, false);
    const auto add = [&](int start, int size, Family family) {
        if (size < 0 || start < 0 || start > nq_ - size) {
            throw std::invalid_argument("Newmark coordinate layout: coordinate range is outside [0,nq)");
        }
        for (int i = start; i < start + size; ++i) {
            if (covered[i]) {
                throw std::invalid_argument("Newmark coordinate layout: overlapping coordinate ranges");
            }
            covered[i] = true;
        }
        if (size != 0) blocks_.push_back({start, size, family});
    };
    for (int i = 0; i < model.num_rigid_bodies(); ++i) {
        const auto body = model.GetRigidBody(i);
        if (!model.IsFreeBody(body)) continue;
        const auto range = model.GetFreeBodyPositionRange(body);
        if (range.size() != 7) {
            throw std::invalid_argument("Newmark coordinate layout: unsupported free-body range");
        }
        add(range.start(), 4, Family::kQuaternion);
        add(range.start() + 4, 3, Family::kTranslation);
    }
    using multibody_model::JointType;
    for (int i = 0; i < model.num_joints(); ++i) {
        const auto joint = model.GetJoint(i);
        const auto range = model.GetJointPositionRange(joint);
        const auto type = model.GetJointType(joint);
        Family family;
        int expected_size;
        switch (type) {
            case JointType::kRevolute: family = Family::kAngle; expected_size = 1; break;
            case JointType::kPrismatic: family = Family::kTranslation; expected_size = 1; break;
            case JointType::kBallRpy: family = Family::kAngle; expected_size = 3; break;
            case JointType::kWeld: family = Family::kAngle; expected_size = 0; break;
            default: throw std::invalid_argument("Newmark coordinate layout: unknown joint type");
        }
        if (range.size() != expected_size) {
            throw std::invalid_argument("Newmark coordinate layout: joint type and coordinate range disagree");
        }
        add(range.start(), range.size(), family);
    }
    for (bool present : covered) {
        if (!present) throw std::invalid_argument("Newmark coordinate layout: incomplete coordinate coverage");
    }
}

NewmarkCoreConfiguration NewmarkCoordinateLayout::Expand(
    const NewmarkConfiguration& configuration,
    const Eigen::Ref<const Eigen::VectorXd>& reference_q) const {
    ValidateNewmarkConfiguration(configuration);
    if (reference_q.size() != nq_ || !reference_q.allFinite()) {
        throw std::invalid_argument("Newmark configuration: reference_q has invalid size or non-finite entries");
    }
    NewmarkCoreConfiguration result;
    result.step_size_seconds = configuration.nominal_step_size_seconds;
    auto& expanded = result.nonlinear_solver;
    const auto& solver = configuration.nonlinear_solver;
    expanded.maximum_iterations = solver.maximum_iterations;
    expanded.position_correction_scales.resize(nq_);
    expanded.velocity_correction_scales.resize(nq_);
    expanded.acceleration_residual_scales.resize(nq_);
    expanded.unknown_reference_scales.resize(nq_ + nz_);
    for (const auto& block : blocks_) {
        const CoordinateNewtonScales* scales;
        double factor = 1.0;
        const char* family;
        switch (block.family) {
            case Family::kTranslation: scales = &solver.translation; family = "translation"; break;
            case Family::kAngle: scales = &solver.angle; family = "angle"; break;
            case Family::kQuaternion:
                scales = &solver.quaternion;
                family = "quaternion";
                factor = reference_q.segment<4>(block.start).norm();
                RequirePositive(factor, "reference quaternion norm");
                break;
            default: throw std::logic_error("unknown Newmark coordinate family");
        }
        const auto put = [&](Eigen::VectorXd& target, double value, const char* field) {
            const double scaled = factor * value;
            RequirePositive(scaled, std::string("expanded ") + family + "." + field);
            target.segment(block.start, block.size).setConstant(scaled);
        };
        put(expanded.position_correction_scales, scales->position_correction, "position_correction");
        put(expanded.velocity_correction_scales, scales->velocity_correction, "velocity_correction");
        put(expanded.acceleration_residual_scales, scales->acceleration_residual, "acceleration_residual");
        put(expanded.unknown_reference_scales, scales->acceleration_reference, "acceleration_reference");
    }
    expanded.internal_state_correction_scales = Eigen::VectorXd::Constant(nz_, solver.force.correction);
    expanded.internal_state_residual_scales = Eigen::VectorXd::Constant(nz_, solver.force.residual);
    expanded.unknown_reference_scales.tail(nz_).setConstant(solver.force.reference);
    return result;
}

}  // namespace orvd::integrators::internal
