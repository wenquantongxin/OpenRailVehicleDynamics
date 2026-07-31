// The generalized mass matrix through the public modelling boundary.
//
// The main oracle does not call another mass-matrix implementation. It recovers
// every matrix entry from the kinetic energy of basis velocities and pairwise
// sums, with each body's energy assembled from its public pose and spatial
// velocity. Symmetry then closes the antisymmetric part that energy cannot see.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyEvaluationContext;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

using MassMatrixMember = void (MultibodyModel::*)(
    const MultibodyEvaluationContext&, Eigen::MatrixXd&) const;
static_assert(std::is_same_v<
              decltype(static_cast<MassMatrixMember>(
                  &MultibodyModel::CalcGeneralizedMassMatrix)),
              MassMatrixMember>);

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

template <typename Attempt>
bool InvalidArgumentMentions(Attempt&& attempt, std::string_view fragment) {
    try {
        attempt();
    } catch (const std::invalid_argument& reason) {
        return std::string_view(reason.what()).find(fragment) !=
               std::string_view::npos;
    } catch (...) {
        return false;
    }
    return false;
}

template <typename Attempt>
bool LogicErrorButNotInvalidArgumentMentions(Attempt&& attempt,
                                             std::string_view fragment) {
    try {
        attempt();
    } catch (const std::invalid_argument&) {
        // invalid_argument derives from logic_error; the public contract makes
        // the distinction, so catch the narrower type first.
        return false;
    } catch (const std::logic_error& reason) {
        return std::string_view(reason.what()).find(fragment) !=
               std::string_view::npos;
    } catch (...) {
        return false;
    }
    return false;
}

RigidBodyInertiaParameters MakeInertia(
    double mass_kilograms,
    const Eigen::Vector3d& center_of_mass_in_body_frame,
    const Eigen::Matrix3d& central_unit_inertia) {
    const Eigen::Matrix3d parallel_axis =
        center_of_mass_in_body_frame.squaredNorm() *
            Eigen::Matrix3d::Identity() -
        center_of_mass_in_body_frame *
            center_of_mass_in_body_frame.transpose();
    const Eigen::Matrix3d unit_inertia_about_body_origin =
        central_unit_inertia + parallel_axis;

    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass_kilograms;
    inertia.center_of_mass_in_body_frame = center_of_mass_in_body_frame;
    inertia.unit_inertia_moments =
        unit_inertia_about_body_origin.diagonal();
    inertia.unit_inertia_products = Eigen::Vector3d(
        unit_inertia_about_body_origin(0, 1),
        unit_inertia_about_body_origin(0, 2),
        unit_inertia_about_body_origin(1, 2));
    return inertia;
}

struct BodyEnergyData {
    RigidBodyHandle body;
    double mass_kilograms{};
    Eigen::Vector3d center_of_mass_in_body_frame{};
    Eigen::Matrix3d central_unit_inertia{};
};

void SetJointCoordinate(const MultibodyModel& model, JointHandle joint,
                        double value, Eigen::VectorXd& positions) {
    const auto range = model.GetJointPositionRange(joint);
    positions[range.start()] = value;
}

void SetJointVelocity(const MultibodyModel& model, JointHandle joint,
                      double value, Eigen::VectorXd& velocities) {
    const auto range = model.GetJointVelocityRange(joint);
    velocities[range.start()] = value;
}

class RichMassFixture {
   public:
    RichMassFixture() {
        Eigen::Matrix3d leaf_central;
        leaf_central << 0.24, 0.012, -0.009, 0.012, 0.31, 0.014, -0.009,
            0.014, 0.37;
        Eigen::Matrix3d root_central;
        root_central << 0.29, -0.011, 0.008, -0.011, 0.36, 0.013, 0.008,
            0.013, 0.43;
        Eigen::Matrix3d welded_central;
        welded_central << 0.19, 0.007, -0.006, 0.007, 0.23, 0.005, -0.006,
            0.005, 0.27;
        Eigen::Matrix3d free_central;
        free_central << 0.21, -0.008, 0.006, -0.008, 0.26, 0.009, 0.006,
            0.009, 0.32;

        // Deliberately add the leaf before its root. Public body order is thus
        // [leaf, root, welded, free], whereas the forest must mobilize root
        // before leaf. A body-ordinal-as-Mobod implementation cannot hide here.
        const Eigen::Vector3d leaf_com(-0.09, 0.06, 0.04);
        leaf = model.AddRigidBody(
            "leaf", MakeInertia(1.4, leaf_com, leaf_central));
        bodies.push_back({leaf, 1.4, leaf_com, leaf_central});

        const Eigen::Vector3d root_com(0.08, -0.05, 0.03);
        root = model.AddRigidBody(
            "root", MakeInertia(2.1, root_com, root_central));
        bodies.push_back({root, 2.1, root_com, root_central});

        const Eigen::Vector3d welded_com(0.05, 0.07, -0.04);
        welded = model.AddRigidBody(
            "welded", MakeInertia(0.8, welded_com, welded_central));
        bodies.push_back({welded, 0.8, welded_com, welded_central});

        const Eigen::Vector3d free_com(-0.04, -0.06, 0.08);
        free = model.AddRigidBody(
            "free", MakeInertia(1.1, free_com, free_central));
        bodies.push_back({free, 1.1, free_com, free_central});

        root_joint = model.AddRevoluteJoint(
            "root_joint", model.world_frame(), model.body_frame(root),
            Eigen::Vector3d(0.31, -0.47, 0.83), 0.0);

        FixedFramePoseParameters slider_mount_pose;
        slider_mount_pose.R_PF =
            Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitY())
                .toRotationMatrix();
        slider_mount_pose.p_PoFo_P = Eigen::Vector3d(0.34, -0.16, 0.22);
        const auto slider_mount =
            model.AddFixedFrame("slider_mount", root, slider_mount_pose);
        slider_joint = model.AddPrismaticJoint(
            "slider_joint", slider_mount, model.body_frame(leaf),
            Eigen::Vector3d(0.91, 0.28, -0.19), 0.0);

        FixedFramePoseParameters weld_mount_pose;
        weld_mount_pose.R_PF =
            Eigen::AngleAxisd(-0.29, Eigen::Vector3d::UnitX())
                .toRotationMatrix();
        weld_mount_pose.p_PoFo_P = Eigen::Vector3d(-0.21, 0.18, 0.13);
        const auto weld_mount =
            model.AddFixedFrame("weld_mount", leaf, weld_mount_pose);
        model.AddWeldJoint("weld_joint", weld_mount,
                           model.body_frame(welded));
        model.DeclareFreeBody(free);
        model.Finalize();

        positions = Eigen::VectorXd::Zero(model.num_generalized_positions());
        SetJointCoordinate(model, root_joint, 0.43, positions);
        SetJointCoordinate(model, slider_joint, -0.24, positions);
        const auto free_positions = model.GetFreeBodyPositionRange(free);
        positions.segment<7>(free_positions.start()) << 1.6, -0.3, 0.7, 0.4,
            0.38, -0.27, 0.19;

        velocities = Eigen::VectorXd::Zero(model.num_generalized_velocities());
        SetJointVelocity(model, root_joint, 0.76, velocities);
        SetJointVelocity(model, slider_joint, -0.39, velocities);
        const auto free_velocities = model.GetFreeBodyVelocityRange(free);
        velocities.segment<6>(free_velocities.start()) << 0.41, -0.28, 0.33,
            -0.22, 0.37, 0.26;
    }

    std::unique_ptr<MultibodyEvaluationContext> MakeContext() const {
        auto context = model.CreateDefaultContext();
        model.SetGeneralizedPositions(context.get(), positions);
        model.SetGeneralizedVelocities(context.get(), velocities);
        return context;
    }

    double CalcKineticEnergy(const MultibodyEvaluationContext& context) const {
        double energy_joules = 0.0;
        for (const BodyEnergyData& data : bodies) {
            const auto pose = model.CalcPoseInWorld(context, data.body);
            const auto velocity =
                model
                    .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        context, data.body);
            const Eigen::Vector3d angular_velocity =
                velocity.angular_velocity_radians_per_second();
            const Eigen::Vector3d center_offset_in_world =
                pose.rotation() * data.center_of_mass_in_body_frame;
            const Eigen::Vector3d center_velocity =
                velocity
                    .translational_velocity_at_frame_origin_meters_per_second() +
                angular_velocity.cross(center_offset_in_world);
            const Eigen::Matrix3d central_inertia_in_world =
                data.mass_kilograms * pose.rotation() *
                data.central_unit_inertia * pose.rotation().transpose();
            energy_joules +=
                0.5 * data.mass_kilograms * center_velocity.squaredNorm() +
                0.5 * angular_velocity.dot(central_inertia_in_world *
                                           angular_velocity);
        }
        return energy_joules;
    }

    MultibodyModel model;
    RigidBodyHandle leaf;
    RigidBodyHandle root;
    RigidBodyHandle welded;
    RigidBodyHandle free;
    JointHandle root_joint;
    JointHandle slider_joint;
    std::vector<BodyEnergyData> bodies;
    Eigen::VectorXd positions;
    Eigen::VectorXd velocities;
};

void ExpectMatrixNear(const Eigen::MatrixXd& actual,
                      const Eigen::MatrixXd& expected, double multiplier,
                      const std::string& description) {
    const double scale = std::max(1.0, expected.cwiseAbs().maxCoeff());
    const double tolerance =
        multiplier * std::numeric_limits<double>::epsilon() * scale;
    const double error = (actual - expected).cwiseAbs().maxCoeff();
    Expect(error <= tolerance,
           description + ": max error " + std::to_string(error) +
               ", tolerance " + std::to_string(tolerance));
}

Eigen::MatrixXd RecoverMassMatrixFromKineticEnergy(
    const RichMassFixture& fixture, MultibodyEvaluationContext* context) {
    const int velocity_count = fixture.model.num_generalized_velocities();
    Eigen::VectorXd basis_energies(velocity_count);
    Eigen::MatrixXd recovered =
        Eigen::MatrixXd::Zero(velocity_count, velocity_count);
    for (int column = 0; column < velocity_count; ++column) {
        Eigen::VectorXd velocity =
            Eigen::VectorXd::Zero(velocity_count);
        velocity[column] = 1.0;
        fixture.model.SetGeneralizedVelocities(context, velocity);
        basis_energies[column] = fixture.CalcKineticEnergy(*context);
        recovered(column, column) = 2.0 * basis_energies[column];
    }
    for (int row = 0; row < velocity_count; ++row) {
        for (int column = row + 1; column < velocity_count; ++column) {
            Eigen::VectorXd velocity =
                Eigen::VectorXd::Zero(velocity_count);
            velocity[row] = 1.0;
            velocity[column] = 1.0;
            fixture.model.SetGeneralizedVelocities(context, velocity);
            const double cross_term = fixture.CalcKineticEnergy(*context) -
                                      basis_energies[row] -
                                      basis_energies[column];
            recovered(row, column) = cross_term;
            recovered(column, row) = cross_term;
        }
    }
    return recovered;
}

void CheckRichMassMatrix() {
    RichMassFixture fixture;
    Expect(fixture.model.GetRigidBody(0) == fixture.leaf &&
               fixture.model.GetRigidBody(1) == fixture.root,
           "the public body order keeps the deliberately leaf-before-root "
           "construction order");
    auto context = fixture.MakeContext();
    const int velocity_count = fixture.model.num_generalized_velocities();
    Eigen::MatrixXd mass_matrix(velocity_count, velocity_count);
    const double* const storage_before = mass_matrix.data();
    fixture.model.CalcGeneralizedMassMatrix(*context, mass_matrix);
    Expect(mass_matrix.data() == storage_before,
           "a correctly sized mass-matrix output retains its allocation");

    const Eigen::MatrixXd recovered =
        RecoverMassMatrixFromKineticEnergy(fixture, context.get());
    ExpectMatrixNear(mass_matrix, recovered, 4096.0,
                     "kinetic-energy polarization recovers every mass-matrix "
                     "entry");
    ExpectMatrixNear(mass_matrix, mass_matrix.transpose(), 256.0,
                     "the mass matrix is symmetric to representation precision");

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(
        0.5 * (mass_matrix + mass_matrix.transpose()));
    Expect(eigensolver.info() == Eigen::Success,
           "the qualified mass matrix has a finite eigendecomposition");
    if (eigensolver.info() == Eigen::Success) {
        Expect(eigensolver.eigenvalues().minCoeff() > 1e-3,
               "the qualified fixture is strictly away from a singular mass "
               "matrix");
    }

    // The energy oracle changed v many times while q stayed fixed. M(q) must
    // remain identical; this also makes the velocity-independence claim
    // sensitive rather than comparing two equal initial states.
    Eigen::MatrixXd after_velocity_writes(velocity_count, velocity_count);
    fixture.model.CalcGeneralizedMassMatrix(*context, after_velocity_writes);
    ExpectMatrixNear(after_velocity_writes, mass_matrix, 64.0,
                     "changing only generalized velocity does not change M(q)");
}

void CheckSingleRevoluteAnalyticMass() {
    MultibodyModel model;
    Eigen::Matrix3d central_unit_inertia = Eigen::Matrix3d::Zero();
    central_unit_inertia.diagonal() << 0.18, 0.23, 0.29;
    const Eigen::Vector3d center_of_mass(0.14, -0.09, 0.0);
    constexpr double mass_kilograms = 2.5;
    const auto inertia = MakeInertia(mass_kilograms, center_of_mass,
                                     central_unit_inertia);
    const auto body = model.AddRigidBody("body", inertia);
    const auto joint = model.AddRevoluteJoint(
        "joint", model.world_frame(), model.body_frame(body),
        Eigen::Vector3d::UnitZ(), 0.0);
    model.Finalize();
    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions(1);
    positions << 0.47;
    model.SetGeneralizedPositions(context.get(), positions);

    Eigen::MatrixXd mass_matrix(1, 1);
    model.CalcGeneralizedMassMatrix(*context, mass_matrix);
    const double expected =
        mass_kilograms *
        (central_unit_inertia(2, 2) + center_of_mass.squaredNorm());
    const double tolerance = 128.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(expected));
    Expect(std::abs(mass_matrix(0, 0) - expected) <= tolerance,
           "a single revolute body's mass is its inertia about the joint axis");
    (void)joint;
}

void CheckFailureSemanticsAndZeroVelocityModel() {
    RichMassFixture fixture;
    auto context = fixture.MakeContext();
    const int velocity_count = fixture.model.num_generalized_velocities();

    Eigen::MatrixXd wrong_shape = Eigen::MatrixXd::Constant(2, 3, 17.0);
    const Eigen::MatrixXd wrong_shape_before = wrong_shape;
    Expect(InvalidArgumentMentions(
               [&] {
                   fixture.model.CalcGeneralizedMassMatrix(*context,
                                                           wrong_shape);
               },
               "must be " + std::to_string(velocity_count) + " x " +
                   std::to_string(velocity_count)),
           "a wrong output shape is refused with its required dimensions");
    Expect(wrong_shape.rows() == wrong_shape_before.rows() &&
               wrong_shape.cols() == wrong_shape_before.cols() &&
               (wrong_shape.array() == wrong_shape_before.array()).all(),
           "a refused output shape is not resized or modified");

    RichMassFixture other_fixture;
    auto foreign_context = other_fixture.MakeContext();
    Eigen::MatrixXd foreign_output =
        Eigen::MatrixXd::Constant(velocity_count, velocity_count, 23.0);
    const Eigen::MatrixXd foreign_before = foreign_output;
    Expect(InvalidArgumentMentions(
               [&] {
                   fixture.model.CalcGeneralizedMassMatrix(*foreign_context,
                                                           foreign_output);
               },
               "different model"),
           "a foreign context is refused by the public model boundary");
    Expect(foreign_output == foreign_before,
           "a foreign-context refusal leaves the output unchanged");

    MultibodyModel unfinished;
    Eigen::MatrixXd unfinished_output =
        Eigen::MatrixXd::Constant(velocity_count, velocity_count, 31.0);
    const Eigen::MatrixXd unfinished_before = unfinished_output;
    Expect(LogicErrorButNotInvalidArgumentMentions(
               [&] {
                   unfinished.CalcGeneralizedMassMatrix(*context,
                                                        unfinished_output);
               },
               "before Finalize"),
           "an unfinalized model refuses the mass-matrix query at its boundary");
    Expect(unfinished_output == unfinished_before,
           "an unfinalized-model refusal leaves the output unchanged");

    MultibodyModel empty_model;
    empty_model.Finalize();
    auto empty_context = empty_model.CreateDefaultContext();
    Eigen::MatrixXd empty_mass_matrix(0, 0);
    empty_model.CalcGeneralizedMassMatrix(*empty_context, empty_mass_matrix);
    Expect(empty_mass_matrix.rows() == 0 && empty_mass_matrix.cols() == 0,
           "a zero-velocity model has a valid 0 x 0 mass matrix");
}

}  // namespace

int main() {
    CheckRichMassMatrix();
    CheckSingleRevoluteAnalyticMass();
    CheckFailureSemanticsAndZeroVelocityModel();
    if (failure_count != 0) {
        std::printf("%d mass-matrix check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("PASS multibody generalized mass matrix\n");
    return 0;
}
