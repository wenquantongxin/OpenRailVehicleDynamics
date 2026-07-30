// The state store's contract: what it owns, what it refuses, and what it leaves
// alone when it refuses.
//
// The refusals matter more than the acceptances here. A store that quietly took
// a non-rotation, a negative damping coefficient or an inertia no real body can
// have would hand a plausible-looking wrong answer to everything downstream, and
// the place it went wrong would be many passes behind wherever the symptom
// appeared. Each rejection below therefore has a matching acceptance that pins
// the boundary from the other side — zero mass is legitimate, a negative gear
// ratio is legitimate, a negative product of inertia is legitimate — so the
// checks cannot be satisfied by a store that simply refuses everything unusual.
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace {

using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::JointActuatorParameters;
using orvd::multibody_runtime::LinearBushingRollPitchYawParameters;
using orvd::multibody_runtime::MultibodyStateInstance;
using orvd::multibody_runtime::MultibodyStateLayout;
using orvd::multibody_runtime::MultibodyStateLayoutDescription;
using orvd::multibody_runtime::RevoluteSpringParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

/// Runs `attempt` and reports whether it refused.
template <typename Attempt>
bool WasRefused(Attempt&& attempt) {
    try {
        attempt();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

void ExpectRefused(bool refused, const std::string& what) {
    ExpectTrue(refused, what + ": expected a refusal, but the value was taken");
}

void ExpectAccepted(bool refused, const std::string& what) {
    ExpectTrue(!refused, what + ": expected acceptance, but the value was refused");
}

MultibodyStateLayoutDescription SampleDescription() {
    MultibodyStateLayoutDescription description;
    // A quaternion floating body plus two revolute joints: seven-plus-two
    // positions against six-plus-two velocities, which is the case where a
    // store that assumed q and v have the same length would fail.
    description.generalized_position_count = 9;
    description.generalized_velocity_count = 8;
    description.rigid_body_count = 3;
    description.fixed_frame_count = 2;
    description.joint_velocity_counts = {6, 1, 1};
    description.joint_actuator_count = 2;
    description.revolute_spring_count = 1;
    description.linear_bushing_count = 1;
    return description;
}

RigidBodyInertiaParameters ValidInertia() {
    RigidBodyInertiaParameters parameters;
    parameters.mass_kilograms = 2.5;
    parameters.center_of_mass_in_body_frame = Eigen::Vector3d(0.1, -0.2, 0.3);
    parameters.unit_inertia_moments = Eigen::Vector3d(0.4, 0.5, 0.6);
    parameters.unit_inertia_products = Eigen::Vector3d(0.01, -0.02, 0.03);
    return parameters;
}

// --- Layout -----------------------------------------------------------------

void CheckLayoutReportsStableSegments() {
    const MultibodyStateLayout layout(SampleDescription());
    ExpectTrue(layout.generalized_position_count() == 9,
               "the layout keeps the generalized position count");
    ExpectTrue(layout.generalized_velocity_count() == 8,
               "the layout keeps the generalized velocity count");
    ExpectTrue(layout.damped_joint_count() == 3,
               "the layout counts the damped joints");
    ExpectTrue(layout.joint_damping_start(0) == 0 &&
                   layout.joint_damping_start(1) == 6 &&
                   layout.joint_damping_start(2) == 7,
               "the layout places each joint's damping after the previous one");
    ExpectTrue(layout.total_joint_damping_count() == 8,
               "the layout's flat damping store spans every joint");
    ExpectRefused(WasRefused([&] { (void)layout.joint_velocity_count(3); }),
                  "a joint index past the end");
    ExpectRefused(WasRefused([&] { (void)layout.joint_velocity_count(-1); }),
                  "a negative joint index");
}

void CheckLayoutRefusesNegativeCounts() {
    MultibodyStateLayoutDescription description = SampleDescription();
    description.generalized_position_count = -1;
    ExpectRefused(WasRefused([&] { MultibodyStateLayout layout(description); }),
                  "a negative generalized position count");

    description = SampleDescription();
    description.joint_velocity_counts = {6, -1};
    ExpectRefused(WasRefused([&] { MultibodyStateLayout layout(description); }),
                  "a negative joint velocity count");
}

// --- Generalized coordinates ------------------------------------------------

void CheckCoordinatesStartAtZeroAndRoundTrip() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    ExpectTrue(state.generalized_positions().size() == 9 &&
                   state.generalized_positions().isZero(),
               "a new state's positions are the layout's length and zero");
    ExpectTrue(state.generalized_velocities().size() == 8 &&
                   state.generalized_velocities().isZero(),
               "a new state's velocities are the layout's length and zero");

    const Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(9, 1.0, 9.0);
    state.set_generalized_positions(positions);
    ExpectTrue(state.generalized_positions() == positions,
               "positions read back as they were written");
}

void CheckRefusedWriteLeavesTheStateAlone() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    const Eigen::VectorXd accepted = Eigen::VectorXd::LinSpaced(9, 1.0, 9.0);
    state.set_generalized_positions(accepted);

    // A vector whose last entry is not finite. Everything before it is fine, so
    // a store that validated as it wrote would have committed eight of nine
    // values before noticing.
    Eigen::VectorXd partly_bad = Eigen::VectorXd::LinSpaced(9, 10.0, 18.0);
    partly_bad[8] = std::numeric_limits<double>::quiet_NaN();
    ExpectRefused(WasRefused([&] { state.set_generalized_positions(partly_bad); }),
                  "a position vector ending in a non-finite value");
    ExpectTrue(state.generalized_positions() == accepted,
               "a refused position write leaves every earlier entry unchanged");

    ExpectRefused(
        WasRefused([&] {
            state.set_generalized_positions(Eigen::VectorXd::Zero(8));
        }),
        "a position vector of the wrong length");
    ExpectTrue(state.generalized_positions() == accepted,
               "a refused length leaves the state unchanged");
}

void CheckNonUnitQuaternionIsNeitherRefusedNorRepaired() {
    // This layer does not know which entries are a quaternion, so it must not
    // act as if it did. Whether a non-unit quaternion is an error, and what to
    // do about it, is settled where the joints are known.
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    Eigen::VectorXd positions = Eigen::VectorXd::Zero(9);
    positions.head<4>() = Eigen::Vector4d(2.0, 0.0, 0.0, 0.0);
    ExpectAccepted(WasRefused([&] { state.set_generalized_positions(positions); }),
                   "a position vector holding a non-unit quaternion");
    ExpectTrue(state.generalized_positions()[0] == 2.0,
               "the non-unit quaternion is stored as given, not normalised");
}

// --- Instance isolation ------------------------------------------------------

void CheckInstancesDoNotShareState() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance first(layout);
    MultibodyStateInstance second(layout);
    first.set_generalized_velocities(Eigen::VectorXd::Constant(8, 3.0));
    ExpectTrue(second.generalized_velocities().isZero(),
               "writing one state does not reach another built on the same layout");
}

// --- Rigid body inertia ------------------------------------------------------

void CheckInertiaValidation() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    ExpectAccepted(
        WasRefused([&] { state.set_rigid_body_inertia_parameters(0, ValidInertia()); }),
        "a realisable inertia");
    ExpectTrue(state.rigid_body_inertia_parameters(0).mass_kilograms == 2.5,
               "the inertia reads back as written");

    // A massless frame carrier is a normal modelling device.
    ExpectAccepted(WasRefused([&] {
                       state.set_rigid_body_inertia_parameters(
                           1, RigidBodyInertiaParameters{});
                   }),
                   "a body with zero mass and zero inertia");

    // Products of inertia are signed; refusing a negative one would forbid most
    // real bodies.
    RigidBodyInertiaParameters negative_product = ValidInertia();
    negative_product.unit_inertia_products = Eigen::Vector3d(-0.05, -0.05, -0.05);
    ExpectAccepted(WasRefused([&] {
                       state.set_rigid_body_inertia_parameters(2, negative_product);
                   }),
                   "a negative product of inertia");

    RigidBodyInertiaParameters negative_mass = ValidInertia();
    negative_mass.mass_kilograms = -1.0;
    ExpectRefused(
        WasRefused([&] { state.set_rigid_body_inertia_parameters(0, negative_mass); }),
        "a negative mass");

    RigidBodyInertiaParameters negative_moment = ValidInertia();
    negative_moment.unit_inertia_moments = Eigen::Vector3d(-0.1, 0.5, 0.6);
    ExpectRefused(WasRefused([&] {
                      state.set_rigid_body_inertia_parameters(0, negative_moment);
                  }),
                  "a negative moment of inertia");

    // Gxx + Gyy < Gzz: no mass distribution has these moments.
    RigidBodyInertiaParameters impossible = ValidInertia();
    impossible.unit_inertia_moments = Eigen::Vector3d(0.1, 0.1, 1.0);
    ExpectRefused(
        WasRefused([&] { state.set_rigid_body_inertia_parameters(0, impossible); }),
        "moments of inertia that violate the triangle inequality");

    ExpectTrue(state.rigid_body_inertia_parameters(0).mass_kilograms == 2.5,
               "every refused inertia left the stored one alone");

    ExpectRefused(
        WasRefused([&] { state.set_rigid_body_inertia_parameters(3, ValidInertia()); }),
        "a rigid body index past the end");
}

// --- Fixed frame pose --------------------------------------------------------

void CheckFixedFramePoseValidation() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    FixedFramePoseParameters pose;
    pose.R_PF = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized())
                    .toRotationMatrix();
    pose.p_PoFo_P = Eigen::Vector3d(0.5, -1.5, 2.0);
    ExpectAccepted(
        WasRefused([&] { state.set_fixed_frame_pose_parameters(0, pose); }),
        "a genuine rotation and a finite translation");

    FixedFramePoseParameters scaled = pose;
    scaled.R_PF *= 1.001;
    ExpectRefused(
        WasRefused([&] { state.set_fixed_frame_pose_parameters(0, scaled); }),
        "a scaled rotation matrix");
    ExpectTrue(state.fixed_frame_pose_parameters(0).R_PF.isApprox(pose.R_PF),
               "the refused rotation left the stored pose alone");

    // A reflection: orthonormal, but with determinant -1. It passes an
    // orthonormality test alone, which is why the determinant is checked too.
    FixedFramePoseParameters reflected = pose;
    reflected.R_PF.col(0) *= -1.0;
    ExpectRefused(
        WasRefused([&] { state.set_fixed_frame_pose_parameters(0, reflected); }),
        "an orthonormal matrix with determinant -1");

    FixedFramePoseParameters infinite_translation = pose;
    infinite_translation.p_PoFo_P[1] = std::numeric_limits<double>::infinity();
    ExpectRefused(WasRefused([&] {
                      state.set_fixed_frame_pose_parameters(0, infinite_translation);
                  }),
                  "a non-finite translation");
}

// --- Joint damping, actuators, springs, bushings ------------------------------

void CheckJointDampingValidation() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    const Eigen::VectorXd six = Eigen::VectorXd::Constant(6, 0.5);
    ExpectAccepted(WasRefused([&] { state.set_joint_damping(0, six); }),
                   "a six-entry damping vector for a six-velocity joint");
    ExpectTrue(state.joint_damping(0).isApprox(six),
               "the damping reads back from its own segment");
    ExpectTrue(state.joint_damping(1).size() == 1 && state.joint_damping(1)[0] == 0.0,
               "writing one joint's damping does not disturb the next joint's");

    ExpectRefused(WasRefused([&] {
                      state.set_joint_damping(1, Eigen::VectorXd::Constant(2, 1.0));
                  }),
                  "a damping vector of the wrong length for that joint");
    ExpectRefused(WasRefused([&] {
                      state.set_joint_damping(1, Eigen::VectorXd::Constant(1, -1.0));
                  }),
                  "a negative damping coefficient");
    ExpectTrue(state.joint_damping(0).isApprox(six),
               "the refused damping writes left the earlier joint alone");
}

void CheckElementParameterValidation() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    JointActuatorParameters actuator;
    actuator.rotor_inertia = 1e-4;
    actuator.gear_ratio = -50.0;  // A sign is a direction, not an error.
    ExpectAccepted(
        WasRefused([&] { state.set_joint_actuator_parameters(0, actuator); }),
        "an actuator with a negative gear ratio");

    JointActuatorParameters negative_rotor = actuator;
    negative_rotor.rotor_inertia = -1.0;
    ExpectRefused(
        WasRefused([&] { state.set_joint_actuator_parameters(0, negative_rotor); }),
        "a negative rotor inertia");

    RevoluteSpringParameters spring;
    spring.stiffness = 12.0;
    spring.nominal_angle_radians = -0.3;
    ExpectAccepted(
        WasRefused([&] { state.set_revolute_spring_parameters(0, spring); }),
        "a spring with a negative nominal angle");

    RevoluteSpringParameters negative_stiffness = spring;
    negative_stiffness.stiffness = -1.0;
    ExpectRefused(WasRefused([&] {
                      state.set_revolute_spring_parameters(0, negative_stiffness);
                  }),
                  "a negative spring stiffness");

    LinearBushingRollPitchYawParameters bushing;
    bushing.torque_stiffness = Eigen::Vector3d(1.0, 2.0, 3.0);
    bushing.torque_damping = Eigen::Vector3d(0.1, 0.2, 0.3);
    bushing.force_stiffness = Eigen::Vector3d(10.0, 20.0, 30.0);
    bushing.force_damping = Eigen::Vector3d(1.0, 2.0, 3.0);
    ExpectAccepted(
        WasRefused([&] { state.set_linear_bushing_parameters(0, bushing); }),
        "a bushing with non-negative stiffness and damping");

    LinearBushingRollPitchYawParameters negative_damping = bushing;
    negative_damping.force_damping[2] = -1.0;
    ExpectRefused(WasRefused([&] {
                      state.set_linear_bushing_parameters(0, negative_damping);
                  }),
                  "a negative bushing damping term");
    ExpectTrue(state.linear_bushing_parameters(0).force_damping.isApprox(
                   bushing.force_damping),
               "the refused bushing write left the stored one alone");
}

}  // namespace

int main() {
    CheckLayoutReportsStableSegments();
    CheckLayoutRefusesNegativeCounts();
    CheckCoordinatesStartAtZeroAndRoundTrip();
    CheckRefusedWriteLeavesTheStateAlone();
    CheckNonUnitQuaternionIsNeitherRefusedNorRepaired();
    CheckInstancesDoNotShareState();
    CheckInertiaValidation();
    CheckFixedFramePoseValidation();
    CheckJointDampingValidation();
    CheckElementParameterValidation();

    if (failure_count > 0) {
        std::printf("%d multibody state check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "the multibody state store owns q, v and the typed parameters, refuses"
        " what no model can mean, and leaves the live state alone when it does\n");
    return 0;
}
