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
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"
#include "orvd/multibody_runtime/multibody_state_version.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace {

using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::JointActuatorParameters;
using orvd::multibody_runtime::LinearBushingRollPitchYawParameters;
using orvd::multibody_runtime::MultibodyStateInstance;
using orvd::multibody_runtime::MultibodyStateLayout;
using orvd::multibody_runtime::MultibodyStateLayoutDescription;
using orvd::multibody_runtime::MultibodyStateVersion;
using orvd::multibody_runtime::RevoluteSpringParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

// A version answers "is this the same one I recorded?" and nothing else. An
// implicit conversion would let it be mistaken for a count or an index; an
// ordering would invite "is it newer?", which is only meaningful within a single
// source and is a category error across two.
static_assert(!std::is_convertible_v<MultibodyStateVersion, std::uint64_t>);
static_assert(!std::is_convertible_v<std::uint64_t, MultibodyStateVersion>);
template <typename T>
concept HasOrdering = requires(T a, T b) { a < b; };
static_assert(!HasOrdering<MultibodyStateVersion>);
template <typename T>
concept HasArithmetic = requires(T a, T b) { a + b; };
static_assert(!HasArithmetic<MultibodyStateVersion>);

static_assert(
    !std::is_constructible_v<MultibodyStateInstance, MultibodyStateLayout&&>);
static_assert(!std::is_constructible_v<MultibodyStateInstance,
                                       const MultibodyStateLayout&&>);
static_assert(!std::is_copy_constructible_v<MultibodyStateInstance>);
static_assert(!std::is_copy_assignable_v<MultibodyStateInstance>);
static_assert(!std::is_move_constructible_v<MultibodyStateInstance>);
static_assert(!std::is_move_assignable_v<MultibodyStateInstance>);

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
    // Dimensions shaped like a quaternion floating body plus two revolute
    // joints: seven-plus-two positions against six-plus-two velocities. This
    // layer uses only the dimensional mismatch; it has no joint segmentation.
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

bool SameInertia(const RigidBodyInertiaParameters& first,
                 const RigidBodyInertiaParameters& second) {
    return first.mass_kilograms == second.mass_kilograms &&
           first.center_of_mass_in_body_frame ==
               second.center_of_mass_in_body_frame &&
           first.unit_inertia_moments == second.unit_inertia_moments &&
           first.unit_inertia_products == second.unit_inertia_products;
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
    ExpectTrue(&state.layout() == &layout,
               "the state remains bound to the exact layout object");
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
    // This raw store does not know which entries are a quaternion, so it must
    // not act as if it did. The model-aware adapter checks the non-zero
    // precondition before evaluation, without normalizing the stored value.
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

    // A point mass one metre from the body origin has G_BBo =
    // diag(0, 1, 1); shifting to its centre of mass leaves zero inertia.
    RigidBodyInertiaParameters offset_point_mass;
    offset_point_mass.mass_kilograms = 1.0;
    offset_point_mass.center_of_mass_in_body_frame =
        Eigen::Vector3d(1.0, 0.0, 0.0);
    offset_point_mass.unit_inertia_moments =
        Eigen::Vector3d(0.0, 1.0, 1.0);
    ExpectAccepted(WasRefused([&] {
                       state.set_rigid_body_inertia_parameters(
                           1, offset_point_mass);
                   }),
                   "a point mass expressed about an offset body origin");

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

    // The diagonal moments alone look valid, but the full symmetric matrix has
    // principal moments (-1, 1, 3).
    RigidBodyInertiaParameters impossible_products;
    impossible_products.mass_kilograms = 1.0;
    impossible_products.unit_inertia_moments =
        Eigen::Vector3d(1.0, 1.0, 1.0);
    impossible_products.unit_inertia_products =
        Eigen::Vector3d(2.0, 0.0, 0.0);
    ExpectRefused(WasRefused([&] {
                      state.set_rigid_body_inertia_parameters(
                          0, impossible_products);
                  }),
                  "products that make a principal moment negative");

    // This one is positive semidefinite, but its principal moments (1, 1, 3)
    // still cannot be a real mass distribution.
    RigidBodyInertiaParameters impossible_principal_triangle;
    impossible_principal_triangle.mass_kilograms = 1.0;
    impossible_principal_triangle.unit_inertia_moments =
        Eigen::Vector3d::Constant(5.0 / 3.0);
    impossible_principal_triangle.unit_inertia_products =
        Eigen::Vector3d::Constant(2.0 / 3.0);
    ExpectRefused(WasRefused([&] {
                      state.set_rigid_body_inertia_parameters(
                          0, impossible_principal_triangle);
                  }),
                  "principal moments that violate the triangle inequality");

    // The about-origin inertia is zero, but shifting it to a non-zero centre of
    // mass produces negative central moments.
    RigidBodyInertiaParameters impossible_centre_of_mass;
    impossible_centre_of_mass.mass_kilograms = 1.0;
    impossible_centre_of_mass.center_of_mass_in_body_frame =
        Eigen::Vector3d(1.0, 0.0, 0.0);
    ExpectRefused(WasRefused([&] {
                      state.set_rigid_body_inertia_parameters(
                          0, impossible_centre_of_mass);
                  }),
                  "an about-origin inertia that is impossible at the centre "
                  "of mass");

    ExpectTrue(SameInertia(state.rigid_body_inertia_parameters(0), ValidInertia()),
               "every refused inertia left the stored one alone");

    ExpectRefused(WasRefused([&] {
                      state.set_rigid_body_inertia_parameters(3, ValidInertia());
                  }),
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

    FixedFramePoseParameters outside_rotation_contract = pose;
    outside_rotation_contract.R_PF = Eigen::Matrix3d::Identity();
    outside_rotation_contract.R_PF(0, 0) += 1e-13;
    ExpectRefused(WasRefused([&] {
                      state.set_fixed_frame_pose_parameters(
                          0, outside_rotation_contract);
                  }),
                  "a matrix outside the rotation representation's tolerance");

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
    ExpectTrue(
        state.fixed_frame_pose_parameters(0).R_PF.isApprox(pose.R_PF) &&
            state.fixed_frame_pose_parameters(0).p_PoFo_P.isApprox(
                pose.p_PoFo_P),
        "every refused fixed-frame pose left the complete stored pose alone");
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
    ExpectTrue(state.joint_damping(1).size() == 1 &&
                   state.joint_damping(1)[0] == 0.0,
               "writing one joint's damping does not disturb the next joint's");
    state.set_joint_damping(1, Eigen::VectorXd::Constant(1, 0.25));

    ExpectRefused(WasRefused([&] {
                      state.set_joint_damping(1, Eigen::VectorXd::Constant(2, 1.0));
                  }),
                  "a damping vector of the wrong length for that joint");
    ExpectRefused(WasRefused([&] {
                      state.set_joint_damping(1, Eigen::VectorXd::Constant(1, -1.0));
                  }),
                  "a negative damping coefficient");
    ExpectTrue(state.joint_damping(0).isApprox(six) &&
                   state.joint_damping(1)[0] == 0.25,
               "the refused damping writes left both stored segments alone");
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
        WasRefused(
            [&] { state.set_joint_actuator_parameters(0, negative_rotor); }),
        "a negative rotor inertia");
    ExpectTrue(state.joint_actuator_parameters(0).rotor_inertia ==
                       actuator.rotor_inertia &&
                   state.joint_actuator_parameters(0).gear_ratio ==
                       actuator.gear_ratio,
               "the refused actuator write left the stored parameters alone");

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
    ExpectTrue(state.revolute_spring_parameters(0).stiffness ==
                       spring.stiffness &&
                   state.revolute_spring_parameters(0).nominal_angle_radians ==
                       spring.nominal_angle_radians,
               "the refused spring write left the stored parameters alone");

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

// --- Versions ---------------------------------------------------------------

/// The five sources that carry a version, in a fixed order.
///
/// Naming them individually is what lets a check say *which* version moved. A
/// check that only asked "did something advance?" would pass just as happily
/// when a position write bumped the inertia counter, and downstream that reads
/// as an inertia change that never happened.
enum class VersionedSource {
    kGeneralizedPositions,
    kGeneralizedVelocities,
    kFixedFramePoses,
    kRigidBodyInertias,
    kJointActuatorParameters,
    /// Past the end: "this write must advance nothing".
    kNone
};

const char* NameOf(VersionedSource source) {
    switch (source) {
        case VersionedSource::kGeneralizedPositions:
            return "the generalized positions version";
        case VersionedSource::kGeneralizedVelocities:
            return "the generalized velocities version";
        case VersionedSource::kFixedFramePoses:
            return "the fixed frame poses version";
        case VersionedSource::kRigidBodyInertias:
            return "the rigid body inertias version";
        case VersionedSource::kJointActuatorParameters:
            return "the joint actuator parameters version";
        case VersionedSource::kNone:
            break;
    }
    return "no version";
}

std::array<std::uint64_t, 5> AllRevisions(const MultibodyStateInstance& state) {
    return {state.generalized_positions_version().revision_number(),
            state.generalized_velocities_version().revision_number(),
            state.fixed_frame_poses_version().revision_number(),
            state.rigid_body_inertias_version().revision_number(),
            state.joint_actuator_parameters_version().revision_number()};
}

/// Runs `write`, then checks every version against what it should now be:
/// `expected` up by exactly one, the rest untouched.
///
/// Checking all five against exact values, rather than checking that the
/// intended one changed, is what makes this discriminating. It fails if the
/// write advances the wrong category, if it advances two, and if it advances the
/// right one twice — three mistakes that a "did it change?" check cannot tell
/// apart from correct behaviour.
template <typename Write>
void ExpectAdvancesOnly(MultibodyStateInstance& state, VersionedSource expected,
                        const std::string& what, Write&& write) {
    const std::array<std::uint64_t, 5> before = AllRevisions(state);
    write();
    const std::array<std::uint64_t, 5> after = AllRevisions(state);
    for (std::size_t i = 0; i < before.size(); ++i) {
        const bool advances = i == static_cast<std::size_t>(expected);
        const std::uint64_t want = before[i] + (advances ? 1u : 0u);
        ExpectTrue(after[i] == want,
                   what + ": " + NameOf(static_cast<VersionedSource>(i)) +
                       " went " + std::to_string(before[i]) + " -> " +
                       std::to_string(after[i]) + ", expected " +
                       std::to_string(want));
    }
}

void CheckVersionValueSemantics() {
    const MultibodyStateVersion fresh;
    ExpectTrue(fresh.revision_number() == 0,
               "a source that has accepted no write is at revision 0");
    ExpectTrue(fresh.Next().revision_number() == 1,
               "the successor of revision 0 is revision 1");
    ExpectTrue(fresh.Next() == fresh.Next(),
               "the same successor computed twice compares equal");
    ExpectTrue(!(fresh == fresh.Next()),
               "a version and its successor do not compare equal");
    ExpectTrue(fresh.revision_number() == 0,
               "asking for the successor does not change the version asked");
}

void CheckEachVersionedWriteAdvancesExactlyItsOwn() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    ExpectTrue(AllRevisions(state) == std::array<std::uint64_t, 5>{0, 0, 0, 0, 0},
               "a fresh state starts every version at 0");

    ExpectAdvancesOnly(
        state, VersionedSource::kGeneralizedPositions, "a position write", [&] {
            state.set_generalized_positions(Eigen::VectorXd::Constant(9, 0.25));
        });
    ExpectAdvancesOnly(
        state, VersionedSource::kGeneralizedVelocities, "a velocity write", [&] {
            state.set_generalized_velocities(Eigen::VectorXd::Constant(8, -1.5));
        });
    ExpectAdvancesOnly(state, VersionedSource::kFixedFramePoses,
                       "a fixed frame pose write", [&] {
                           FixedFramePoseParameters pose;
                           pose.p_PoFo_P = Eigen::Vector3d(1.0, 2.0, 3.0);
                           state.set_fixed_frame_pose_parameters(1, pose);
                       });
    ExpectAdvancesOnly(state, VersionedSource::kRigidBodyInertias,
                       "an inertia write",
                       [&] { state.set_rigid_body_inertia_parameters(2, ValidInertia()); });
    ExpectAdvancesOnly(state, VersionedSource::kJointActuatorParameters,
                       "an actuator write", [&] {
                           JointActuatorParameters actuator;
                           actuator.rotor_inertia = 0.5;
                           actuator.gear_ratio = -8.0;
                           state.set_joint_actuator_parameters(0, actuator);
                       });
}

void CheckWritingTheSameValueStillAdvances() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    // A version records an accepted write, not the value it left behind. If it
    // skipped when the numbers happened to match, the write path would have to
    // scan the whole vector to find out, and "unchanged" would become a question
    // about value equality rather than about whether anything was stored.
    const Eigen::VectorXd positions = Eigen::VectorXd::Constant(9, 0.75);
    ExpectAdvancesOnly(state, VersionedSource::kGeneralizedPositions,
                       "the first write of a value",
                       [&] { state.set_generalized_positions(positions); });
    ExpectAdvancesOnly(state, VersionedSource::kGeneralizedPositions,
                       "writing the identical value again",
                       [&] { state.set_generalized_positions(positions); });

    const RigidBodyInertiaParameters inertia = ValidInertia();
    ExpectAdvancesOnly(state, VersionedSource::kRigidBodyInertias,
                       "the first write of an inertia",
                       [&] { state.set_rigid_body_inertia_parameters(0, inertia); });
    ExpectAdvancesOnly(state, VersionedSource::kRigidBodyInertias,
                       "writing the identical inertia again",
                       [&] { state.set_rigid_body_inertia_parameters(0, inertia); });
}

void CheckRefusedWritesLeaveEveryVersionAlone() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    // Get every counter off zero first, so that "unchanged" is a real claim
    // rather than one that a state which never advances anything would satisfy.
    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 0.1));
    state.set_generalized_velocities(Eigen::VectorXd::Constant(8, 0.2));
    FixedFramePoseParameters pose;
    state.set_fixed_frame_pose_parameters(0, pose);
    state.set_rigid_body_inertia_parameters(0, ValidInertia());
    state.set_joint_actuator_parameters(0, JointActuatorParameters{});

    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a position write of the wrong size", [&] {
                           ExpectRefused(WasRefused([&] {
                               state.set_generalized_positions(
                                   Eigen::VectorXd::Zero(8));
                           }),
                                         "a position vector of the wrong size");
                       });
    // Both failure modes, for both coordinate vectors. One case per setter would
    // leave the other validator untested: a version advance that slipped in
    // between the size check and the finiteness check would still be caught by
    // the case that fails on size, and missed entirely by the case that does not
    // reach the second check at all.
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a position write holding a NaN", [&] {
                           Eigen::VectorXd positions =
                               Eigen::VectorXd::Zero(9);
                           positions[6] =
                               std::numeric_limits<double>::quiet_NaN();
                           ExpectRefused(WasRefused([&] {
                               state.set_generalized_positions(positions);
                           }),
                                         "a non-finite position");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a velocity write of the wrong size", [&] {
                           ExpectRefused(WasRefused([&] {
                               state.set_generalized_velocities(
                                   Eigen::VectorXd::Zero(9));
                           }),
                                         "a velocity vector of the wrong size");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a velocity write holding an infinity", [&] {
                           Eigen::VectorXd velocities =
                               Eigen::VectorXd::Zero(8);
                           velocities[3] =
                               std::numeric_limits<double>::infinity();
                           ExpectRefused(WasRefused([&] {
                               state.set_generalized_velocities(velocities);
                           }),
                                         "a non-finite velocity");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a pose write whose rotation is not a rotation", [&] {
                           FixedFramePoseParameters bad;
                           bad.R_PF(0, 1) = 0.5;
                           ExpectRefused(WasRefused([&] {
                               state.set_fixed_frame_pose_parameters(0, bad);
                           }),
                                         "a non-orthonormal rotation");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "an inertia write no real body can have", [&] {
                           RigidBodyInertiaParameters bad = ValidInertia();
                           bad.unit_inertia_products = Eigen::Vector3d(10.0, 0.0, 0.0);
                           ExpectRefused(WasRefused([&] {
                               state.set_rigid_body_inertia_parameters(0, bad);
                           }),
                                         "an indefinite unit inertia");
                       });
    // The translation is the pose setter's last validator, and the gear ratio the
    // actuator setter's. Refusals that land on the last check are the ones that
    // pin the advance to a position after *all* of the validation rather than
    // merely after the first of it.
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a pose write with a non-finite translation", [&] {
                           FixedFramePoseParameters bad;
                           bad.p_PoFo_P[2] =
                               std::numeric_limits<double>::quiet_NaN();
                           ExpectRefused(WasRefused([&] {
                               state.set_fixed_frame_pose_parameters(0, bad);
                           }),
                                         "a non-finite translation");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "an actuator write with negative rotor inertia", [&] {
                           JointActuatorParameters bad;
                           bad.rotor_inertia = -1.0;
                           ExpectRefused(WasRefused([&] {
                               state.set_joint_actuator_parameters(0, bad);
                           }),
                                         "a negative rotor inertia");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "an actuator write with a non-finite gear ratio", [&] {
                           JointActuatorParameters bad;
                           bad.gear_ratio =
                               std::numeric_limits<double>::infinity();
                           ExpectRefused(WasRefused([&] {
                               state.set_joint_actuator_parameters(0, bad);
                           }),
                                         "a non-finite gear ratio");
                       });
    ExpectAdvancesOnly(state, VersionedSource::kNone,
                       "a write to an out-of-range index", [&] {
                           ExpectRefused(WasRefused([&] {
                               state.set_rigid_body_inertia_parameters(
                                   3, ValidInertia());
                           }),
                                         "a rigid body index past the end");
                       });
}

void CheckUnversionedParametersAdvanceNothing() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);

    // These three have no cache consumer, so they have no version of their own —
    // and, just as importantly, they must not disturb anyone else's. A write
    // that bumped an unrelated counter would force a recomputation that the
    // recompute count would then report as a genuine dependency.
    ExpectAdvancesOnly(state, VersionedSource::kNone, "a damping write", [&] {
        state.set_joint_damping(1, Eigen::VectorXd::Constant(1, 3.0));
    });
    ExpectAdvancesOnly(state, VersionedSource::kNone, "a spring write", [&] {
        RevoluteSpringParameters spring;
        spring.stiffness = 12.0;
        spring.nominal_angle_radians = 0.3;
        state.set_revolute_spring_parameters(0, spring);
    });
    ExpectAdvancesOnly(state, VersionedSource::kNone, "a bushing write", [&] {
        LinearBushingRollPitchYawParameters bushing;
        bushing.torque_stiffness = Eigen::Vector3d(1.0, 2.0, 3.0);
        state.set_linear_bushing_parameters(0, bushing);
    });

    ExpectTrue(state.joint_damping(1)[0] == 3.0,
               "the unversioned damping write still stored its value");
    ExpectTrue(state.revolute_spring_parameters(0).stiffness == 12.0,
               "the unversioned spring write still stored its value");
}

void CheckEqualVersionNumbersDoNotMakeTwoInstancesOne() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance first(layout);
    MultibodyStateInstance second(layout);

    // Two states on the same model, driven to the same revision numbers. The
    // numbers agreeing is exactly the situation in which a shared counter, or a
    // version that silently identified an instance, would go unnoticed.
    first.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    second.set_generalized_positions(Eigen::VectorXd::Constant(9, 2.0));
    ExpectTrue(first.generalized_positions_version() ==
                   second.generalized_positions_version(),
               "two instances written once each are at the same revision");
    ExpectTrue(first.generalized_positions()[0] == 1.0 &&
                   second.generalized_positions()[0] == 2.0,
               "equal revisions do not imply equal contents");

    const std::array<std::uint64_t, 5> second_before = AllRevisions(second);
    first.set_generalized_positions(Eigen::VectorXd::Constant(9, 3.0));
    first.set_rigid_body_inertia_parameters(0, ValidInertia());
    ExpectTrue(AllRevisions(second) == second_before,
               "writing to one instance advances no version of the other");
    ExpectTrue(second.generalized_positions()[0] == 2.0,
               "writing to one instance does not change the other's state");
    ExpectTrue(!(first.generalized_positions_version() ==
                 second.generalized_positions_version()),
               "the two instances' versions have now diverged");
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
    CheckVersionValueSemantics();
    CheckEachVersionedWriteAdvancesExactlyItsOwn();
    CheckWritingTheSameValueStillAdvances();
    CheckRefusedWritesLeaveEveryVersionAlone();
    CheckUnversionedParametersAdvanceNothing();
    CheckEqualVersionNumbersDoNotMakeTwoInstancesOne();

    if (failure_count > 0) {
        std::printf("%d multibody state check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "the multibody state store owns q, v and the typed parameters, refuses"
        " what no model can mean, leaves the live state alone when it does, and"
        " advances exactly the versions the write touched\n");
    return 0;
}
