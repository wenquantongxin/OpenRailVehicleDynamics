// Measures the reference implementation's real kinematics cache invalidation
// semantics on the pinned Drake, so later ORVD cache passes are built against
// measured behaviour rather than assumed behaviour.
//
// Measured reality (Drake 1.54.0): mutating either q or v through the public
// setters marks BOTH the position and velocity kinematics cache entries out of
// date. Drake can currently only obtain the mobilizer state as one mutable q/v
// block and says so in a source TODO, so the joint invalidation is a
// conservative property of the reference today — not an ORVD requirement.
// ORVD's planned cache will separate the two precisely. What must carry over:
// repeated evaluation never recomputes, stale entries recompute separately on
// demand, a serial number moves only when a recomputation actually happened,
// and Contexts are isolated from each other.
//
// Reference-only probe against the pinned Drake. The cache-entry accessors used
// here are Drake public API but must not shape any ORVD interface. Serial
// numbers are compared only as "unchanged" or "strictly increased": absolute
// values, cache indices and exact increments are implementation detail.
#include <cstdio>
#include <memory>
#include <string_view>

#include <Eigen/Dense>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/spatial_inertia.h>

namespace {

using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::systems::Context;

int failure_count = 0;

void RecordFailureUnless(bool condition, std::string_view failure_description) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %.*s\n",
                     static_cast<int>(failure_description.size()),
                     failure_description.data());
        ++failure_count;
    }
}

struct KinematicsCacheSnapshot {
    bool position_out_of_date;
    bool velocity_out_of_date;
    long long position_serial;
    long long velocity_serial;
};

KinematicsCacheSnapshot ReadKinematicsCaches(const MultibodyPlant<double>& plant,
                                             const Context<double>& context) {
    const auto& position_value =
        plant.position_kinematics_cache_entry().get_cache_entry_value(context);
    const auto& velocity_value =
        plant.velocity_kinematics_cache_entry().get_cache_entry_value(context);
    return {position_value.is_out_of_date(), velocity_value.is_out_of_date(),
            static_cast<long long>(position_value.serial_number()),
            static_cast<long long>(velocity_value.serial_number())};
}

}  // namespace

int main() {
    // A single free rigid body: nq=7 (quaternion + translation) differs from
    // nv=6 (angular + translational velocity), so a probe that confused the two
    // state segments could not pass by accident.
    MultibodyPlant<double> plant(0.0);
    const RigidBody<double>& free_body =
        plant.AddRigidBody("free_body", SpatialInertia<double>::MakeUnitary());
    plant.Finalize();
    RecordFailureUnless(plant.num_positions() == 7 && plant.num_velocities() == 6,
                        "the free body must expose distinct q and v dimensions");

    const auto make_warmed_context = [&plant, &free_body] {
        std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
        plant.EvalBodyPoseInWorld(*context, free_body);
        plant.EvalBodySpatialVelocityInWorld(*context, free_body);
        return context;
    };

    Eigen::VectorXd nonzero_velocities(6);
    nonzero_velocities << 0.11, -0.07, 0.05, 0.4, -0.3, 0.2;

    {  // Repeated evaluation must not recompute.
        const auto context = make_warmed_context();
        const KinematicsCacheSnapshot warmed = ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            !warmed.position_out_of_date && !warmed.velocity_out_of_date,
            "warmup must leave both kinematics caches fresh");
        plant.EvalBodyPoseInWorld(*context, free_body);
        plant.EvalBodySpatialVelocityInWorld(*context, free_body);
        const KinematicsCacheSnapshot repeated = ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            !repeated.position_out_of_date && !repeated.velocity_out_of_date &&
                repeated.position_serial == warmed.position_serial &&
                repeated.velocity_serial == warmed.velocity_serial,
            "repeated evaluation must not recompute either kinematics cache");
    }

    {  // Mutating v invalidates both caches; re-evaluation recomputes both.
        const auto context = make_warmed_context();
        const KinematicsCacheSnapshot warmed = ReadKinematicsCaches(plant, *context);
        plant.SetVelocities(context.get(), nonzero_velocities);
        const KinematicsCacheSnapshot mutated = ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            mutated.position_out_of_date && mutated.velocity_out_of_date,
            "SetVelocities must mark both kinematics caches out of date "
            "(Drake's conservative whole-q/v invalidation)");
        RecordFailureUnless(
            mutated.position_serial == warmed.position_serial &&
                mutated.velocity_serial == warmed.velocity_serial,
            "invalidation alone must not move a serial number");
        plant.EvalBodySpatialVelocityInWorld(*context, free_body);
        const KinematicsCacheSnapshot evaluated = ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            !evaluated.position_out_of_date && !evaluated.velocity_out_of_date,
            "evaluating velocities must leave both kinematics caches fresh");
        RecordFailureUnless(
            evaluated.position_serial > mutated.position_serial &&
                evaluated.velocity_serial > mutated.velocity_serial,
            "recomputation after a velocity change must strictly increase "
            "both serials");
    }

    {  // Mutating q invalidates both; stale entries recompute separately.
        const auto context = make_warmed_context();
        const KinematicsCacheSnapshot warmed = ReadKinematicsCaches(plant, *context);
        Eigen::VectorXd positions = plant.GetPositions(*context);
        positions(4) += 0.5;  // x translation of the floating base
        plant.SetPositions(context.get(), positions);
        const KinematicsCacheSnapshot mutated = ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            mutated.position_out_of_date && mutated.velocity_out_of_date,
            "SetPositions must mark both kinematics caches out of date");
        RecordFailureUnless(
            mutated.position_serial == warmed.position_serial &&
                mutated.velocity_serial == warmed.velocity_serial,
            "invalidation alone must not move a serial number");
        plant.EvalBodyPoseInWorld(*context, free_body);
        const KinematicsCacheSnapshot position_evaluated =
            ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            !position_evaluated.position_out_of_date &&
                position_evaluated.position_serial > mutated.position_serial,
            "evaluating the pose must refresh the position kinematics cache");
        RecordFailureUnless(
            position_evaluated.velocity_out_of_date &&
                position_evaluated.velocity_serial == mutated.velocity_serial,
            "evaluating the pose must leave the velocity kinematics cache "
            "stale and untouched");
        plant.EvalBodySpatialVelocityInWorld(*context, free_body);
        const KinematicsCacheSnapshot velocity_evaluated =
            ReadKinematicsCaches(plant, *context);
        RecordFailureUnless(
            !velocity_evaluated.velocity_out_of_date &&
                velocity_evaluated.velocity_serial >
                    position_evaluated.velocity_serial,
            "evaluating velocities must refresh the velocity kinematics cache");
        RecordFailureUnless(
            !velocity_evaluated.position_out_of_date &&
                velocity_evaluated.position_serial ==
                    position_evaluated.position_serial,
            "evaluating velocities must leave the already fresh position "
            "kinematics cache fresh and untouched");
    }

    {  // Mutating and evaluating one Context must not touch another.
        const auto mutated_context = make_warmed_context();
        const auto observed_context = make_warmed_context();
        const KinematicsCacheSnapshot before =
            ReadKinematicsCaches(plant, *observed_context);
        plant.SetVelocities(mutated_context.get(), nonzero_velocities);
        plant.EvalBodyPoseInWorld(*mutated_context, free_body);
        plant.EvalBodySpatialVelocityInWorld(*mutated_context, free_body);
        const KinematicsCacheSnapshot after =
            ReadKinematicsCaches(plant, *observed_context);
        RecordFailureUnless(
            before.position_out_of_date == after.position_out_of_date &&
                before.velocity_out_of_date == after.velocity_out_of_date &&
                before.position_serial == after.position_serial &&
                before.velocity_serial == after.velocity_serial,
            "mutating and evaluating one Context must not move the other "
            "Context's kinematics caches");
    }

    if (failure_count > 0) {
        std::fprintf(stderr, "%d cache semantics check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("drake multibody kinematics cache invalidation semantics verified\n");
    return 0;
}
