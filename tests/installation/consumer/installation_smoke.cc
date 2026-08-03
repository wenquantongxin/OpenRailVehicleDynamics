#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

#include <Eigen/Dense>

#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "orvd/track_geometry/track_geometry.h"

namespace {

using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemContinuousStateAdvancer;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackScalarProfile;
using orvd::track_geometry::TrackScalarSegment;
using orvd::track_geometry::TrackScalarSegmentShape;

bool Near(double measured, double expected) {
    return std::abs(measured - expected) <=
           1.0e-8 *
               std::max({1.0, std::abs(measured), std::abs(expected)});
}

// The installed line layer, exercised through its own public header rather than
// merely linked: a straight, level stretch whose centerline and track frame
// have closed-form values, so a consumer that compiled against a stale header
// or an empty archive would not reach the end of this function.
int RunInstalledLineSmoke() {
    TrackScalarSegment level;
    level.length_meters = 100.0;
    level.shape = TrackScalarSegmentShape::kConstant;
    level.start_value = 0.0;
    level.end_value = 0.0;
    const TrackGeometry line(TrackScalarProfile(0.0, {level}, {}),
                             TrackScalarProfile(0.0, {level}, {}),
                             TrackScalarProfile(0.0, {level}, {}), 1.5, 1.0);
    const auto kinematics = line.EvaluateTrackFrame(40.0);
    const Eigen::Vector3d origin = kinematics.pose().origin_in_inertial_meters();
    const Eigen::Matrix3d rotation =
        kinematics.pose().rotation_inertial_from_track();
    if (!Near(origin.x(), 40.0) || !Near(origin.y(), 0.0) ||
        !Near(origin.z(), 0.0) ||
        (rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1.0e-12) {
        std::fprintf(stderr,
                     "installed ORVD line smoke produced origin (% .17g, % .17g,"
                     " % .17g) on a straight level line\n",
                     origin.x(), origin.y(), origin.z());
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    try {
        constexpr double kMassKilograms = 2.0;
        constexpr double kUnitInertia = 0.5;
        constexpr double kDamping = 0.4;
        constexpr double kInitialPosition = 0.2;
        constexpr double kInitialVelocity = 1.4;
        constexpr double kTargetTime = 0.1;

        RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = kMassKilograms;
        inertia.center_of_mass_in_body_frame.setZero();
        inertia.unit_inertia_moments.setConstant(kUnitInertia);
        inertia.unit_inertia_products.setZero();

        MultibodyModel model;
        const auto rotor = model.AddRigidBody("rotor", inertia);
        model.AddRevoluteJoint(
            "bearing", model.world_frame(), model.body_frame(rotor),
            Eigen::Vector3d::UnitZ(), kDamping);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();

        const SystemAssemblyDescription description(model);
        const SystemInstance system(description);
        const CompiledSystemPlan plan(system);
        auto context = system.CreateDefaultRuntimeContext(0.0);

        Eigen::Vector2d initial_state;
        initial_state << kInitialPosition, kInitialVelocity;
        system.SetContinuousState(*context, initial_state);

        SystemContinuousStateAdvancer advancer(
            system, plan, *context,
            ContinuousStateErrorTolerances(
                1.0e-10, Eigen::VectorXd::Constant(2, 1.0e-12)),
            NoCallTimeAppliedForces{});
        advancer.AdvanceTo(kTargetTime);

        Eigen::VectorXd observed(system.continuous_state_size());
        system.CopyContinuousState(*context, observed);

        const double inertia_about_axis = kMassKilograms * kUnitInertia;
        const double decay_rate = kDamping / inertia_about_axis;
        const double expected_velocity =
            kInitialVelocity * std::exp(-decay_rate * kTargetTime);
        const double expected_position =
            kInitialPosition +
            kInitialVelocity *
                (1.0 - std::exp(-decay_rate * kTargetTime)) / decay_rate;

        if (context->time_seconds() != kTargetTime || observed.size() != 2 ||
            !observed.allFinite() ||
            !Near(observed[0], expected_position) ||
            !Near(observed[1], expected_velocity)) {
            std::fprintf(
                stderr,
                "installed ORVD smoke produced t=% .17g q=% .17g v=% .17g; "
                "expected t=% .17g q=% .17g v=% .17g\n",
                context->time_seconds(), observed[0], observed[1],
                kTargetTime, expected_position, expected_velocity);
            return 1;
        }
        if (RunInstalledLineSmoke() != 0) {
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed ORVD smoke failed: %s\n", error.what());
        return 1;
    }
    return 0;
}
