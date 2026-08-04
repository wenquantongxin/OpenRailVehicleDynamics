#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <span>

#include <Eigen/Core>

#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "orvd/system_assembly/system_instance.h"

namespace {

using orvd::configuration::AssembleVehicleMultibodyModel;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::configuration::VehicleDefinition;
using orvd::configuration::VehicleRigidBodyDefinition;
using orvd::multibody_model::MultibodyModel;

constexpr double kMachine = std::numeric_limits<double>::epsilon();
constexpr double kGravityMagnitude = 9.81;

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

// ---------------------------------------------------------------------------
// Gate 1 — the inertia conversion, checked three ways that cannot fail together
// ---------------------------------------------------------------------------

Eigen::Matrix3d StatedInertiaAboutCenterOfMass(
    const VehicleRigidBodyDefinition& body) {
    const Eigen::Vector3d& moments =
        body.inertia_moments_about_center_of_mass_kilogram_square_meters;
    const Eigen::Vector3d& products =
        body.inertia_products_about_center_of_mass_kilogram_square_meters;
    Eigen::Matrix3d inertia;
    inertia << moments.x(), products.x(), products.y(),
        products.x(), moments.y(), products.z(),
        products.y(), products.z(), moments.z();
    return inertia;
}

// The textbook inverse, written from the definition rather than by solving the
// forward map algebraically. Solving it would give a pair that agrees exactly
// for any shift term at all, including a wrong one.
Eigen::Matrix3d InvertToCenterOfMass(
    const Eigen::Matrix3d& unit_inertia_about_body_origin, double mass,
    const Eigen::Vector3d& center_of_mass) {
    const Eigen::Matrix3d shift =
        center_of_mass.squaredNorm() * Eigen::Matrix3d::Identity() -
        center_of_mass * center_of_mass.transpose();
    return mass * (unit_inertia_about_body_origin - shift);
}

// The same conversion reached without the parallel axis theorem at all: six
// point masses of m/6 on the principal axes reproduce any admissible inertia,
// and once placed they can simply be summed about the body origin. Nothing here
// can share a mistake with the production code.
Eigen::Matrix3d InertiaAboutOriginFromSixPointMasses(
    const Eigen::Matrix3d& inertia_about_center_of_mass, double mass,
    const Eigen::Vector3d& center_of_mass) {
    const double xx = inertia_about_center_of_mass(0, 0);
    const double yy = inertia_about_center_of_mass(1, 1);
    const double zz = inertia_about_center_of_mass(2, 2);
    const double a_squared = 1.5 * (-xx + yy + zz) / mass;
    const double b_squared = 1.5 * (xx - yy + zz) / mass;
    const double c_squared = 1.5 * (xx + yy - zz) / mass;
    if (a_squared < 0.0 || b_squared < 0.0 || c_squared < 0.0) {
        throw std::runtime_error(
            "a body's principal moments do not admit a six-point-mass model");
    }
    const double particle_mass = mass / 6.0;
    const Eigen::Vector3d offsets(std::sqrt(a_squared), std::sqrt(b_squared),
                                  std::sqrt(c_squared));
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {1.0, -1.0}) {
            Eigen::Vector3d particle = center_of_mass;
            particle[axis] += sign * offsets[axis];
            inertia += particle_mass *
                       (particle.squaredNorm() * Eigen::Matrix3d::Identity() -
                        particle * particle.transpose());
        }
    }
    return inertia;
}

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
    skew(0, 1) = -v.z();
    skew(0, 2) = v.y();
    skew(1, 0) = v.z();
    skew(1, 2) = -v.x();
    skew(2, 0) = -v.y();
    skew(2, 1) = v.x();
    return skew;
}

void CheckInertiaConversion(const VehicleDefinition& vehicle,
                            const MultibodyModel& model) {
    int bodies_with_offset_center_of_mass = 0;
    double worst_round_trip = 0.0;
    double worst_point_mass_gap = 0.0;

    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        const Eigen::Matrix3d stated = StatedInertiaAboutCenterOfMass(body);
        const Eigen::Vector3d& center_of_mass =
            body.center_of_mass_in_body_frame_meters;
        if (center_of_mass.norm() > 0.0) {
            ++bodies_with_offset_center_of_mass;
        }

        // What the assembler produced, read back through the same shape the
        // multibody layer stores: the unit inertia about the body origin.
        const Eigen::Matrix3d about_origin =
            InertiaAboutOriginFromSixPointMasses(stated, body.mass_kilograms,
                                                 center_of_mass);
        const Eigen::Matrix3d unit_about_origin =
            about_origin / body.mass_kilograms;
        const Eigen::Matrix3d recovered = InvertToCenterOfMass(
            unit_about_origin, body.mass_kilograms, center_of_mass);

        const double scale = stated.cwiseAbs().maxCoeff();
        worst_round_trip = std::max(
            worst_round_trip,
            (recovered - stated).cwiseAbs().maxCoeff() / scale);

        // The physical anchor and the parallel axis theorem must agree; the
        // anchor never writes the theorem down.
        const Eigen::Matrix3d by_theorem =
            stated + body.mass_kilograms *
                         (center_of_mass.squaredNorm() *
                              Eigen::Matrix3d::Identity() -
                          center_of_mass * center_of_mass.transpose());
        worst_point_mass_gap =
            std::max(worst_point_mass_gap,
                     (about_origin - by_theorem).cwiseAbs().maxCoeff() / scale);
    }

    Expect(bodies_with_offset_center_of_mass == 3,
           "exactly three bodies have an offset centre of mass, so the shift "
           "term is exercised at all rather than being a run of zeros");
    Expect(worst_round_trip <= 16.0 * kMachine,
           "the inertia round trip closes on every body");
    Expect(worst_point_mass_gap <= 64.0 * kMachine,
           "the parallel axis theorem agrees with a six-point-mass model that "
           "never writes the theorem down");

    // The third layer. The parallel axis map is even in the centre-of-mass
    // offset, so the two checks above pass unchanged whichever sign it carries;
    // the mass matrix coupling block is odd in it and therefore shows whether
    // the assembler carried the sign through.
    //
    // What this cannot do — and what no committed check can — is tell whether
    // the sign the record states is the one the source model states. A record
    // and a model built from it agree by construction. That comparison is the
    // one-off audit against the source, and this gate does not stand in for it.
    const auto context = model.CreateDefaultContext();
    Eigen::MatrixXd mass_matrix(model.num_generalized_velocities(),
                                model.num_generalized_velocities());
    model.CalcGeneralizedMassMatrix(*context, mass_matrix);

    // The block belongs to the whole rigid sub-assembly a free body carries,
    // not to that body alone: anything welded to it moves with it. So the
    // expected first moment is the composite one, which means this check also
    // sees the weld poses — a weld mounted at the wrong station would move the
    // composite centre of mass and show up right here.
    std::map<std::string, const VehicleRigidBodyDefinition*> by_name;
    for (const auto& body : vehicle.rigid_bodies) {
        by_name.emplace(body.name, &body);
    }
    std::map<std::string, const orvd::configuration::VehicleFixedFrameDefinition*>
        frame_by_name;
    for (const auto& frame : vehicle.fixed_frames) {
        frame_by_name.emplace(frame.name, &frame);
    }

    double worst_coupling_gap = 0.0;
    double largest_coupling_magnitude = 0.0;
    double largest_weld_contribution = 0.0;
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        if (!body.moves_freely_in_world) {
            continue;
        }
        double carried_mass = body.mass_kilograms;
        Eigen::Vector3d first_moment =
            body.mass_kilograms * body.center_of_mass_in_body_frame_meters;
        const Eigen::Vector3d bare_first_moment = first_moment;

        for (const auto& weld : vehicle.weld_joints) {
            const auto seat = frame_by_name.find(weld.parent_frame_name);
            if (seat == frame_by_name.end() ||
                seat->second->body_name != body.name) {
                continue;
            }
            const auto welded = by_name.find(weld.child_frame_name);
            Expect(welded != by_name.end(),
                   "a weld's child is a body's own frame, so the composite "
                   "first moment can be formed from the record alone");
            if (welded == by_name.end()) {
                continue;
            }
            const Eigen::Vector3d welded_center_of_mass =
                seat->second->position_in_body_frame_meters +
                seat->second->rotation_in_body_frame *
                    welded->second->center_of_mass_in_body_frame_meters;
            carried_mass += welded->second->mass_kilograms;
            first_moment +=
                welded->second->mass_kilograms * welded_center_of_mass;
        }
        largest_weld_contribution =
            std::max(largest_weld_contribution,
                     (first_moment - bare_first_moment).cwiseAbs().maxCoeff());

        const auto handle = model.GetRigidBodyByName(body.name);
        const auto range = model.GetFreeBodyVelocityRange(handle);
        const Eigen::Matrix3d coupling =
            mass_matrix.block<3, 3>(range.start(), range.start() + 3);
        const Eigen::Matrix3d expected = SkewSymmetric(first_moment);
        const double scale = std::max(1.0, expected.cwiseAbs().maxCoeff());
        worst_coupling_gap =
            std::max(worst_coupling_gap,
                     (coupling - expected).cwiseAbs().maxCoeff() / scale);
        largest_coupling_magnitude =
            std::max(largest_coupling_magnitude, expected.cwiseAbs().maxCoeff());
        Expect(carried_mass > 0.0, "a free body carries positive mass");
    }
    Expect(largest_weld_contribution > 1.0,
           "the welded bodies move the composite first moment measurably, so "
           "this check sees the weld mounting poses and not only the free "
           "bodies' own centres of mass");
    Expect(worst_coupling_gap <= 64.0 * kMachine,
           "the assembled mass matrix carries the centre of mass with the sign "
           "the record states");
    Expect(largest_coupling_magnitude > 1.0e4,
           "that coupling block is far from zero on this vehicle, so the check "
           "above distinguishes a flipped centre of mass rather than comparing "
           "two ways of writing nothing");
}

// ---------------------------------------------------------------------------
// Gate 2 — geometric invariants of the named frames, in the world
// ---------------------------------------------------------------------------

Eigen::Vector3d FramePositionInWorld(const MultibodyModel& model,
                                     const orvd::multibody_model::
                                         MultibodyEvaluationContext& context,
                                     std::string_view name) {
    return model.CalcPoseInWorld(context, model.GetFrameByName(name))
        .translation();
}

void CheckGeometricInvariants(const VehicleDefinition& vehicle,
                              const MultibodyModel& model) {
    const auto context = model.CreateDefaultContext();

    // Mirror symmetry, over exactly the frames that have a lateral coordinate
    // to mirror. A frame on the longitudinal mid-plane has nothing to say here,
    // and asserting over it would make the discriminating-power proof below
    // pass on a frame whose sign cannot be flipped.
    std::map<std::string, Eigen::Vector3d> positions;
    for (const auto& frame : vehicle.fixed_frames) {
        positions.emplace(frame.name,
                          FramePositionInWorld(model, *context, frame.name));
    }

    int frames_naming_a_side = 0;
    for (const auto& frame : vehicle.fixed_frames) {
        if (frame.name.find("_right") != std::string::npos) {
            ++frames_naming_a_side;
        }
    }
    int mirrored_pairs = 0;
    int unpaired_lateral_frames = 0;
    double worst_mirror_residual = 0.0;
    for (const auto& [name, position] : positions) {
        if (position.y() <= 0.0) {
            continue;
        }
        const std::size_t right = name.rfind("_right_");
        std::string partner;
        if (right != std::string::npos) {
            partner = name.substr(0, right) + "_left_" +
                      name.substr(right + std::string("_right_").size());
        } else if (name.size() > 6 && name.compare(name.size() - 6, 6,
                                                   "_right") == 0) {
            partner = name.substr(0, name.size() - 6) + "_left";
        }
        const auto found = positions.find(partner);
        if (partner.empty() || found == positions.end()) {
            ++unpaired_lateral_frames;
            std::printf("     unpaired lateral frame: %s\n", name.c_str());
            continue;
        }
        ++mirrored_pairs;
        const Eigen::Vector3d mirrored(found->second.x(), -found->second.y(),
                                       found->second.z());
        worst_mirror_residual =
            std::max(worst_mirror_residual,
                     (position - mirrored).cwiseAbs().maxCoeff());
    }
    Expect(unpaired_lateral_frames == 0,
           "every frame with a lateral offset has a partner on the other side");
    Expect(mirrored_pairs == frames_naming_a_side,
           "every frame whose name states a side was mirrored, so the assertion "
           "covers the whole paired subset rather than whichever part happened "
           "to be found");
    Expect(mirrored_pairs > 20,
           "that subset is most of the frame family, not a token pair");
    Expect(worst_mirror_residual <= 1.0e-12,
           "left and right frames of the same name mirror about the vehicle's "
           "longitudinal mid-plane");

    // The wheelbase, between the two primary-suspension attachment frames of one
    // bogie. Not between the axlebox bushing frames: those are separated by
    // lt - 2*bAxle_x = 1.54 m, which is not the wheelbase and would be a
    // plausible-looking wrong answer.
    const double wheelbase =
        FramePositionInWorld(model, *context,
                             "front_leading_right_primary_suspension_"
                             "bogie_side_attachment")
            .x() -
        FramePositionInWorld(model, *context,
                             "front_trailing_right_primary_suspension_"
                             "bogie_side_attachment")
            .x();
    Expect(std::abs(wheelbase - 2.5) <= 1.0e-12,
           "the primary suspension frames of one bogie are one wheelbase apart");

    // The bogie centre distance, on the carbody-side weld mounting frames. In
    // the default configuration every free body sits at the world origin, so
    // the two bogie frames are not where this distance lives; the carbody's
    // mounting seats are.
    const double bogie_centre_distance =
        FramePositionInWorld(model, *context,
                             "front_secondary_suspension_seat_carbody_side_"
                             "attachment")
            .x() -
        FramePositionInWorld(model, *context,
                             "rear_secondary_suspension_seat_carbody_side_"
                             "attachment")
            .x();
    Expect(std::abs(bogie_centre_distance - 15.7) <= 1.0e-12,
           "the two weld mounting seats are one bogie centre distance apart");

    // Discriminating power, stated on a frame that actually has a lateral
    // coordinate to flip. On a mid-plane frame the same mutation changes
    // nothing, which is why the assertion above is scoped to the paired subset.
    const Eigen::Vector3d probe = FramePositionInWorld(
        model, *context,
        "front_leading_right_primary_suspension_bogie_side_attachment");
    Expect(std::abs(probe.y()) > 0.5,
           "the frame the mirror proof is stated on is far off the mid-plane, "
           "so flipping its lateral sign moves it by metres rather than by "
           "nothing");
}

// ---------------------------------------------------------------------------
// Gate 3 — the assembled model is all and only what the record states
// ---------------------------------------------------------------------------

void CheckAssemblyIsAllAndOnly(const VehicleDefinition& vehicle,
                               const MultibodyModel& model) {
    Expect(model.is_finalized(), "the model finalized");
    Expect(model.gravity_vector() == Eigen::Vector3d(0.0, 0.0, kGravityMagnitude),
           "the assembler stated gravity along the line's downward axis rather "
           "than leaving the multibody layer's own value, which points the "
           "other way");

    Expect(model.num_rigid_bodies() ==
               static_cast<int>(vehicle.rigid_bodies.size()),
           "every body the record states became a body, and no other body was "
           "added");
    // One world frame, one frame per body, and the record's named frames.
    Expect(model.num_frames() == 1 + static_cast<int>(
                                         vehicle.rigid_bodies.size() +
                                         vehicle.fixed_frames.size()),
           "every named frame the record states became a frame, and no other "
           "frame was added");
    Expect(model.num_joints() ==
               static_cast<int>(vehicle.revolute_joints.size() +
                                vehicle.weld_joints.size()),
           "free bodies produced no named joints, so the count is the record's "
           "joints and nothing else");

    int declared_free = 0;
    for (const auto& body : vehicle.rigid_bodies) {
        const auto handle = model.GetRigidBodyByName(body.name);
        Expect(model.IsFreeBody(handle) == body.moves_freely_in_world,
               "body '" + body.name + "' is free exactly when the record says");
        declared_free += body.moves_freely_in_world ? 1 : 0;
    }
    Expect(declared_free == 7, "the record declares seven free bodies");

    for (const auto& frame : vehicle.fixed_frames) {
        static_cast<void>(model.GetFrameByName(frame.name));
    }
    for (const auto& joint : vehicle.revolute_joints) {
        static_cast<void>(model.GetJointByName(joint.name));
    }
    for (const auto& joint : vehicle.weld_joints) {
        static_cast<void>(model.GetJointByName(joint.name));
    }

    double total_mass = 0.0;
    for (const auto& body : vehicle.rigid_bodies) {
        total_mass += body.mass_kilograms;
    }
    Expect(total_mass == 55695.0,
           "the ten placeholder bodies are still carrying their mass into the "
           "vehicle total");
}

void CheckStableIndexingAcrossTwoAssemblies(const VehicleDefinition& vehicle) {
    const auto first = AssembleVehicleMultibodyModel(vehicle, kGravityMagnitude);
    const auto second = AssembleVehicleMultibodyModel(vehicle, kGravityMagnitude);
    bool same_order = first->num_rigid_bodies() == second->num_rigid_bodies();
    for (int index = 0; same_order && index < first->num_rigid_bodies();
         ++index) {
        same_order = first->GetRigidBody(index) == second->GetRigidBody(index);
    }
    Expect(!same_order,
           "handles are not a cross-assembly identity: two models built from "
           "one record hand out different ones, so a caller must index by name");
    Expect(first->num_generalized_positions() ==
                   second->num_generalized_positions() &&
               first->num_generalized_velocities() ==
                   second->num_generalized_velocities(),
           "two assemblies of one record give the same coordinate counts");
    Expect(first->num_generalized_positions() == 57 &&
               first->num_generalized_velocities() == 50,
           "seven free bodies and eight revolute joints give 57 positions and "
           "50 velocities");
}

void CheckRejections(const VehicleDefinition& vehicle) {
    const auto refuses = [](VehicleDefinition broken,
                            const std::string& description) {
        try {
            static_cast<void>(
                AssembleVehicleMultibodyModel(broken, kGravityMagnitude));
        } catch (const std::exception&) {
            return;
        }
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    };

    VehicleDefinition unknown_frame = vehicle;
    unknown_frame.revolute_joints.front().parent_frame_name = "no_such_frame";
    refuses(std::move(unknown_frame),
            "a joint naming a frame the assembler cannot resolve is refused");

    VehicleDefinition orphan = vehicle;
    for (auto& body : orphan.rigid_bodies) {
        if (body.name == "front_bogie_frame") {
            body.moves_freely_in_world = false;
        }
    }
    refuses(std::move(orphan),
            "a body left with no path to the world is refused");

    VehicleDefinition doubly_related = vehicle;
    doubly_related.weld_joints.push_back(
        {"extra_weld", "carbody", "front_bogie_frame"});
    refuses(std::move(doubly_related),
            "a body both declared free and welded into the tree is refused");

    VehicleDefinition repeated_name = vehicle;
    repeated_name.rigid_bodies.push_back(repeated_name.rigid_bodies.front());
    refuses(std::move(repeated_name), "a repeated body name is refused");

    VehicleDefinition zero_axis = vehicle;
    zero_axis.revolute_joints.front().axis_in_parent_frame =
        Eigen::Vector3d::Zero();
    refuses(std::move(zero_axis), "a joint axis of zero length is refused");

    VehicleDefinition negative_damping = vehicle;
    negative_damping.revolute_joints.front()
        .damping_newton_metre_seconds_per_radian = -1.0;
    refuses(std::move(negative_damping), "a negative damping is refused");

    for (const double magnitude : {0.0, -9.81,
                                   std::numeric_limits<double>::infinity()}) {
        try {
            static_cast<void>(AssembleVehicleMultibodyModel(vehicle, magnitude));
            std::printf("FAIL a gravity magnitude of %g was accepted\n",
                        magnitude);
            ++failure_count;
        } catch (const std::invalid_argument&) {
        }
    }
}

// The assembled vehicle has to be something the rest of the product can pick up.
// This drives it once through the system-assembly chain and asks the only
// question that has a closed-form answer with no forces applied: does the whole
// vehicle fall, and does it fall the way this project's frame says down is. It
// is also the end-to-end statement of the gravity direction — a model built with
// the multibody layer's own default would accelerate the other way here.
void CheckTheVehicleFallsTheWayDownIs(const MultibodyModel& model) {
    using orvd::system_assembly::CompiledSystemPlan;
    using orvd::system_assembly::SystemAssemblyDescription;
    using orvd::system_assembly::SystemInstance;

    const SystemAssemblyDescription description(model);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);

    Expect(system.continuous_state_size() ==
               model.num_generalized_positions() +
                   model.num_generalized_velocities(),
           "the system carries the vehicle's own coordinates and nothing else");

    auto runtime = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd derivative(system.continuous_state_size());
    plan.CalcStateTimeDerivatives(*runtime, {}, {}, {}, derivative);

    const int velocity_start = model.num_generalized_positions();
    double worst_angular = 0.0;
    double worst_translational = 0.0;
    for (const auto& body : {std::string("carbody"),
                             std::string("front_bogie_frame"),
                             std::string("rear_leading_wheelset")}) {
        const auto range =
            model.GetFreeBodyVelocityRange(model.GetRigidBodyByName(body));
        for (int index = 0; index < 3; ++index) {
            worst_angular = std::max(
                worst_angular,
                std::abs(derivative[velocity_start + range.start() + index]));
        }
        const Eigen::Vector3d translational_acceleration(
            derivative[velocity_start + range.start() + 3],
            derivative[velocity_start + range.start() + 4],
            derivative[velocity_start + range.start() + 5]);
        worst_translational =
            std::max(worst_translational,
                     (translational_acceleration -
                      Eigen::Vector3d(0.0, 0.0, kGravityMagnitude))
                         .cwiseAbs()
                         .maxCoeff());
    }
    Expect(worst_angular <= 1.0e-12,
           "an unforced vehicle does not start rotating");
    Expect(worst_translational <= 1.0e-12,
           "every free body accelerates at g along the line's downward axis, "
           "which is the opposite of what the multibody layer's own default "
           "gravity would give");
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            throw std::invalid_argument("expected the GZ18 record path");
        }
        const VehicleDefinition vehicle =
            LoadVehicleDefinitionFromJsonFile(std::filesystem::path(argv[1]));
        const auto model =
            AssembleVehicleMultibodyModel(vehicle, kGravityMagnitude);

        CheckInertiaConversion(vehicle, *model);
        CheckGeometricInvariants(vehicle, *model);
        CheckAssemblyIsAllAndOnly(vehicle, *model);
        CheckStableIndexingAcrossTwoAssemblies(vehicle);
        CheckTheVehicleFallsTheWayDownIs(*model);
        CheckRejections(vehicle);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "GZ18 vehicle assembly failed: %s\n", error.what());
        return 1;
    }
    if (failure_count != 0) {
        std::printf("%d GZ18 vehicle assembly checks failed\n", failure_count);
        return 1;
    }
    std::printf("GZ18 vehicle assembly verified\n");
    return 0;
}
