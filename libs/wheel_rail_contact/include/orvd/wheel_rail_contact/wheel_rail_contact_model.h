#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

#include "orvd/wheel_rail_contact/contact_creepage.h"
#include "orvd/wheel_rail_contact/contact_geometry.h"
#include "orvd/wheel_rail_contact/contact_wrench.h"
#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"
#include "orvd/wheel_rail_contact/normal_contact_force.h"
#include "orvd/wheel_rail_contact/profile_points.h"
#include "orvd/wheel_rail_contact/tangential_contact_force.h"
#include "orvd/wheel_rail_contact/wheel_profile_preprocessing.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

// One wheel against one rail: from where the wheelset is to what the contact
// does to it.
//
// This is the whole contact model, assembled from the pieces around it. It owns
// the geometry solver, the normal law, the creep coefficients and the
// tangential solver, and it is immutable after construction — two evaluations
// in any order give the same answers, provided each has its own workspace.
//
// ## What the caller must supply, and why it is not less
//
// Four things, and none of them is derivable from the others:
//
// - **The pose**, reduced to the four scalars the geometry solves against.
// - **The rail profile's frame** — where the rail's own cross-section sits in
//   the track frame and how it is turned. On a perfect line this is the laying
//   cant at the nominal offset. Under track irregularity it is not: the
//   irregularity displaces the rail and turns it, and it is *here* that the
//   difference enters the relative velocity. Taking it as an input rather than
//   reconstructing it from the pose is what keeps this model usable when an
//   irregularity model arrives.
// - **The wheel profile frame's rigid motion** — where the authored profile's
//   datum on the axle is, how its non-spinning basis is turned, and how that
//   datum is moving. The relative velocity at a contact point is a property of
//   the rigid body, not of the contact, and it cannot be recovered from the
//   pose scalars.
// - **The advance rate and the wheel's own rotation rate**, which set the speed
//   the creepages are measured against.
//
// ## The rail is a construction, not a body
//
// The rail has no state and no inertia in this model: it is a shape the line
// carries. So its material point at the contact is stationary by construction,
// and the relative velocity is the wheel's material velocity alone. That is a
// modelling decision with a consequence worth naming: a rail that deflects
// under load, or a track that moves, is not represented, and adding one means
// giving the rail a velocity here rather than correcting the forces later.
//
// ## What comes out
//
// Two points deliberately remain distinct. The rail material reference point
// is where the wheel's rigid-body velocity is evaluated. The wheel surface
// point is where the force acts, and is the point the paired wrench is reduced
// about. They differ through the two profiles' geometric constructions and the
// contact compliance. Treating them as one loses the force's lever arm when a
// consumer later reduces it about a body origin.
//
// One paired wrench per patch is therefore reduced about the wheel surface
// point and written in the track frame, plus the intermediate quantities each
// stage produced. The pair is emitted from one set of numbers, so the two
// halves cancel exactly at that point by construction.

namespace orvd::wheel_rail_contact {

struct WheelRailContactConfiguration {
    ContactGeometryConfiguration geometry;
    NormalContactConfiguration normal;
    CreepageConfiguration creepage;
    FrictionLaw friction;
    TangentialContactConfiguration tangential;
    // The material both force laws use. It overrides whatever the two
    // sub-configurations carry, so that the normal law and the tangential
    // solver cannot be given different steel.
    ContactMaterial material;
    // What the creep coefficients do when the contact ellipse leaves the
    // tabulated range of shapes.
    OutsideTableRule creep_coefficients_outside_table{OutsideTableRule::kAsymptotic};
};

// Where the rail's own cross-section sits, in the track frame.
struct RailProfileFrame {
    Eigen::Vector3d origin_track_meters{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d rotation_track_from_profile{Eigen::Matrix3d::Identity()};
};

// The rigid motion of the wheel profile's datum, in the track frame.
//
// The origin is on the axle at authored profile coordinates y=0, z=0. Putting
// the lateral wheelset-to-profile datum into this origin avoids carrying the
// same datum a second time through the contact model. The orientation maps the
// profile coordinates [circumferential x, lateral y, radial-down z] into the
// track frame and deliberately excludes wheel spin: an axisymmetric profile
// does not change its geometry when the wheel spins. The angular velocity, in
// contrast, is the wheel body's actual angular velocity and includes spin.
struct WheelProfileRigidMotion {
    Eigen::Vector3d origin_track_meters{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d rotation_track_from_profile{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d origin_velocity_track_meters_per_second{
        Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity_track_radians_per_second{
        Eigen::Vector3d::Zero()};
    // How fast the contact advances along the line.
    double arc_rate_meters_per_second{0.0};
    // The wheel's rotation about its own axle, negative running forward.
    double wheel_pitch_rate_radians_per_second{0.0};
};

struct WheelRailContactInput {
    ContactPoseScalars pose;
    RailProfileFrame rail_frame;
    WheelProfileRigidMotion wheel;
};

struct WheelRailContactPatchResult {
    // The interaction, both halves, about the compliant wheel-surface point P
    // and in the track frame. This is not the rail material reference point R
    // at which the relative velocity was evaluated.
    PairedContactWrench wrench;

    // What each stage produced, kept because a force that comes out wrong is
    // almost never wrong in the last stage.
    ContactPatch geometry;
    NormalContactResult normal;
    Creepages creepages;
    TangentialContactResult tangential;
    double friction_coefficient{0.0};
    double approach_speed_meters_per_second{0.0};
    // The angle of the frame the creepages were resolved in and the force was
    // written in. It is the rail's surface slope angle, and is reported because
    // a consumer resolving anything else into this contact's axes must use the
    // same angle rather than rebuild it.
    double contact_frame_angle_radians{0.0};
};

struct WheelRailContactResult {
    std::array<WheelRailContactPatchResult, kMaxContactPatches> patches{};
    std::size_t count{0};
    // The patches the geometry found before any force law rejected one. A count
    // that differs from `count` means a patch was in contact geometrically but
    // carried no load.
    std::size_t geometric_patch_count{0};
    // Per evaluation, not cumulative. The geometry attempts one
    // three-dimensional longitudinal resolution for every geometric patch,
    // so `geometric_patch_count` is also the attempt count. This count says
    // how many force-law evaluations retained the analytic baseline because
    // that measurement was unavailable.
    std::size_t analytic_longitudinal_length_fallback_count{0};
};

class WheelRailContactModel;

// The buffers one evaluation needs. One per context; never shared between
// threads.
class WheelRailContactWorkspace {
  public:
    WheelRailContactWorkspace() = default;

  private:
    friend class WheelRailContactModel;

    ContactGeometryWorkspace geometry_;
    // A numerical Jacobian frequently changes only wheel motion, while the
    // four pose scalars that define the contact geometry remain exactly the
    // same. Cache that geometry per interface. The model identity belongs to
    // the key because this workspace is default-constructible and may be used
    // sequentially with different immutable models. The four scalar object
    // representations preserve signed zero and admit no tolerance.
    bool geometry_cache_valid_{false};
    const WheelRailContactModel* geometry_cache_model_{nullptr};
    std::array<std::uint64_t, 4> geometry_cache_pose_key_{};
    ContactPatchSet geometry_cache_patches_;
    TangentialContactWorkspace tangential_;
};

class WheelRailContactModel {
  public:
    // Throws std::invalid_argument when a profile or a configuration number is
    // unusable; the message names which.
    WheelRailContactModel(const ProfilePoints& wheel_profile,
                          const ProfilePoints& rail_profile, WheelSide side,
                          const WheelProfilePreprocessingConfiguration& preparation,
                          const WheelRailContactConfiguration& configuration);

    // Evaluates every patch at one pose. PrepareWorkspace sizes the complete
    // hot-path workspace; subsequent evaluations allocate nothing. An
    // unavailable three-dimensional longitudinal extent uses and reports the
    // analytic baseline. The call may still propagate a configured coefficient
    // table's documented refusal outside its supported range or an internal
    // workspace-bound violation.
    [[nodiscard]] WheelRailContactResult Evaluate(
        const WheelRailContactInput& input,
        WheelRailContactWorkspace& workspace) const;

    void PrepareWorkspace(WheelRailContactWorkspace& workspace) const;

    [[nodiscard]] const ContactGeometrySolver& geometry() const & {
        return geometry_;
    }
    [[nodiscard]] const ContactGeometrySolver& geometry() const && = delete;
    [[nodiscard]] const NormalContactLaw& normal_law() const & {
        return normal_law_;
    }
    [[nodiscard]] const NormalContactLaw& normal_law() const && = delete;
    [[nodiscard]] const TangentialContactSolver& tangential_solver() const & {
        return tangential_solver_;
    }
    [[nodiscard]] const TangentialContactSolver& tangential_solver() const && =
        delete;
    [[nodiscard]] const KalkerCoefficientTable& creep_coefficients() const & {
        return creep_coefficients_;
    }
    [[nodiscard]] const KalkerCoefficientTable& creep_coefficients() const && =
        delete;

  private:
    ContactGeometrySolver geometry_;
    NormalContactLaw normal_law_;
    KalkerCoefficientTable creep_coefficients_;
    TangentialContactSolver tangential_solver_;
    CreepageConfiguration creepage_;
    FrictionLaw friction_;
};

}  // namespace orvd::wheel_rail_contact
