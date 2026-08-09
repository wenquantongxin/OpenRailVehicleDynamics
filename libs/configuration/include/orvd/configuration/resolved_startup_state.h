#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// A start-up state somebody else already resolved.
//
// This module loads such a state; it does not solve for one. There is no
// equilibrium search, no preload solver and no iteration here, and none is
// coming: the numbers below are inputs. The only expansion is the explicit
// Vehicle-Globals-style relation stated below; it is part of the record's
// contract, not an equilibrium or contact solve.
//
// Two consequences run through the field names.
//
// First, the vehicle start velocity and the common start-up effective rolling
// radius are the two authorities for the level-tangent GZ18 start-up. Their
// quotient is expanded into the four wheelset spins and the eight axlebox
// carrier pivot rates. The effective radius is not the nominal wheel radius:
// it belongs to the wheel/rail profiles, contact strategy, load condition and
// resolved rail offset named by this record. Other angular velocity components
// remain explicit and are added after that expansion.
//
// Second, nothing carries absolute mileage. Where the vehicle sits on a line is
// a property of the run, not of the resolved state, so a body's longitudinal
// place is written as an offset from the vehicle's mechanical layout, and the
// layout itself belongs to the vehicle definition. The same resolved identity
// can then be assembled at any finite station; only the bundled straight,
// level demonstration carries reference-tool qualification.
//
// The state is expressed in the local track frame at each body's own station,
// because that is the frame a human author can write down and check. The
// inertial values the multibody layer wants are formed when a context is
// assembled, not stored here twice.

namespace orvd::configuration {

// Along which way the track station runs as the vehicle moves forward.
//
// Only the increasing direction is admitted, and zero or negative V0 is
// refused. Near-zero and later through-zero wheel-rail physics has not been
// qualified against the reference tool; accepting a positive scalar is not a
// claim that those regimes have been validated. This is a physics debt, not a
// missing accessor: adding a support-end query to the line would not discharge
// it.
enum class StartupRunningDirection {
    kIncreasingTrackStation,
};

// Which vehicle this state was resolved for. Both are compared for equality
// when a start-up context is assembled.
struct StartupVehicleBinding {
    std::string vehicle_name;
    std::string mechanical_definition_identifier;
};

// Which wheel-rail objects this state was resolved against.
//
// The objects themselves do not exist yet, so these are checked for shape here
// and matched against real assets when a contact goal loads them. Saying so is
// the point: a check that cannot fail should not be described as one that
// passed.
struct StartupWheelRailBinding {
    std::string wheel_profile_identifier;
    std::string rail_profile_identifier;
    std::string wheel_rail_contact_strategy_identifier;
};

// One free body's resolved place and motion, in the local track frame at that
// body's own station.
//
// The station itself is not here. It is
//
//     vehicle layout reference station
//   + the body's mechanical station offset, from the vehicle definition
//   + `resolved_track_station_offset_from_mechanical_layout_meters`
//
// so the mechanical layout keeps its single authority and this record states
// only what the resolution moved.
//
// The additional angular velocity is an inertial angular velocity expressed in
// the body frame. For a wheelset it is added to the pure-rolling spin generated
// from the common effective radius; for another body it is the whole stated
// angular velocity. The lateral and vertical origin velocities are inertial
// velocity components expressed in the local track frame. The longitudinal
// component is generated from the vehicle start velocity instead of being
// copied into every body as a second authority.
struct FreeBodyStartupState {
    std::string body_name;
    // The rotation taking body-frame components to local-track-frame
    // components. Written as named w, x, y, z so no reader has to know which
    // order a library stores a quaternion in.
    Eigen::Quaterniond rotation_local_track_from_body{
        Eigen::Quaterniond::Identity()};
    double resolved_track_station_offset_from_mechanical_layout_meters{0.0};
    double lateral_offset_in_local_track_frame_meters{0.0};
    double vertical_offset_in_local_track_frame_meters{0.0};
    Eigen::Vector3d
        additional_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second{
            Eigen::Vector3d::Zero()};
    double
        body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second{
            0.0};
    double
        body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second{
            0.0};
};

// How one wheelset spins when the vehicle moves toward increasing station.
// The axis is a unit vector in the wheelset body frame and includes the sign;
// GZ18 therefore states (0,-1,0). Its magnitude is V0 divided by the common
// start-up effective rolling radius.
struct WheelsetStartupKinematics {
    std::string wheelset_body_name;
    Eigen::Vector3d forward_spin_axis_in_body_frame{Eigen::Vector3d::Zero()};
};

// One revolute joint's resolved coordinate and rate. Weld joints have no
// coordinate, so they are not listed: an entry for one would state a number
// nothing owns.
struct RevoluteJointStartupState {
    std::string joint_name;
    double position_radians{0.0};
    // Multiplied by the positive common wheel-spin magnitude V0/r_eff. GZ18's
    // eight axlebox carrier pivots use +1 so that their physical rotation
    // cancels the wheelset's -y spin.
    double rate_per_common_wheel_spin_magnitude{0.0};
};

// One series spring-viscous element's internal force state, along that
// element's stated axis and acting on its reference end.
//
// The reference tool stores the series spring's elongation instead; the two are
// one multiplication apart and the difference is recorded in the difference
// watchlist rather than reconciled here.
struct SeriesSpringViscousDamperForceState {
    std::string element_name;
    double reference_end_axial_force_newtons{0.0};
};

// One translational element's resolved preload: the force on its reference end,
// stated in that end's frame, exactly as the force plan will apply it.
struct TranslationalSpringDamperNominalForce {
    std::string element_name;
    Eigen::Vector3d force_on_reference_end_in_reference_frame_newtons{
        Eigen::Vector3d::Zero()};
};

// The support force each of one wheelset's two wheels is resolved to carry.
//
// Both are magnitudes of compressive support along -z_T, so both are positive:
// the direction is in the definition, not in the sign. Left and right are as
// seen facing increasing track station, and the track frame's +y is right.
//
// This is a constraint to check, never a force to apply. Nothing in this module
// puts it into a force slot, and a later goal compares it against a contact
// solution rather than injecting it.
struct WheelsetTargetSupportForces {
    std::string wheelset_body_name;
    double left_support_force_newtons{0.0};
    double right_support_force_newtons{0.0};
};

struct ResolvedStartupState {
    StartupVehicleBinding vehicle_binding;
    StartupWheelRailBinding wheel_rail_binding;
    // Names the load condition this state was resolved under. It is a name, not
    // a second copy of the masses: the vehicle definition already states those,
    // and a resolved state for another load condition is another record beside
    // another vehicle definition.
    std::string load_condition_identifier;
    // The gravity this state was resolved under. Every preload, every target
    // wheel load and the rail offset that produces them follow from it, so a
    // system assembled under a different gravity is not one this state
    // describes. Gravity is not part of a vehicle's mechanical identity, which
    // is why it is stated here and not in the vehicle definition.
    double gravitational_acceleration_meters_per_second_squared{0.0};
    StartupRunningDirection running_direction{
        StartupRunningDirection::kIncreasingTrackStation};
    // The Vehicle Globals style station speed applied to every GZ18 free body
    // represented by a General Rail Track Joint in the source model. Strictly
    // positive: the direction is carried by the enumeration above.
    //
    // This expansion is qualified for the bundled straight, level, zero-
    // superelevation GZ18 start-up. Other line geometries remain accepted for
    // research use, but this module does not claim that simply using this value
    // reproduces the source tool's complete Joint-7 transport kinematics there.
    double initial_longitudinal_speed_meters_per_second{0.0};
    // The common rolling radius that reproduces the qualified moving start-up
    // spin under this record's wheel/rail identities, load condition and rail
    // profile offset. It is a consumed model input, not a nominal wheel radius
    // and not an output snapshot used as a test oracle.
    double common_startup_effective_rolling_radius_meters{0.0};
    // Where the vehicle sits on the line is deliberately absent. A resolved
    // start-up state is an identity, not a placement: the same one can be
    // assembled at every finite station of a line or its native straight
    // continuation, so the station of the layout reference body is a run's
    // argument to the assembly, not a field here. Storing it would make this
    // record disagree with the next scene that used it.
    //
    // The signed vertical shift applied to the rail profile reference frame,
    // along +z_T, before contact geometry is evaluated. Positive lowers the
    // rail profile, negative raises it. It is passed through exactly as
    // written: taking its magnitude, clamping it or repairing its sign would
    // discard the resolution this record exists to carry.
    double rail_profile_reference_vertical_offset_meters{0.0};
    std::vector<WheelsetTargetSupportForces> target_wheel_support_forces;
    std::vector<WheelsetStartupKinematics> wheelset_startup_kinematics;
    std::vector<FreeBodyStartupState> free_body_startup_states;
    std::vector<RevoluteJointStartupState> revolute_joint_startup_states;
    std::vector<SeriesSpringViscousDamperForceState>
        series_spring_viscous_damper_force_states;
    std::vector<TranslationalSpringDamperNominalForce>
        translational_spring_damper_nominal_forces;
};

}  // namespace orvd::configuration
