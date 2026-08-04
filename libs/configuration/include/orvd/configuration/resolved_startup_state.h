#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// A start-up state somebody else already resolved.
//
// This module loads such a state; it does not solve for one. There is no
// equilibrium search, no preload solver and no iteration here, and none is
// coming: the numbers below are inputs, and a routine that recomputed one of
// them from the others would be answering a question this record has already
// answered.
//
// Two consequences run through the field names.
//
// First, nothing is derived. The wheelset spin is stated, not computed from the
// speed and a nominal rolling radius — the resolved value follows from the
// contact rolling radius under the resolved rail offset, which is not the
// nominal radius, and a loader that recomputed it would silently change the
// physics. That the two happen to agree in some other vehicle is not an error
// either; equality is not the test.
//
// Second, nothing carries absolute mileage. Where the vehicle sits on a line is
// a property of the run, not of the resolved state, so a body's longitudinal
// place is written as an offset from the vehicle's mechanical layout, and the
// layout itself belongs to the vehicle definition. The same resolved identity
// can then start at any admissible station of any admissible line.
//
// The state is expressed in the local track frame at each body's own station,
// because that is the frame a human author can write down and check. The
// inertial values the multibody layer wants are formed when a context is
// assembled, not stored here twice.

namespace orvd::configuration {

// Along which way the track station runs as the vehicle moves forward.
//
// Only the increasing direction is admitted. Reverse and through-zero start-up
// wait on qualification of the longitudinal creepage and creep force near and
// across zero speed, which has not been settled against the reference tool;
// running strictly forward avoids the question entirely. This is a physics
// debt, not a missing accessor: adding a support-end query to the line would
// not discharge it.
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
// The two velocities are inertial velocities. Only the basis differs: the
// angular one is written in the body frame because that is where a wheelset's
// spin is legible, and the origin velocity in the local track frame because
// that is where a forward speed is. Neither is a velocity relative to the track
// frame, and assembling one adds no transport term.
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
        body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second{
            Eigen::Vector3d::Zero()};
    Eigen::Vector3d
        body_origin_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second{
            Eigen::Vector3d::Zero()};
};

// One revolute joint's resolved coordinate and rate. Weld joints have no
// coordinate, so they are not listed: an entry for one would state a number
// nothing owns.
struct RevoluteJointStartupState {
    std::string joint_name;
    double position_radians{0.0};
    double rate_radians_per_second{0.0};
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
    // The forward speed of the layout reference body along the line. Strictly
    // positive: the direction is carried by the enumeration above, and a zero
    // or negative value would be a second, disagreeing authority for it.
    double initial_longitudinal_speed_meters_per_second{0.0};
    // Where the vehicle sits on the line is deliberately absent. A resolved
    // start-up state is an identity, not a placement: the same one is valid at
    // every admissible station of every admissible line, so the station of the
    // layout reference body is a run's argument to the assembly, not a field
    // here. Storing it would make this record disagree with the next scene that
    // used it.
    //
    // The signed vertical shift applied to the rail profile reference frame,
    // along +z_T, before contact geometry is evaluated. Positive lowers the
    // rail profile, negative raises it. It is passed through exactly as
    // written: taking its magnitude, clamping it or repairing its sign would
    // discard the resolution this record exists to carry.
    double rail_profile_reference_vertical_offset_meters{0.0};
    std::vector<WheelsetTargetSupportForces> target_wheel_support_forces;
    std::vector<FreeBodyStartupState> free_body_startup_states;
    std::vector<RevoluteJointStartupState> revolute_joint_startup_states;
    std::vector<SeriesSpringViscousDamperForceState>
        series_spring_viscous_damper_force_states;
    std::vector<TranslationalSpringDamperNominalForce>
        translational_spring_damper_nominal_forces;
};

}  // namespace orvd::configuration
