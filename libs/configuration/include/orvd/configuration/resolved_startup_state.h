#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// A start-up state somebody else already resolved.
//
// This module loads such a state; it does not solve for one. There is no
// equilibrium search, no preload solver and no iteration here: the numbers
// below are inputs. A record may explicitly request one algebraic expansion of
// those inputs, the common wheel-spin relation used by a rigid wheelset. That
// relation is part of the record's contract, not an equilibrium or contact
// solve.
//
// Two consequences run through the field names.
//
// First, a rigid-wheelset record may carry a common start-up effective rolling
// radius. The vehicle start velocity divided by that radius is expanded through
// explicitly stated body-axis coefficients and joint-rate multipliers. An
// independently rotating wheelset does not carry that relation: its axle body
// angular velocity and its two wheel-to-axle joint rates are separate state.
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

// The one optional generated angular-speed authority.
//
// Its magnitude is the positive start speed divided by the stated effective
// rolling radius. GZ18 consumes it through the coefficients carried by its
// rigid wheelset bodies and axlebox-pivot joints. IRW has no such authority:
// its axle bodies and wheel-to-axle joints state their own initial velocities.
struct CommonWheelSpinGeneration {
    double effective_rolling_radius_meters{0.0};
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
// The explicit angular velocity is an inertial angular velocity expressed in
// the body frame. When the record has a common wheel-spin generation, the
// dimensionless body-frame coefficient multiplies that one generated magnitude
// and is added to the explicit value. GZ18's four rigid wheelsets use
// `(0,-1,0)`; IRW's independently rotating wheelset axle bodies use zero and
// keep their own free-body angular velocity here. The lateral and vertical
// origin velocities are inertial velocity components expressed in the local
// track frame. The longitudinal component is generated from the vehicle start
// velocity instead of being copied into every body as a second authority.
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
        explicit_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second{
            Eigen::Vector3d::Zero()};
    Eigen::Vector3d common_wheel_spin_coefficient_in_body_frame{
        Eigen::Vector3d::Zero()};
    double
        body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second{
            0.0};
    double
        body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second{
            0.0};
};

struct ExplicitRevoluteJointRate {
    double angular_rate_radians_per_second{0.0};
};

struct RevoluteJointRatePerCommonWheelSpin {
    double multiplier{0.0};
};

using RevoluteJointStartupRate =
    std::variant<ExplicitRevoluteJointRate,
                 RevoluteJointRatePerCommonWheelSpin>;

// One revolute joint's resolved coordinate and rate. Weld joints have no
// coordinate, so they are not listed: an entry for one would state a number
// nothing owns.
struct RevoluteJointStartupState {
    std::string joint_name;
    double position_radians{0.0};
    // An explicit wheel-to-axle relative rate for IRW, or a multiplier of the
    // common generated magnitude for GZ18's axlebox pivots. These are distinct
    // physical definitions, not two spellings of one value.
    RevoluteJointStartupRate rate{ExplicitRevoluteJointRate{}};
};

// One Ball-RPY joint's resolved coordinates and physical relative angular
// velocity. The three positions state R_FM = Rz(yaw) Ry(pitch) Rx(roll). The
// velocity is omega_FM expressed in the parent frame F; it is not an RPY angle
// derivative.
struct BallRpyJointStartupState {
    std::string joint_name;
    Eigen::Vector3d roll_pitch_yaw_angles_radians{Eigen::Vector3d::Zero()};
    Eigen::Vector3d
        angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second{
            Eigen::Vector3d::Zero()};
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

// The support force each of one left/right wheel pair is resolved to carry.
//
// Both are magnitudes of compressive support along -z_T, so both are positive:
// the direction is in the definition, not in the sign. Left and right are as
// seen facing increasing track station, and the track frame's +y is right.
//
// `station_reference_body_name` is the rigid wheelset body for GZ18 and the
// axle body for IRW. It provides the common longitudinal station only; it does
// not claim that both contact wrenches act on that body. This is a constraint
// to check, never a force to apply.
struct WheelPairTargetSupportForces {
    std::string station_reference_body_name;
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
    // The Vehicle Globals style station speed applied to every vehicle free
    // body represented by a General Rail Track Joint in the source model.
    // Strictly positive: the direction is carried by the enumeration above.
    //
    // This expansion is qualified for the bundled GZ18 and IRW H3 start-ups on
    // their straight, level, zero-superelevation starting segments. Other line
    // geometries remain accepted for research use, but this module does not
    // claim that simply using this value reproduces the source tool's complete
    // Joint-7 transport kinematics there.
    double initial_longitudinal_speed_meters_per_second{0.0};
    // Absent for an independently rotating wheelset whose axle and wheel-joint
    // velocities are already explicit. No inactive radius or sentinel is
    // carried in that case.
    std::optional<CommonWheelSpinGeneration> common_wheel_spin_generation;
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
    std::vector<WheelPairTargetSupportForces>
        wheel_pair_target_support_forces;
    std::vector<FreeBodyStartupState> free_body_startup_states;
    std::vector<RevoluteJointStartupState> revolute_joint_startup_states;
    std::vector<BallRpyJointStartupState> ball_rpy_joint_startup_states;
    std::vector<SeriesSpringViscousDamperForceState>
        series_spring_viscous_damper_force_states;
    std::vector<TranslationalSpringDamperNominalForce>
        translational_spring_damper_nominal_forces;
};

}  // namespace orvd::configuration
