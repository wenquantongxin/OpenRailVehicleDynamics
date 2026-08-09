#pragma once

#include <memory>
#include <string>
#include <vector>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/system_assembly/system_instance.h"
#include "orvd/track_geometry/track_geometry.h"

namespace orvd::configuration {

// Where one wheelset ended up, and what its two wheels are resolved to carry.
//
// The station is derived here rather than stated in the record: it is the
// vehicle's placement plus that body's mechanical offset plus whatever the
// resolution moved. A later goal seeding a per-axle observer reads it from
// here, so the placement is computed once and no second party repeats the sum.
//
// The two forces travel beside the station because they belong to the same
// wheelset. Two four-element vectors could fall out of step; one list of named
// records cannot.
struct ResolvedWheelsetPlacement {
    std::string wheelset_body_name;
    double track_station_meters{0.0};
    double left_support_force_newtons{0.0};
    double right_support_force_newtons{0.0};
};

// One assembled start-up: a runtime context holding the resolved state, and the
// facts about that resolution which are not state.
//
// The metadata is read-only. A caller may edit a `ResolvedStartupState` freely,
// but once one has been assembled the context and the identity, placement and
// target loads that came with it are one transaction: letting a caller change
// the recorded rail offset while the context stayed put would produce a pair
// that describes nothing.
//
// Lifetime: the `AssembledVehicleSystem` this came from must outlive it, and it
// must outlive any integrator borrowing its context. A later goal building a
// contact context must keep using the same line this was assembled against.
class ResolvedInitialContext {
   public:
    ResolvedInitialContext(ResolvedInitialContext&&) noexcept;
    ResolvedInitialContext& operator=(ResolvedInitialContext&&) noexcept;
    ResolvedInitialContext(const ResolvedInitialContext&) = delete;
    ResolvedInitialContext& operator=(const ResolvedInitialContext&) = delete;
    ~ResolvedInitialContext();

    [[nodiscard]] system_assembly::SystemRuntimeContext& context() const {
        return *context_;
    }

    [[nodiscard]] const std::vector<ResolvedWheelsetPlacement>&
    wheelset_placements() const {
        return wheelset_placements_;
    }

    // Signed, along +z_T, exactly as the record stated it.
    [[nodiscard]] double rail_profile_reference_vertical_offset_meters() const {
        return rail_profile_reference_vertical_offset_meters_;
    }

    [[nodiscard]] const StartupWheelRailBinding& wheel_rail_binding() const {
        return wheel_rail_binding_;
    }
    [[nodiscard]] const std::string& load_condition_identifier() const {
        return load_condition_identifier_;
    }

   private:
    friend ResolvedInitialContext AssembleResolvedInitialContext(
        const AssembledVehicleSystem&, const ResolvedStartupState&,
        const track_geometry::TrackGeometry&, double);
    friend ResolvedInitialContext AssembleResolvedInitialContext(
        const AssembledVehicleSystem&, const ResolvedStartupState&, double);

    ResolvedInitialContext(
        std::unique_ptr<system_assembly::SystemRuntimeContext> context,
        std::vector<ResolvedWheelsetPlacement> wheelset_placements,
        double rail_profile_reference_vertical_offset_meters,
        StartupWheelRailBinding wheel_rail_binding,
        std::string load_condition_identifier);

    std::unique_ptr<system_assembly::SystemRuntimeContext> context_;
    std::vector<ResolvedWheelsetPlacement> wheelset_placements_;
    double rail_profile_reference_vertical_offset_meters_{0.0};
    StartupWheelRailBinding wheel_rail_binding_;
    std::string load_condition_identifier_;
};

// Places a resolved start-up state on a line and writes it into a fresh runtime
// context.
//
// The station of the vehicle's layout reference body is this run's argument,
// not a field of the record: the same resolved identity can be assembled at any
// finite placement, and a record carrying an absolute mileage would disagree
// with the next scene that used it. Every other body's station is that argument
// plus its mechanical offset, from the vehicle definition, plus whatever the
// resolution moved.
//
// Everything is checked before anything is written, and the context is not the
// caller's until all of it succeeded. A failure part-way therefore leaves no
// half-written state anywhere: the context it would have gone into is destroyed
// with the exception.
//
// The order of the checks is itself a contract. The record's own invariants
// come first, so an ill-formed value gets its own diagnostic rather than a
// misleading identity mismatch. Identity and gravity come next, so a state
// resolved for another vehicle or another planet is refused before its stations
// are computed and reported. Only then are names resolved and the line
// consulted.
//
// Gravity is compared exactly. The preloads, the target wheel loads and the
// rail offset were all resolved under one value of it, so a system assembled
// under another is not the system this state describes, and no tolerance makes
// that less true.
//
// Every free body must have one finite station. It may lie inside the finite
// definition interval declared by the base-track asset or on TrackGeometry's native straight
// continuation beyond either boundary. The bundled GZ18 demonstration is
// qualified on straight, level, zero-superelevation track; this general
// research entry point deliberately also accepts a body or a wheelset on
// curved, graded or superelevated track. Such a modified start is assembled by
// the stated formulas but is not claimed to reproduce the source tool's
// complete type-7 railway-joint transport kinematics.
//
// Call this before constructing a `SystemContinuousStateAdvancer` on the
// returned context. An advancer copies the state and the context-local
// parameters when it is built; a context assembled after one already exists
// would need an explicit resynchronisation that nothing here can perform.
//
// Throws std::invalid_argument when the record violates its own invariants,
// when an identifier or gravity disagrees with the assembled system, when a
// name family is not exactly the system's, when a name resolves to nothing,
// when a body's station is not finite, or when a contact-enabled system is
// paired with a line other than the one its contact plan owns; the
// diagnostic states the offending amount where it applies.
[[nodiscard]] ResolvedInitialContext AssembleResolvedInitialContext(
    const AssembledVehicleSystem& system,
    const ResolvedStartupState& startup_state,
    const track_geometry::TrackGeometry& line,
    double vehicle_layout_reference_track_station_meters);

// The single-authority form for a contact-enabled system.  The line is owned
// by that system's wheel--rail force plan, so it cannot disagree with the line
// used to resolve the initial context.
//
// Throws std::invalid_argument if `system` has no contact force plan.
[[nodiscard]] ResolvedInitialContext AssembleResolvedInitialContext(
    const AssembledVehicleSystem& system,
    const ResolvedStartupState& startup_state,
    double vehicle_layout_reference_track_station_meters);

}  // namespace orvd::configuration
