#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"

// The document a graphical modelling tool edits for one rail vehicle.
//
// A project holds the vehicle's mechanical definition and, once provided, its
// wheel-rail contact record. Those two are what the solver side receives,
// through generate_assembly_inputs.h. Beside them it holds two things that
// exist only for the person editing: where each free body is drawn, and how
// the elements are organised into groups. Neither reaches the solver side, so
// redrawing or reorganising a vehicle cannot change a number the dynamics
// sees.
//
// Like the vehicle definition it carries, this is deliberately a mutable value
// record: a caller may build any part of it directly and edit it in place.
// Nothing here keeps a reference to a project, and nothing generated from one
// refers back to it. The parts this library owns are checked when the caller
// asks for it (vehicle_project_invariants.h), not on every assignment.

namespace orvd::vehicle_project {

// Where the person editing puts one free body: its lateral and vertical
// offsets and its rotation in the local track frame at the body's own station,
// with the conventions of configuration::FreeBodyStartupState. That frame has
// +y to the right of an observer facing increasing track station and +z
// downward, so a body above the track plane has a negative vertical offset.
//
// The longitudinal place is deliberately absent. A body's station is the
// layout reference station plus the body's mechanical station offset, and the
// vehicle definition stays the single authority for that offset; unlike a
// start-up state, a placement carries no resolved correction to it either.
//
// A placement is not a start-up state. It is read for drawing and editing only,
// carries no velocity, takes no part in dynamics, and nothing generated from a
// project depends on it.
struct FreeBodyPlacement {
    std::string body_name;
    // The rotation taking body-frame components to local-track-frame
    // components.
    Eigen::Quaterniond rotation_local_track_from_body{Eigen::Quaterniond::Identity()};
    double lateral_offset_in_local_track_frame_meters{0.0};
    double vertical_offset_in_local_track_frame_meters{0.0};
};

// Every kind of element a project contains: each element family of
// configuration::VehicleDefinition, named there by `name`, and the carriers and
// interfaces of the contact record's declaration, named by `carrier_name` and
// `interface_name`.
enum class ElementKind {
    kRigidBody,
    kFixedFrame,
    kRevoluteJoint,
    kBallRpyJoint,
    kWeldJoint,
    kTranslationalSpringDamper,
    kRollSpringDamperCouple,
    kSeriesSpringViscousDamper,
    kPointToPointSeriesSpringBilinearDamper,
    kSaturatedPiecewiseLinearDamper,
    kReferenceFrameLinearBushing,
    kPointToPointLinearSpringDamper,
    kPointToPointHystereticSpring,
    kContactCarrier,
    kContactInterface,
};

// One element, identified by its kind and by the name its declaring record
// gives it. A name alone does not identify an element: GZ18's contact carriers
// carry the names of their wheelset bodies, and IRW's contact interfaces those
// of their wheel bodies.
struct ElementReference {
    ElementKind kind{ElementKind::kRigidBody};
    std::string name;
};

// One node of the grouping tree.
//
// A group only organises. It renames nothing, carries no pose and has no
// effect on what a project generates: its members keep their names and their
// places in the records that declare them. Groups nest to any depth, an
// element belongs to at most one group, and an element no group names is
// simply ungrouped.
struct ElementGroup {
    std::string name;
    std::vector<ElementReference> members{};
    std::vector<ElementGroup> subgroups{};
};

struct VehicleProject {
    // The complete mechanical content, in the form the vehicle definition
    // loader produces. While `contact_record` is present, the wheel-pair
    // station reference list of its mechanical track station layout is not
    // consulted: generation takes that list from the contact carriers, which
    // are the one place it is edited.
    configuration::VehicleDefinition vehicle_definition;

    // Absent until the contact record has been provided. Absence says nothing
    // about whether the vehicle runs in wheel-rail contact.
    std::optional<configuration::VehicleContactRecord> contact_record{};

    // At most one per free body. A free body without one has not been placed
    // yet.
    std::vector<FreeBodyPlacement> free_body_placements{};

    // The top level of the grouping tree.
    std::vector<ElementGroup> element_groups{};
};

}  // namespace orvd::vehicle_project
