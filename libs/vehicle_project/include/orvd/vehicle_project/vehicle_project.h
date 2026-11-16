#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"

// The document a graphical modelling tool edits for one rail vehicle, and
// saves as one project file (vehicle_project_file_reading_and_writing.h).
//
// A project holds:
// - the vehicle's mechanical definition and, once provided, its wheel-rail
//   contact record, which are what the solver side receives, through
//   generate_assembly_inputs_from_vehicle_project.h;
// - the magnitude of the gravitational acceleration the vehicle is checked
//   under, once someone has stated it;
// - where each free body is drawn, the name each rigid body's shapes are
//   stated under, and how the elements are organised into groups;
// - the visual definition its bodies are drawn with, a document of the
//   drawing's own that this library keeps as text and does not interpret.
// Placements, shape names, groups and the visual definition exist only for the
// person editing. None of them reaches the solver side, so redrawing or
// reorganising a vehicle cannot change a number the dynamics sees.
//
// Like the vehicle definition it carries, this is deliberately a mutable value
// record: a caller may build any part of it directly and edit it in place.
// Nothing here keeps a reference to a project, and nothing generated from one
// refers back to it. The parts this library owns are checked when the caller
// asks for it (vehicle_project_invariants.h), not on every assignment, and
// neither writing nor reading a project file checks them: an unfinished
// project is saved and reopened as it is.

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

// The name one rigid body's shapes are stated under. A visual definition,
// which only a drawing reads, states the shapes of a vehicle's bodies under
// the bodies' names in the vehicle definition it was made for. This record
// keeps that name for a body, so that the shapes stay with the body when it
// is renamed.
struct VisualDefinitionBodyName {
    std::string body_name;
    std::string visual_definition_body_name;
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

    friend bool operator==(const ElementReference&, const ElementReference&) = default;
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

    // The magnitude of the gravitational acceleration the vehicle is checked
    // under. Absent until someone states it, as a start-up state or a project
    // file does; checking passes it to the vehicle assembly, which is where it
    // is judged.
    std::optional<double> gravitational_acceleration_meters_per_second_squared{};

    // At most one per free body. A free body without one has not been placed
    // yet.
    std::vector<FreeBodyPlacement> free_body_placements{};

    // At most one per rigid body. A project made from a vehicle definition
    // (make_vehicle_project_from_vehicle_definition.h) gives every body its
    // own name; renaming the body keeps the name its shapes are stated under,
    // and a body without one, such as a body created later, has no shapes.
    std::vector<VisualDefinitionBodyName> visual_definition_body_names{};

    // The top level of the grouping tree.
    std::vector<ElementGroup> element_groups{};

    // The visual definition the bodies are drawn with: the text of a JSON
    // object a drawing reads, which this library carries and does not
    // interpret (read_visual_definition_document_from_file.h). Empty when the
    // project has none.
    std::string visual_definition_document{};
};

}  // namespace orvd::vehicle_project
