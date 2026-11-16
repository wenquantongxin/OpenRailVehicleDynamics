#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_element_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_element_properties.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

// The command layer a graphical modelling tool drives: one project being
// edited, the gravity it is checked under, its undo and redo history, a
// revision number, and the preview of the last check that passed.
//
// An interface sends commands and shows what comes back; nothing here draws,
// and nothing here knows a window. Whatever an interface can do through a
// session, a test can do without one.
//
// An edit sets a property, creates an element or removes one. Each is made on
// a copy of the project, which becomes the project only when the whole edit
// is accepted: one undo step and one new revision. A refused edit changes
// neither the project, the history nor the revision.
//
// An edit introduces no reference to an element the project does not contain,
// but it does not judge the references the project already holds: a project
// that opens with problems is repaired one edit at a time, and checking keeps
// reporting what is still wrong. Physical plausibility, a negative stiffness
// or a non-positive mass for instance, and the station layout's own
// consistency, such as its reference body keeping a zero station offset, are
// judged by checking as well.

namespace orvd::vehicle_project_editing {

class VehicleProjectEditingSession {
  public:
    // Starts at revision 0, with an empty history and no preview.
    //
    // The gravitational acceleration magnitude is the caller's to state, for
    // the reason configuration::AssembleVehicleSystem gives; checking passes it
    // to that assembly unchanged, which is also where it is judged.
    VehicleProjectEditingSession(
        vehicle_project::VehicleProject project,
        double gravitational_acceleration_magnitude_meters_per_second_squared);

    [[nodiscard]] const vehicle_project::VehicleProject& project() const { return project_; }

    // Increases by one with every accepted edit, undo and redo, and never
    // decreases: undoing restores an earlier project, not its revision.
    [[nodiscard]] std::uint64_t revision() const { return revision_; }

    [[nodiscard]] bool can_undo() const { return !undo_snapshots_.empty(); }
    [[nodiscard]] bool can_redo() const { return !redo_snapshots_.empty(); }

    // Sets one property of one element (vehicle_project_element_properties.h) and returns the
    // element as it is named afterwards.
    //
    // The edit is checked in this order: the project contains the element,
    // the element's kind has the property, the property is editable, the value
    // holds the property's value kind, and then by what the property holds:
    // - a number, a 3-vector or a curve has only finite numbers;
    // - a station offset or a placement offset needs the record holding it,
    //   the body's entry in the mechanical track station layout or its
    //   placement;
    // - a reference names an element of a kind it may name, or names none
    //   where it may;
    // - a token is one of its choices;
    // - a new name is not empty, is not `world` for a rigid body or a fixed
    //   frame, and is held by no other element of the element's name space,
    //   which rigid bodies and fixed frames share, as the three joint kinds and
    //   the eight force element kinds do.
    // Renaming changes every reference to the element and every record that
    // follows it: its group memberships and, for a rigid body, its station
    // layout entry, the layout's reference body name and wheel-pair station
    // list, its placement and the name its shapes are stated under, which
    // keeps naming the shapes it named. Making a rigid body move freely gives
    // it a station layout entry and a placement where it has none, at zero
    // offsets and the identity rotation; making it no longer move freely takes
    // both away and is refused for the station layout's reference body.
    [[nodiscard]] std::expected<vehicle_project::ElementReference, ElementEditRefusal>
    SetElementProperty(const vehicle_project::ElementReference& element,
                       std::string_view property_name, const PropertyValue& value);

    // Appends a new element of `kind` named `name` to its kind's records and
    // returns it. The element is its record type's default-constructed value
    // with that name and the given properties, which are set as
    // SetElementProperty sets them, in the order DescribeElementProperties
    // lists them; a free body thus gets its station layout entry and placement
    // before their offsets are set.
    //
    // Refused, naming the property at fault where one is, when the kind
    // belongs to a contact record the project does not have, when the name is
    // refused as a rename's would be, when a property given is not one of the
    // kind's or is the element's own name, when a reference that cannot name
    // none is not given, or when one given value is refused. A property that
    // is not editable cannot be given either.
    [[nodiscard]] std::expected<vehicle_project::ElementReference, ElementEditRefusal> AddElement(
        vehicle_project::ElementKind kind, const std::string& name,
        const PropertyValuesByName& properties);

    // Removes `element`, every element holding a reference to an element
    // removed, transitively, and the records following each of them, keeping
    // their groups. Returns everything removed, `element` included, in outline
    // order: kind by kind in ElementKind's order, each kind in declaration
    // order.
    //
    // Refused when the project does not contain the element, and for the
    // station layout's reference body.
    [[nodiscard]] std::expected<std::vector<vehicle_project::ElementReference>, ElementEditRefusal>
    RemoveElement(const vehicle_project::ElementReference& element);

    // What RemoveElement would remove, or why it would be refused. Changes
    // nothing.
    [[nodiscard]] std::expected<std::vector<vehicle_project::ElementReference>, ElementEditRefusal>
    CollectRemovalConsequences(const vehicle_project::ElementReference& element) const;

    // Returns to the project before the most recent step of the undo history
    // and moves that step to the redo history.
    //
    // Throws std::logic_error when there is no step to undo.
    void Undo();

    // Repeats the most recently undone step.
    //
    // Throws std::logic_error when there is no step to redo.
    void Redo();

    // Checks the current project (vehicle_project_mechanical_check_report.h). When the check
    // passes, the preview of the current revision replaces the previous one;
    // when it does not, the previous preview stays, marked with the revision it
    // was computed from.
    [[nodiscard]] MechanicalCheckReport CheckMechanics();

    // The preview of the last check that passed; absent until one has.
    [[nodiscard]] const std::optional<VehicleProjectPreview>& preview() const { return preview_; }

  private:
    // Makes an accepted edit's project the current one: one undo step, an
    // empty redo history and a new revision.
    void AdoptEdit(vehicle_project::VehicleProject edited);

    vehicle_project::VehicleProject project_;
    double gravitational_acceleration_magnitude_meters_per_second_squared_;
    // Whole-project snapshots, the most recent last: the project before each
    // step that can be undone, and the project after each step that can be
    // redone.
    std::vector<vehicle_project::VehicleProject> undo_snapshots_;
    std::vector<vehicle_project::VehicleProject> redo_snapshots_;
    std::uint64_t revision_{0};
    std::optional<VehicleProjectPreview> preview_;
};

}  // namespace orvd::vehicle_project_editing
