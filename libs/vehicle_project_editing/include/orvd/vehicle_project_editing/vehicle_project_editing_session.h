#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

// The command layer a graphical modelling tool drives over one vehicle
// project: the project being edited, the project file it is associated with if
// any, its undo and redo history, a revision number, whether it has unsaved
// changes, and the preview of the last check that passed.
//
// An interface sends commands and shows what comes back; nothing here draws,
// and nothing here knows a window. Whatever an interface can do through a
// session, a test can do without one, saving included.
//
// An edit sets a property of an element or of the project's settings, changes
// how a rigid body is attached, creates an element or removes one. Each is
// made on a copy of the project, which becomes the project only when the
// whole edit is accepted: one undo step and one new revision. A refused edit
// changes neither the project, the history nor the revision.
//
// An edit introduces no reference to an element the project does not contain,
// but it does not judge the references the project already holds: a project
// that opens with problems is repaired one edit at a time, and checking keeps
// reporting what is still wrong. Physical plausibility, a negative stiffness
// or a non-positive mass for instance, and the station layout's own
// consistency, such as every free body having one station offset, are judged
// by checking as well.
//
// Saving writes the project, whatever state it is in, to a project file
// (vehicle_project_file_reading_and_writing.h). A session opened from a
// vehicle's own files has no file until it is saved as a named project; one
// opened from a project file is associated with that file. Saving as a named
// project gives the project that name as its vehicle name and mechanical
// definition identifier. Those two names are the project's identity rather
// than an edit: every project of the undo and redo history carries the
// current one, so no undo or redo brings an earlier identity back. The session
// knows whether its project differs from the state last saved or, before any
// save, from the state it was opened with, through undo and redo: undoing back
// to that state leaves no unsaved changes, and an edit made after undoing past
// it makes it unreachable, so the changes stay unsaved until the next save.

namespace orvd::vehicle_project_editing {

class VehicleProjectEditingSession {
  public:
    // A session over a project opened from a vehicle's own files, which has no
    // project file yet. Starts at revision 0, with an empty history, no
    // preview and no unsaved changes.
    explicit VehicleProjectEditingSession(vehicle_project::VehicleProject project);

    // A session over a project read from the project file at `associated_file`,
    // with which it stays associated. Starts as the other constructor does.
    VehicleProjectEditingSession(vehicle_project::VehicleProject project,
                                 vehicle_project::VehicleProjectFileLocation associated_file);

    [[nodiscard]] const vehicle_project::VehicleProject& project() const { return project_; }

    // The project file the session was opened from or last saved as; absent
    // for a project opened from a vehicle's own files and not saved yet.
    [[nodiscard]] const std::optional<vehicle_project::VehicleProjectFileLocation>&
    associated_file() const {
        return associated_file_;
    }

    // Whether the project differs from the state last saved or, before any
    // save, from the state the session was opened with.
    [[nodiscard]] bool has_unsaved_changes() const {
        return saved_state_position_ != undo_snapshots_.size();
    }

    // Increases by one with every accepted edit, undo and redo and with every
    // Save As that changes the project's names, and never decreases: undoing
    // restores an earlier project, not its revision.
    [[nodiscard]] std::uint64_t revision() const { return revision_; }

    [[nodiscard]] bool can_undo() const { return !undo_snapshots_.empty(); }
    [[nodiscard]] bool can_redo() const { return !redo_snapshots_.empty(); }

    // Sets one property of one subject, an element or settings
    // (vehicle_project_subject_properties.h), and returns the subject as it is
    // named afterwards: an element renamed under its new name.
    //
    // The edit is checked in this order: the project contains the element, or
    // holds the record of the settings, which the wheel, the rail and the
    // contact computation lack while the project has no contact record; the
    // subject has the property, the property is editable, the value holds the
    // property's value kind, and then by what the property holds:
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
    // layout entry and the layout's wheel-pair station list, its placement and
    // the name its shapes are stated under, which keeps naming the shapes it
    // named. Whether a rigid body moves freely is
    // no property: SetRigidBodyAttachment changes it. Settings are judged no
    // further: the check judges the gravity, and a run the wheel and rail
    // values it assembles.
    [[nodiscard]] std::expected<PropertySubject, SubjectEditRefusal> SetSubjectProperty(
        const PropertySubject& subject, std::string_view property_name, const PropertyValue& value);

    // Appends a new element of `kind` named `name` to its kind's records and
    // returns it. The element is its record type's default-constructed value
    // with that name and the given properties, which are set as
    // SetSubjectProperty sets them, in the order DescribeElementProperties
    // lists them.
    //
    // Refused for a rigid body, which AddRigidBody creates with its
    // attachment, and for a joint kind, a joint being created only as a rigid
    // body's attachment. Refused, naming the property at fault where one is,
    // when the kind belongs to a contact record the project does not have,
    // when the name is refused as a rename's would be, when a property given
    // is not one of the kind's or is the element's own name, when a reference
    // that cannot name none is not given, or when one given value is refused.
    // A property that is not editable cannot be given either.
    [[nodiscard]] std::expected<vehicle_project::ElementReference, SubjectEditRefusal> AddElement(
        vehicle_project::ElementKind kind, const std::string& name,
        const PropertyValuesByName& properties);

    // Appends a new rigid body named `name` with the given properties,
    // attached as `attachment` states: free in the world, or by a new joint
    // whose end on the body, its child_frame_name, names the new body and is
    // therefore not given. One edit, the joint included. The properties only a
    // free body has, its station offset and placement offsets, are set once it
    // is attached, so that a body attached by a joint has none to set.
    //
    // Refused as AddElement refuses an element's name and properties, as
    // SetRigidBodyAttachment refuses taking the joint, and when the joint's
    // properties give its end on the body.
    [[nodiscard]] std::expected<vehicle_project::ElementReference, SubjectEditRefusal> AddRigidBody(
        const std::string& name, const PropertyValuesByName& properties,
        const RigidBodyAttachmentChoice& attachment);

    // Changes how the rigid body named `body_name` is attached
    // (vehicle_project_rigid_body_attachments.h) and returns the joint it hangs
    // on afterwards, none when it moves freely. One edit.
    //
    // Becoming free: the body is stated to move freely, and gets a station
    // layout entry and a placement where it has none, at zero offsets and the
    // identity rotation; the joint it hung on is removed. Refused for a body
    // already free and for a conflicting one, and, naming them, while other
    // elements name the joint it hangs on: such a joint goes, with what names
    // it, only by RemoveElement.
    //
    // Taking a joint: a joint of the attachment's kind is created with its
    // name and properties, checked as AddElement checks an element's; its
    // child_frame_name names the body or one of its fixed frames, and its
    // parent_frame_name a frame on a body the body does not reach through
    // joints once the joint it hangs on is taken away. A free body stops
    // moving freely and loses its station layout entry and placement; a body
    // hanging on a joint has that joint removed first, so that the new one may
    // take its name, and what named it names the new one. Refused for a
    // conflicting body and for a kind that is no joint kind; naming them, when
    // an element naming the replaced joint cannot name the new one's kind; and,
    // with the joint as the refusal's subject, for the joint's name and
    // properties and for an end that is not where it must be.
    [[nodiscard]] std::expected<std::optional<vehicle_project::ElementReference>,
                                SubjectEditRefusal>
    SetRigidBodyAttachment(const std::string& body_name,
                           const RigidBodyAttachmentChoice& attachment);

    // Removes `element`, every element holding a reference to an element
    // removed, transitively, and the records following each of them, keeping
    // their groups. Returns everything removed, `element` included, in outline
    // order: kind by kind in ElementKind's order, each kind in declaration
    // order.
    //
    // Refused when the project does not contain the element.
    [[nodiscard]] std::expected<std::vector<vehicle_project::ElementReference>, SubjectEditRefusal>
    RemoveElement(const vehicle_project::ElementReference& element);

    // What RemoveElement would remove, or why it would be refused. Changes
    // nothing.
    [[nodiscard]] std::expected<std::vector<vehicle_project::ElementReference>, SubjectEditRefusal>
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

    // Writes the project to the file the session is associated with, and
    // marks the current state saved. Changes nothing in the project, its
    // history or its revision.
    //
    // Throws std::logic_error when the session has no file, and what writing
    // throws when the file cannot be written (WriteVehicleProjectFile); either
    // way nothing changes.
    void SaveVehicleProject();

    // Saves the project as the project named `file.name` in `file.directory`:
    // the project is written with the vehicle definition's vehicle_name and
    // mechanical_definition_identifier set to that name, replacing a project
    // file of that name, and only once it has been written do the two names
    // become those of the current project and of every project in the undo
    // and redo history. No step is added, so can_undo and can_redo stay as
    // they were; the revision increases by one when the names change. The
    // session is then associated with the file and its state is saved.
    // Nothing else is renamed or rewritten.
    //
    // Throws what writing throws (WriteVehicleProjectFile), an unacceptable
    // name included, leaving the project, its names, the history, the
    // revision and the association as they were.
    void SaveVehicleProjectAs(const vehicle_project::VehicleProjectFileLocation& file);

    // Checks the current project (vehicle_project_mechanical_check_report.h).
    // When the check passes, the preview of the current revision replaces the
    // previous one; when it does not, the previous preview stays, marked with
    // the revision it was computed from.
    [[nodiscard]] MechanicalCheckReport CheckMechanics();

    // The preview of the last check that passed; absent until one has.
    [[nodiscard]] const std::optional<VehicleProjectPreview>& preview() const { return preview_; }

  private:
    // Makes an accepted edit's project the current one: one undo step, an
    // empty redo history and a new revision.
    void AdoptEdit(vehicle_project::VehicleProject edited);

    vehicle_project::VehicleProject project_;
    std::optional<vehicle_project::VehicleProjectFileLocation> associated_file_;
    // Whole-project snapshots, the most recent last: the project before each
    // step that can be undone, and the project after each step that can be
    // redone.
    std::vector<vehicle_project::VehicleProject> undo_snapshots_;
    std::vector<vehicle_project::VehicleProject> redo_snapshots_;
    // Where in the history the saved state is: the number of undo snapshots
    // while it is the current project. Absent once an edit has discarded the
    // redo history it was in.
    std::optional<std::size_t> saved_state_position_{0};
    std::uint64_t revision_{0};
    std::optional<VehicleProjectPreview> preview_;
};

}  // namespace orvd::vehicle_project_editing
