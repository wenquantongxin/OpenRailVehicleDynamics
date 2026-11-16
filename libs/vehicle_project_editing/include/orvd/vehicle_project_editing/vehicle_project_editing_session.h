#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/element_properties.h"
#include "orvd/vehicle_project_editing/mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

// The command layer a graphical modelling tool drives: one project being
// edited, the gravity it is checked under, its undo and redo history, a
// revision number, and the preview of the last check that passed.
//
// An interface sends commands and shows what comes back; nothing here draws,
// and nothing here knows a window. Whatever an interface can do through a
// session, a test can do without one.

namespace orvd::vehicle_project_editing {

// Why one property edit was refused: which element, which property, and what
// is wrong.
struct PropertyEditRefusal {
    vehicle_project::ElementReference element;
    std::string property_name;
    std::string reason;
};

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

    // Sets one property of one element (element_properties.h).
    //
    // The edit is checked here, in this order: the project contains the
    // element, the element's kind has the property, the property is editable,
    // the value holds the property's value kind, every number in the value is
    // finite, and, for a station offset or a placement offset, the record
    // holding it exists: a station offset needs the body's entry in the
    // mechanical track station layout, a placement offset an existing
    // placement. Physical plausibility, a negative stiffness or a non-positive
    // mass for instance, and the layout's own consistency, such as its
    // reference body keeping a zero station offset, are not judged here: that
    // is what checking is for.
    //
    // An accepted edit is one undo step: it empties the redo history and
    // starts a new revision. A refused edit changes neither the project, the
    // history nor the revision.
    [[nodiscard]] std::expected<void, PropertyEditRefusal> SetElementProperty(
        const vehicle_project::ElementReference& element, std::string_view property_name,
        const PropertyValue& value);

    // Returns to the project before the most recent step of the undo history
    // and moves that step to the redo history.
    //
    // Throws std::logic_error when there is no step to undo.
    void Undo();

    // Repeats the most recently undone step.
    //
    // Throws std::logic_error when there is no step to redo.
    void Redo();

    // Checks the current project (mechanical_check_report.h). When the check
    // passes, the preview of the current revision replaces the previous one;
    // when it does not, the previous preview stays, marked with the revision it
    // was computed from.
    [[nodiscard]] MechanicalCheckReport CheckMechanics();

    // The preview of the last check that passed; absent until one has.
    [[nodiscard]] const std::optional<VehicleProjectPreview>& preview() const { return preview_; }

  private:
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
