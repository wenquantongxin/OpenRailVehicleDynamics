#pragma once

#include <optional>
#include <string>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "orvd/vehicle_project_editing/vehicle_project_titles_in_english_and_chinese.h"

// The model tree an interface shows a project as: the project's records
// arranged in domains, as SIMPACK's model tree arranges elements in
// categories, so that the interface draws it without knowing a domain, an
// element kind or a settings word.
//
// The tree is made from the records alone, whatever a check would say of
// them, and every element of the outline is one row of it, exactly once. Its
// first level is the domains, in this order:
// - Globals: the gravity and the vehicle identity, which are settings; the
//   vehicle layout zero, which the free bodies' station offsets are measured
//   from and which no record names, a row selecting nothing; initial
//   conditions, views and simulation settings, not provided yet.
// - Bodies: every rigid body, with its markers, the fixed frames naming it,
//   and its geometry, a note on the name its shapes are stated under. An
//   unattached or conflicting body's row carries its attachment's note
//   (vehicle_project_rigid_body_attachments.h). Fixed frames naming a body the
//   project does not have are listed under "Owner not found", where they can
//   be seen and repaired.
// - Joints and force elements: each one list, in outline order. A kind is a
//   tag of the rows of such a list, not a level of it, and a kind without
//   elements has no row. The joints row creates no joint on its own: it
//   attaches one of the bodies it lists, the free bodies that can take a joint
//   and the unattached ones.
// - Constraints: not provided yet.
// - Wheel-rail: the wheel, the rail and the contact computation, which are
//   settings, and the contact carriers, each with the contact interfaces
//   naming it; interfaces naming a carrier the project does not have are
//   listed under "Owner not found". Without a contact record the domain has no
//   rows and a note saying so.
// - Track and substructures: not provided yet.
// A row's offers say what its "+" creates and with which properties fixed: a
// marker on its body, a contact interface on its carrier. A row with neither
// offers nor bodies to attach has no "+".
//
// A row keeps its key across edits; an element row's key follows its
// element's name, and the rows below an element row follow it.

namespace orvd::vehicle_project_editing {

// One element a row creates: its kind, and the properties it is created with
// that the creation cannot change.
struct ModelTreeCreationOffer {
    vehicle_project::ElementKind kind{vehicle_project::ElementKind::kRigidBody};
    PropertyValuesByName fixed_properties{};
};

struct ModelTreeNode {
    // Unique in the tree.
    std::string key;
    // Absent for an element row, which shows its element's name.
    std::optional<Title> title{};
    // What selecting the row selects; absent for a row that only groups or
    // only says what it is.
    std::optional<PropertySubject> subject{};
    // False for what the product does not provide yet: such a row has no
    // subject, no offers and no children, and its note says what will be
    // there.
    bool provided{true};
    // One line shown with the row.
    std::optional<Title> note{};
    std::vector<ModelTreeCreationOffer> creates{};
    // The rigid bodies the row's "+" attaches, in outline order: empty but for
    // the joints row.
    std::vector<vehicle_project::ElementReference> attaches{};
    std::vector<ModelTreeNode> children{};
};

// The domains of the project's model tree, in the order above.
[[nodiscard]] std::vector<ModelTreeNode> DescribeModelTree(
    const vehicle_project::VehicleProject& project);

// The name people know `kind` by: a row's tag in a list of several kinds, and
// a choice among the kinds a row creates.
[[nodiscard]] Title ElementKindTitle(vehicle_project::ElementKind kind);

}  // namespace orvd::vehicle_project_editing
