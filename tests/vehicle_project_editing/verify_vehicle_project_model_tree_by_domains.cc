#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <functional>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_model_tree_by_domains.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// The model tree by domains, on both bundled vehicles as the modelling
// service opens them: every element of the outline is one row, exactly once;
// each marker hangs under the body it names and each contact interface under
// the carrier it names; joints and force elements are each one list in
// outline order; the domains come in the contract's order, provided or not;
// keys are unique; each row offers what the contract says it creates, the
// joints row nothing; the joints row attaches the free bodies but the station
// layout's reference body; and no body's row has a note. The tree follows
// edits, keeping the keys of the rows an edit leaves alone; a body left
// unattached by a removed joint, and the bodies a joint joins to two free
// bodies, carry their attachment's note, and the joints row attaches the
// former; markers and interfaces whose owner the project does not have are
// listed under "Owner not found", and records repeating a kind and a name each
// have a row of their own; a project without a contact record has an empty
// wheel-rail domain with its note; and a project that fails its check has the
// same tree.

namespace {

using namespace orvd;
using editing_verification::Fail;
using editing_verification::PlacedProjectWithContact;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project_editing::ModelTreeCreationOffer;
using vehicle_project_editing::ModelTreeNode;
using vehicle_project_editing::ProjectSettings;
using vehicle_project_editing::PropertySubject;
using vehicle_project_editing::PropertyValue;
using vehicle_project_editing::ReferencedElementName;
using vehicle_project_editing::Title;
using vehicle_project_editing::VehicleProjectEditingSession;

// The joint and force element kinds, as the contract groups them, stated here
// independently of the library.
constexpr std::array kJointKinds{ElementKind::kRevoluteJoint, ElementKind::kBallRpyJoint,
                                 ElementKind::kWeldJoint};
constexpr std::array kForceElementKinds{
    ElementKind::kTranslationalSpringDamper,
    ElementKind::kRollSpringDamperCouple,
    ElementKind::kSeriesSpringViscousDamper,
    ElementKind::kPointToPointSeriesSpringBilinearDamper,
    ElementKind::kSaturatedPiecewiseLinearDamper,
    ElementKind::kReferenceFrameLinearBushing,
    ElementKind::kPointToPointLinearSpringDamper,
    ElementKind::kPointToPointHystereticSpring,
};

std::string Describe(const ElementReference& element) {
    return std::string(vehicle_project::DescribeElement(element));
}

// One row of the tree and the rows above it, the domain first.
struct VisitedRow {
    const ModelTreeNode* row;
    std::vector<const ModelTreeNode*> above;
};

void CollectRows(const std::vector<ModelTreeNode>& nodes, std::vector<const ModelTreeNode*>& above,
                 std::vector<VisitedRow>& rows) {
    for (const ModelTreeNode& node : nodes) {
        rows.push_back({&node, above});
        above.push_back(&node);
        CollectRows(node.children, above, rows);
        above.pop_back();
    }
}

// Every row, depth first.
std::vector<VisitedRow> RowsOf(const std::vector<ModelTreeNode>& tree) {
    std::vector<VisitedRow> rows;
    std::vector<const ModelTreeNode*> above;
    CollectRows(tree, above, rows);
    return rows;
}

const ElementReference* ElementOf(const ModelTreeNode& row) {
    return row.subject.has_value() ? std::get_if<ElementReference>(&*row.subject) : nullptr;
}

// The elements the rows select, in tree order.
std::vector<ElementReference> ElementsOf(const std::vector<ModelTreeNode>& rows) {
    std::vector<ElementReference> elements;
    for (const ModelTreeNode& row : rows) {
        if (const ElementReference* const element = ElementOf(row)) {
            elements.push_back(*element);
        }
    }
    return elements;
}

std::vector<ElementReference> ElementsInTree(const std::vector<ModelTreeNode>& tree) {
    std::vector<ElementReference> elements;
    for (const VisitedRow& visited : RowsOf(tree)) {
        if (const ElementReference* const element = ElementOf(*visited.row)) {
            elements.push_back(*element);
        }
    }
    return elements;
}

// The outline's elements: kind by kind in ElementKind's order, each kind in
// declaration order.
std::vector<ElementReference> OutlineElements(const VehicleProject& project,
                                              std::span<const ElementKind> kinds) {
    std::vector<ElementReference> elements;
    for (const ElementKind kind : kinds) {
        for (const std::string& name :
             vehicle_project_editing::CollectElementNames(project, kind)) {
            elements.push_back({kind, name});
        }
    }
    return elements;
}

bool SameElements(std::vector<ElementReference> first, std::vector<ElementReference> second) {
    const auto order = [](const ElementReference& left, const ElementReference& right) {
        return std::pair(left.kind, left.name) < std::pair(right.kind, right.name);
    };
    std::ranges::sort(first, order);
    std::ranges::sort(second, order);
    return first == second;
}

const ModelTreeNode& DomainTitled(const std::vector<ModelTreeNode>& tree, std::string_view title) {
    const auto domain = std::ranges::find_if(tree, [title](const ModelTreeNode& node) {
        return node.title.has_value() && node.title->en == title;
    });
    Require(domain != tree.end(), "the tree has no domain titled '" + std::string(title) + "'");
    return *domain;
}

const ModelTreeNode* ChildTitled(const ModelTreeNode& row, std::string_view title) {
    const auto child = std::ranges::find_if(row.children, [title](const ModelTreeNode& node) {
        return node.title.has_value() && node.title->en == title;
    });
    return child == row.children.end() ? nullptr : &*child;
}

// The row selecting `element`, or null.
const ModelTreeNode* RowOf(const std::vector<ModelTreeNode>& tree,
                           const ElementReference& element) {
    for (const VisitedRow& visited : RowsOf(tree)) {
        const ElementReference* const selected = ElementOf(*visited.row);
        if (selected != nullptr && *selected == element) {
            return visited.row;
        }
    }
    return nullptr;
}

// The markers listed under `body`'s row.
std::vector<ElementReference> MarkersOf(const std::vector<ModelTreeNode>& tree,
                                        const ElementReference& body) {
    const ModelTreeNode* const row = RowOf(tree, body);
    Require(row != nullptr, "the tree has no row for " + Describe(body));
    const ModelTreeNode* const markers = ChildTitled(*row, "Markers");
    Require(markers != nullptr, Describe(body) + " has no markers row");
    return ElementsOf(markers->children);
}

std::set<std::string> KeysOf(const std::vector<ModelTreeNode>& tree) {
    std::set<std::string> keys;
    for (const VisitedRow& visited : RowsOf(tree)) {
        keys.insert(visited.row->key);
    }
    return keys;
}

bool OffersExactly(const ModelTreeNode& row, std::span<const ElementKind> kinds) {
    return row.creates.size() == kinds.size() &&
           std::ranges::equal(row.creates, kinds, {}, &ModelTreeCreationOffer::kind) &&
           std::ranges::all_of(row.creates, [](const ModelTreeCreationOffer& offer) {
               return offer.fixed_properties.empty();
           });
}

// The row offers to create one element of `kind` with `property` fixed to
// name `owner`.
bool OffersOneOnOwner(const ModelTreeNode& row, ElementKind kind, const std::string& property,
                      const std::string& owner) {
    return row.creates.size() == 1 && row.creates.front().kind == kind &&
           row.creates.front().fixed_properties.size() == 1 &&
           row.creates.front().fixed_properties.contains(property) &&
           std::get<ReferencedElementName>(row.creates.front().fixed_properties.at(property)) ==
               ReferencedElementName{owner};
}

// The rigid bodies, in outline order, whose rows carry a note.
std::vector<ElementReference> BodiesWithNotes(const std::vector<ModelTreeNode>& tree) {
    std::vector<ElementReference> bodies;
    for (const ModelTreeNode& row : DomainTitled(tree, "Bodies").children) {
        const ElementReference* const body = ElementOf(row);
        if (body != nullptr && row.note.has_value()) {
            bodies.push_back(*body);
        }
    }
    return bodies;
}

// The rigid bodies stating that they move freely, in outline order, but the
// station layout's reference body, and those named in `also`.
std::vector<ElementReference> FreeBodiesButTheReferenceBody(const VehicleProject& project,
                                                            std::span<const std::string> also) {
    std::vector<ElementReference> bodies;
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        if ((body.moves_freely_in_world &&
             body.name !=
                 project.vehicle_definition.mechanical_track_station_layout.reference_body_name) ||
            std::ranges::find(also, body.name) != also.end()) {
            bodies.push_back({ElementKind::kRigidBody, body.name});
        }
    }
    return bodies;
}

std::optional<ProjectSettings> SettingsOf(const ModelTreeNode& row) {
    if (!row.subject.has_value() || !std::holds_alternative<ProjectSettings>(*row.subject)) {
        return std::nullopt;
    }
    return std::get<ProjectSettings>(*row.subject);
}

// Two trees with the same rows, keys, titles, subjects, notes, offers and
// bodies to attach.
bool SameTree(const std::vector<ModelTreeNode>& first, const std::vector<ModelTreeNode>& second) {
    return std::ranges::equal(
        first, second, [](const ModelTreeNode& left, const ModelTreeNode& right) {
            const auto same_offer = [](const ModelTreeCreationOffer& a,
                                       const ModelTreeCreationOffer& b) {
                return a.kind == b.kind &&
                       std::ranges::equal(a.fixed_properties, b.fixed_properties,
                                          [](const auto& x, const auto& y) {
                                              return x.first == y.first &&
                                                     std::get<ReferencedElementName>(x.second) ==
                                                         std::get<ReferencedElementName>(y.second);
                                          });
            };
            return left.key == right.key && left.title == right.title &&
                   left.subject == right.subject && left.provided == right.provided &&
                   left.note == right.note &&
                   std::ranges::equal(left.creates, right.creates, same_offer) &&
                   left.attaches == right.attaches && SameTree(left.children, right.children);
        });
}

void VerifyEveryElementOnceInItsPlace(const VehicleProject& project) {
    const std::string& vehicle_name = project.vehicle_definition.vehicle_name;
    const std::vector<ModelTreeNode> tree = vehicle_project_editing::DescribeModelTree(project);

    // The domains, in the contract's order and titles, provided or not.
    const std::vector<Title> domain_titles = {
        {.en = "Globals", .zh = "全局"},     {.en = "Bodies", .zh = "刚体"},
        {.en = "Joints", .zh = "铰"},        {.en = "Force elements", .zh = "力元"},
        {.en = "Constraints", .zh = "约束"}, {.en = "Wheel-rail", .zh = "轮轨"},
        {.en = "Track", .zh = "线路"},       {.en = "Substructures", .zh = "子结构"}};
    const std::vector<bool> domains_provided = {true, true, true, true, false, true, false, false};
    Require(tree.size() == domain_titles.size(), vehicle_name + ": the tree has not eight domains");
    for (std::size_t index = 0; index < tree.size(); ++index) {
        Require(tree[index].title == domain_titles[index] &&
                    tree[index].provided == domains_provided[index],
                vehicle_name + ": domain " + std::to_string(index) + " is not '" +
                    domain_titles[index].en + "', provided as the contract says");
    }
    // A row not provided yet selects nothing, offers nothing, holds nothing
    // and says what will be there.
    for (const VisitedRow& visited : RowsOf(tree)) {
        const ModelTreeNode& row = *visited.row;
        if (!row.provided) {
            Require(!row.subject.has_value() && row.creates.empty() && row.children.empty() &&
                        row.note.has_value() && !row.note->en.empty() && !row.note->zh.empty(),
                    vehicle_name + ": the row '" + row.title->en +
                        "', not provided yet, is not empty with its note");
        }
    }
    Require(KeysOf(tree).size() == RowsOf(tree).size(), vehicle_name + ": two rows share a key");

    // Every element exactly once.
    Require(SameElements(ElementsInTree(tree),
                         OutlineElements(project, vehicle_project::kElementKinds)),
            vehicle_name + ": the tree does not hold every element of the outline exactly once");

    // The settings rows.
    const ModelTreeNode& globals = DomainTitled(tree, "Globals");
    Require(globals.children.size() == 6 &&
                SettingsOf(globals.children[0]) == ProjectSettings::kGravity &&
                SettingsOf(globals.children[1]) == ProjectSettings::kVehicleIdentity &&
                SettingsOf(globals.children[2]) == ProjectSettings::kStationLayout &&
                !globals.children[3].provided && !globals.children[4].provided &&
                !globals.children[5].provided &&
                globals.children[5].title == Title{.en = "Simulation settings", .zh = "仿真设置"} &&
                globals.children[5].note ==
                    Title{.en = "Duration, solver, tolerances and parallel threads of a run",
                          .zh = "一次运行的时长、求解器、容差与并行线程数"},
            vehicle_name +
                ": the globals are not gravity, vehicle identity, station layout and "
                "three rows not provided yet, the simulation settings last");

    // Bodies with their markers; each marker under the body it names.
    const ModelTreeNode& bodies = DomainTitled(tree, "Bodies");
    Require(OffersExactly(bodies, std::array{ElementKind::kRigidBody}),
            vehicle_name + ": the bodies row does not offer exactly a rigid body");
    Require(ElementsOf(bodies.children) ==
                OutlineElements(project, std::array{ElementKind::kRigidBody}),
            vehicle_name + ": the bodies row does not list every rigid body in outline order");
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        const ElementReference body_element{ElementKind::kRigidBody, body.name};
        const ModelTreeNode& row = *RowOf(tree, body_element);
        const ModelTreeNode* const markers = ChildTitled(row, "Markers");
        Require(markers != nullptr && ChildTitled(row, "Geometry") != nullptr,
                vehicle_name + ": " + Describe(body_element) + " has no markers or geometry row");
        Require(OffersOneOnOwner(*markers, ElementKind::kFixedFrame, "body_name", body.name),
                vehicle_name + ": the markers of " + Describe(body_element) +
                    " do not offer a marker on it");
        std::vector<ElementReference> expected;
        for (const configuration::VehicleFixedFrameDefinition& frame :
             project.vehicle_definition.fixed_frames) {
            if (frame.body_name == body.name) {
                expected.push_back({ElementKind::kFixedFrame, frame.name});
            }
        }
        Require(ElementsOf(markers->children) == expected,
                vehicle_name + ": the markers of " + Describe(body_element) +
                    " are not the fixed frames naming it");
        Require(ChildTitled(row, "Geometry")->note.has_value(),
                vehicle_name + ": the geometry of " + Describe(body_element) + " has no note");
    }
    Require(ChildTitled(bodies, "Owner not found") == nullptr,
            vehicle_name + ": every marker's body exists, yet some are under 'Owner not found'");
    Require(BodiesWithNotes(tree).empty(),
            vehicle_name + ": a body that is free or hangs on a joint has a note on its row");

    // Joints and force elements, each one list in outline order. The joints
    // row creates nothing; it attaches the free bodies that may take a joint.
    const ModelTreeNode& joints = DomainTitled(tree, "Joints");
    Require(joints.creates.empty() &&
                ElementsOf(joints.children) == OutlineElements(project, kJointKinds) &&
                joints.children.size() == OutlineElements(project, kJointKinds).size(),
            vehicle_name + ": the joints are not one list in outline order creating nothing");
    Require(joints.attaches == FreeBodiesButTheReferenceBody(project, {}),
            vehicle_name +
                ": the joints row does not attach exactly the free bodies but the station "
                "layout's reference body, in outline order");
    const ModelTreeNode& force_elements = DomainTitled(tree, "Force elements");
    Require(
        OffersExactly(force_elements, kForceElementKinds) &&
            ElementsOf(force_elements.children) == OutlineElements(project, kForceElementKinds) &&
            force_elements.children.size() == OutlineElements(project, kForceElementKinds).size(),
        vehicle_name +
            ": the force elements are not one list in outline order offering "
            "every force element kind");

    // The wheel-rail settings and the carriers with their interfaces.
    const ModelTreeNode& wheel_rail = DomainTitled(tree, "Wheel-rail");
    Require(wheel_rail.children.size() == 4 &&
                SettingsOf(wheel_rail.children[0]) == ProjectSettings::kWheel &&
                SettingsOf(wheel_rail.children[1]) == ProjectSettings::kRail &&
                SettingsOf(wheel_rail.children[2]) == ProjectSettings::kContactComputation,
            vehicle_name +
                ": the wheel-rail domain does not begin with the wheel, the rail and "
                "the contact computation");
    const ModelTreeNode* const carriers = ChildTitled(wheel_rail, "Contact carriers");
    Require(carriers != nullptr && carriers->note.has_value() &&
                OffersExactly(*carriers, std::array{ElementKind::kContactCarrier}),
            vehicle_name +
                ": the contact carriers row is missing, has no note or does not offer "
                "a carrier");
    const configuration::VehicleContactDeclaration& declaration =
        project.contact_record->declaration;
    Require(ElementsOf(carriers->children) ==
                OutlineElements(project, std::array{ElementKind::kContactCarrier}),
            vehicle_name + ": the carriers row does not list every carrier in outline order");
    for (const ModelTreeNode& carrier_row : carriers->children) {
        const ElementReference& carrier = *ElementOf(carrier_row);
        Require(OffersOneOnOwner(carrier_row, ElementKind::kContactInterface, "carrier_name",
                                 carrier.name),
                vehicle_name + ": " + Describe(carrier) + " does not offer an interface on it");
        std::vector<ElementReference> expected;
        for (const configuration::VehicleContactInterfaceDeclaration& interface :
             declaration.interfaces) {
            if (interface.carrier_name == carrier.name) {
                expected.push_back({ElementKind::kContactInterface, interface.interface_name});
            }
        }
        Require(!expected.empty() && ElementsOf(carrier_row.children) == expected,
                vehicle_name + ": the interfaces under " + Describe(carrier) +
                    " are not those naming it");
    }

    // No element row but a carrier's creates anything, and no row but the
    // joints row attaches anything.
    for (const VisitedRow& visited : RowsOf(tree)) {
        const ElementReference* const element = ElementOf(*visited.row);
        if (element != nullptr && element->kind != ElementKind::kContactCarrier) {
            Require(visited.row->creates.empty() && !visited.row->title.has_value(),
                    vehicle_name + ": the row of " + Describe(*element) +
                        " has a title or offers to create");
        }
        Require(visited.row == &joints || visited.row->attaches.empty(),
                vehicle_name + ": a row other than the joints row attaches bodies");
    }
}

void VerifyTreeFollowsEdits(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const auto tree = [&session] {
        return vehicle_project_editing::DescribeModelTree(session.project());
    };

    // A frame pointed at another body moves under it, keeping its key, and no
    // key changes.
    const ElementReference carbody{ElementKind::kRigidBody, "carbody"};
    const ElementReference rear_bogie{ElementKind::kRigidBody, "rear_bogie_frame"};
    const std::vector<ElementReference> carbody_markers = MarkersOf(tree(), carbody);
    Require(!carbody_markers.empty(), "GZ18's carbody has no markers");
    const ElementReference moved = carbody_markers.front();
    const std::vector<ModelTreeNode> before_move = tree();
    Require(session.SetSubjectProperty(moved, "body_name", ReferencedElementName{rear_bogie.name})
                .has_value(),
            "pointing a marker at another body was refused");
    const std::vector<ModelTreeNode> after_move = tree();
    const std::vector<ElementReference> rear_markers = MarkersOf(after_move, rear_bogie);
    const std::vector<ElementReference> carbody_markers_after = MarkersOf(after_move, carbody);
    Require(std::ranges::find(rear_markers, moved) != rear_markers.end() &&
                std::ranges::find(carbody_markers_after, moved) == carbody_markers_after.end(),
            "a marker pointed at another body did not move under it");
    Require(KeysOf(before_move) == KeysOf(after_move) &&
                RowOf(before_move, moved)->key == RowOf(after_move, moved)->key,
            "moving a marker changed the keys of the tree's rows");

    // A renamed body keeps its markers under it; the rows it does not hold
    // keep their keys.
    const ElementReference front_bogie{ElementKind::kRigidBody, "front_bogie_frame"};
    const std::vector<ElementReference> front_markers = MarkersOf(after_move, front_bogie);
    Require(!front_markers.empty(), "GZ18's front bogie frame has no markers");
    const auto renamed = session.SetSubjectProperty(
        front_bogie, "name", vehicle_project_editing::ElementOwnName{"front_bogie_frame_renamed"});
    Require(renamed.has_value(), "renaming the front bogie frame was refused");
    const ElementReference renamed_body = std::get<ElementReference>(*renamed);
    const std::vector<ModelTreeNode> after_rename = tree();
    Require(RowOf(after_rename, front_bogie) == nullptr &&
                MarkersOf(after_rename, renamed_body) == front_markers,
            "a renamed body did not keep its markers under it");
    // Only the body's own row and the rows it holds directly, its markers and
    // geometry rows, may change their keys; its markers keep theirs.
    const std::set<std::string> renamed_keys = KeysOf(after_rename);
    const auto is_front_bogie = [&front_bogie](const ModelTreeNode& row) {
        return ElementOf(row) != nullptr && *ElementOf(row) == front_bogie;
    };
    for (const VisitedRow& visited : RowsOf(after_move)) {
        if (!is_front_bogie(*visited.row) &&
            (visited.above.empty() || !is_front_bogie(*visited.above.back()))) {
            Require(renamed_keys.contains(visited.row->key),
                    "renaming a body changed the key of a row it does not hold");
        }
    }

    // A created force element takes its place in the one list.
    const ElementReference spring{ElementKind::kTranslationalSpringDamper, "tree_probe_spring"};
    Require(session
                .AddElement(spring.kind, spring.name,
                            {{"reference_frame_name", ReferencedElementName{"carbody"}},
                             {"opposite_frame_name", ReferencedElementName{rear_bogie.name}}})
                .has_value(),
            "creating a force element was refused");
    const std::vector<ModelTreeNode> after_creation = tree();
    Require(ElementsOf(DomainTitled(after_creation, "Force elements").children) ==
                    OutlineElements(session.project(), kForceElementKinds) &&
                RowOf(after_creation, spring) != nullptr,
            "a created force element is not in the one list of force elements");

    // A removed body takes the rows of its markers with it.
    const auto removed = session.RemoveElement(rear_bogie);
    Require(removed.has_value(), "removing the rear bogie frame was refused");
    const std::vector<ModelTreeNode> after_removal = tree();
    for (const ElementReference& element : *removed) {
        Require(RowOf(after_removal, element) == nullptr,
                Describe(element) + " was removed but still has a row");
    }
    Require(std::ranges::find(*removed, moved) != removed->end() &&
                SameElements(ElementsInTree(after_removal),
                             OutlineElements(session.project(), vehicle_project::kElementKinds)),
            "removing a body did not take its markers' rows with it");
}

// A body's row carries its attachment's note while the body is unattached or
// conflicting, and only then.
void VerifyBodyRowsCarryAttachmentNotes(const ReferenceVehicle& gz18) {
    // Removing the pivot an axlebox carrier hangs on leaves the carrier
    // unattached: its row says so, and the joints row attaches it.
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const std::string carrier = "front_leading_right_axlebox_carrier";
    Require(session.RemoveElement({ElementKind::kRevoluteJoint, carrier + "_pivot"}).has_value(),
            "removing the pivot of an axlebox carrier was refused");
    const std::vector<ModelTreeNode> unattached =
        vehicle_project_editing::DescribeModelTree(session.project());
    Require(BodiesWithNotes(unattached) ==
                    std::vector<ElementReference>{{ElementKind::kRigidBody, carrier}} &&
                RowOf(unattached, {ElementKind::kRigidBody, carrier})->note ==
                    Title{.en = "Neither free nor held by a joint",
                          .zh = "既没有声明自由，也没有铰把它接到别的刚体上"},
            "the row of a body left without its joint does not alone carry the note of an "
            "unattached body");
    const std::array also_attached{carrier};
    Require(DomainTitled(unattached, "Joints").attaches ==
                FreeBodiesButTheReferenceBody(session.project(), also_attached),
            "the joints row does not attach a body left without its joint");

    // A weld joining the car body to a free bogie frame joins the bogie
    // frame, the car body and the bodies welded to it to two free bodies:
    // their rows say so, and the joints row attaches neither free body.
    VehicleProject joined = PlacedProjectWithContact(gz18);
    joined.vehicle_definition.weld_joints.push_back({.name = "carbody_to_front_bogie_frame",
                                                     .parent_frame_name = "carbody",
                                                     .child_frame_name = "front_bogie_frame"});
    const std::vector<ModelTreeNode> conflicting =
        vehicle_project_editing::DescribeModelTree(joined);
    const std::vector<ElementReference> joined_bodies = {
        {ElementKind::kRigidBody, "carbody"},
        {ElementKind::kRigidBody, "front_bogie_frame"},
        {ElementKind::kRigidBody, "front_carbody_interface_body"},
        {ElementKind::kRigidBody, "rear_carbody_interface_body"}};
    Require(BodiesWithNotes(conflicting) == joined_bodies,
            "the rows carrying a note are not those of the bodies joined to two free bodies");
    for (const ElementReference& body : joined_bodies) {
        Require(RowOf(conflicting, body)->note ==
                    Title{.en = "It is joined through joints to more than one free body; remove "
                                "one of the joints between them",
                          .zh = "它经由铰连到了不止一个自由刚体，请删除其间的一个铰"},
                "the row of " + Describe(body) +
                    ", joined to two free bodies, does not carry that note");
    }
    std::vector<ElementReference> attachable = FreeBodiesButTheReferenceBody(joined, {});
    std::erase(attachable, joined_bodies[1]);
    Require(DomainTitled(conflicting, "Joints").attaches == attachable,
            "the joints row attaches a body joined to two free bodies, or leaves out another "
            "free body");
}

void VerifyRecordsAProjectFileCanHold(const ReferenceVehicle& gz18) {
    const VehicleProject opened = PlacedProjectWithContact(gz18);

    // A marker naming no body of the project and an interface naming no
    // carrier of it are listed under "Owner not found" in their domains.
    VehicleProject orphaned = opened;
    configuration::VehicleFixedFrameDefinition orphan_marker;
    orphan_marker.name = "marker_without_body";
    orphan_marker.body_name = "body_not_in_project";
    orphaned.vehicle_definition.fixed_frames.push_back(orphan_marker);
    configuration::VehicleContactInterfaceDeclaration orphan_contact;
    orphan_contact.interface_name = "contact_without_carrier";
    orphan_contact.carrier_name = "carrier_not_in_project";
    orphan_contact.wheel_body_name = "carbody";
    orphaned.contact_record->declaration.interfaces.push_back(orphan_contact);
    const std::vector<ModelTreeNode> tree = vehicle_project_editing::DescribeModelTree(orphaned);
    const ModelTreeNode* const markers_not_found =
        ChildTitled(DomainTitled(tree, "Bodies"), "Owner not found");
    const ModelTreeNode* const contacts_not_found = ChildTitled(
        *ChildTitled(DomainTitled(tree, "Wheel-rail"), "Contact carriers"), "Owner not found");
    Require(markers_not_found != nullptr &&
                ElementsOf(markers_not_found->children) ==
                    std::vector<ElementReference>{{ElementKind::kFixedFrame, orphan_marker.name}},
            "a marker naming no body of the project is not under 'Owner not found'");
    Require(contacts_not_found != nullptr &&
                ElementsOf(contacts_not_found->children) ==
                    std::vector<ElementReference>{
                        {ElementKind::kContactInterface, orphan_contact.interface_name}},
            "an interface naming no carrier of the project is not under 'Owner not found'");
    Require(SameElements(ElementsInTree(tree),
                         OutlineElements(orphaned, vehicle_project::kElementKinds)),
            "a project with records whose owner is missing does not have every element once");

    // Two records of one kind under one name each have a row, under keys of
    // their own.
    VehicleProject repeated = opened;
    repeated.vehicle_definition.rigid_bodies.push_back(
        repeated.vehicle_definition.rigid_bodies.front());
    const std::vector<ModelTreeNode> repeated_tree =
        vehicle_project_editing::DescribeModelTree(repeated);
    Require(KeysOf(repeated_tree).size() == RowsOf(repeated_tree).size() &&
                SameElements(ElementsInTree(repeated_tree),
                             OutlineElements(repeated, vehicle_project::kElementKinds)),
            "a project with a body's record repeated has rows sharing a key or missing");

    // Without a contact record the wheel-rail domain has no rows, and a note.
    VehicleProject without_contact = opened;
    without_contact.contact_record.reset();
    const std::vector<ModelTreeNode> tree_without_contact =
        vehicle_project_editing::DescribeModelTree(without_contact);
    const ModelTreeNode& wheel_rail = DomainTitled(tree_without_contact, "Wheel-rail");
    Require(wheel_rail.children.empty() && wheel_rail.creates.empty() &&
                wheel_rail.note.has_value() &&
                wheel_rail.note->en.find("contact record") != std::string::npos,
            "the wheel-rail domain of a project without a contact record has rows or no note");

    // A project failing its check has the same tree.
    VehicleProject failing = opened;
    failing.vehicle_definition.rigid_bodies.front().mass_kilograms = 0.0;
    VehicleProjectEditingSession failing_session(failing);
    Require(!failing_session.CheckMechanics().refusals.empty(),
            "a body of zero mass did not fail the check");
    Require(SameTree(vehicle_project_editing::DescribeModelTree(opened),
                     vehicle_project_editing::DescribeModelTree(failing)),
            "the tree of a project failing its check differs");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_model_tree_by_domains GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const ReferenceVehicle irw =
            editing_verification::LoadReferenceVehicle(argv[4], argv[5], argv[6]);
        VerifyEveryElementOnceInItsPlace(PlacedProjectWithContact(gz18));
        VerifyEveryElementOnceInItsPlace(PlacedProjectWithContact(irw));
        VerifyTreeFollowsEdits(gz18);
        VerifyBodyRowsCarryAttachmentNotes(gz18);
        VerifyRecordsAProjectFileCanHold(gz18);
        std::puts("vehicle project model tree by domains passed for GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project model tree by domains: %s\n", error.what());
        return 1;
    }
}
