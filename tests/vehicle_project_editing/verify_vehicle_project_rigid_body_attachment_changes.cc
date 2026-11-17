#include <Eigen/Core>
#include <algorithm>
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "vehicle_project/vehicle_project_comparison.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// Changing how a rigid body is attached, and creating a rigid body with its
// attachment, through the editing session on the bundled vehicles; each
// accepted change is one edit, which one undo takes back whole, and each
// refused one names the body, or the joint and its property at fault, and
// changes nothing. A free body takes a joint, losing its station layout entry
// and placement, the joint's child end on it; the station layout's reference
// body, a joint held on the body's own branch and a joint whose end on the
// body is elsewhere are refused. A hanging body becomes free, its joint
// removed and the records of a free body added, unless another element names
// the joint, as an IRW contact interface names its wheel's pivot; it takes
// a joint of another kind under the same name, and another joint in place of
// its own, which what named the old one then names, unless that cannot name
// the new kind. An unattached body becomes free and takes a joint; a
// conflicting body does neither. A rigid body is created free with its
// station offset and placement, or with a joint, in one edit; a joint name
// taken already, a body end given, a station offset for a body created with a
// joint, a rigid body or a joint created through AddElement are refused.
// Hanging a free bogie frame on the car body, making an axlebox carrier free
// and creating a body with a joint each leave GZ18 passing its check.

namespace {

using namespace orvd;
using editing_verification::PlacedProjectWithContact;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using editing_verification::RequireAccepted;
using editing_verification::RequireCheckPasses;
using editing_verification::RequireRefused;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project_comparison::SameProject;
using vehicle_project_editing::FreeInWorldAttachment;
using vehicle_project_editing::JointAttachment;
using vehicle_project_editing::PropertyValuesByName;
using vehicle_project_editing::ReadRigidBodyAttachment;
using vehicle_project_editing::ReferencedElementName;
using vehicle_project_editing::RigidBodyAttachmentState;
using vehicle_project_editing::VehicleProjectEditingSession;

ElementReference RigidBody(const std::string& name) {
    return {ElementKind::kRigidBody, name};
}

// The record of `records` named `name`.
template <typename Record>
const Record& Named(const std::vector<Record>& records, std::string_view name) {
    const auto found = std::ranges::find(records, name, &Record::name);
    Require(found != records.end(), "no record is named '" + std::string(name) + "'");
    return *found;
}

bool MovesFreely(const VehicleProject& project, const std::string& body) {
    return Named(project.vehicle_definition.rigid_bodies, body).moves_freely_in_world;
}

const configuration::VehicleFreeBodyStationOffsetDefinition* FindStationOffset(
    const VehicleProject& project, const std::string& body) {
    const auto& offsets =
        project.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets;
    const auto found = std::ranges::find(
        offsets, body, &configuration::VehicleFreeBodyStationOffsetDefinition::body_name);
    return found == offsets.end() ? nullptr : std::to_address(found);
}

const vehicle_project::FreeBodyPlacement* FindPlacement(const VehicleProject& project,
                                                        const std::string& body) {
    const auto found = std::ranges::find(project.free_body_placements, body,
                                         &vehicle_project::FreeBodyPlacement::body_name);
    return found == project.free_body_placements.end() ? nullptr : std::to_address(found);
}

// The body has a free body's records: a station layout entry and a placement,
// at zero offsets and the identity rotation.
bool HasZeroFreeBodyRecords(const VehicleProject& project, const std::string& body) {
    const configuration::VehicleFreeBodyStationOffsetDefinition* const offset =
        FindStationOffset(project, body);
    const vehicle_project::FreeBodyPlacement* const placement = FindPlacement(project, body);
    return offset != nullptr && offset->station_offset_meters == 0.0 && placement != nullptr &&
           vehicle_project_comparison::SamePlacement({.body_name = body}, *placement);
}

bool HasNoFreeBodyRecords(const VehicleProject& project, const std::string& body) {
    return FindStationOffset(project, body) == nullptr && FindPlacement(project, body) == nullptr;
}

PropertyValuesByName Ends(const std::string& other_end, const std::string& body_end) {
    return {{"parent_frame_name", ReferencedElementName{other_end}},
            {"child_frame_name", ReferencedElementName{body_end}}};
}

// The properties of a small box, its mass and inertia.
PropertyValuesByName Box() {
    return {{"mass_kilograms", 10.0},
            {"inertia_moments_about_center_of_mass_kilogram_square_meters",
             Eigen::Vector3d::Ones().eval()}};
}

// --- Taking a joint ----------------------------------------------------------

// GZ18's free front bogie frame hung on the car body by a weld: it no longer
// states that it moves freely, its station layout entry and placement are
// gone and the weld's child end is on it, in one edit that one undo takes
// back whole, placement included; the project passes its check.
void VerifyFreeBodyTakingAJoint(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    RequireCheckPasses(session, "GZ18 as opened");
    const std::string bogie = "front_bogie_frame";
    const ElementReference weld{ElementKind::kWeldJoint, "front_bogie_frame_weld"};

    const std::optional<ElementReference> hung_on =
        RequireAccepted(session.SetRigidBodyAttachment(
                            bogie, JointAttachment{weld.kind, weld.name, Ends("carbody", bogie)}),
                        "hanging the free front bogie frame on the car body by a weld");
    const VehicleProject& hung = session.project();
    const configuration::VehicleWeldJointDefinition& record =
        Named(hung.vehicle_definition.weld_joints, weld.name);
    Require(hung_on == weld && session.revision() == 1 && session.can_undo(),
            "hanging a free body on a joint did not return the joint as one edit");
    Require(!MovesFreely(hung, bogie) && HasNoFreeBodyRecords(hung, bogie) &&
                record.child_frame_name == bogie && record.parent_frame_name == "carbody",
            "a free body hung on a weld still moves freely, keeps a free body's records, or is "
            "not on the weld's child end");
    Require(ReadRigidBodyAttachment(hung, bogie).joint == weld &&
                ReadRigidBodyAttachment(hung, bogie).held_at == RigidBody("carbody"),
            "a free body hung on a weld does not read as hanging on it, held at the car body");
    RequireCheckPasses(session, "GZ18 with its front bogie frame welded to the car body");

    session.Undo();
    Require(SameProject(opened, session.project()),
            "one undo did not bring back the free front bogie frame with its station layout entry "
            "and placement, without the weld");
}

// Refused, changing nothing: the station layout's reference body taking a
// joint, naming the body; a joint whose other end is on the body's own
// branch, or whose end on the body is not the body's, naming the joint and
// that end.
void VerifyTakingAJointRefusals(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject before = session.project();
    const ElementReference weld{ElementKind::kWeldJoint, "refused_weld"};
    const auto take_weld = [&](const std::string& body, const std::string& other_end,
                               const std::string& body_end) {
        return session.SetRigidBodyAttachment(
            body, JointAttachment{weld.kind, weld.name, Ends(other_end, body_end)});
    };
    const auto require_refused = [&](const auto& outcome, const ElementReference& subject,
                                     const std::optional<std::string>& property_name,
                                     const std::string& what) {
        RequireRefused(outcome, subject, property_name, session, before, 0, false, what);
    };

    require_refused(take_weld("carbody", "front_bogie_frame", "carbody"), RigidBody("carbody"),
                    std::nullopt, "the station layout's reference body taking a joint");
    // The axlebox carrier hangs below the wheelset.
    require_refused(take_weld("front_leading_wheelset", "front_leading_right_axlebox_carrier",
                              "front_leading_wheelset"),
                    weld, "parent_frame_name",
                    "a wheelset held at a frame of an axlebox carrier hanging below it");
    require_refused(take_weld("front_bogie_frame", "front_bogie_frame", "front_bogie_frame"), weld,
                    "parent_frame_name", "a body held at its own frame");
    require_refused(take_weld("front_bogie_frame", "carbody", "rear_bogie_frame"), weld,
                    "child_frame_name", "a joint whose end on the body names another body");
    require_refused(
        session.SetRigidBodyAttachment(
            "front_bogie_frame", JointAttachment{ElementKind::kFixedFrame, "not_a_joint", {}}),
        RigidBody("front_bogie_frame"), std::nullopt,
        "a body attached by a kind that is not a joint kind");
}

// GZ18's front carbody interface body, welded to the car body, takes a
// revolute joint under the weld's name, with the same ends: the weld is gone
// and the body hangs on the revolute joint.
void VerifyHangingBodyTakingAnotherKind(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const std::string body = "front_carbody_interface_body";
    const configuration::VehicleWeldJointDefinition& weld =
        Named(gz18.definition.weld_joints, body + "_weld");
    const ElementReference revolute{ElementKind::kRevoluteJoint, weld.name};

    const std::optional<ElementReference> hung_on = RequireAccepted(
        session.SetRigidBodyAttachment(
            body, JointAttachment{revolute.kind, revolute.name,
                                  Ends(weld.parent_frame_name, weld.child_frame_name)}),
        "a welded body taking a revolute joint under the weld's name");
    const VehicleProject& changed = session.project();
    const configuration::VehicleRevoluteJointDefinition& record =
        Named(changed.vehicle_definition.revolute_joints, revolute.name);
    Require(hung_on == revolute && session.revision() == 1 &&
                !vehicle_project::ContainsElement(changed, {ElementKind::kWeldJoint, weld.name}) &&
                record.parent_frame_name == weld.parent_frame_name &&
                record.child_frame_name == weld.child_frame_name && !MovesFreely(changed, body) &&
                HasNoFreeBodyRecords(changed, body) &&
                ReadRigidBodyAttachment(changed, body).joint == revolute &&
                ReadRigidBodyAttachment(changed, body).held_at ==
                    ElementReference{ElementKind::kFixedFrame, weld.parent_frame_name},
            "a welded body taking a revolute joint under the weld's name does not hang on that "
            "joint alone, held where the weld held it, in one edit");
    session.Undo();
    Require(SameProject(opened, session.project()),
            "one undo did not bring back the weld in place of the revolute joint");
}

// --- Becoming free -----------------------------------------------------------

// A GZ18 axlebox carrier made free: its pivot is gone, and it states that it
// moves freely, with a station layout entry and a placement at zero offsets
// and the identity rotation; the project passes its check.
void VerifyHangingBodyBecomingFree(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const std::string carrier = "front_leading_right_axlebox_carrier";
    const ElementReference pivot{ElementKind::kRevoluteJoint, carrier + "_pivot"};

    const std::optional<ElementReference> hung_on =
        RequireAccepted(session.SetRigidBodyAttachment(carrier, FreeInWorldAttachment{}),
                        "making an axlebox carrier free");
    const VehicleProject& freed = session.project();
    Require(!hung_on.has_value() && session.revision() == 1 &&
                !vehicle_project::ContainsElement(freed, pivot) && MovesFreely(freed, carrier) &&
                HasZeroFreeBodyRecords(freed, carrier) &&
                ReadRigidBodyAttachment(freed, carrier).state == RigidBodyAttachmentState::kFree,
            "an axlebox carrier made free still has its pivot, or lacks a free body's records "
            "at zero offsets and the identity rotation");
    RequireCheckPasses(session, "GZ18 with an axlebox carrier made free");
    session.Undo();
    Require(SameProject(opened, session.project()),
            "one undo did not bring back the axlebox carrier hanging on its pivot");
}

// The contact interface naming `joint` as its independent wheel's pivot.
const configuration::VehicleContactInterfaceDeclaration& InterfaceNaming(
    const ReferenceVehicle& vehicle, const std::string& joint) {
    const auto& interfaces = vehicle.contact_record.declaration.interfaces;
    const auto found = std::ranges::find(
        interfaces, std::optional(joint),
        &configuration::VehicleContactInterfaceDeclaration::independent_wheel_revolute_joint_name);
    Require(found != interfaces.end(), "no contact interface names '" + joint + "'");
    return *found;
}

// An IRW wheel hangs on a pivot its contact interface names: it is refused
// becoming free, and taking a weld, each refusal naming the interface; it
// takes a revolute joint under a new name, and the interface then names that.
void VerifyIrwWheelPivot(const ReferenceVehicle& irw) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(irw));
    const VehicleProject before = session.project();
    const configuration::VehicleRevoluteJointDefinition& pivot =
        irw.definition.revolute_joints.front();
    const std::string wheel = pivot.child_frame_name;
    const std::string interface_name = InterfaceNaming(irw, pivot.name).interface_name;
    const auto require_refused_naming_interface = [&](const auto& outcome,
                                                      const std::string& what) {
        RequireRefused(outcome, RigidBody(wheel), std::nullopt, session, before, 0, false, what);
        Require(
            outcome.error().reason.find("contact interface '" + interface_name + "'") !=
                std::string::npos,
            what + " was refused without naming the contact interface: " + outcome.error().reason);
    };

    require_refused_naming_interface(session.SetRigidBodyAttachment(wheel, FreeInWorldAttachment{}),
                                     "an IRW wheel becoming free");
    require_refused_naming_interface(
        session.SetRigidBodyAttachment(
            wheel, JointAttachment{ElementKind::kWeldJoint, pivot.name + "_weld",
                                   Ends(pivot.parent_frame_name, pivot.child_frame_name)}),
        "an IRW wheel taking a weld");

    const ElementReference replacement{ElementKind::kRevoluteJoint, pivot.name + "_replacement"};
    PropertyValuesByName properties = Ends(pivot.parent_frame_name, pivot.child_frame_name);
    properties.emplace("axis_in_parent_frame", pivot.axis_in_parent_frame);
    const std::optional<ElementReference> hung_on =
        RequireAccepted(session.SetRigidBodyAttachment(
                            wheel, JointAttachment{replacement.kind, replacement.name, properties}),
                        "an IRW wheel taking a revolute joint under a new name");
    const VehicleProject& replaced = session.project();
    const auto& interfaces = replaced.contact_record->declaration.interfaces;
    const auto interface =
        std::ranges::find(interfaces, interface_name,
                          &configuration::VehicleContactInterfaceDeclaration::interface_name);
    Require(
        hung_on == replacement &&
            !vehicle_project::ContainsElement(replaced,
                                              {ElementKind::kRevoluteJoint, pivot.name}) &&
            interface != interfaces.end() &&
            interface->independent_wheel_revolute_joint_name == replacement.name &&
            ReadRigidBodyAttachment(replaced, wheel).joint == replacement &&
            ReadRigidBodyAttachment(replaced, wheel).held_at == RigidBody(pivot.parent_frame_name),
        "an IRW wheel taking a revolute joint under a new name does not hang on it alone, "
        "its contact interface naming it");
}

// --- Unattached and conflicting bodies ---------------------------------------

// A GZ18 axlebox carrier whose pivot was removed is unattached: it becomes
// free, and, that undone, takes its pivot again.
void VerifyUnattachedBody(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const std::string carrier = "front_leading_right_axlebox_carrier";
    const configuration::VehicleRevoluteJointDefinition& pivot =
        Named(gz18.definition.revolute_joints, carrier + "_pivot");
    const ElementReference pivot_element{ElementKind::kRevoluteJoint, pivot.name};
    RequireAccepted(session.RemoveElement(pivot_element), "removing an axlebox carrier's pivot");
    Require(ReadRigidBodyAttachment(session.project(), carrier).state ==
                RigidBodyAttachmentState::kUnattached,
            "an axlebox carrier without its pivot is not unattached");

    RequireAccepted(session.SetRigidBodyAttachment(carrier, FreeInWorldAttachment{}),
                    "making an unattached body free");
    Require(MovesFreely(session.project(), carrier) &&
                HasZeroFreeBodyRecords(session.project(), carrier),
            "an unattached body made free does not move freely with a free body's records");
    session.Undo();

    PropertyValuesByName properties = Ends(pivot.parent_frame_name, pivot.child_frame_name);
    properties.emplace("axis_in_parent_frame", pivot.axis_in_parent_frame);
    const std::optional<ElementReference> hung_on =
        RequireAccepted(session.SetRigidBodyAttachment(
                            carrier, JointAttachment{pivot_element.kind, pivot.name, properties}),
                        "an unattached body taking its pivot again");
    Require(hung_on == pivot_element && !MovesFreely(session.project(), carrier) &&
                HasNoFreeBodyRecords(session.project(), carrier) &&
                ReadRigidBodyAttachment(session.project(), carrier).joint == pivot_element,
            "an unattached body taking a joint does not hang on it, without a free body's "
            "records");
}

// GZ18 with a weld joining the car body to the free front bogie frame, as a
// project file can hold: the bogie frame, joined to two free bodies, neither
// becomes free nor takes a joint.
void VerifyConflictingBodyChangesNothing(const ReferenceVehicle& gz18) {
    VehicleProject joined = PlacedProjectWithContact(gz18);
    joined.vehicle_definition.weld_joints.push_back({.name = "carbody_to_front_bogie_frame",
                                                     .parent_frame_name = "carbody",
                                                     .child_frame_name = "front_bogie_frame"});
    VehicleProjectEditingSession session(joined);
    const std::string bogie = "front_bogie_frame";
    Require(ReadRigidBodyAttachment(joined, bogie).state == RigidBodyAttachmentState::kConflicting,
            "a free body welded to another free body is not conflicting");
    RequireRefused(session.SetRigidBodyAttachment(bogie, FreeInWorldAttachment{}), RigidBody(bogie),
                   std::nullopt, session, joined, 0, false, "a conflicting body becoming free");
    RequireRefused(session.SetRigidBodyAttachment(
                       bogie, JointAttachment{ElementKind::kRevoluteJoint, "conflicting_hinge",
                                              Ends("rear_bogie_frame", bogie)}),
                   RigidBody(bogie), std::nullopt, session, joined, 0, false,
                   "a conflicting body taking a joint");
}

// --- Creating ----------------------------------------------------------------

// A rigid body created free has its station layout entry and placement, at
// the offsets given.
void VerifyCreatingAFreeBody(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const std::string body = "created_free_box";
    PropertyValuesByName properties = Box();
    properties.emplace("mechanical_track_station_layout.station_offset_meters", 2.5);
    properties.emplace("placement.lateral_offset_in_local_track_frame_meters", 0.25);
    properties.emplace("placement.vertical_offset_in_local_track_frame_meters", 1.5);
    Require(RequireAccepted(session.AddRigidBody(body, properties, FreeInWorldAttachment{}),
                            "creating a free body") == RigidBody(body) &&
                session.revision() == 1,
            "creating a free body did not return it as one edit");
    const VehicleProject& created = session.project();
    const configuration::VehicleFreeBodyStationOffsetDefinition* const offset =
        FindStationOffset(created, body);
    const vehicle_project::FreeBodyPlacement* const placement = FindPlacement(created, body);
    Require(MovesFreely(created, body) && offset != nullptr &&
                offset->station_offset_meters == 2.5 && placement != nullptr &&
                placement->lateral_offset_in_local_track_frame_meters == 0.25 &&
                placement->vertical_offset_in_local_track_frame_meters == 1.5 &&
                ReadRigidBodyAttachment(created, body).state == RigidBodyAttachmentState::kFree,
            "a body created free does not move freely with its station offset and placement");
}

// A rigid body created with a revolute joint on the car body: one edit
// creating both, the joint's child end the new body, without a free body's
// records; the project passes its check, and one undo takes both away.
void VerifyCreatingABodyWithAJoint(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const std::string body = "hung_box";
    const ElementReference hinge{ElementKind::kRevoluteJoint, "hung_box_hinge"};

    RequireAccepted(
        session.AddRigidBody(
            body, Box(),
            JointAttachment{
                hinge.kind, hinge.name, {{"parent_frame_name", ReferencedElementName{"carbody"}}}}),
        "creating a body hanging on a revolute joint on the car body");
    const VehicleProject& created = session.project();
    const configuration::VehicleRevoluteJointDefinition& record =
        Named(created.vehicle_definition.revolute_joints, hinge.name);
    Require(session.revision() == 1 && !MovesFreely(created, body) &&
                HasNoFreeBodyRecords(created, body) && record.child_frame_name == body &&
                record.parent_frame_name == "carbody" &&
                ReadRigidBodyAttachment(created, body).joint == hinge &&
                ReadRigidBodyAttachment(created, body).held_at == RigidBody("carbody"),
            "a body created with a joint is not one edit creating both, the joint's child end on "
            "the body and its other end on the car body, without a free body's records");
    RequireCheckPasses(session, "GZ18 with a box hanging on the car body");
    session.Undo();
    Require(SameProject(opened, session.project()),
            "one undo did not take away both the created body and its joint");
}

// Refused, changing nothing: a joint name already taken, naming the joint and
// its name, whether a body is created with it or takes it; a new body's joint
// whose end on the body is given; a station offset given to a body created
// with a joint, naming the body and that property; and a rigid body or a
// joint through AddElement.
void VerifyCreationRefusals(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject before = session.project();
    const auto require_refused = [&](const auto& outcome, const ElementReference& subject,
                                     const std::optional<std::string>& property_name,
                                     const std::string& what) {
        RequireRefused(outcome, subject, property_name, session, before, 0, false, what);
    };
    const ElementReference taken{ElementKind::kRevoluteJoint,
                                 gz18.definition.revolute_joints.front().name};
    const PropertyValuesByName on_carbody{{"parent_frame_name", ReferencedElementName{"carbody"}}};

    require_refused(session.AddRigidBody("clashing_box", Box(),
                                         JointAttachment{taken.kind, taken.name, on_carbody}),
                    taken, "name", "a body created with a joint under a joint's name");
    require_refused(session.SetRigidBodyAttachment(
                        "front_bogie_frame", JointAttachment{taken.kind, taken.name,
                                                             Ends("carbody", "front_bogie_frame")}),
                    taken, "name", "a body taking a joint under a joint's name");
    const ElementReference hinge{ElementKind::kRevoluteJoint, "box_hinge"};
    require_refused(
        session.AddRigidBody("box", Box(),
                             JointAttachment{hinge.kind, hinge.name, Ends("carbody", "carbody")}),
        hinge, "child_frame_name", "a new body's joint with its end on the body given");
    PropertyValuesByName placed = Box();
    placed.emplace("mechanical_track_station_layout.station_offset_meters", 1.0);
    require_refused(session.AddRigidBody("placed_box", placed,
                                         JointAttachment{hinge.kind, hinge.name, on_carbody}),
                    RigidBody("placed_box"),
                    "mechanical_track_station_layout.station_offset_meters",
                    "a station offset given to a body created with a joint");
    require_refused(session.AddElement(ElementKind::kRigidBody, "box", Box()), RigidBody("box"),
                    std::nullopt, "a rigid body created through AddElement");
    const ElementReference weld{ElementKind::kWeldJoint, "lone_weld"};
    require_refused(session.AddElement(weld.kind, weld.name, Ends("carbody", "front_bogie_frame")),
                    weld, std::nullopt, "a joint created through AddElement");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_rigid_body_attachment_changes GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const ReferenceVehicle irw =
            editing_verification::LoadReferenceVehicle(argv[4], argv[5], argv[6]);
        VerifyFreeBodyTakingAJoint(gz18);
        VerifyTakingAJointRefusals(gz18);
        VerifyHangingBodyTakingAnotherKind(gz18);
        VerifyHangingBodyBecomingFree(gz18);
        VerifyIrwWheelPivot(irw);
        VerifyUnattachedBody(gz18);
        VerifyConflictingBodyChangesNothing(gz18);
        VerifyCreatingAFreeBody(gz18);
        VerifyCreatingABodyWithAJoint(gz18);
        VerifyCreationRefusals(gz18);
        std::puts("vehicle project rigid body attachment changes passed for GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project rigid body attachment changes: %s\n", error.what());
        return 1;
    }
}
