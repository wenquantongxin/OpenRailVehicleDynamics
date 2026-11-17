#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/generate_assembly_inputs_from_vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project/vehicle_project_invariants.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "vehicle_project/configuration_record_comparison.h"
#include "vehicle_project/vehicle_project_comparison.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// Structural edits on the bundled vehicles, verified against records built
// here from the loaded ones rather than against recorded outputs: a force
// element added to GZ18 is exactly one more record and comes and goes with
// undo and redo; removing an IRW wheel body or a GZ18 wheelset takes exactly
// what refers to it, transitively, as the consequences said beforehand, with
// the records that followed it; renaming a body renames it wherever it is
// referred to, its contact carrier of the same name keeping its own name, and
// its station layout entry, placement, shape name and group membership
// follow, by kind; every edit
// the contract refuses is refused by element and property and changes
// nothing; edits alone give a GZ18 bogie a third axle that passes the check
// and is drawn with two more wheels; an optional reference is set and cleared;
// a new damper's empty curve is judged by the check until a curve is set; a
// project whose contact record does not fit is repaired one edit at a time; a
// body created in the session has no shapes; and every axis token is one the
// check accepts.

namespace {

using namespace orvd;
using configuration_record_comparison::Same;
using editing_verification::PlacedProject;
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
using vehicle_project_editing::ElementOwnName;
using vehicle_project_editing::FreeInWorldAttachment;
using vehicle_project_editing::MechanicalCheckReport;
using vehicle_project_editing::PropertyToken;
using vehicle_project_editing::PropertyValue;
using vehicle_project_editing::PropertyValuesByName;
using vehicle_project_editing::ReferencedElementName;
using vehicle_project_editing::VehicleProjectEditingSession;
using vehicle_project_editing::VehicleProjectPreview;

// The record of `records` named `name`.
template <typename Record>
const Record& Named(const std::vector<Record>& records, std::string_view name) {
    const auto found = std::ranges::find(records, name, &Record::name);
    Require(found != records.end(), "no record is named '" + std::string(name) + "'");
    return *found;
}

const configuration::VehicleFreeBodyStationOffsetDefinition& StationOffsetOf(
    const VehicleProject& project, const std::string& body_name) {
    const auto& offsets =
        project.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets;
    const auto found = std::ranges::find(
        offsets, body_name, &configuration::VehicleFreeBodyStationOffsetDefinition::body_name);
    Require(found != offsets.end(), "the layout has no entry for '" + body_name + "'");
    return *found;
}

const vehicle_project::FreeBodyPlacement* FindPlacement(const VehicleProject& project,
                                                        const std::string& body_name) {
    const auto found = std::ranges::find(project.free_body_placements, body_name,
                                         &vehicle_project::FreeBodyPlacement::body_name);
    return found == project.free_body_placements.end() ? nullptr : std::to_address(found);
}

const vehicle_project::VisualDefinitionBodyName* FindVisualDefinitionName(
    const VehicleProject& project, const std::string& body_name) {
    const auto found = std::ranges::find(project.visual_definition_body_names, body_name,
                                         &vehicle_project::VisualDefinitionBodyName::body_name);
    return found == project.visual_definition_body_names.end() ? nullptr : std::to_address(found);
}

// Every element of the project in outline order.
std::vector<ElementReference> Outline(const VehicleProject& project) {
    std::vector<ElementReference> elements;
    for (const ElementKind kind : vehicle_project::kElementKinds) {
        for (const std::string& name :
             vehicle_project_editing::CollectElementNames(project, kind)) {
            elements.push_back({kind, name});
        }
    }
    return elements;
}

// --- Adding ------------------------------------------------------------------

// A translational spring-damper added beside a GZ18 primary suspension, with
// its ends and values: the generated definition is the original with exactly
// that record appended, undo restores the opened project and redo the edit.
void VerifyAddingAForceElement(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const configuration::VehicleTranslationalSpringDamperDefinition& beside = Named(
        gz18.definition.translational_spring_dampers, "front_leading_left_primary_suspension");
    const configuration::VehicleTranslationalSpringDamperDefinition added{
        .name = beside.name + "_parallel",
        .reference_frame_name = beside.reference_frame_name,
        .opposite_frame_name = beside.opposite_frame_name,
        .stiffness_newtons_per_meter = beside.stiffness_newtons_per_meter,
        .damping_newton_seconds_per_meter = beside.damping_newton_seconds_per_meter};
    const ElementReference element{ElementKind::kTranslationalSpringDamper, added.name};

    const ElementReference created = RequireAccepted(
        session.AddElement(
            element.kind, element.name,
            {{"reference_frame_name", ReferencedElementName{added.reference_frame_name}},
             {"opposite_frame_name", ReferencedElementName{added.opposite_frame_name}},
             {"stiffness_newtons_per_meter", added.stiffness_newtons_per_meter},
             {"damping_newton_seconds_per_meter", added.damping_newton_seconds_per_meter}}),
        "adding a GZ18 spring-damper");
    Require(created == element && session.revision() == 1 && session.can_undo(),
            "adding a spring-damper did not return it as one edit of one revision");
    configuration::VehicleDefinition expected = vehicle_project::GenerateVehicleDefinition(opened);
    expected.translational_spring_dampers.push_back(added);
    Require(Same(expected, vehicle_project::GenerateVehicleDefinition(session.project())),
            "the definition with an added spring-damper is not the original with exactly that "
            "record appended");
    RequireCheckPasses(session, "GZ18 with an added spring-damper");

    session.Undo();
    Require(SameProject(opened, session.project()),
            "undoing the addition did not restore the opened project");
    session.Redo();
    Require(Same(expected, vehicle_project::GenerateVehicleDefinition(session.project())),
            "redoing the addition did not bring the added record back");
}

// --- Removing ----------------------------------------------------------------

bool AmongNames(const std::vector<std::string>& names, const std::string& name) {
    return std::ranges::find(names, name) != names.end();
}

// What removing `body` takes with it, stated here from the records: the body,
// the frames fixed to it, every joint and force element with an end at the
// body or one of those frames, the carriers on the body, and every interface
// whose wheel body is the body or whose carrier or independent-wheel joint is
// taken, in outline order.
std::vector<ElementReference> ExpectedRemovalOfBody(const VehicleProject& project,
                                                    const std::string& body) {
    const configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    std::vector<std::string> frames{body};
    for (const auto& frame : vehicle.fixed_frames) {
        if (frame.body_name == body) {
            frames.push_back(frame.name);
        }
    }
    std::vector<ElementReference> taken{{ElementKind::kRigidBody, body}};
    for (const auto& frame : vehicle.fixed_frames) {
        if (frame.body_name == body) {
            taken.push_back({ElementKind::kFixedFrame, frame.name});
        }
    }
    std::vector<std::string> taken_joints;
    const auto take_joints = [&](ElementKind kind, const auto& joints) {
        for (const auto& joint : joints) {
            if (AmongNames(frames, joint.parent_frame_name) ||
                AmongNames(frames, joint.child_frame_name)) {
                taken.push_back({kind, joint.name});
                taken_joints.push_back(joint.name);
            }
        }
    };
    take_joints(ElementKind::kRevoluteJoint, vehicle.revolute_joints);
    take_joints(ElementKind::kBallRpyJoint, vehicle.ball_rpy_joints);
    take_joints(ElementKind::kWeldJoint, vehicle.weld_joints);
    const auto take_force_elements = [&](ElementKind kind, const auto& elements) {
        for (const auto& element : elements) {
            if (AmongNames(frames, element.reference_frame_name) ||
                AmongNames(frames, element.opposite_frame_name)) {
                taken.push_back({kind, element.name});
            }
        }
    };
    take_force_elements(ElementKind::kTranslationalSpringDamper,
                        vehicle.translational_spring_dampers);
    take_force_elements(ElementKind::kRollSpringDamperCouple, vehicle.roll_spring_damper_couples);
    take_force_elements(ElementKind::kSeriesSpringViscousDamper,
                        vehicle.series_spring_viscous_dampers);
    take_force_elements(ElementKind::kPointToPointSeriesSpringBilinearDamper,
                        vehicle.point_to_point_series_spring_bilinear_dampers);
    take_force_elements(ElementKind::kSaturatedPiecewiseLinearDamper,
                        vehicle.saturated_piecewise_linear_dampers);
    take_force_elements(ElementKind::kReferenceFrameLinearBushing,
                        vehicle.reference_frame_linear_bushings);
    take_force_elements(ElementKind::kPointToPointLinearSpringDamper,
                        vehicle.point_to_point_linear_spring_dampers);
    take_force_elements(ElementKind::kPointToPointHystereticSpring,
                        vehicle.point_to_point_hysteretic_springs);
    std::vector<std::string> taken_carriers;
    for (const auto& carrier : project.contact_record->declaration.carriers) {
        if (carrier.body_name == body) {
            taken.push_back({ElementKind::kContactCarrier, carrier.carrier_name});
            taken_carriers.push_back(carrier.carrier_name);
        }
    }
    for (const auto& interface : project.contact_record->declaration.interfaces) {
        if (interface.wheel_body_name == body ||
            AmongNames(taken_carriers, interface.carrier_name) ||
            (interface.independent_wheel_revolute_joint_name.has_value() &&
             AmongNames(taken_joints, *interface.independent_wheel_revolute_joint_name))) {
            taken.push_back({ElementKind::kContactInterface, interface.interface_name});
        }
    }
    return taken;
}

// Removing a body takes exactly what refers to it, as the consequences listed
// beforehand, as one edit, with the station layout entry, placement and shape
// name that followed it; undo brings all of it back. `required` must be among
// what goes.
void VerifyRemovingABody(const ReferenceVehicle& reference, const std::string& body_name,
                         const std::vector<ElementReference>& required) {
    const std::string what = reference.definition.vehicle_name + " body '" + body_name + "'";
    VehicleProjectEditingSession session(PlacedProjectWithContact(reference));
    const VehicleProject opened = session.project();
    const ElementReference body{ElementKind::kRigidBody, body_name};
    const std::vector<ElementReference> expected = ExpectedRemovalOfBody(opened, body_name);
    Require(std::ranges::all_of(required,
                                [&expected](const ElementReference& element) {
                                    return std::ranges::find(expected, element) != expected.end();
                                }),
            what + " is not referred to by everything this test expects to go with it");

    const std::vector<ElementReference> consequences = RequireAccepted(
        session.CollectRemovalConsequences(body), "asking what removing " + what + " takes");
    Require(consequences == expected,
            "the consequences of removing " + what + " are not exactly what refers to it");
    Require(
        SameProject(opened, session.project()) && session.revision() == 0 && !session.can_undo(),
        "asking for the consequences of removing " + what + " changed the session");

    const std::vector<ElementReference> removed =
        RequireAccepted(session.RemoveElement(body), "removing " + what);
    Require(removed == consequences && session.revision() == 1 && session.can_undo(),
            "removing " + what + " did not take what its consequences listed, as one edit");
    std::vector<ElementReference> remaining = Outline(opened);
    std::erase_if(remaining, [&removed](const ElementReference& element) {
        return std::ranges::find(removed, element) != removed.end();
    });
    Require(Outline(session.project()) == remaining,
            "removing " + what + " took other elements than it listed, or left one of them");
    const configuration::VehicleMechanicalTrackStationLayoutDefinition& layout =
        session.project().vehicle_definition.mechanical_track_station_layout;
    Require(std::ranges::find(layout.free_body_station_offsets, body_name,
                              &configuration::VehicleFreeBodyStationOffsetDefinition::body_name) ==
                    layout.free_body_station_offsets.end() &&
                std::ranges::find(layout.wheel_pair_station_reference_body_names, body_name) ==
                    layout.wheel_pair_station_reference_body_names.end() &&
                FindPlacement(session.project(), body_name) == nullptr &&
                FindVisualDefinitionName(session.project(), body_name) == nullptr,
            "removing " + what + " left its station layout entry, placement or shape name");

    session.Undo();
    Require(SameProject(opened, session.project()),
            "undoing the removal of " + what + " did not restore the opened project");
}

// An IRW wheel body goes with its revolute joint and contact interface; a
// GZ18 wheelset with its frames, the axlebox carrier pivots on those frames,
// its contact carrier and its interfaces.
void VerifyRemovingBodies(const ReferenceVehicle& gz18, const ReferenceVehicle& irw) {
    const configuration::VehicleContactInterfaceDeclaration& irw_interface =
        irw.contact_record.declaration.interfaces.front();
    VerifyRemovingABody(
        irw, irw_interface.wheel_body_name,
        {{ElementKind::kContactInterface, irw_interface.interface_name},
         {ElementKind::kRevoluteJoint, *irw_interface.independent_wheel_revolute_joint_name}});
    const configuration::VehicleContactCarrierDeclaration& gz18_carrier =
        gz18.contact_record.declaration.carriers.front();
    const configuration::VehicleRevoluteJointDefinition& pivot =
        gz18.definition.revolute_joints.front();
    Require(Named(gz18.definition.fixed_frames, pivot.parent_frame_name).body_name ==
                gz18_carrier.body_name,
            "GZ18's first revolute joint no longer hangs from a frame on its first wheelset");
    VerifyRemovingABody(gz18, gz18_carrier.body_name,
                        {{ElementKind::kFixedFrame, pivot.parent_frame_name},
                         {ElementKind::kRevoluteJoint, pivot.name},
                         {ElementKind::kContactCarrier, gz18_carrier.carrier_name}});
}

// Group memberships follow their elements by kind: renaming GZ18's first
// wheelset body renames its membership but not that of its contact carrier of
// the same name, and removing a frame drops its membership and keeps the
// groups.
void VerifyGroupsFollowElements(const ReferenceVehicle& gz18) {
    const configuration::VehicleContactCarrierDeclaration& carrier =
        gz18.contact_record.declaration.carriers.front();
    const ElementReference body{ElementKind::kRigidBody, carrier.body_name};
    const ElementReference carrier_element{ElementKind::kContactCarrier, carrier.carrier_name};
    const ElementReference frame{ElementKind::kFixedFrame,
                                 gz18.definition.fixed_frames.front().name};
    VehicleProject project = PlacedProjectWithContact(gz18);
    project.element_groups = {{.name = "running gear",
                               .members = {carrier_element, frame},
                               .subgroups = {{.name = "wheelset", .members = {body}}}}};
    VehicleProjectEditingSession session(project);

    const auto renamed = std::get<ElementReference>(RequireAccepted(
        session.SetSubjectProperty(body, "name", ElementOwnName{body.name + "_renamed"}),
        "renaming a grouped body"));
    std::vector<vehicle_project::ElementGroup> expected = project.element_groups;
    expected.front().subgroups.front().members = {renamed};
    Require(vehicle_project_comparison::SameGroups(expected, session.project().element_groups),
            "the renamed body's membership did not follow it, or its carrier's did");

    RequireAccepted(session.RemoveElement(frame), "removing a grouped frame");
    expected.front().members = {carrier_element};
    Require(vehicle_project_comparison::SameGroups(expected, session.project().element_groups),
            "removing a grouped frame did not drop exactly its membership");
    vehicle_project::RequireVehicleProjectInvariants(session.project());
}

// --- Renaming ----------------------------------------------------------------

// The definition with `old_name` substituted by `new_name` wherever a body is
// referred to: the body's own record, the frames fixed to it, joint and force
// element ends, and the station layout.
void SubstituteBodyName(configuration::VehicleDefinition& vehicle, const std::string& old_name,
                        const std::string& new_name) {
    const auto substitute = [&old_name, &new_name](std::string& name) {
        if (name == old_name) {
            name = new_name;
        }
    };
    for (auto& body : vehicle.rigid_bodies) {
        substitute(body.name);
    }
    for (auto& frame : vehicle.fixed_frames) {
        substitute(frame.body_name);
    }
    const auto substitute_ends = [&substitute](auto& joints_or_force_elements) {
        for (auto& element : joints_or_force_elements) {
            if constexpr (requires { element.parent_frame_name; }) {
                substitute(element.parent_frame_name);
                substitute(element.child_frame_name);
            } else {
                substitute(element.reference_frame_name);
                substitute(element.opposite_frame_name);
            }
        }
    };
    substitute_ends(vehicle.revolute_joints);
    substitute_ends(vehicle.ball_rpy_joints);
    substitute_ends(vehicle.weld_joints);
    substitute_ends(vehicle.translational_spring_dampers);
    substitute_ends(vehicle.roll_spring_damper_couples);
    substitute_ends(vehicle.series_spring_viscous_dampers);
    substitute_ends(vehicle.point_to_point_series_spring_bilinear_dampers);
    substitute_ends(vehicle.saturated_piecewise_linear_dampers);
    substitute_ends(vehicle.reference_frame_linear_bushings);
    substitute_ends(vehicle.point_to_point_linear_spring_dampers);
    substitute_ends(vehicle.point_to_point_hysteretic_springs);
    auto& layout = vehicle.mechanical_track_station_layout;
    substitute(layout.reference_body_name);
    for (auto& offset : layout.free_body_station_offsets) {
        substitute(offset.body_name);
    }
    std::ranges::for_each(layout.wheel_pair_station_reference_body_names, substitute);
}

// Renaming GZ18's first wheelset body renames it wherever it is referred to;
// its contact carrier of the same name keeps its own name and names the
// renamed body; the station layout, the placement and the shape name follow,
// the shape name still being the one the body had; the check still passes.
void VerifyRenamingARigidBody(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const configuration::VehicleContactCarrierDeclaration& carrier =
        gz18.contact_record.declaration.carriers.front();
    Require(carrier.carrier_name == carrier.body_name,
            "GZ18's first contact carrier no longer shares its body's name");
    const std::string old_name = carrier.body_name;
    const std::string new_name = old_name + "_renamed";

    const auto renamed = std::get<ElementReference>(RequireAccepted(
        session.SetSubjectProperty(ElementReference{ElementKind::kRigidBody, old_name}, "name",
                                   ElementOwnName{new_name}),
        "renaming a GZ18 wheelset body"));
    Require(renamed == ElementReference{ElementKind::kRigidBody, new_name},
            "a rename did not return the renamed body");

    configuration::VehicleDefinition expected = vehicle_project::GenerateVehicleDefinition(opened);
    SubstituteBodyName(expected, old_name, new_name);
    Require(Same(expected, vehicle_project::GenerateVehicleDefinition(session.project())),
            "the renamed project's definition is not the original with the body's name "
            "substituted wherever the body is referred to");
    // The project's own definition too, whose wheel-pair station list the
    // contact carriers stand in for in generation.
    configuration::VehicleDefinition expected_held = opened.vehicle_definition;
    SubstituteBodyName(expected_held, old_name, new_name);
    Require(Same(expected_held, session.project().vehicle_definition),
            "the renamed project holds a definition that still names the body by its old name");
    configuration::VehicleContactRecord expected_contact = gz18.contact_record;
    for (auto& expected_carrier : expected_contact.declaration.carriers) {
        if (expected_carrier.body_name == old_name) {
            expected_carrier.body_name = new_name;
        }
    }
    for (auto& expected_interface : expected_contact.declaration.interfaces) {
        if (expected_interface.wheel_body_name == old_name) {
            expected_interface.wheel_body_name = new_name;
        }
    }
    Require(Same(expected_contact, *session.project().contact_record),
            "the renamed body's carrier did not keep its own name while naming the renamed body, "
            "or its interfaces did not follow");

    const vehicle_project::FreeBodyPlacement* const old_placement = FindPlacement(opened, old_name);
    const vehicle_project::FreeBodyPlacement* const new_placement =
        FindPlacement(session.project(), new_name);
    Require(
        old_placement != nullptr && new_placement != nullptr &&
            FindPlacement(session.project(), old_name) == nullptr &&
            vehicle_project_comparison::SamePlacement(
                {.body_name = new_name,
                 .rotation_local_track_from_body = old_placement->rotation_local_track_from_body,
                 .lateral_offset_in_local_track_frame_meters =
                     old_placement->lateral_offset_in_local_track_frame_meters,
                 .vertical_offset_in_local_track_frame_meters =
                     old_placement->vertical_offset_in_local_track_frame_meters},
                *new_placement),
        "the renamed body's placement did not follow it");
    const vehicle_project::VisualDefinitionBodyName* const shapes =
        FindVisualDefinitionName(session.project(), new_name);
    Require(shapes != nullptr && shapes->visual_definition_body_name == old_name &&
                FindVisualDefinitionName(session.project(), old_name) == nullptr,
            "the renamed body does not keep the name its shapes are stated under");

    RequireCheckPasses(session, "GZ18 with a renamed wheelset body");
    const VehicleProjectPreview& preview = *session.preview();
    const auto drawn = std::ranges::find(preview.rigid_bodies, new_name,
                                         &vehicle_project_editing::RigidBodyDrawing::body_name);
    Require(drawn != preview.rigid_bodies.end() && drawn->visual_definition_body_name == old_name,
            "the renamed body is not drawn with the shapes it had");
}

// --- Refusals ----------------------------------------------------------------

// Each edit the contract refuses is refused by element and property and
// changes neither the project, its history nor its revision.
void VerifyStructuralRefusals(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const configuration::VehicleDefinition& vehicle = gz18.definition;
    const ElementReference suspension{ElementKind::kTranslationalSpringDamper,
                                      vehicle.translational_spring_dampers.front().name};
    // One accepted edit first, so that the history holds a step a refusal
    // could disturb.
    RequireAccepted(session.SetSubjectProperty(suspension, "damping_newton_seconds_per_meter",
                                               Eigen::Vector3d(1.0e3, 2.0e3, 3.0e4)),
                    "a GZ18 damping edit");
    const VehicleProject before = session.project();
    const std::uint64_t revision = session.revision();
    const auto require_refused = [&](const auto& outcome, const ElementReference& element,
                                     const std::optional<std::string>& property_name,
                                     const std::string& what) {
        RequireRefused(outcome, element, property_name, session, before, revision, true, what);
    };

    require_refused(session.SetSubjectProperty(suspension, "reference_frame_name",
                                               ReferencedElementName{"absent_frame"}),
                    suspension, "reference_frame_name",
                    "a force element end set to a frame that does not exist");
    const ElementReference frame{ElementKind::kFixedFrame, vehicle.fixed_frames.front().name};
    require_refused(
        session.SetSubjectProperty(frame, "body_name",
                                   ReferencedElementName{vehicle.revolute_joints.front().name}),
        frame, "body_name", "a fixed frame pointed at a joint");
    require_refused(session.SetSubjectProperty(frame, "name",
                                               ElementOwnName{vehicle.rigid_bodies.front().name}),
                    frame, "name", "a frame named like a body");
    const ElementReference weld{ElementKind::kWeldJoint, vehicle.weld_joints.front().name};
    require_refused(session.SetSubjectProperty(
                        weld, "name", ElementOwnName{vehicle.revolute_joints.front().name}),
                    weld, "name", "a weld named like a revolute joint");
    const ElementReference reference_body{
        ElementKind::kRigidBody, vehicle.mechanical_track_station_layout.reference_body_name};
    require_refused(session.SetSubjectProperty(reference_body, "name", ElementOwnName{"world"}),
                    reference_body, "name", "a body named 'world'");
    require_refused(session.SetSubjectProperty(reference_body, "name", ElementOwnName{""}),
                    reference_body, "name", "a body given an empty name");
    const ElementReference new_frame{ElementKind::kFixedFrame, "frame_without_body"};
    require_refused(
        session.AddElement(new_frame.kind, new_frame.name,
                           {{"position_in_body_frame_meters", Eigen::Vector3d::Zero().eval()}}),
        new_frame, "body_name", "a fixed frame created without its body");
    const ElementReference damper{ElementKind::kSeriesSpringViscousDamper,
                                  vehicle.series_spring_viscous_dampers.front().name};
    require_refused(session.SetSubjectProperty(damper, "axis", PropertyToken{"diagonal"}), damper,
                    "axis", "an axis outside its choices");
    require_refused(session.RemoveElement(reference_body), reference_body, std::nullopt,
                    "removing the station layout's reference body");
    require_refused(session.CollectRemovalConsequences(reference_body), reference_body,
                    std::nullopt, "asking what removing the station layout's reference body takes");
    const ElementReference body_named_like_frame{ElementKind::kRigidBody, frame.name};
    require_refused(session.AddRigidBody(body_named_like_frame.name, {}, FreeInWorldAttachment{}),
                    body_named_like_frame, "name", "a body created under a frame's name");
    const ElementReference named_twice{ElementKind::kRigidBody, "named_twice"};
    require_refused(
        session.AddRigidBody(named_twice.name, {{"name", ElementOwnName{"named_twice"}}},
                             FreeInWorldAttachment{}),
        named_twice, "name", "a body created with its own name among its properties");
}

// A contact element needs the contact record its kind belongs to.
void VerifyContactElementNeedsContactRecord(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProject(gz18));
    const VehicleProject before = session.project();
    const ElementReference carrier{ElementKind::kContactCarrier, "carrier_without_record"};
    RequireRefused(session.AddElement(carrier.kind, carrier.name,
                                      {{"body_name", ReferencedElementName{"carbody"}}}),
                   carrier, std::nullopt, session, before, 0, false,
                   "a contact carrier created in a project without a contact record");
}

// --- Building ----------------------------------------------------------------

// Only through edits, GZ18's front bogie gets a third axle, ahead of its
// leading one by the spacing of its two axles: a free wheelset body copying
// the leading one, two frames on it, two frames on the bogie frame, the two
// primary suspensions between them, a contact carrier and its two interfaces.
// The check passes and the preview draws two more wheels.
void VerifyBuildingAThirdAxle(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject& project = session.project();
    const configuration::VehicleDefinition& vehicle = gz18.definition;
    RequireCheckPasses(session, "GZ18 as opened");
    const std::size_t opened_wheel_count = session.preview()->wheels.size();

    const std::string leading_name = "front_leading_wheelset";
    const configuration::VehicleRigidBodyDefinition& leading =
        Named(vehicle.rigid_bodies, leading_name);
    const double leading_station = StationOffsetOf(project, leading_name).station_offset_meters;
    const double axle_spacing =
        leading_station - StationOffsetOf(project, "front_trailing_wheelset").station_offset_meters;
    const vehicle_project::FreeBodyPlacement& leading_placement =
        *FindPlacement(project, leading_name);
    const std::string wheelset = "front_third_wheelset";
    RequireAccepted(
        session.AddRigidBody(
            wheelset,
            {{"mass_kilograms", leading.mass_kilograms},
             {"center_of_mass_in_body_frame_meters", leading.center_of_mass_in_body_frame_meters},
             {"inertia_moments_about_center_of_mass_kilogram_square_meters",
              leading.inertia_moments_about_center_of_mass_kilogram_square_meters},
             {"inertia_products_about_center_of_mass_kilogram_square_meters",
              leading.inertia_products_about_center_of_mass_kilogram_square_meters},
             {"mechanical_track_station_layout.station_offset_meters",
              leading_station + axle_spacing},
             {"placement.lateral_offset_in_local_track_frame_meters",
              leading_placement.lateral_offset_in_local_track_frame_meters},
             {"placement.vertical_offset_in_local_track_frame_meters",
              leading_placement.vertical_offset_in_local_track_frame_meters}},
            FreeInWorldAttachment{}),
        "adding the third wheelset body");

    for (const std::string side : {"left", "right"}) {
        const std::string leading_side = "front_leading_" + side;
        const std::string third_side = "front_third_" + side;
        const configuration::VehicleFixedFrameDefinition& wheelset_side = Named(
            vehicle.fixed_frames, leading_side + "_axlebox_carrier_pivot_wheelset_side_attachment");
        const configuration::VehicleFixedFrameDefinition& bogie_side =
            Named(vehicle.fixed_frames, leading_side + "_primary_suspension_bogie_side_attachment");
        const configuration::VehicleTranslationalSpringDamperDefinition& suspension =
            Named(vehicle.translational_spring_dampers, leading_side + "_primary_suspension");
        const std::string wheelset_frame =
            third_side + "_primary_suspension_wheelset_side_attachment";
        const std::string bogie_frame = third_side + "_primary_suspension_bogie_side_attachment";
        RequireAccepted(session.AddElement(ElementKind::kFixedFrame, wheelset_frame,
                                           {{"body_name", ReferencedElementName{wheelset}},
                                            {"position_in_body_frame_meters",
                                             wheelset_side.position_in_body_frame_meters}}),
                        "adding a frame on the third wheelset");
        RequireAccepted(
            session.AddElement(ElementKind::kFixedFrame, bogie_frame,
                               {{"body_name", ReferencedElementName{bogie_side.body_name}},
                                {"position_in_body_frame_meters",
                                 Eigen::Vector3d(bogie_side.position_in_body_frame_meters +
                                                 Eigen::Vector3d(axle_spacing, 0.0, 0.0))}}),
            "adding a bogie-side frame for the third axle");
        RequireAccepted(
            session.AddElement(
                ElementKind::kTranslationalSpringDamper, third_side + "_primary_suspension",
                {{"reference_frame_name", ReferencedElementName{bogie_frame}},
                 {"opposite_frame_name", ReferencedElementName{wheelset_frame}},
                 {"stiffness_newtons_per_meter", suspension.stiffness_newtons_per_meter},
                 {"damping_newton_seconds_per_meter",
                  suspension.damping_newton_seconds_per_meter}}),
            "adding a primary suspension of the third axle");
    }

    RequireAccepted(session.AddElement(ElementKind::kContactCarrier, wheelset,
                                       {{"body_name", ReferencedElementName{wheelset}}}),
                    "adding the third axle's contact carrier");
    // Each interface is named after its carrier and its side, as GZ18's are.
    for (const std::string side : {"left", "right"}) {
        RequireAccepted(session.AddElement(ElementKind::kContactInterface, wheelset + "." + side,
                                           {{"carrier_name", ReferencedElementName{wheelset}},
                                            {"wheel_body_name", ReferencedElementName{wheelset}},
                                            {"side", PropertyToken{side}}}),
                        "adding a contact interface of the third axle");
    }

    RequireCheckPasses(session, "GZ18 with a third axle on its front bogie");
    const VehicleProjectPreview& preview = *session.preview();
    Require(preview.wheels.size() == opened_wheel_count + 2,
            "the preview does not draw exactly two more wheels for the third axle");
    for (const std::string side : {"left", "right"}) {
        Require(std::ranges::find(preview.wheels, wheelset + "." + side,
                                  &vehicle_project_editing::WheelDrawing::interface_name) !=
                    preview.wheels.end(),
                "the third axle's " + side + " wheel is not drawn");
    }
}

// --- References and curves ---------------------------------------------------

// An interface's optional independent-wheel joint is set, which the check
// then judges, and cleared again, which restores the opened project.
void VerifyOptionalReference(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const ElementReference interface{
        ElementKind::kContactInterface,
        gz18.contact_record.declaration.interfaces.front().interface_name};
    const std::string joint = gz18.definition.revolute_joints.front().name;
    RequireAccepted(session.SetSubjectProperty(interface, "independent_wheel_revolute_joint_name",
                                               ReferencedElementName{joint}),
                    "naming an independent-wheel joint for a GZ18 interface");
    Require(session.project()
                    .contact_record->declaration.interfaces.front()
                    .independent_wheel_revolute_joint_name == joint,
            "the interface does not name the joint it was given");
    Require(!session.CheckMechanics().refusals.empty(),
            "the check accepted a rigid wheelset's interface naming an independent-wheel joint");
    RequireAccepted(session.SetSubjectProperty(interface, "independent_wheel_revolute_joint_name",
                                               ReferencedElementName{std::nullopt}),
                    "clearing the interface's independent-wheel joint");
    Require(SameProject(opened, session.project()),
            "clearing the joint did not restore the opened project");
    RequireCheckPasses(session, "GZ18 with the joint cleared again");
}

// A new saturated piecewise-linear damper has an empty curve, which the check
// refuses; with the curve of an existing yaw damper set, the check passes.
void VerifyCurve(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const configuration::VehicleSaturatedPiecewiseLinearDamperDefinition& beside =
        gz18.definition.saturated_piecewise_linear_dampers.front();
    const ElementReference damper{ElementKind::kSaturatedPiecewiseLinearDamper,
                                  beside.name + "_parallel"};
    RequireAccepted(
        session.AddElement(
            damper.kind, damper.name,
            {{"reference_frame_name", ReferencedElementName{beside.reference_frame_name}},
             {"opposite_frame_name", ReferencedElementName{beside.opposite_frame_name}},
             {"axis", PropertyToken{beside.axis}}}),
        "adding a saturated piecewise-linear damper");
    Require(
        Named(session.project().vehicle_definition.saturated_piecewise_linear_dampers, damper.name)
            .curve.empty(),
        "a new saturated piecewise-linear damper does not start with an empty curve");
    const MechanicalCheckReport refused = session.CheckMechanics();
    Require(std::ranges::any_of(refused.refusals,
                                [&damper](const std::string& refusal) {
                                    return refusal.find(damper.name) != std::string::npos;
                                }),
            "the check did not judge the new damper's empty curve");
    RequireAccepted(session.SetSubjectProperty(damper, "curve", beside.curve),
                    "setting the new damper's curve");
    RequireCheckPasses(session, "GZ18 with a new damper and its curve");
}

// --- Repairing and shapes ----------------------------------------------------

// GZ18 opened with IRW's contact record, whose carriers and interfaces name
// bodies GZ18 does not have, is edited one step at a time: an edit is judged
// only by the references it gives.
void VerifyRepairingAMismatchedContactRecord(const ReferenceVehicle& gz18,
                                             const ReferenceVehicle& irw) {
    VehicleProject project = PlacedProject(gz18);
    project.contact_record = irw.contact_record;
    VehicleProjectEditingSession session(std::move(project));
    Require(!session.CheckMechanics().refusals.empty(),
            "GZ18 with IRW's contact record passed the check");

    const ElementReference suspension{ElementKind::kTranslationalSpringDamper,
                                      gz18.definition.translational_spring_dampers.front().name};
    RequireAccepted(session.SetSubjectProperty(suspension, "damping_newton_seconds_per_meter",
                                               Eigen::Vector3d(1.0e3, 2.0e3, 3.0e4)),
                    "a damping edit beside a mismatched contact record");
    const configuration::VehicleContactCarrierDeclaration& irw_carrier =
        irw.contact_record.declaration.carriers.front();
    const ElementReference carrier{ElementKind::kContactCarrier, irw_carrier.carrier_name};
    RequireAccepted(
        session.SetSubjectProperty(
            carrier, "body_name",
            ReferencedElementName{gz18.contact_record.declaration.carriers.front().body_name}),
        "pointing a mismatched carrier at a GZ18 wheelset");
    const ElementReference interface{
        ElementKind::kContactInterface,
        irw.contact_record.declaration.interfaces.back().interface_name};
    RequireAccepted(session.SetSubjectProperty(interface, "interface_name",
                                               ElementOwnName{interface.name + "_renamed"}),
                    "renaming an interface whose wheel body GZ18 does not have");
    const std::vector<ElementReference> removed =
        RequireAccepted(session.RemoveElement(carrier), "removing the repointed carrier");
    Require(
        std::ranges::count(removed, ElementKind::kContactInterface, &ElementReference::kind) ==
            std::ranges::count(irw.contact_record.declaration.interfaces, irw_carrier.carrier_name,
                               &configuration::VehicleContactInterfaceDeclaration::carrier_name),
        "removing a carrier did not take exactly its interfaces");
    Require(!session.CheckMechanics().refusals.empty(),
            "the check stopped reporting the contact record's remaining mismatches");
}

// A body created in the session has no shapes; the bodies the vehicle opened
// with have the shapes stated under their own names.
void VerifyCreatedBodyHasNoShapes(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const ElementReference body{ElementKind::kRigidBody, "created_box"};
    RequireAccepted(
        session.AddRigidBody(body.name,
                             {{"mass_kilograms", 1.0},
                              {"inertia_moments_about_center_of_mass_kilogram_square_meters",
                               Eigen::Vector3d::Ones().eval()}},
                             FreeInWorldAttachment{}),
        "creating a free body");
    Require(FindVisualDefinitionName(session.project(), body.name) == nullptr,
            "a body created in the session has a name its shapes are stated under");
    RequireCheckPasses(session, "GZ18 with a created body");
    for (const vehicle_project_editing::RigidBodyDrawing& drawing :
         session.preview()->rigid_bodies) {
        const std::optional<std::string> expected =
            drawing.body_name == body.name ? std::nullopt : std::optional(drawing.body_name);
        Require(drawing.visual_definition_body_name == expected,
                "body '" + drawing.body_name + "' is drawn with other shapes than its own");
    }
}

// Every axis token is one the check accepts.
void VerifyAxisTokensPassTheCheck(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const ElementReference damper{ElementKind::kSeriesSpringViscousDamper,
                                  gz18.definition.series_spring_viscous_dampers.front().name};
    const auto axis = vehicle_project_editing::DescribeElementProperty(damper.kind, "axis");
    Require(axis.has_value() && !(*axis)->choices.empty(), "the damper's axis states no choices");
    for (const std::string_view token : (*axis)->choices) {
        RequireAccepted(
            session.SetSubjectProperty(damper, "axis", PropertyToken{std::string(token)}),
            "setting the axis '" + std::string(token) + "'");
        RequireCheckPasses(session,
                           "GZ18 with its lateral damper along '" + std::string(token) + "'");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_structural_editing GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const ReferenceVehicle irw =
            editing_verification::LoadReferenceVehicle(argv[4], argv[5], argv[6]);

        VerifyAddingAForceElement(gz18);
        VerifyRemovingBodies(gz18, irw);
        VerifyRenamingARigidBody(gz18);
        VerifyGroupsFollowElements(gz18);
        VerifyStructuralRefusals(gz18);
        VerifyContactElementNeedsContactRecord(gz18);
        VerifyBuildingAThirdAxle(gz18);
        VerifyOptionalReference(gz18);
        VerifyCurve(gz18);
        VerifyRepairingAMismatchedContactRecord(gz18, irw);
        VerifyCreatedBodyHasNoShapes(gz18);
        VerifyAxisTokensPassTheCheck(gz18);
        std::puts(
            "vehicle project structural editing: creation, removal, renaming, reconnection, "
            "refusals and a built third axle passed for GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project structural editing: %s\n", error.what());
        return 1;
    }
}
