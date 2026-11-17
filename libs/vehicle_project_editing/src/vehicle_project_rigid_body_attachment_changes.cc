#include "vehicle_project_rigid_body_attachment_changes.h"

#include <algorithm>
#include <cstddef>
#include <span>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_creation.h"
#include "vehicle_project_element_name_spaces.h"
#include "vehicle_project_element_property_assignment.h"
#include "vehicle_project_element_references.h"
#include "vehicle_project_element_removal.h"
#include "vehicle_project_records_following_elements.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;

using AttachmentChange = std::expected<std::optional<ElementReference>, SubjectEditRefusal>;

bool IsJointKind(ElementKind kind) {
    return std::ranges::find(kJointKinds, kind) != kJointKinds.end();
}

// How a reason names several elements: "contact interface 'a' and contact
// interface 'b'".
std::string DescribeElements(const std::vector<ElementReference>& elements) {
    std::string described;
    for (std::size_t index = 0; index < elements.size(); ++index) {
        if (index > 0) {
            described += index + 1 == elements.size() ? " and " : ", ";
        }
        described += vehicle_project::DescribeElement(elements[index]);
    }
    return described;
}

// The body moves freely in the world, with the records a free body has, or no
// longer does, without them.
void StateMovesFreely(VehicleProject& project, const std::string& body_name, bool moves_freely) {
    vehicle_project::FindElementRecord<
        vehicle_project::ElementKindRecords<ElementKind::kRigidBody>>(project, body_name)
        ->moves_freely_in_world = moves_freely;
    if (moves_freely) {
        AddFreeBodyRecords(project, body_name);
    } else {
        RemoveFreeBodyRecords(project, body_name);
    }
}

// Why nothing of a conflicting body changes here.
std::string RefuseConflictingAttachment(const ElementReference& body,
                                        const RigidBodyAttachment& attachment) {
    return "the attachment of " + vehicle_project::DescribeElement(body) +
           " is unclear, so it changes only once a joint is removed: " + attachment.note->en;
}

// The frame a new joint's end names, among its properties, which creating the
// joint has required to be a reference naming a frame.
std::string FrameNamedBy(const PropertyValuesByName& properties, std::string_view end) {
    return *std::get<ReferencedElementName>(properties.find(end)->second).name;
}

bool Lists(const std::vector<ElementReference>& frames, const std::string& frame_name) {
    return std::ranges::find(frames, frame_name, &ElementReference::name) != frames.end();
}

AttachmentChange BecomeFree(VehicleProject& project, const ElementReference& body,
                            const RigidBodyAttachment& attachment) {
    const auto refuse = [&body](std::string reason) {
        return std::unexpected(SubjectEditRefusal{body, std::nullopt, std::move(reason)});
    };
    if (attachment.state == RigidBodyAttachmentState::kFree) {
        return refuse(vehicle_project::DescribeElement(body) +
                      " already moves freely in the world");
    }
    if (attachment.state == RigidBodyAttachmentState::kConflicting) {
        return refuse(RefuseConflictingAttachment(body, attachment));
    }
    if (attachment.joint.has_value()) {
        // Nothing is removed behind the user's back: what names the joint
        // goes with it only through an ordinary removal.
        const std::vector<ElementReference> naming =
            CollectReferringElements(project, *attachment.joint);
        if (!naming.empty()) {
            return refuse(DescribeElements(naming) + (naming.size() == 1 ? " names " : " name ") +
                          vehicle_project::DescribeElement(*attachment.joint) + ", which " +
                          vehicle_project::DescribeElement(body) +
                          " hangs on; removing that joint removes what names it as well");
        }
        RemoveElements(project, std::span(&*attachment.joint, 1));
    }
    StateMovesFreely(project, body.name, true);
    return std::nullopt;
}

AttachmentChange TakeJoint(VehicleProject& project, const ElementReference& body,
                           const RigidBodyAttachment& attachment, const JointAttachment& joint) {
    const auto refuse = [&body](std::string reason) {
        return std::unexpected(SubjectEditRefusal{body, std::nullopt, std::move(reason)});
    };
    const ElementReference new_joint{joint.kind, joint.name};
    const auto refuse_joint = [&new_joint](std::string_view property_name, std::string reason) {
        return std::unexpected(
            SubjectEditRefusal{new_joint, std::string(property_name), std::move(reason)});
    };
    if (attachment.state == RigidBodyAttachmentState::kConflicting) {
        return refuse(RefuseConflictingAttachment(body, attachment));
    }
    if (!attachment.can_take_joint) {
        return refuse(
            RequireLayoutKeepsReferenceBody(project, body.name, "attached by a joint").error());
    }
    if (!IsJointKind(joint.kind)) {
        return refuse("a rigid body is attached by a " + DescribeReferencedKinds(kJointKinds) +
                      ", not by a " +
                      std::string(vehicle_project::DescribeElementKind(joint.kind)));
    }
    if (attachment.joint.has_value()) {
        // The joint it hangs on goes first, so that the new one may take its
        // name; what names it must be able to name the new one.
        const std::vector<ElementReference> unable =
            CollectReferringElementsUnableToName(project, *attachment.joint, joint.kind);
        if (!unable.empty()) {
            return refuse(DescribeElements(unable) + (unable.size() == 1 ? " names " : " name ") +
                          vehicle_project::DescribeElement(*attachment.joint) +
                          ", which the new joint replaces, and cannot name a " +
                          std::string(vehicle_project::DescribeElementKind(joint.kind)));
        }
        RemoveElements(project, std::span(&*attachment.joint, 1));
    }
    const std::expected<ElementReference, SubjectEditRefusal> created =
        CreateElement(project, joint.kind, joint.name, joint.properties);
    if (!created.has_value()) {
        return std::unexpected(created.error());
    }
    const RigidBodyAttachmentDescription description = DescribeRigidBodyAttachment();
    const std::string body_end = FrameNamedBy(joint.properties, description.body_end_property);
    if (!Lists(attachment.frames_on_body, body_end)) {
        return refuse_joint(description.body_end_property,
                            "the joint's end on " + vehicle_project::DescribeElement(body) +
                                " names '" + body_end +
                                "', which is neither the body nor one of its fixed frames");
    }
    const std::string other_end = FrameNamedBy(joint.properties, description.other_end_property);
    if (!Lists(attachment.frames_it_can_be_held_at, other_end)) {
        return refuse_joint(description.other_end_property,
                            "the joint's other end names '" + other_end + "', which is on " +
                                vehicle_project::DescribeElement(body) +
                                " or on a body it reaches through joints; holding it there "
                                "would close a loop");
    }
    if (attachment.joint.has_value()) {
        RedirectReferences(project, *attachment.joint, joint.name);
    }
    if (attachment.state == RigidBodyAttachmentState::kFree) {
        StateMovesFreely(project, body.name, false);
    }
    return new_joint;
}

}  // namespace

AttachmentChange ChangeRigidBodyAttachment(VehicleProject& project, const std::string& body_name,
                                           const RigidBodyAttachmentChoice& choice) {
    const ElementReference body{ElementKind::kRigidBody, body_name};
    if (!vehicle_project::ContainsElement(project, body)) {
        return std::unexpected(SubjectEditRefusal{
            body, std::nullopt,
            "this project contains no " + vehicle_project::DescribeElement(body)});
    }
    const RigidBodyAttachment attachment = ReadRigidBodyAttachment(project, body_name);
    if (const auto* const joint = std::get_if<JointAttachment>(&choice)) {
        return TakeJoint(project, body, attachment, *joint);
    }
    return BecomeFree(project, body, attachment);
}

std::expected<ElementReference, SubjectEditRefusal> CreateRigidBodyWithAttachment(
    VehicleProject& project, const std::string& name, const PropertyValuesByName& properties,
    const RigidBodyAttachmentChoice& choice) {
    const ElementReference body{ElementKind::kRigidBody, name};
    const RigidBodyAttachmentDescription description = DescribeRigidBodyAttachment();
    // The properties only a free body has are set once the body is attached.
    PropertyValuesByName record_properties;
    PropertyValuesByName free_only_properties;
    for (const auto& [property_name, value] : properties) {
        (std::ranges::find(description.free_only_properties, property_name) !=
                 description.free_only_properties.end()
             ? free_only_properties
             : record_properties)
            .emplace(property_name, value);
    }
    const std::expected<ElementReference, SubjectEditRefusal> created =
        CreateElement(project, ElementKind::kRigidBody, name, record_properties);
    if (!created.has_value()) {
        return created;
    }
    RigidBodyAttachmentChoice attached = choice;
    if (auto* const joint = std::get_if<JointAttachment>(&attached)) {
        // The new joint's end on the body names the new body.
        if (joint->properties.contains(description.body_end_property)) {
            return std::unexpected(SubjectEditRefusal{
                ElementReference{joint->kind, joint->name},
                std::string(description.body_end_property),
                "the joint's end on the new body names the new body; it is not given"});
        }
        joint->properties.emplace(description.body_end_property, ReferencedElementName{name});
    }
    const AttachmentChange changed = ChangeRigidBodyAttachment(project, name, attached);
    if (!changed.has_value()) {
        return std::unexpected(changed.error());
    }
    for (const std::string_view property_name : description.free_only_properties) {
        const auto given = free_only_properties.find(property_name);
        if (given == free_only_properties.end()) {
            continue;
        }
        const std::expected<ElementReference, SubjectEditRefusal> assigned =
            AssignElementProperty(project, body, property_name, given->second);
        if (!assigned.has_value()) {
            return assigned;
        }
    }
    return body;
}

std::optional<std::string> RefuseCreationWithoutAttachment(ElementKind kind) {
    if (kind == ElementKind::kRigidBody) {
        return "a rigid body is created with its attachment: free in the world, or by a joint";
    }
    if (IsJointKind(kind)) {
        return "a " + std::string(vehicle_project::DescribeElementKind(kind)) +
               " is created only as a rigid body's attachment";
    }
    return std::nullopt;
}

}  // namespace orvd::vehicle_project_editing::internal
