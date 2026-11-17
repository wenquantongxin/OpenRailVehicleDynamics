#pragma once

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "orvd/vehicle_project_editing/vehicle_project_titles_in_english_and_chinese.h"

// How a project's rigid bodies are attached, and how a change of attachment
// is stated.
//
// The kernel takes a loop-free tree of rigid bodies: a body moves freely in
// the world, which its record states, or hangs on one joint relating it to
// another body. Here the attachment is read from the records alone, whatever
// a check would say of them. A joint relates two bodies when its two ends name
// frames on two different bodies of the project: a body's own name is its
// frame, and a fixed frame is on the body it names. A joint with an end naming
// no such frame, or with both ends on one body, relates nothing. Bodies
// related directly or through others form a group, read as a whole:
// - joints of the group close a loop: every body of it is conflicting;
// - no loop and exactly one free body: that body is free, and every other
//   body hangs on the one joint on its way to the free body, whichever end of
//   the record is on it;
// - no loop and no free body: every body of it is unattached;
// - no loop and more than one free body: every body of it is conflicting.
//
// A free body can take a joint, except the station layout's reference body,
// which keeps moving freely. A body hanging on a joint can become free, or
// take another joint in place of the one it hangs on. An unattached body can
// become free, or take a joint. Nothing of a conflicting body changes here:
// removing one of the joints concerned, an ordinary removal, repairs it. A
// joint is created only as a body's attachment, its end on the body being its
// child_frame_name and its other end its parent_frame_name; each change is one
// edit of the session (VehicleProjectEditingSession::SetRigidBodyAttachment
// and AddRigidBody).

namespace orvd::vehicle_project_editing {

enum class RigidBodyAttachmentState {
    kFree,
    kHangsOnJoint,
    kUnattached,
    kConflicting,
};

struct RigidBodyAttachment {
    RigidBodyAttachmentState state{RigidBodyAttachmentState::kUnattached};
    // One short line saying how the body is held; it names no element.
    Title summary{};
    // One line more: what is the matter, or why something cannot be changed.
    std::optional<Title> note{};
    // The joint the body hangs on, and the frame it is held at, at that
    // joint's other end: a rigid body or a fixed frame.
    std::optional<vehicle_project::ElementReference> joint{};
    std::optional<vehicle_project::ElementReference> held_at{};
    bool can_become_free{false};
    bool can_take_joint{false};
    // What a new joint's end on the body may name: the body, then its fixed
    // frames.
    std::vector<vehicle_project::ElementReference> frames_on_body{};
    // What a new joint's other end may name: every frame on a body this body
    // does not reach through joints once the joint it hangs on is taken away,
    // body by body in declaration order, each body before its fixed frames.
    std::vector<vehicle_project::ElementReference> frames_it_can_be_held_at{};
};

// The attachment of the rigid body named `body_name`.
//
// Throws std::invalid_argument, naming the body, when the project has no
// rigid body of that name.
[[nodiscard]] RigidBodyAttachment ReadRigidBodyAttachment(
    const vehicle_project::VehicleProject& project, const std::string& body_name);

// What every rigid body's attachment is stated with: its title, the title of
// moving freely, the titles the joint a body hangs on and the frame it is held
// at are shown under, the joint kinds it can take, the joint properties naming
// the end on the body and the other end, and the body's properties only a free
// body has, its station offset and placement offsets.
struct RigidBodyAttachmentDescription {
    Title title;
    Title free_title;
    Title joint_title;
    Title held_at_title;
    std::span<const vehicle_project::ElementKind> joint_kinds;
    std::string_view body_end_property;
    std::string_view other_end_property;
    std::span<const std::string_view> free_only_properties;
};

[[nodiscard]] RigidBodyAttachmentDescription DescribeRigidBodyAttachment();

// How a rigid body is to be attached: free in the world, or by a new joint of
// a joint kind, with its name and properties.
struct FreeInWorldAttachment {};

struct JointAttachment {
    vehicle_project::ElementKind kind{vehicle_project::ElementKind::kRevoluteJoint};
    std::string name;
    PropertyValuesByName properties{};
};

using RigidBodyAttachmentChoice = std::variant<FreeInWorldAttachment, JointAttachment>;

}  // namespace orvd::vehicle_project_editing
