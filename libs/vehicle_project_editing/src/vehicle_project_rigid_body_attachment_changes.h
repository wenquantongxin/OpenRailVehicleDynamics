#pragma once

#include <expected>
#include <optional>
#include <string>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

// Changing how a rigid body is attached, and creating a rigid body with its
// attachment: the one way a body's statement that it moves freely in the
// world changes, together with the records a free body has, and the one way a
// joint is created. A change takes the attachment it starts from as reading
// gives it (vehicle_project_rigid_body_attachments.h). A refusal may leave the
// project partly changed: the session edits a copy and drops it.

namespace orvd::vehicle_project_editing::internal {

// Changes how the rigid body named `body_name` is attached, checked as
// VehicleProjectEditingSession::SetRigidBodyAttachment states, and returns the
// joint it hangs on afterwards: none when it moves freely.
[[nodiscard]] std::expected<std::optional<vehicle_project::ElementReference>, SubjectEditRefusal>
ChangeRigidBodyAttachment(vehicle_project::VehicleProject& project, const std::string& body_name,
                          const RigidBodyAttachmentChoice& choice);

// Appends a new rigid body named `name` with `properties`, attached as
// `choice` states, checked as VehicleProjectEditingSession::AddRigidBody
// states.
[[nodiscard]] std::expected<vehicle_project::ElementReference, SubjectEditRefusal>
CreateRigidBodyWithAttachment(vehicle_project::VehicleProject& project, const std::string& name,
                              const PropertyValuesByName& properties,
                              const RigidBodyAttachmentChoice& choice);

// Why VehicleProjectEditingSession::AddElement does not create an element of
// `kind`: a rigid body is created with its attachment, and a joint only as a
// rigid body's attachment. None for every other kind.
[[nodiscard]] std::optional<std::string> RefuseCreationWithoutAttachment(
    vehicle_project::ElementKind kind);

}  // namespace orvd::vehicle_project_editing::internal
