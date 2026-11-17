#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"

#include <string>
#include <type_traits>
#include <utility>
#include <variant>

static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::ReadRigidBodyAttachment(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>(),
                                 std::declval<const std::string&>())),
                             orvd::vehicle_project_editing::RigidBodyAttachment>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::DescribeRigidBodyAttachment()),
                             orvd::vehicle_project_editing::RigidBodyAttachmentDescription>);
static_assert(std::is_same_v<orvd::vehicle_project_editing::RigidBodyAttachmentChoice,
                             std::variant<orvd::vehicle_project_editing::FreeInWorldAttachment,
                                          orvd::vehicle_project_editing::JointAttachment>>);
