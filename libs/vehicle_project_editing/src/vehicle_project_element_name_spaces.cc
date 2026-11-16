#include "vehicle_project_element_name_spaces.h"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::ElementKind;

// Bodies and frames share the multibody model's frame names, and the vehicle
// force plan refuses one name for two force elements of any of its kinds.
constexpr std::array kBodyAndFrameNameSpace{ElementKind::kRigidBody, ElementKind::kFixedFrame};
constexpr std::array kJointNameSpace{ElementKind::kRevoluteJoint, ElementKind::kBallRpyJoint,
                                     ElementKind::kWeldJoint};
constexpr std::array kForceElementNameSpace{
    ElementKind::kTranslationalSpringDamper,
    ElementKind::kRollSpringDamperCouple,
    ElementKind::kSeriesSpringViscousDamper,
    ElementKind::kPointToPointSeriesSpringBilinearDamper,
    ElementKind::kSaturatedPiecewiseLinearDamper,
    ElementKind::kReferenceFrameLinearBushing,
    ElementKind::kPointToPointLinearSpringDamper,
    ElementKind::kPointToPointHystereticSpring,
};
constexpr std::array kContactCarrierNameSpace{ElementKind::kContactCarrier};
constexpr std::array kContactInterfaceNameSpace{ElementKind::kContactInterface};

constexpr std::array<std::span<const ElementKind>, 5> kNameSpaces{
    kBodyAndFrameNameSpace, kJointNameSpace, kForceElementNameSpace, kContactCarrierNameSpace,
    kContactInterfaceNameSpace};

// The name the multibody model gives the world's own body and frame
// (libs/multibody_model/src/multibody_model.cc), which it exports no constant
// for. No rigid body or fixed frame may take it.
constexpr std::string_view kWorldName = "world";

std::span<const ElementKind> NameSpaceOf(ElementKind kind) {
    for (const std::span<const ElementKind> name_space : kNameSpaces) {
        if (std::ranges::find(name_space, kind) != name_space.end()) {
            return name_space;
        }
    }
    throw std::logic_error("element kind " + std::to_string(std::to_underlying(kind)) +
                           " belongs to no name space");
}

}  // namespace

bool ContainsElementOfKinds(const vehicle_project::VehicleProject& project,
                            std::span<const ElementKind> kinds, const std::string& name) {
    return std::ranges::any_of(kinds, [&project, &name](ElementKind kind) {
        return vehicle_project::ContainsElement(project, {kind, name});
    });
}

std::expected<void, std::string> RequireNameAvailable(
    const vehicle_project::VehicleProject& project, ElementKind kind, const std::string& name) {
    if (name.empty()) {
        return std::unexpected("an element's name cannot be empty");
    }
    const std::span<const ElementKind> name_space = NameSpaceOf(kind);
    if (name == kWorldName && name_space.data() == kBodyAndFrameNameSpace.data()) {
        return std::unexpected(
            "'world' is the name of the world's own body and frame, which no rigid body or fixed "
            "frame can take");
    }
    for (const ElementKind sharing_kind : name_space) {
        if (vehicle_project::ContainsElement(project, {sharing_kind, name})) {
            return std::unexpected("'" + name + "' already names " +
                                   vehicle_project::DescribeElement({sharing_kind, name}));
        }
    }
    return {};
}

}  // namespace orvd::vehicle_project_editing::internal
