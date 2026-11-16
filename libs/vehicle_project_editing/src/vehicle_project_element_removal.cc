#include "vehicle_project_element_removal.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <string>
#include <utility>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_references.h"
#include "vehicle_project_records_following_elements.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;

// Where the element stands among its kind's records.
std::size_t DeclarationIndex(const VehicleProject& project, const ElementReference& element) {
    return vehicle_project::VisitElementKind(
        element.kind, [&project, &element]<typename KindRecords>(KindRecords) {
            const auto& records = *KindRecords::Records(project);
            return static_cast<std::size_t>(
                std::distance(records.begin(),
                              std::ranges::find(records, element.name, KindRecords::kElementName)));
        });
}

}  // namespace

std::expected<std::vector<ElementReference>, ElementEditRefusal> CollectElementsRemovedWith(
    const VehicleProject& project, const ElementReference& element) {
    const auto refuse = [&element](std::string reason) {
        return std::unexpected(ElementEditRefusal{element, std::nullopt, std::move(reason)});
    };
    if (!vehicle_project::ContainsElement(project, element)) {
        return refuse("this project contains no " + vehicle_project::DescribeElement(element));
    }
    if (element.kind == ElementKind::kRigidBody) {
        const std::expected<void, std::string> kept =
            RequireLayoutKeepsReferenceBody(project, element.name, "removed");
        if (!kept.has_value()) {
            return refuse(kept.error());
        }
    }

    std::vector<ElementReference> removed{element};
    for (std::size_t next = 0; next < removed.size(); ++next) {
        for (ElementReference& referring : CollectReferringElements(project, removed[next])) {
            if (std::ranges::find(removed, referring) == removed.end()) {
                removed.push_back(std::move(referring));
            }
        }
    }
    std::ranges::sort(removed, {}, [&project](const ElementReference& taken) {
        return std::pair(std::to_underlying(taken.kind), DeclarationIndex(project, taken));
    });
    return removed;
}

void RemoveElements(VehicleProject& project, std::span<const ElementReference> elements) {
    for (const ElementReference& element : elements) {
        vehicle_project::VisitElementKind(
            element.kind, [&project, &element]<typename KindRecords>(KindRecords) {
                std::erase_if(*KindRecords::Records(project), [&element](const auto& record) {
                    return record.*KindRecords::kElementName == element.name;
                });
            });
        RemoveRecordsFollowingElement(project, element);
    }
}

}  // namespace orvd::vehicle_project_editing::internal
