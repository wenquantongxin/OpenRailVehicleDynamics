#include "vehicle_project_element_creation.h"

#include <algorithm>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_name_spaces.h"
#include "vehicle_project_element_property_assignment.h"

namespace orvd::vehicle_project_editing::internal {

std::expected<vehicle_project::ElementReference, SubjectEditRefusal> CreateElement(
    vehicle_project::VehicleProject& project, vehicle_project::ElementKind kind,
    const std::string& name, const PropertyValuesByName& properties) {
    const vehicle_project::ElementReference element{kind, name};
    const auto refuse = [&element](std::optional<std::string> property_name, std::string reason) {
        return std::unexpected(
            SubjectEditRefusal{element, std::move(property_name), std::move(reason)});
    };
    const std::string kind_description(vehicle_project::DescribeElementKind(kind));
    const std::span<const PropertyDescription> descriptions = DescribeElementProperties(kind);

    const bool has_records =
        vehicle_project::VisitElementKind(kind, [&project]<typename KindRecords>(KindRecords) {
            return KindRecords::Records(project) != nullptr;
        });
    if (!has_records) {
        return refuse(std::nullopt, "a " + kind_description +
                                        " belongs to the project's wheel-rail contact record, "
                                        "which has not been provided");
    }
    // The element's own name is the one property every kind has exactly one
    // of.
    const auto own_name = std::ranges::find(descriptions, PropertyValueKind::kElementName,
                                            &PropertyDescription::value_kind);
    const std::expected<void, std::string> available = RequireNameAvailable(project, kind, name);
    if (!available.has_value()) {
        return refuse(std::string(own_name->name), available.error());
    }
    for (const auto& [property_name, value] : properties) {
        const std::expected<const PropertyDescription*, std::string> described =
            DescribeElementProperty(kind, property_name);
        if (!described.has_value()) {
            return refuse(property_name, described.error());
        }
        if ((*described)->value_kind == PropertyValueKind::kElementName) {
            return refuse(property_name,
                          "the element is named by the name it is created with, not by '" +
                              property_name + "' among its properties");
        }
    }
    for (const PropertyDescription& description : descriptions) {
        if (description.value_kind == PropertyValueKind::kElementReference &&
            !description.may_name_none && !properties.contains(description.name)) {
            return refuse(std::string(description.name),
                          "'" + std::string(description.name) + "' must be given: a " +
                              kind_description + " names a " +
                              DescribeReferencedKinds(description.referenced_kinds));
        }
    }

    vehicle_project::VisitElementKind(kind, [&project, &name]<typename KindRecords>(KindRecords) {
        typename KindRecords::Record record{};
        record.*KindRecords::kElementName = name;
        KindRecords::Records(project)->push_back(std::move(record));
    });
    for (const PropertyDescription& description : descriptions) {
        const auto given = properties.find(description.name);
        if (given == properties.end()) {
            continue;
        }
        const auto assigned =
            AssignElementProperty(project, element, description.name, given->second);
        if (!assigned.has_value()) {
            return std::unexpected(assigned.error());
        }
    }
    return element;
}

}  // namespace orvd::vehicle_project_editing::internal
