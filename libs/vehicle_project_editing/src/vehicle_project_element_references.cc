#include "vehicle_project_element_references.h"

#include <algorithm>
#include <optional>
#include <span>
#include <variant>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_property_tables.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::ElementKind;
using vehicle_project::ElementReference;

template <typename Member>
constexpr bool kIsReferenceMember = false;

template <typename Record, typename Name>
constexpr bool kIsReferenceMember<ReferenceMember<Record, Name>> = true;

bool NamesTarget(std::span<const ElementKind> referenced_kinds, const std::string& name,
                 const ElementReference& target) {
    return name == target.name &&
           std::ranges::find(referenced_kinds, target.kind) != referenced_kinds.end();
}

bool NamesTarget(std::span<const ElementKind> referenced_kinds,
                 const std::optional<std::string>& name, const ElementReference& target) {
    return name.has_value() && NamesTarget(referenced_kinds, *name, target);
}

// Calls `visit(holder, name, referenced_kinds)` for every reference of every
// element of the project: the element holding it, the name it holds, which
// shares the project's constness, and the kinds it may name.
template <vehicle_project::ConstOrMutableVehicleProject Project, typename Visitor>
void ForEachReference(Project& project, Visitor&& visit) {
    for (const ElementKind kind : vehicle_project::kElementKinds) {
        vehicle_project::VisitElementKind(kind, [&]<typename KindRecords>(KindRecords) {
            using Record = typename KindRecords::Record;
            auto* const records = KindRecords::Records(project);
            if (records == nullptr) {
                return;
            }
            for (auto& record : *records) {
                const ElementReference holder{kind, record.*KindRecords::kElementName};
                for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
                    std::visit(
                        [&]<typename Member>(const Member& member) {
                            if constexpr (kIsReferenceMember<Member>) {
                                visit(holder, record.*member.member, member.referenced_kinds);
                            }
                        },
                        property.member);
                }
            }
        });
    }
}

}  // namespace

std::vector<ElementReference> CollectReferringElements(
    const vehicle_project::VehicleProject& project, const ElementReference& target) {
    std::vector<ElementReference> referring;
    ForEachReference(project,
                     [&referring, &target](const ElementReference& holder, const auto& name,
                                           std::span<const ElementKind> referenced_kinds) {
                         if (NamesTarget(referenced_kinds, name, target) &&
                             std::ranges::find(referring, holder) == referring.end()) {
                             referring.push_back(holder);
                         }
                     });
    return referring;
}

std::vector<ElementReference> CollectReferringElementsUnableToName(
    const vehicle_project::VehicleProject& project, const ElementReference& target,
    ElementKind kind) {
    std::vector<ElementReference> unable;
    ForEachReference(project,
                     [&unable, &target, kind](const ElementReference& holder, const auto& name,
                                              std::span<const ElementKind> referenced_kinds) {
                         if (NamesTarget(referenced_kinds, name, target) &&
                             std::ranges::find(referenced_kinds, kind) == referenced_kinds.end() &&
                             std::ranges::find(unable, holder) == unable.end()) {
                             unable.push_back(holder);
                         }
                     });
    return unable;
}

void RedirectReferences(vehicle_project::VehicleProject& project, const ElementReference& target,
                        const std::string& new_name) {
    ForEachReference(project, [&target, &new_name](const ElementReference&, auto& name,
                                                   std::span<const ElementKind> referenced_kinds) {
        if (NamesTarget(referenced_kinds, name, target)) {
            name = new_name;
        }
    });
}

}  // namespace orvd::vehicle_project_editing::internal
