#include "orvd/vehicle_project/vehicle_project_invariants.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "orvd/vehicle_project/element_kind_records.h"

namespace orvd::vehicle_project {
namespace {

// The criterion a resolved start-up state applies to its rotations: a few
// rounding steps of the four-term norm, not a repair budget.
constexpr double kUnitQuaternionNormTolerance = 1.0e-12;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("vehicle project." + detail);
}

void RequireFinite(double value, const std::string& what) {
    if (!std::isfinite(value)) {
        Reject(what + " is " + std::to_string(value) + ", which is not finite");
    }
}

void RequirePlacementInvariants(const VehicleProject& project) {
    std::unordered_set<std::string> free_body_names;
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        if (body.moves_freely_in_world) {
            free_body_names.insert(body.name);
        }
    }

    std::unordered_set<std::string> placed_body_names;
    for (const FreeBodyPlacement& placement : project.free_body_placements) {
        const std::string what = "free_body_placements['" + placement.body_name + "']";
        if (!free_body_names.contains(placement.body_name)) {
            Reject(what +
                   " names a body this vehicle definition does not declare as moving freely in "
                   "the world");
        }
        if (!placed_body_names.insert(placement.body_name).second) {
            Reject("free_body_placements repeats '" + placement.body_name + "'");
        }
        RequireFinite(placement.lateral_offset_in_local_track_frame_meters,
                      what + ".lateral_offset_in_local_track_frame_meters");
        RequireFinite(placement.vertical_offset_in_local_track_frame_meters,
                      what + ".vertical_offset_in_local_track_frame_meters");
        if (!(std::abs(placement.rotation_local_track_from_body.norm() - 1.0) <=
              kUnitQuaternionNormTolerance)) {
            Reject(what +
                   ".rotation_local_track_from_body is not a unit quaternion; it is refused "
                   "rather than normalised, because normalising would draw a rotation nobody "
                   "stated");
        }
    }
}

// Checks one level of the grouping tree and everything below it. `path`
// locates `groups` in the project. `containing_group_of_element` maps every
// member met so far, by kind and name, to the path of its group.
void RequireGroupInvariants(
    const VehicleProject& project, const std::vector<ElementGroup>& groups, const std::string& path,
    std::map<std::pair<ElementKind, std::string>, std::string>& containing_group_of_element) {
    std::unordered_set<std::string> sibling_names;
    for (std::size_t index = 0; index < groups.size(); ++index) {
        const ElementGroup& group = groups[index];
        if (group.name.empty()) {
            Reject(path + "[" + std::to_string(index) + "].name is empty");
        }
        if (!sibling_names.insert(group.name).second) {
            Reject(path + " repeats the group name '" + group.name + "'");
        }
        const std::string group_path = path + "['" + group.name + "']";
        for (const ElementReference& member : group.members) {
            // How a diagnostic names the member's kind, and whether the
            // project contains the member.
            const auto [kind_description, contained] = VisitElementKind(
                member.kind, [&project, &member]<typename KindRecords>(KindRecords) {
                    const auto* const records = KindRecords::Records(project);
                    return std::pair{
                        KindRecords::kKindDescription,
                        records != nullptr &&
                            std::ranges::find(*records, member.name, KindRecords::kElementName) !=
                                records->end()};
                });
            const std::string element = std::string(kind_description) + " '" + member.name + "'";
            if (!contained) {
                Reject(group_path + ".members names " + element +
                       ", which this project does not contain");
            }
            const auto [containing_group, first_membership] = containing_group_of_element.emplace(
                std::pair(member.kind, member.name), group_path);
            if (!first_membership) {
                Reject(group_path + ".members names " + element + ", which " +
                       containing_group->second +
                       " already contains; an element belongs to at most one group");
            }
        }
        RequireGroupInvariants(project, group.subgroups, group_path + ".subgroups",
                               containing_group_of_element);
    }
}

}  // namespace

void RequireVehicleProjectInvariants(const VehicleProject& project) {
    RequirePlacementInvariants(project);
    std::map<std::pair<ElementKind, std::string>, std::string> containing_group_of_element;
    RequireGroupInvariants(project, project.element_groups, "element_groups",
                           containing_group_of_element);
}

}  // namespace orvd::vehicle_project
