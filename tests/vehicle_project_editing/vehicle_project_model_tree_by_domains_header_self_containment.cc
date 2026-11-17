#include "orvd/vehicle_project_editing/vehicle_project_model_tree_by_domains.h"

#include <type_traits>
#include <utility>
#include <vector>

static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::DescribeModelTree(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>())),
                             std::vector<orvd::vehicle_project_editing::ModelTreeNode>>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::ElementKindTitle(
                                 std::declval<orvd::vehicle_project::ElementKind>())),
                             orvd::vehicle_project_editing::Title>);
