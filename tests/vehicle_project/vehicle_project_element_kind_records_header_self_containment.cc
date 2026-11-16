#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"

#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(std::is_same_v<orvd::vehicle_project::ElementKindRecords<
                                 orvd::vehicle_project::ElementKind::kRigidBody>::Record,
                             orvd::configuration::VehicleRigidBodyDefinition>);
static_assert(std::is_same_v<
              decltype(orvd::vehicle_project::ElementKindRecords<
                       orvd::vehicle_project::ElementKind::kContactCarrier>::
                           Records(std::declval<const orvd::vehicle_project::VehicleProject&>())),
              const std::vector<orvd::configuration::VehicleContactCarrierDeclaration>*>);
static_assert(
    std::is_same_v<decltype(orvd::vehicle_project::ElementKindRecords<
                            orvd::vehicle_project::ElementKind::kWeldJoint>::
                                Records(std::declval<orvd::vehicle_project::VehicleProject&>())),
                   std::vector<orvd::configuration::VehicleWeldJointDefinition>*>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project::VisitElementKind(
                                 std::declval<orvd::vehicle_project::ElementKind>(),
                                 []<typename KindRecords>(KindRecords) {
                                     return KindRecords::kKindDescription;
                                 })),
                             std::string_view>);
