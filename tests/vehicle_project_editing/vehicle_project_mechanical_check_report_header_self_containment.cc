#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"

#include <string>
#include <type_traits>
#include <vector>

static_assert(
    std::is_same_v<decltype(orvd::vehicle_project_editing::MechanicalCheckReport{}.refusals),
                   std::vector<std::string>>);
static_assert(std::is_same_v<
              decltype(orvd::vehicle_project_editing::MechanicalCheckReport{}.coverage_statements),
              std::vector<std::string>>);
