#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

#include <cstdint>
#include <type_traits>
#include <vector>

static_assert(std::is_same_v<
              decltype(orvd::vehicle_project_editing::VehicleProjectPreview{}.project_revision),
              std::uint64_t>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::VehicleProjectPreview{}
                                          .not_drawable_elements),
                             std::vector<orvd::vehicle_project::ElementReference>>);
