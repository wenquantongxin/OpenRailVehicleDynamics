#include "orvd/configuration/assembled_vehicle_contact_scenario.h"

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public vehicle-contact scenario header leaks the JSON implementation"
#endif

namespace {
[[maybe_unused]] void HeaderIsSelfContained(
    const orvd::configuration::AssembledVehicleContactScenario& scenario) {
    (void)scenario.vehicle_system();
    (void)scenario.initial_context();
}
}  // namespace
