#include "orvd/configuration/assembled_gz18_contact_scenario.h"

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public GZ18 scenario header leaks the JSON implementation"
#endif

namespace {
[[maybe_unused]] void HeaderIsSelfContained(
    const orvd::configuration::AssembledGz18ContactScenario& scenario) {
    (void)scenario.vehicle_system();
    (void)scenario.initial_context();
}
}  // namespace
