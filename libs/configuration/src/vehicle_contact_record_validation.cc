#include "vehicle_contact_record_validation.h"

#include "configuration_identifier_validation.h"

namespace orvd::configuration::internal {

void RequireVehicleContactRecordInvariants(const VehicleContactRecord& record,
                                           const std::string& path) {
    RequireConfigurationIdentifier(record.wheel.profile.profile_identifier,
                                   path + ".wheel.profile.profile_identifier",
                                   "profile identifier");
    RequireConfigurationIdentifier(record.rail.profile.profile_identifier,
                                   path + ".rail.profile.profile_identifier", "profile identifier");
    RequireConfigurationIdentifier(record.computation.contact_strategy_identifier,
                                   path + ".computation.contact_strategy_identifier",
                                   "contact strategy identifier");
}

}  // namespace orvd::configuration::internal
