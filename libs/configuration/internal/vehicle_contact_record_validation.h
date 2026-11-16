#pragma once

#include <string>

#include "orvd/configuration/vehicle_contact_record.h"

namespace orvd::configuration::internal {

// Identifier requirements of the formal contact-file loader.
void RequireVehicleContactRecordInvariants(const VehicleContactRecord& record,
                                           const std::string& path);

}  // namespace orvd::configuration::internal
