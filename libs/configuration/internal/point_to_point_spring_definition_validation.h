#pragma once

#include <string>

#include "orvd/configuration/point_to_point_spring_definitions.h"

namespace orvd::configuration::internal {

void RequirePointToPointLinearSpringDamperInvariants(
    const PointToPointLinearSpringDamperDefinition& element, const std::string& path);
void RequirePointToPointHystereticSpringInvariants(
    const PointToPointHystereticSpringDefinition& element, const std::string& path);

}  // namespace orvd::configuration::internal
