#pragma once

#include <string>

#include "orvd/configuration/point_to_point_spring_definitions.h"
#include "strict_json.h"

namespace orvd::configuration::internal {

PointToPointLinearSpringDamperDefinition ParsePointToPointLinearSpringDamper(
    const strict_json::Json& value, const std::string& path);
PointToPointHystereticSpringDefinition ParsePointToPointHystereticSpring(
    const strict_json::Json& value, const std::string& path);

}  // namespace orvd::configuration::internal
