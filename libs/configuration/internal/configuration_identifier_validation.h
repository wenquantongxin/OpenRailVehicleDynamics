#pragma once

#include <string>
#include <string_view>

namespace orvd::configuration::internal {

void RequireConfigurationIdentifier(const std::string& identifier, const std::string& path,
                                    std::string_view what);

}  // namespace orvd::configuration::internal
