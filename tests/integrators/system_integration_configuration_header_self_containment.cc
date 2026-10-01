#include "orvd/integrators/system_integration_configuration.h"

#include <type_traits>
#include <variant>

static_assert(std::variant_size_v<orvd::integrators::SystemIntegrationMethodConfiguration> == 5);
static_assert(std::is_copy_constructible_v<orvd::integrators::SystemIntegrationConfiguration>);
static_assert(!std::is_default_constructible_v<orvd::integrators::SystemIntegrationConfiguration>);
