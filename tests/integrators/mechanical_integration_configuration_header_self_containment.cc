#include "orvd/integrators/mechanical_integration_configuration.h"

#include <type_traits>

static_assert(std::is_trivially_copyable_v<orvd::integrators::NewmarkConfiguration>);
static_assert(std::is_trivially_copyable_v<orvd::integrators::ZhaiConfiguration>);
