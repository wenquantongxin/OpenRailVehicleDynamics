#include "orvd/scene_observation/scalar_definition.h"

#include <type_traits>

static_assert(static_cast<int>(orvd::scene_observation::ScalarStatus::kNotReady) == 0);
static_assert(static_cast<int>(orvd::scene_observation::ScalarStatus::kValid) == 1);
static_assert(static_cast<int>(orvd::scene_observation::ScalarStatus::kPlaceholder) == 2);
static_assert(std::is_aggregate_v<orvd::scene_observation::ScalarDefinition>);
