#include "orvd/scene_observation/wheel_spin_sampling.h"

#include <type_traits>

static_assert(!std::is_default_constructible_v<
              orvd::scene_observation::WheelSpinAngleSampler>);
