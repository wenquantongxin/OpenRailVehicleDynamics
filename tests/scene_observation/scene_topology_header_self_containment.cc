#include "orvd/scene_observation/scene_topology.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::scene_observation::DescribeSceneTopology(
                  std::declval<const orvd::multibody_model::MultibodyModel&>(),
                  nullptr)),
              orvd::scene_observation::SceneTopology>);
