#include "orvd/scene_observation/body_state.h"

#include <type_traits>
#include <utility>
#include <vector>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::scene_observation::
                                        BodyStateSampler&>()
                           .Sample(std::declval<const orvd::multibody_model::
                                                    MultibodyEvaluationContext&>())),
              std::vector<orvd::scene_observation::BodyState>>);
static_assert(sizeof(orvd::scene_observation::BodyState) ==
              13 * sizeof(double));
