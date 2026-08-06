#include "orvd/wheel_rail_contact/contact_geometry.h"

#include <type_traits>
#include <utility>

template <typename Solver>
concept ExposesConfigurationFromRvalue = requires(Solver&& solver) {
    std::forward<Solver>(solver).configuration();
};

template <typename Solver>
concept ExposesNodeViewFromRvalue = requires(Solver&& solver) {
    std::forward<Solver>(solver).wheel_node_lateral_meters();
};

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::ContactGeometrySolver&>()
                           .Solve(std::declval<const orvd::wheel_rail_contact::ContactPoseScalars&>(),
                                  std::declval<orvd::wheel_rail_contact::ContactGeometryWorkspace&>())),
              orvd::wheel_rail_contact::ContactPatchSet>);

static_assert(!ExposesConfigurationFromRvalue<
              const orvd::wheel_rail_contact::ContactGeometrySolver>);
static_assert(!ExposesNodeViewFromRvalue<
              const orvd::wheel_rail_contact::ContactGeometrySolver>);
