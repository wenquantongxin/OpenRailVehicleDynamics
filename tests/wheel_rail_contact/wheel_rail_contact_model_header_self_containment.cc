#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"

#include <type_traits>
#include <utility>

template <typename Model>
concept BorrowsGeometryFromRvalue = requires(Model&& model) {
    std::forward<Model>(model).geometry();
};

static_assert(std::is_aggregate_v<
              orvd::wheel_rail_contact::WheelProfileRigidMotion>);
static_assert(!BorrowsGeometryFromRvalue<
              const orvd::wheel_rail_contact::WheelRailContactModel>);

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::WheelRailContactModel&>()
                           .Evaluate(std::declval<const orvd::wheel_rail_contact::WheelRailContactInput&>(),
                                     std::declval<orvd::wheel_rail_contact::WheelRailContactWorkspace&>())),
              orvd::wheel_rail_contact::WheelRailContactResult>);
