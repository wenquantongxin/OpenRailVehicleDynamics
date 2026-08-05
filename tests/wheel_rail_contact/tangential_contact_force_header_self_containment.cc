#include "orvd/wheel_rail_contact/tangential_contact_force.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::TangentialContactSolver&>()
                           .Solve(std::declval<const orvd::wheel_rail_contact::TangentialContactPatch&>(),
                                  std::declval<const orvd::wheel_rail_contact::Creepages&>(),
                                  std::declval<orvd::wheel_rail_contact::TangentialContactWorkspace&>())),
              orvd::wheel_rail_contact::TangentialContactResult>);
