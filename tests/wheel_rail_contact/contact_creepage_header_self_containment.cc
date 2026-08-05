#include "orvd/wheel_rail_contact/contact_creepage.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::ComputeCreepages(
                  std::declval<const orvd::wheel_rail_contact::ContactRelativeMotion&>(),
                  std::declval<const orvd::wheel_rail_contact::ContactFrame&>(),
                  std::declval<double>(),
                  std::declval<const orvd::wheel_rail_contact::CreepageConfiguration&>())),
              orvd::wheel_rail_contact::Creepages>);
