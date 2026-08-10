#include <type_traits>

#include "orvd/configuration/irw_wheel_rail_contact.h"

namespace {
using IrwContact = orvd::configuration::IrwWheelRailContact;
static_assert(!std::is_copy_constructible_v<IrwContact>);
}  // namespace
