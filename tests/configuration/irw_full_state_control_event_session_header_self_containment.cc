#include "orvd/configuration/irw_full_state_control_event_session.h"

#include <type_traits>

static_assert(std::is_class_v<
              orvd::configuration::IrwFullStateControlEventSession>);
static_assert(std::is_class_v<
              orvd::configuration::IrwFullStateControlEventAudit>);
