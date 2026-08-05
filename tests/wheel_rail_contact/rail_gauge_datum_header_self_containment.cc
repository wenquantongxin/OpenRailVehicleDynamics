#include "orvd/wheel_rail_contact/rail_gauge_datum.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::ComputeRailGaugeDatum(
                  std::declval<const orvd::wheel_rail_contact::ProfilePoints&>(),
                  1.435, 0.016, 0.0, orvd::wheel_rail_contact::WheelSide::kRight)),
              orvd::wheel_rail_contact::RailGaugeDatum>);
