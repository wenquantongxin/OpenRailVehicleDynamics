#include "orvd/wheel_rail_contact/wheel_profile_preprocessing.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::
                                        WheelProfilePreprocessing&>()
                           .SampleVisibleOutline(
                               std::declval<const orvd::wheel_rail_contact::
                                                ProfilePoints&>(),
                               orvd::wheel_rail_contact::WheelSide::kRight, 2)),
              orvd::wheel_rail_contact::WheelProfileOutline>);
