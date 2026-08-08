#include "orvd/wheel_rail_contact/track_irregularity_field.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::
                                        TrackIrregularityField&>()
                           .LateralDisplacementMeters(0.0)),
              double>);
static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::
                                        TrackIrregularityField&>()
                           .VerticalSlopeMetersPerMeter(0.0)),
              double>);
