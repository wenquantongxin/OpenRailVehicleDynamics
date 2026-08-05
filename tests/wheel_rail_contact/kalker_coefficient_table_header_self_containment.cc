#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(std::declval<const orvd::wheel_rail_contact::KalkerCoefficientTable&>()
                           .At(std::declval<double>())),
              orvd::wheel_rail_contact::KalkerCoefficients>);
static_assert(std::is_same_v<
              decltype(orvd::wheel_rail_contact::KalkerCoefficientTable::ForPoissonRatio(
                  std::declval<double>(),
                  std::declval<orvd::wheel_rail_contact::OutsideTableRule>())),
              orvd::wheel_rail_contact::KalkerCoefficientTable>);
