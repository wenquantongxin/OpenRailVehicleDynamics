#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

#include <type_traits>
#include <utility>

static_assert(orvd::control::kIrwGuidanceAxleCount == 4);
static_assert(orvd::control::kIrwGuidanceWheelCount == 8);
static_assert(std::is_same_v<
              decltype(std::declval<const orvd::control::
                                        IrwFullStateWheelSpeedGuidanceController&>()
                           .Step(
                               std::declval<const orvd::control::
                                   IrwFullStateWheelSpeedGuidanceControllerInput&>(),
                               std::declval<const orvd::control::
                                   IrwFullStateWheelSpeedGuidanceControllerState&>())),
              orvd::control::
                  IrwFullStateWheelSpeedGuidanceControllerResult>);
