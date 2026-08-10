#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"

#include <array>
#include <type_traits>
#include <utility>

static_assert(
    orvd::actuation::kWheelDriveTorqueChannelCount == 8 &&
    orvd::actuation::kWheelDriveTorqueSpeedNodeCount == 10);
static_assert(std::is_same_v<
              decltype(std::declval<const orvd::actuation::
                                        WheelDriveTorqueCommandConditioner&>()
                           .Step(
                               std::declval<const orvd::actuation::
                                   WheelDriveTorqueChannelValues&>(),
                               std::declval<const orvd::actuation::
                                   WheelDriveTorqueChannelValues&>(),
                               std::declval<const orvd::actuation::
                                   WheelDriveTorqueChannelValues&>())),
              orvd::actuation::WheelDriveTorqueConditioningResult>);
