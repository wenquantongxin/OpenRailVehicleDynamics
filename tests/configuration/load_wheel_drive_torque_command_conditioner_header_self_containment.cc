#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"

#include <filesystem>
#include <type_traits>
#include <utility>

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public configuration header leaks nlohmann/json"
#endif

static_assert(std::is_same_v<
              decltype(orvd::configuration::
                           LoadWheelDriveTorqueCommandConditionerFromJsonFile(
                               std::declval<const std::filesystem::path&>())),
              orvd::actuation::WheelDriveTorqueCommandConditioner>);
