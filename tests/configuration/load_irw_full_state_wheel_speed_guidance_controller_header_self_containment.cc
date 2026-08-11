#include "orvd/configuration/load_irw_full_state_wheel_speed_guidance_controller.h"

#include <filesystem>
#include <type_traits>
#include <utility>

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public configuration header leaks nlohmann/json"
#endif

static_assert(std::is_same_v<
              decltype(orvd::configuration::
                           LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
                               std::declval<const std::filesystem::path&>())),
              orvd::control::
                  IrwFullStateWheelSpeedGuidanceController>);
