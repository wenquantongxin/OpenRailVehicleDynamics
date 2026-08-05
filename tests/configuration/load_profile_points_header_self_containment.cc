#include "orvd/configuration/load_profile_points.h"

#include <filesystem>
#include <type_traits>
#include <utility>

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public configuration header leaks nlohmann/json"
#endif

static_assert(std::is_same_v<
              decltype(orvd::configuration::LoadProfilePointsFromJsonFile(
                  std::declval<const std::filesystem::path&>())),
              orvd::wheel_rail_contact::ProfilePoints>);
