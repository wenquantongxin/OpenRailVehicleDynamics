#include "orvd/configuration/load_track_irregularity_field.h"

#include <filesystem>
#include <string_view>
#include <type_traits>
#include <utility>

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public configuration header leaks nlohmann/json"
#endif

static_assert(std::is_same_v<
              decltype(orvd::configuration::
                           LoadTrackIrregularityFieldFromDataRoot(
                               std::declval<const std::filesystem::path&>(),
                               std::declval<std::string_view>())),
              orvd::wheel_rail_contact::TrackIrregularityField>);
