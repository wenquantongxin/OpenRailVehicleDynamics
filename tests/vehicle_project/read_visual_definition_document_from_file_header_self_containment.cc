#include "orvd/vehicle_project/read_visual_definition_document_from_file.h"

#include <filesystem>
#include <string>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<decltype(orvd::vehicle_project::ReadVisualDefinitionDocumentFromFile(
                                 std::declval<const std::filesystem::path&>())),
                             std::string>);
