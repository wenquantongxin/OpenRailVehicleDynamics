#include "orvd/vehicle_project/read_visual_definition_document_from_file.h"

#include "strict_json.h"

namespace orvd::vehicle_project {

std::string ReadVisualDefinitionDocumentFromFile(const std::filesystem::path& path) {
    const configuration::strict_json::Json document = configuration::strict_json::ParseStrictJson(
        configuration::strict_json::ReadWholeFile(path, "visual definition file"));
    configuration::strict_json::RequireObject(document, "$");
    return document.dump();
}

}  // namespace orvd::vehicle_project
