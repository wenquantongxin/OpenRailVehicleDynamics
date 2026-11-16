#pragma once

#include <filesystem>
#include <string>

// A visual definition file holds one JSON object, the document a drawing reads
// for a vehicle's shapes, which a project carries as text
// (VehicleProject::visual_definition_document) without interpreting it. The
// file is read as strictly as a project file is, so that whatever a project
// takes in from it can be saved and read back.

namespace orvd::vehicle_project {

// The JSON object the visual definition file at `path` holds, as the text a
// project carries: compact, its keys sorted, the text a project file read
// back gives as well.
//
// Throws std::runtime_error naming the file when it cannot be read, and
// std::invalid_argument when it does not hold one JSON object: a syntax error,
// a repeated key or a number binary64 cannot hold included.
[[nodiscard]] std::string ReadVisualDefinitionDocumentFromFile(const std::filesystem::path& path);

}  // namespace orvd::vehicle_project
