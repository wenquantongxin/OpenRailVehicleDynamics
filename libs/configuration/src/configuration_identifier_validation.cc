#include "configuration_identifier_validation.h"

#include <stdexcept>

namespace orvd::configuration::internal {

void RequireConfigurationIdentifier(const std::string& identifier, const std::string& path,
                                    std::string_view what) {
    if (identifier.empty()) {
        throw std::invalid_argument(path + " must be a non-empty " + std::string(what));
    }
    for (const char character : identifier) {
        const bool admitted = (character >= 'A' && character <= 'Z') ||
                              (character >= 'a' && character <= 'z') ||
                              (character >= '0' && character <= '9') || character == '.' ||
                              character == '_' || character == '-';
        if (!admitted) {
            throw std::invalid_argument(path + " is a " + std::string(what) + " '" + identifier +
                                        "' containing a character outside [A-Za-z0-9._-]; an "
                                        "identifier is a name, not a path or a sentence");
        }
    }
}

}  // namespace orvd::configuration::internal
