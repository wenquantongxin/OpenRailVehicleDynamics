#include "profile_json_writer.h"

#include <cstddef>
#include <fstream>
#include <ios>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>

namespace orvd::profile_conversion {
namespace {

// Seventeen significant digits name a binary64 uniquely, so the text is a
// lossless statement of the number rather than a rounded picture of it.
std::string ExactDecimal(double value) {
    std::ostringstream stream;
    stream.precision(17);
    stream << value;
    return stream.str();
}

void WriteColumn(std::ofstream& output, const std::string& key,
                 std::span<const double> column, bool trailing_comma) {
    output << "  \"" << key << "\": [\n";
    for (std::size_t index = 0; index < column.size(); ++index) {
        output << "    " << ExactDecimal(column[index]);
        if (index + 1 < column.size()) {
            output << ",";
        }
        output << "\n";
    }
    output << "  ]" << (trailing_comma ? "," : "") << "\n";
}

}  // namespace

void WriteProfilePointsJson(const std::filesystem::path& json_path,
                            const wheel_rail_contact::ProfilePoints& profile) {
    std::ofstream output(json_path, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("profile record '" + json_path.string() +
                                 "' cannot be opened for writing");
    }

    output << "{\n";
    output << "  \"schema_version\": 1,\n";
    output << "  \"profile_identifier\": \"" << profile.identifier() << "\",\n";
    output << "  \"profile_role\": \""
           << (profile.role() == wheel_rail_contact::ProfileRole::kWheel ? "wheel"
                                                                        : "rail")
           << "\",\n";
    output << "  \"coordinate_frame\": \"lateral_right_vertical_down\",\n";
    output << "  \"length_unit\": \"meter\",\n";
    WriteColumn(output, "lateral_meters", profile.authored_lateral_meters(), true);
    WriteColumn(output, "vertical_meters", profile.authored_vertical_meters(),
                false);
    output << "}\n";

    if (!output) {
        throw std::runtime_error("profile record '" + json_path.string() +
                                 "' could not be written to its end");
    }
}

}  // namespace orvd::profile_conversion
