#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"

#include <filesystem>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <string>
#include <system_error>

#include "strict_json.h"
#include "vehicle_project_json_reading_and_writing.h"

namespace orvd::vehicle_project {
namespace {

// Removes the pending file and refuses the write, the target untouched.
[[noreturn]] void RefuseWrite(const std::filesystem::path& pending,
                              const std::filesystem::path& target, const std::string& reason) {
    std::error_code ignored;
    std::filesystem::remove(pending, ignored);
    throw std::runtime_error("the vehicle project file '" + target.string() +
                             "' was not written: " + reason);
}

}  // namespace

void WriteVehicleProjectFile(const VehicleProject& project,
                             const VehicleProjectFileLocation& location) {
    const std::filesystem::path target = VehicleProjectFilePath(location);
    // Completed in memory first, so that a document that cannot be completed
    // never reaches the disk.
    const std::string text = internal::EncodeVehicleProjectToJson(project).dump(2) + "\n";

    std::filesystem::path pending = target;
    pending += ".pending";
    std::ofstream stream(pending, std::ios::binary | std::ios::trunc);
    if (!stream.is_open()) {
        throw std::runtime_error("the vehicle project file '" + target.string() +
                                 "' was not written: '" + pending.string() + "' cannot be created");
    }
    stream.write(text.data(), static_cast<std::streamsize>(text.size()));
    stream.close();
    if (stream.fail()) {
        RefuseWrite(pending, target, "'" + pending.string() + "' could not be written in full");
    }
    std::error_code renamed;
    std::filesystem::rename(pending, target, renamed);
    if (renamed) {
        RefuseWrite(pending, target,
                    "'" + pending.string() + "' could not replace it: " + renamed.message());
    }
}

VehicleProject ReadVehicleProjectFile(const VehicleProjectFileLocation& location) {
    return internal::DecodeVehicleProjectFromJson(
        configuration::strict_json::ParseStrictJson(configuration::strict_json::ReadWholeFile(
            VehicleProjectFilePath(location), "vehicle project file")));
}

}  // namespace orvd::vehicle_project
