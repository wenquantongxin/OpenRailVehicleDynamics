// Converts one profile between the reference multibody tool's format and this
// project's strict JSON record, in either direction.
//
// A development-time convenience, not a product entry point. It resolves no
// paths, searches nothing, and reads no environment: both files are named on
// the command line, exactly as given.

#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>
#include <string_view>

#include "orvd/configuration/load_profile_points.h"
#include "profile_json_writer.h"
#include "simpack_profile_io.h"

namespace {

int Usage() {
    std::fputs(
        "usage:\n"
        "  convert_profile to-json   <input.prw|input.prr> <output.json> "
        "<identifier>\n"
        "  convert_profile to-simpack <input.json> <output.prw|output.prr>\n",
        stderr);
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        return Usage();
    }
    const std::string_view direction = argv[1];

    try {
        if (direction == "to-json") {
            if (argc != 5) {
                return Usage();
            }
            const orvd::profile_conversion::SimpackProfile profile =
                orvd::profile_conversion::ReadSimpackProfile(argv[2], argv[4]);
            orvd::profile_conversion::WriteProfilePointsJson(argv[3],
                                                             profile.points);
            std::printf("wrote %s with %zu points\n", argv[3],
                        profile.points.size());
            return 0;
        }
        if (direction == "to-simpack") {
            if (argc != 4) {
                return Usage();
            }
            orvd::profile_conversion::SimpackProfile profile{
                orvd::configuration::LoadProfilePointsFromJsonFile(argv[2]),
                orvd::profile_conversion::SimpackProfileMetadata{}};
            profile.metadata.comment = profile.points.identifier();
            orvd::profile_conversion::WriteSimpackProfile(argv[3], profile);
            std::printf("wrote %s with %zu points\n", argv[3],
                        profile.points.size());
            return 0;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "profile conversion failed: %s\n", error.what());
        return 1;
    }
    return Usage();
}
