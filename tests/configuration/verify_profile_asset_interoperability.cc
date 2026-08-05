// The product's strict JSON profile record, and its semantic round trip against
// the reference multibody tool's format.
//
// Both directions are checked by meaning, not by text: what must survive a
// conversion is the role, the frame, the unit, the order and the points.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/configuration/load_profile_points.h"
#include "profile_json_writer.h"
#include "simpack_profile_io.h"

namespace {

using orvd::configuration::LoadProfilePointsFromJsonFile;
using orvd::profile_conversion::ReadSimpackProfile;
using orvd::profile_conversion::SimpackProfile;
using orvd::profile_conversion::WriteProfilePointsJson;
using orvd::profile_conversion::WriteSimpackProfile;
using orvd::wheel_rail_contact::ProfilePoints;
using orvd::wheel_rail_contact::ProfileRole;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "profile asset interoperability: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireRefusal(const std::function<void()>& action, std::string_view fragment,
                    std::string_view what) {
    try {
        action();
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "profile asset interoperability: %.*s was refused for "
                         "another reason: %s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

void Write(const std::filesystem::path& path, const std::string& document) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << document;
    Require(static_cast<bool>(output), "a fixture could not be written");
}

std::string ReplaceOnce(std::string document, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = document.find(needle);
    if (position == std::string::npos ||
        document.find(needle, position + needle.size()) != std::string::npos) {
        Require(false, "a fixture mutation is not unique");
        return document;
    }
    document.replace(position, needle.size(), replacement);
    return document;
}

// A minimal but complete file in the reference format. The rows descend in the
// lateral coordinate and the file declares the order inversion that reverses
// them, which is the one preprocessing step the qualified assets actually use.
std::string SimpackWheelDocument() {
    return R"(! a synthetic wheel profile
  header.begin
    version        =  1
    type           =  1                       ! 0=rail profile, 1=wheel profile
    meas.pos.e     = +1.000000000000000e-02
    meas.pos.qr    = +2.000000000000000e-03
  header.end

  spline.begin
    approx.smooth  = +0.000000000000000e+00
    file           = '-'
    file.mtime     =  0
    comment        = 'synthetic wheel, not a real asset'
    type           =  0
    point.dist.min = +0.000000000000000e+00
    shift.y        = +0.000000000000000e+00
    shift.z        = +0.000000000000000e+00
    rotate         = +0.000000000000000e+00
    bound.y.min    = +1.000000000000000e+00
    bound.y.max    = +0.000000000000000e+00
    bound.z.min    = +1.000000000000000e+00
    bound.z.max    = +0.000000000000000e+00
    mirror.y       =  0
    mirror.z       =  0
    inversion      =  1
    units.len      = 'm'
    units.ang      = 'rad'
    units.len.f    = +1.000000000000000e+00
    units.ang.f    = +1.000000000000000e+00
    point.begin
      !
      ! y value (lengths unit)  z value (lengths unit)
 4.00000000000000e-02 -2.00000000000000e-03
 2.00000000000000e-02 -1.00000000000000e-03
 0.00000000000000e+00 0.00000000000000e+00
-2.00000000000000e-02 3.00000000000000e-03
-4.00000000000000e-02 1.10000000000000e-02
    point.end
  spline.end
)";
}

bool SamePoints(const ProfilePoints& left, const ProfilePoints& right) {
    if (left.size() != right.size()) {
        return false;
    }
    for (std::size_t index = 0; index < left.size(); ++index) {
        if (left.authored_lateral_meters()[index] !=
                right.authored_lateral_meters()[index] ||
            left.authored_vertical_meters()[index] !=
                right.authored_vertical_meters()[index]) {
            return false;
        }
    }
    return left.role() == right.role() && left.identifier() == right.identifier();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr,
                     "usage: verify_profile_asset_interoperability <scratch-dir>\n");
        return 2;
    }
    const std::filesystem::path scratch = argv[1];
    std::error_code ignored;
    std::filesystem::create_directories(scratch, ignored);

    const std::filesystem::path simpack_path = scratch / "synthetic_wheel.prw";
    const std::filesystem::path json_path = scratch / "synthetic_wheel.json";
    const std::filesystem::path returned_path = scratch / "returned_wheel.prw";
    const std::filesystem::path precise_path = scratch / "precise_rail.prr";
    const std::filesystem::path wrong_role_path =
        scratch / "wheel_written_as_rail.prr";
    const std::string valid = SimpackWheelDocument();

    ProfilePoints from_simpack = ProfilePoints::FromAuthoredOrder(
        ProfileRole::kWheel, "placeholder", {0.0, 1.0}, {0.0, 0.0});
    try {
        Write(simpack_path, valid);
        const SimpackProfile read = ReadSimpackProfile(simpack_path, "synthetic_wheel");
        from_simpack = read.points;

        Require(read.points.role() == ProfileRole::kWheel,
                "the declared wheel role did not survive reading");
        Require(read.points.size() == 5, "a point was lost while reading");
        // The file's rows descend and it declares the inversion, so the value
        // object must come out ascending. A reader that ignored the flag would
        // land on the same list only by accident; one that ignored it here
        // would come out descending.
        Require(read.points.authored_lateral_meters().front() == -0.04 &&
                    read.points.authored_lateral_meters().back() == 0.04,
                "the declared order inversion was not applied");
        Require(read.points.authored_vertical_meters().front() == 0.011,
                "the point rows were reordered without their partners");
        Require(read.metadata.flange_width_measurement_depth_meters == 0.01 &&
                    read.metadata.flange_slope_measurement_depth_meters == 0.002,
                "the wheel measurement metadata was lost");

        WriteProfilePointsJson(json_path, read.points);
        const ProfilePoints from_json = LoadProfilePointsFromJsonFile(json_path);
        Require(SamePoints(read.points, from_json),
                "a profile written as the product record and read back is not "
                "the same profile");

        WriteSimpackProfile(returned_path, SimpackProfile{from_json, read.metadata});
        const SimpackProfile returned =
            ReadSimpackProfile(returned_path, "synthetic_wheel");
        Require(SamePoints(from_json, returned.points),
                "a profile carried out to the reference format and back is not "
                "the same profile");
        Require(returned.metadata.flange_width_measurement_depth_meters ==
                        read.metadata.flange_width_measurement_depth_meters &&
                    returned.metadata.flange_slope_measurement_depth_meters ==
                        read.metadata.flange_slope_measurement_depth_meters,
                "the measurement metadata did not survive the round trip");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "profile asset interoperability failed: %s\n",
                     error.what());
        return 1;
    }

    {
        // The writer must retain enough decimal digits to reconstruct every
        // binary64 coordinate. These nextafter values sit beside ordinary
        // profile-scale decimals and lose bits with a 15-significant-digit
        // formatter.
        const ProfilePoints precise = ProfilePoints::FromAuthoredOrder(
            ProfileRole::kRail, "precision_rail",
            {std::nextafter(-0.0645,
                            -std::numeric_limits<double>::infinity()),
             0.0,
             std::nextafter(0.0715,
                            std::numeric_limits<double>::infinity())},
            {std::nextafter(-0.0135,
                            -std::numeric_limits<double>::infinity()),
             0.0012345678901234567,
             std::nextafter(0.0045,
                            std::numeric_limits<double>::infinity())});
        WriteSimpackProfile(precise_path, SimpackProfile{precise, {}});
        const SimpackProfile returned =
            ReadSimpackProfile(precise_path, "precision_rail");
        Require(SamePoints(precise, returned.points),
                "the reference-format writer did not preserve binary64 "
                "profile coordinates");
    }

    {
        // Refuse a role/extension mismatch before opening with truncation. The
        // local converter must not destroy the very asset whose name exposed
        // the caller's mistake.
        const std::string sentinel = "keep this local asset\n";
        Write(wrong_role_path, sentinel);
        RequireRefusal(
            [&] {
                WriteSimpackProfile(wrong_role_path,
                                    SimpackProfile{from_simpack, {}});
            },
            "extension contradicts",
            "a wheel profile written through a rail-profile extension");
        std::ifstream preserved(wrong_role_path, std::ios::binary);
        const std::string contents((std::istreambuf_iterator<char>(preserved)),
                                   std::istreambuf_iterator<char>());
        Require(contents == sentinel,
                "a refused role/extension mismatch truncated an existing file");
    }

    // The reference format carries nine preprocessing steps and the qualified
    // assets leave all but one at identity. Every one of them must be refused
    // rather than ignored: a reader that ignored them would be right on exactly
    // the files it was written against.
    const auto expect_simpack_refusal = [&](const std::string& document,
                                            std::string_view fragment,
                                            std::string_view what) {
        Write(simpack_path, document);
        RequireRefusal(
            [&] { (void)ReadSimpackProfile(simpack_path, "synthetic_wheel"); },
            fragment, what);
    };

    expect_simpack_refusal(
        ReplaceOnce(valid, "shift.y        = +0.000000000000000e+00",
                    "shift.y        = +1.000000000000000e-03"),
        "preprocessing is the identity", "a declared translation");
    expect_simpack_refusal(
        ReplaceOnce(valid, "mirror.y       =  0", "mirror.y       =  1"),
        "preprocessing is the identity", "a declared mirroring");
    expect_simpack_refusal(
        ReplaceOnce(valid, "rotate         = +0.000000000000000e+00",
                    "rotate         = +1.000000000000000e-02"),
        "preprocessing is the identity", "a declared rotation");
    expect_simpack_refusal(
        ReplaceOnce(valid, "units.len.f    = +1.000000000000000e+00",
                    "units.len.f    = +1.000000000000000e+03"),
        "preprocessing is the identity", "a non-unit length factor");
    expect_simpack_refusal(ReplaceOnce(valid, "units.len      = 'm'",
                                       "units.len      = 'mm'"),
                           "unit other than metres", "a millimetre profile");
    expect_simpack_refusal(
        ReplaceOnce(valid, "bound.y.min    = +1.000000000000000e+00",
                    "bound.y.min    = -1.000000000000000e+00"),
        "requests a clipping window", "an enabled clipping window");
    expect_simpack_refusal(ReplaceOnce(valid, "file           = '-'",
                                       "file           = 'somewhere.dat'"),
                           "as an external source", "an imported point cache");
    expect_simpack_refusal(
        ReplaceOnce(valid, "    mirror.z       =  0\n",
                    "    mirror.z       =  0\n    split.distance =  0\n"),
        "does not implement", "an unknown preprocessing key");
    expect_simpack_refusal(ReplaceOnce(valid, "    mirror.z       =  0\n", ""),
                           "is missing", "a missing preprocessing key");
    expect_simpack_refusal(ReplaceOnce(valid, "    type           =  1  ",
                                       "    type           =  0  "),
                           "extension contradicts", "a rail role in a wheel file");
    expect_simpack_refusal(
        ReplaceOnce(valid, "-2.00000000000000e-02 3.00000000000000e-03",
                    "-6.00000000000000e-02 3.00000000000000e-03"),
        "does not run monotonically", "a profile that doubles back");
    expect_simpack_refusal(ReplaceOnce(valid, "    point.begin\n", ""),
                           "unreadable line",
                           "point rows outside any point block");
    expect_simpack_refusal(
        ReplaceOnce(valid, "0.00000000000000e+00 0.00000000000000e+00",
                    "0.00000000000000e+00 0.00000000000000e+00 2.0"),
        "weight other than one", "a point carrying an approximation weight");

    // The product record's own refusal surface.
    Write(simpack_path, valid);
    WriteProfilePointsJson(json_path, ReadSimpackProfile(simpack_path,
                                                         "synthetic_wheel")
                                          .points);
    std::ifstream input(json_path, std::ios::binary);
    const std::string record((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());

    const auto expect_json_refusal = [&](const std::string& document,
                                         std::string_view fragment,
                                         std::string_view what) {
        Write(json_path, document);
        RequireRefusal([&] { (void)LoadProfilePointsFromJsonFile(json_path); },
                       fragment, what);
    };

    expect_json_refusal(
        ReplaceOnce(record, "\"schema_version\": 1", "\"schema_version\": 2"),
        "integer 1", "an unsupported schema version");
    expect_json_refusal(
        ReplaceOnce(record, "\"profile_role\": \"wheel\"",
                    "\"profile_role\": \"axle\""),
        "'wheel' or 'rail'", "a role that names no surface");
    expect_json_refusal(
        ReplaceOnce(record, "\"length_unit\": \"meter\"",
                    "\"length_unit\": \"millimeter\""),
        "'meter'", "a record stating another unit");
    expect_json_refusal(
        ReplaceOnce(record, "\"coordinate_frame\": \"lateral_right_vertical_down\"",
                    "\"coordinate_frame\": \"lateral_right_vertical_up\""),
        "lateral_right_vertical_down", "a record stating another frame");
    expect_json_refusal(
        ReplaceOnce(record, "\"schema_version\": 1,",
                    "\"schema_version\": 1, \"schema_version\": 1,"),
        "duplicate JSON object key", "a duplicate key");
    expect_json_refusal(
        ReplaceOnce(record, "\"length_unit\": \"meter\",",
                    "\"length_unit\": \"meter\", \"origin\": \"crown\","),
        "unknown key", "an unknown key");
    expect_json_refusal(ReplaceOnce(record, "  \"length_unit\": \"meter\",\n", ""),
                        "length_unit is required", "a missing key");
    expect_json_refusal(
        ReplaceOnce(record, "\"profile_identifier\": \"synthetic_wheel\"",
                    "\"profile_identifier\": \"../elsewhere\""),
        "outside [A-Za-z0-9._-]", "an identifier that traverses a path");

    // The two columns are separate arrays, so the one thing that can go wrong
    // with that choice has to be refused.
    {
        std::string uneven = record;
        const std::size_t last_bracket = uneven.rfind("  ]\n}");
        Require(last_bracket != std::string::npos, "the record's shape changed");
        const std::size_t last_value = uneven.rfind('\n', last_bracket - 2);
        uneven.erase(last_value, last_bracket - last_value - 1);
        // Removing the final vertical coordinate leaves a trailing comma, so
        // repair it before the parser objects to the syntax instead.
        const std::size_t stray = uneven.rfind(',', last_value);
        if (stray != std::string::npos) {
            uneven.erase(stray, 1);
        }
        expect_json_refusal(uneven, "a point needs both",
                            "a record whose two columns differ in length");
    }

    std::filesystem::remove(simpack_path, ignored);
    std::filesystem::remove(json_path, ignored);
    std::filesystem::remove(returned_path, ignored);
    std::filesystem::remove(precise_path, ignored);
    std::filesystem::remove(wrong_role_path, ignored);

    if (failures != 0) {
        return 1;
    }
    std::puts("profile asset interoperability verified");
    return 0;
}
