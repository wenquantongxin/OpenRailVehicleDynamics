#include "simpack_profile_io.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <ios>
#include <limits>
#include <map>
#include <span>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace orvd::profile_conversion {
namespace {

using wheel_rail_contact::ProfilePoints;
using wheel_rail_contact::ProfileRole;

[[noreturn]] void Reject(const std::filesystem::path& path,
                         const std::string& detail) {
    throw std::invalid_argument("SIMPACK profile '" + path.string() + "': " +
                                detail);
}

std::string Trim(const std::string& line) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = line.find_last_not_of(" \t\r");
    return line.substr(first, last - first + 1);
}

// Drops a trailing comment, respecting a quoted value: the comment character
// also appears inside the descriptive strings the format carries.
std::string StripComment(const std::string& line) {
    bool inside_quotes = false;
    for (std::size_t index = 0; index < line.size(); ++index) {
        if (line[index] == '\'') {
            inside_quotes = !inside_quotes;
        } else if (line[index] == '!' && !inside_quotes) {
            return line.substr(0, index);
        }
    }
    return line;
}

bool ParseDouble(const std::string& text, double& value) {
    std::string trimmed = Trim(text);
    if (trimmed.empty()) {
        return false;
    }
    // The format writes an explicit leading plus on every signed quantity, and
    // the standard library's number reader does not accept one.
    if (trimmed.front() == '+') {
        trimmed.erase(0, 1);
        if (trimmed.empty()) {
            return false;
        }
    }
    const char* begin = trimmed.data();
    const char* end = begin + trimmed.size();
    const std::from_chars_result result = std::from_chars(begin, end, value);
    return result.ec == std::errc{} && result.ptr == end && std::isfinite(value);
}

std::vector<std::string> SplitFields(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t index = 0;
    while (index < line.size()) {
        while (index < line.size() && (line[index] == ' ' || line[index] == '\t')) {
            ++index;
        }
        const std::size_t start = index;
        while (index < line.size() && line[index] != ' ' && line[index] != '\t') {
            ++index;
        }
        if (index > start) {
            fields.push_back(line.substr(start, index - start));
        }
    }
    return fields;
}

using KeyValues = std::map<std::string, std::string>;

double RequireNumber(const std::filesystem::path& path, const KeyValues& values,
                     const std::string& key) {
    const auto entry = values.find(key);
    if (entry == values.end()) {
        Reject(path, "is missing the key '" + key + "'");
    }
    double number = 0.0;
    if (!ParseDouble(entry->second, number)) {
        Reject(path, "states '" + entry->second + "' for '" + key +
                         "', which is not a finite number");
    }
    return number;
}

std::string RequireQuoted(const std::filesystem::path& path,
                          const KeyValues& values, const std::string& key) {
    const auto entry = values.find(key);
    if (entry == values.end()) {
        Reject(path, "is missing the key '" + key + "'");
    }
    const std::string raw = Trim(entry->second);
    if (raw.size() < 2 || raw.front() != '\'' || raw.back() != '\'') {
        Reject(path, "states " + raw + " for '" + key +
                         "', which is not a quoted string");
    }
    return raw.substr(1, raw.size() - 2);
}

void RequireIdentityValue(const std::filesystem::path& path,
                          const KeyValues& values, const std::string& key,
                          double expected) {
    const double actual = RequireNumber(path, values, key);
    if (actual != expected) {
        Reject(path, "sets '" + key + "' to " + std::to_string(actual) +
                         " rather than " + std::to_string(expected) +
                         "; this reader implements only profiles whose "
                         "preprocessing is the identity, and refuses rather "
                         "than ignoring a step it does not apply");
    }
}

void RequireExactKeySet(const std::filesystem::path& path,
                        const KeyValues& values,
                        const std::vector<std::string>& expected,
                        const std::string& block) {
    for (const std::string& key : expected) {
        if (!values.contains(key)) {
            Reject(path, "is missing '" + key + "' from its " + block + " block");
        }
    }
    for (const auto& [key, ignored] : values) {
        if (std::find(expected.begin(), expected.end(), key) == expected.end()) {
            Reject(path, "states '" + key + "' in its " + block +
                             " block, which this reader does not implement; an "
                             "unknown key is a preprocessing step, not a "
                             "decoration");
        }
    }
}

const std::vector<std::string>& SplineKeyOrder() {
    static const std::vector<std::string> keys{
        "approx.smooth", "file",        "file.mtime", "comment",
        "type",          "point.dist.min", "shift.y",  "shift.z",
        "rotate",        "bound.y.min", "bound.y.max", "bound.z.min",
        "bound.z.max",   "mirror.y",    "mirror.z",   "inversion",
        "units.len",     "units.ang",   "units.len.f", "units.ang.f"};
    return keys;
}

void RequireRoleMatchesExtension(const std::filesystem::path& profile_path,
                                 bool is_wheel) {
    const std::string extension = profile_path.extension().string();
    const bool extension_says_wheel = extension == ".prw";
    const bool extension_says_rail = extension == ".prr";
    if (!extension_says_wheel && !extension_says_rail) {
        Reject(profile_path, "has the extension '" + extension +
                                 "', which names neither a wheel nor a rail "
                                 "profile");
    }
    if (is_wheel != extension_says_wheel) {
        Reject(profile_path,
               "declares a role its extension contradicts; the two are the only "
               "statements of what this file is and they must agree");
    }
}

std::string FormatCoordinate(double value) {
    std::ostringstream stream;
    stream.setf(std::ios::scientific, std::ios::floatfield);
    // In scientific notation precision counts digits after the decimal point.
    // max_digits10 significant digits are sufficient to recover any binary64
    // value exactly, so one fewer belongs after the leading digit.
    stream.precision(std::numeric_limits<double>::max_digits10 - 1);
    stream << value;
    return stream.str();
}

}  // namespace

SimpackProfile ReadSimpackProfile(const std::filesystem::path& profile_path,
                                  std::string identifier) {
    std::ifstream input(profile_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("SIMPACK profile '" + profile_path.string() +
                                 "' cannot be opened");
    }
    std::string contents((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
    if (!input.eof() && input.fail()) {
        throw std::runtime_error("SIMPACK profile '" + profile_path.string() +
                                 "' cannot be read to its end");
    }
    for (const char byte : contents) {
        if (static_cast<unsigned char>(byte) > 0x7F) {
            Reject(profile_path,
                   "contains a byte outside the admitted character range");
        }
    }

    enum class Block { kPreamble, kHeader, kSpline, kPoints };
    Block block = Block::kPreamble;
    bool header_seen = false;
    bool header_closed = false;
    bool spline_seen = false;
    bool spline_closed = false;
    bool points_seen = false;
    bool points_closed = false;
    KeyValues header;
    KeyValues spline;
    std::vector<double> lateral;
    std::vector<double> vertical;

    std::istringstream lines(contents);
    std::string raw_line;
    while (std::getline(lines, raw_line)) {
        if (block == Block::kPoints) {
            const std::string trimmed = Trim(raw_line);
            if (trimmed.empty() || trimmed.front() == '!') {
                continue;
            }
            if (trimmed == "point.end") {
                block = Block::kSpline;
                points_closed = true;
                continue;
            }
            const std::vector<std::string> fields = SplitFields(trimmed);
            if (fields.size() < 2 || fields.size() > 3) {
                Reject(profile_path, "has a point row with " +
                                         std::to_string(fields.size()) +
                                         " fields; a row is two coordinates and "
                                         "an optional weight");
            }
            double y = 0.0;
            double z = 0.0;
            if (!ParseDouble(fields[0], y) || !ParseDouble(fields[1], z)) {
                Reject(profile_path,
                       "has a point row whose coordinates are not finite numbers");
            }
            if (fields.size() == 3) {
                double weight = 0.0;
                if (!ParseDouble(fields[2], weight) || weight != 1.0) {
                    Reject(profile_path,
                           "gives a point a weight other than one, which only "
                           "means something for an approximating fit this "
                           "reader does not implement");
                }
            }
            lateral.push_back(y);
            vertical.push_back(z);
            continue;
        }

        const std::string trimmed = Trim(StripComment(raw_line));
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed == "header.begin") {
            if (block != Block::kPreamble || header_seen) {
                Reject(profile_path, "opens a header block out of order");
            }
            header_seen = true;
            block = Block::kHeader;
            continue;
        }
        if (trimmed == "header.end") {
            if (block != Block::kHeader) {
                Reject(profile_path, "closes a header block that is not open");
            }
            header_closed = true;
            block = Block::kPreamble;
            continue;
        }
        if (trimmed == "spline.begin") {
            if (block != Block::kPreamble || spline_seen) {
                Reject(profile_path, "opens a spline block out of order");
            }
            spline_seen = true;
            block = Block::kSpline;
            continue;
        }
        if (trimmed == "point.begin") {
            if (block != Block::kSpline || points_seen) {
                Reject(profile_path, "opens a point block out of order");
            }
            points_seen = true;
            block = Block::kPoints;
            continue;
        }
        if (trimmed == "spline.end") {
            if (block != Block::kSpline) {
                Reject(profile_path, "closes a spline block that is not open");
            }
            spline_closed = true;
            block = Block::kPreamble;
            continue;
        }

        if (block == Block::kPreamble) {
            continue;
        }
        const std::size_t equals = trimmed.find('=');
        if (equals == std::string::npos) {
            Reject(profile_path, "has the unreadable line '" + trimmed + "'");
        }
        const std::string key = Trim(trimmed.substr(0, equals));
        const std::string value = Trim(trimmed.substr(equals + 1));
        KeyValues& target = (block == Block::kHeader) ? header : spline;
        if (!target.emplace(key, value).second) {
            Reject(profile_path, "repeats the key '" + key + "'");
        }
    }

    if (!header_seen || !header_closed || !spline_seen || !spline_closed ||
        !points_seen || !points_closed) {
        Reject(profile_path,
               "does not contain one closed header block, one closed spline "
               "block and one closed point block");
    }

    const double version = RequireNumber(profile_path, header, "version");
    if (version != 1.0) {
        Reject(profile_path, "declares format version " + std::to_string(version) +
                                 "; only version one is implemented");
    }
    const double declared_role = RequireNumber(profile_path, header, "type");
    if (declared_role != 0.0 && declared_role != 1.0) {
        Reject(profile_path, "declares the role " + std::to_string(declared_role) +
                                 ", which is neither rail nor wheel");
    }
    const bool is_wheel = declared_role == 1.0;

    RequireRoleMatchesExtension(profile_path, is_wheel);

    std::vector<std::string> header_keys{"version", "type"};
    if (is_wheel) {
        header_keys.push_back("meas.pos.e");
        header_keys.push_back("meas.pos.qr");
    }
    RequireExactKeySet(profile_path, header, header_keys, "header");
    RequireExactKeySet(profile_path, spline, SplineKeyOrder(), "spline");

    RequireIdentityValue(profile_path, spline, "approx.smooth", 0.0);
    RequireIdentityValue(profile_path, spline, "point.dist.min", 0.0);
    RequireIdentityValue(profile_path, spline, "shift.y", 0.0);
    RequireIdentityValue(profile_path, spline, "shift.z", 0.0);
    RequireIdentityValue(profile_path, spline, "rotate", 0.0);
    RequireIdentityValue(profile_path, spline, "mirror.y", 0.0);
    RequireIdentityValue(profile_path, spline, "mirror.z", 0.0);

    const std::string source_file = RequireQuoted(profile_path, spline, "file");
    if (source_file != "-") {
        Reject(profile_path, "names '" + source_file +
                                 "' as an external source; the point block "
                                 "would then be a cache of an import this "
                                 "reader cannot re-run");
    }

    // The clipping window is stored with its lower limit above its upper one,
    // which is how the format says clipping is switched off. A reader that
    // applied it literally would keep no points, and one that sorted the two
    // limits would silently delete the whole flange.
    const double lateral_clip_low = RequireNumber(profile_path, spline, "bound.y.min");
    const double lateral_clip_high = RequireNumber(profile_path, spline, "bound.y.max");
    const double vertical_clip_low = RequireNumber(profile_path, spline, "bound.z.min");
    const double vertical_clip_high = RequireNumber(profile_path, spline, "bound.z.max");
    if (!(lateral_clip_low > lateral_clip_high) ||
        !(vertical_clip_low > vertical_clip_high)) {
        Reject(profile_path,
               "requests a clipping window; this reader implements only the "
               "disabled encoding, where the lower limit lies above the upper "
               "one");
    }

    if (RequireQuoted(profile_path, spline, "units.len") != "m" ||
        RequireQuoted(profile_path, spline, "units.ang") != "rad") {
        Reject(profile_path,
               "states its coordinates in a unit other than metres and "
               "radians; conversion happens when a profile is written, not "
               "when it is read");
    }
    RequireIdentityValue(profile_path, spline, "units.len.f", 1.0);
    RequireIdentityValue(profile_path, spline, "units.ang.f", 1.0);

    const double inversion = RequireNumber(profile_path, spline, "inversion");
    if (inversion != 0.0 && inversion != 1.0) {
        Reject(profile_path, "sets the order inversion to " +
                                 std::to_string(inversion) +
                                 ", which is neither kept nor reversed");
    }

    if (lateral.size() < 3) {
        Reject(profile_path, "states " + std::to_string(lateral.size()) +
                                 " points; a profile surface needs at least "
                                 "three");
    }
    if (inversion == 1.0) {
        std::reverse(lateral.begin(), lateral.end());
        std::reverse(vertical.begin(), vertical.end());
    }

    // After the declared inversion the row order must be monotone. A profile
    // that is not is a multi-section one, which this reader does not implement,
    // and sorting it into something plausible would invent a surface.
    bool ascending = true;
    bool descending = true;
    for (std::size_t index = 0; index + 1 < lateral.size(); ++index) {
        if (!(lateral[index + 1] > lateral[index])) {
            ascending = false;
        }
        if (!(lateral[index + 1] < lateral[index])) {
            descending = false;
        }
    }
    if (!ascending && !descending) {
        Reject(profile_path,
               "does not run monotonically across the profile once its declared "
               "order inversion is applied");
    }
    if (descending) {
        std::reverse(lateral.begin(), lateral.end());
        std::reverse(vertical.begin(), vertical.end());
    }

    SimpackProfile profile{
        ProfilePoints::FromAuthoredOrder(
            is_wheel ? ProfileRole::kWheel : ProfileRole::kRail,
            std::move(identifier), std::move(lateral), std::move(vertical)),
        SimpackProfileMetadata{}};
    if (is_wheel) {
        profile.metadata.flange_width_measurement_depth_meters =
            RequireNumber(profile_path, header, "meas.pos.e");
        profile.metadata.flange_slope_measurement_depth_meters =
            RequireNumber(profile_path, header, "meas.pos.qr");
    }
    profile.metadata.comment = RequireQuoted(profile_path, spline, "comment");
    return profile;
}

void WriteSimpackProfile(const std::filesystem::path& profile_path,
                         const SimpackProfile& profile) {
    const bool is_wheel =
        profile.points.role() == wheel_rail_contact::ProfileRole::kWheel;
    // Validate before opening with truncation: a role/extension mistake must
    // not destroy an existing local research asset.
    RequireRoleMatchesExtension(profile_path, is_wheel);
    std::ofstream output(profile_path, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("SIMPACK profile '" + profile_path.string() +
                                 "' cannot be opened for writing");
    }
    output << "! Written by OpenRailVehicleDynamics for local research use.\n";
    output << "! Profile identifier : " << profile.points.identifier() << "\n";
    output << "\n";
    output << "  header.begin\n";
    output << "    version        =  1\n";
    output << "    type           =  " << (is_wheel ? 1 : 0) << "\n";
    if (is_wheel) {
        output << "    meas.pos.e     = "
               << FormatCoordinate(
                      profile.metadata.flange_width_measurement_depth_meters)
               << "\n";
        output << "    meas.pos.qr    = "
               << FormatCoordinate(
                      profile.metadata.flange_slope_measurement_depth_meters)
               << "\n";
    }
    output << "  header.end\n";
    output << "\n";
    output << "  spline.begin\n";
    output << "    approx.smooth  = " << FormatCoordinate(0.0) << "\n";
    output << "    file           = '-'\n";
    output << "    file.mtime     =  0\n";
    output << "    comment        = '" << profile.metadata.comment << "'\n";
    output << "    type           =  0\n";
    output << "    point.dist.min = " << FormatCoordinate(0.0) << "\n";
    output << "    shift.y        = " << FormatCoordinate(0.0) << "\n";
    output << "    shift.z        = " << FormatCoordinate(0.0) << "\n";
    output << "    rotate         = " << FormatCoordinate(0.0) << "\n";
    output << "    bound.y.min    = " << FormatCoordinate(1.0) << "\n";
    output << "    bound.y.max    = " << FormatCoordinate(0.0) << "\n";
    output << "    bound.z.min    = " << FormatCoordinate(1.0) << "\n";
    output << "    bound.z.max    = " << FormatCoordinate(0.0) << "\n";
    output << "    mirror.y       =  0\n";
    output << "    mirror.z       =  0\n";
    output << "    inversion      =  0\n";
    output << "    units.len      = 'm'\n";
    output << "    units.ang      = 'rad'\n";
    output << "    units.len.f    = " << FormatCoordinate(1.0) << "\n";
    output << "    units.ang.f    = " << FormatCoordinate(1.0) << "\n";
    output << "    point.begin\n";
    output << "      !\n";
    output << "      ! y value (lengths unit)  z value (lengths unit)\n";
    const std::span<const double> lateral = profile.points.authored_lateral_meters();
    const std::span<const double> vertical =
        profile.points.authored_vertical_meters();
    for (std::size_t index = 0; index < lateral.size(); ++index) {
        output << FormatCoordinate(lateral[index]) << " "
               << FormatCoordinate(vertical[index]) << "\n";
    }
    output << "    point.end\n";
    output << "  spline.end\n";

    if (!output) {
        throw std::runtime_error("SIMPACK profile '" + profile_path.string() +
                                 "' could not be written to its end");
    }
}

}  // namespace orvd::profile_conversion
