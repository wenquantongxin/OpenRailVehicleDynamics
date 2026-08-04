#include "orvd/configuration/load_track_geometry.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace orvd::configuration {
namespace {

using Json = nlohmann::json;
using track_geometry::TrackScalarProfile;
using track_geometry::TrackScalarSegment;
using track_geometry::TrackScalarSegmentShape;
using track_geometry::TrackSeamTransition;

struct ParseFrame {
    enum class Kind { kObject, kArray };

    Kind kind{};
    std::string component_from_parent;
    std::unordered_set<std::string> object_keys;
    std::string pending_object_key;
    std::size_t next_array_index{};
};

std::string PendingValueComponent(const ParseFrame& frame) {
    if (frame.kind == ParseFrame::Kind::kObject) {
        return "." + frame.pending_object_key;
    }
    return "[" + std::to_string(frame.next_array_index) + "]";
}

void CompleteValue(ParseFrame& frame) {
    if (frame.kind == ParseFrame::Kind::kObject) {
        frame.pending_object_key.clear();
    } else {
        ++frame.next_array_index;
    }
}

[[noreturn]] void ThrowExpected(const std::string& path,
                                std::string_view expected) {
    throw std::invalid_argument(path + " must be " + std::string(expected));
}

bool HasNonzeroSignificand(std::string_view token) {
    const std::size_t exponent = token.find_first_of("eE");
    const std::string_view significand = token.substr(0, exponent);
    return std::any_of(significand.begin(), significand.end(), [](char digit) {
        return digit >= '1' && digit <= '9';
    });
}

class StrictJsonValidator final : public nlohmann::json_sax<Json> {
   public:
    bool null() override { return CompleteScalar(); }
    bool boolean(bool) override { return CompleteScalar(); }
    bool number_integer(number_integer_t) override { return CompleteScalar(); }
    bool number_unsigned(number_unsigned_t) override {
        return CompleteScalar();
    }
    bool number_float(number_float_t value, const string_t& token) override {
        if (value == 0.0 && HasNonzeroSignificand(token)) {
            throw std::invalid_argument(
                CurrentValuePath() +
                " contains a non-zero JSON number that underflows binary64");
        }
        return CompleteScalar();
    }
    bool string(string_t&) override { return CompleteScalar(); }
    bool binary(binary_t&) override { return CompleteScalar(); }

    bool start_object(std::size_t) override {
        PushFrame(ParseFrame::Kind::kObject);
        return true;
    }
    bool key(string_t& key) override {
        if (frames_.empty() ||
            frames_.back().kind != ParseFrame::Kind::kObject) {
            throw std::invalid_argument(
                "invalid JSON parser event while reading object key");
        }
        ParseFrame& frame = frames_.back();
        if (!frame.object_keys.insert(key).second) {
            throw std::invalid_argument("duplicate JSON object key at " +
                                        ContainerPath() + "." + key);
        }
        frame.pending_object_key = key;
        return true;
    }
    bool end_object() override { return PopFrame(); }
    bool start_array(std::size_t) override {
        PushFrame(ParseFrame::Kind::kArray);
        return true;
    }
    bool end_array() override { return PopFrame(); }

    bool parse_error(std::size_t position, const std::string&,
                     const nlohmann::detail::exception& error) override {
        if (error.id == 406) {
            throw std::invalid_argument(
                CurrentValuePath() +
                " contains a JSON number that is not representable as a "
                "finite binary64 value: " +
                error.what());
        }
        throw std::invalid_argument("invalid JSON syntax at byte " +
                                    std::to_string(position) + ": " +
                                    error.what());
    }

   private:
    [[nodiscard]] std::string ContainerPath() const {
        std::string path{"$"};
        for (const ParseFrame& frame : frames_) {
            path += frame.component_from_parent;
        }
        return path;
    }

    [[nodiscard]] std::string CurrentValuePath() const {
        if (frames_.empty()) {
            return "$";
        }
        return ContainerPath() + PendingValueComponent(frames_.back());
    }

    void PushFrame(ParseFrame::Kind kind) {
        ParseFrame frame;
        frame.kind = kind;
        if (!frames_.empty()) {
            frame.component_from_parent =
                PendingValueComponent(frames_.back());
        }
        frames_.push_back(std::move(frame));
    }

    bool PopFrame() {
        if (frames_.empty()) {
            throw std::invalid_argument(
                "invalid JSON parser event while closing a container");
        }
        frames_.pop_back();
        if (!frames_.empty()) {
            CompleteValue(frames_.back());
        }
        return true;
    }

    bool CompleteScalar() {
        if (!frames_.empty()) {
            CompleteValue(frames_.back());
        }
        return true;
    }

    std::vector<ParseFrame> frames_;
};

Json ParseStrictJson(const std::string& document) {
    const std::size_t nul_position = document.find('\0');
    if (nul_position != std::string::npos) {
        throw std::invalid_argument(
            "invalid JSON syntax: NUL byte at byte " +
            std::to_string(nul_position + 1) +
            " would terminate the document before the end of the file");
    }

    // The SAX pass sees duplicate keys and raw number tokens, both of which a
    // completed DOM has already normalised away. It stores only one local path
    // component per nesting level; the full path is assembled only on error.
    StrictJsonValidator validator;
    if (!Json::sax_parse(document, &validator, Json::input_format_t::json,
                         true)) {
        throw std::invalid_argument("invalid JSON syntax");
    }
    return Json::parse(document);
}

bool IsExactlyRepresentableAsBinary64(std::uint64_t magnitude) {
    const unsigned int width = std::bit_width(magnitude);
    if (width <= 53U) {
        return true;
    }
    const unsigned int discarded_bits = width - 53U;
    const std::uint64_t discarded_mask =
        (std::uint64_t{1} << discarded_bits) - 1U;
    return (magnitude & discarded_mask) == 0U;
}

void RequireObject(const Json& value, const std::string& path) {
    if (!value.is_object()) {
        ThrowExpected(path, "a JSON object");
    }
}

void RequireArray(const Json& value, const std::string& path) {
    if (!value.is_array()) {
        ThrowExpected(path, "a JSON array");
    }
}

void RequireExactKeys(const Json& object, const std::string& path,
                      std::initializer_list<std::string_view> expected_keys) {
    RequireObject(object, path);
    for (const auto& [key, unused] : object.items()) {
        (void)unused;
        if (std::find(expected_keys.begin(), expected_keys.end(), key) ==
            expected_keys.end()) {
            throw std::invalid_argument(path + "." + key +
                                        " is an unknown key");
        }
    }
    for (const std::string_view key : expected_keys) {
        if (!object.contains(std::string(key))) {
            throw std::invalid_argument(path + "." + std::string(key) +
                                        " is required");
        }
    }
}

double RequireFiniteNumber(const Json& value, const std::string& path) {
    if (!value.is_number()) {
        ThrowExpected(path, "a finite JSON number");
    }
    // A JSON integer states an exact value, so do not silently round it while
    // converting to binary64. Decimal floating tokens intentionally retain the
    // usual binary64 rounding semantics.
    if (value.is_number_unsigned() &&
        !IsExactlyRepresentableAsBinary64(value.get<std::uint64_t>())) {
        ThrowExpected(path, "exactly representable as a finite binary64 number");
    }
    if (value.is_number_integer() && !value.is_number_unsigned()) {
        const std::int64_t signed_value = value.get<std::int64_t>();
        const std::uint64_t magnitude =
            signed_value < 0
                ? static_cast<std::uint64_t>(-(signed_value + 1)) + 1U
                : static_cast<std::uint64_t>(signed_value);
        if (!IsExactlyRepresentableAsBinary64(magnitude)) {
            ThrowExpected(path,
                          "exactly representable as a finite binary64 number");
        }
    }
    double result{};
    try {
        result = value.get<double>();
    } catch (const Json::exception&) {
        ThrowExpected(path, "a finite binary64 number");
    }
    if (!std::isfinite(result)) {
        ThrowExpected(path, "a finite binary64 number");
    }
    return result;
}

std::string RequireString(const Json& value, const std::string& path) {
    if (!value.is_string()) {
        ThrowExpected(path, "a JSON string");
    }
    return value.get<std::string>();
}

std::size_t RequireIndex(const Json& value, const std::string& path) {
    if (!value.is_number_unsigned()) {
        ThrowExpected(path, "a non-negative JSON integer");
    }
    const std::uint64_t result = value.get<std::uint64_t>();
    if (result > std::numeric_limits<std::size_t>::max()) {
        ThrowExpected(path, "an integer representable as size_t");
    }
    return static_cast<std::size_t>(result);
}

struct ProfileFieldNames {
    std::string_view constant_value;
    std::string_view start_value;
    std::string_view end_value;
};

TrackScalarSegment ParseSegment(const Json& value, const std::string& path,
                                const ProfileFieldNames& fields) {
    RequireObject(value, path);
    if (!value.contains("shape")) {
        throw std::invalid_argument(path + ".shape is required");
    }
    const std::string shape = RequireString(value.at("shape"), path + ".shape");

    TrackScalarSegment result;
    if (shape == "constant") {
        RequireExactKeys(value, path,
                         {"shape", "length_meters", fields.constant_value});
        result.shape = TrackScalarSegmentShape::kConstant;
        result.length_meters =
            RequireFiniteNumber(value.at("length_meters"),
                                path + ".length_meters");
        result.start_value =
            RequireFiniteNumber(value.at(std::string(fields.constant_value)),
                                path + "." +
                                    std::string(fields.constant_value));
        result.end_value = result.start_value;
        return result;
    }
    if (shape == "hermite_cubic_blend") {
        RequireExactKeys(value, path,
                         {"shape", "length_meters", fields.start_value,
                          fields.end_value});
        result.shape = TrackScalarSegmentShape::kHermiteCubicBlend;
        result.length_meters =
            RequireFiniteNumber(value.at("length_meters"),
                                path + ".length_meters");
        result.start_value =
            RequireFiniteNumber(value.at(std::string(fields.start_value)),
                                path + "." + std::string(fields.start_value));
        result.end_value =
            RequireFiniteNumber(value.at(std::string(fields.end_value)),
                                path + "." + std::string(fields.end_value));
        return result;
    }
    ThrowExpected(path + ".shape",
                  "'constant' or 'hermite_cubic_blend'");
}

TrackScalarProfile ParseProfile(const Json& value, const std::string& path,
                                double start_track_station_meters,
                                const ProfileFieldNames& fields) {
    RequireExactKeys(value, path, {"segments", "seam_transitions"});
    const Json& segments_json = value.at("segments");
    const Json& seams_json = value.at("seam_transitions");
    RequireArray(segments_json, path + ".segments");
    RequireArray(seams_json, path + ".seam_transitions");

    std::vector<TrackScalarSegment> segments;
    segments.reserve(segments_json.size());
    for (std::size_t index = 0; index < segments_json.size(); ++index) {
        segments.push_back(ParseSegment(
            segments_json[index],
            path + ".segments[" + std::to_string(index) + "]", fields));
    }

    std::vector<TrackSeamTransition> seams;
    seams.reserve(seams_json.size());
    for (std::size_t index = 0; index < seams_json.size(); ++index) {
        const std::string seam_path =
            path + ".seam_transitions[" + std::to_string(index) + "]";
        const Json& seam_json = seams_json[index];
        RequireExactKeys(seam_json, seam_path,
                         {"preceding_segment_index", "window_length_meters"});
        seams.push_back(TrackSeamTransition{
            RequireIndex(seam_json.at("preceding_segment_index"),
                         seam_path + ".preceding_segment_index"),
            RequireFiniteNumber(seam_json.at("window_length_meters"),
                                seam_path + ".window_length_meters")});
    }
    return TrackScalarProfile(start_track_station_meters, std::move(segments),
                              std::move(seams));
}

}  // namespace

track_geometry::TrackGeometry LoadTrackGeometryFromJsonFile(
    const std::filesystem::path& configuration_path) {
    std::ifstream input(configuration_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open track geometry configuration '" +
                                 configuration_path.string() + "'");
    }
    std::string document;
    std::array<char, 4096> buffer{};
    while (input.read(buffer.data(),
                      static_cast<std::streamsize>(buffer.size())) ||
           input.gcount() > 0) {
        document.append(buffer.data(),
                        static_cast<std::size_t>(input.gcount()));
    }
    if (!input.eof()) {
        throw std::runtime_error("could not read track geometry configuration '" +
                                 configuration_path.string() + "'");
    }

    const Json root = ParseStrictJson(document);
    RequireExactKeys(
        root, "$",
        {"schema_version", "start_track_station_meters",
         "station_node_spacing_meters",
         "rail_reference_lateral_span_meters", "curvature_profile",
         "superelevation_profile", "centerline_upward_grade_profile"});

    if (!root.at("schema_version").is_number_integer() ||
        root.at("schema_version") != 1) {
        ThrowExpected("$.schema_version", "the integer 1");
    }
    const double start_track_station_meters = RequireFiniteNumber(
        root.at("start_track_station_meters"), "$.start_track_station_meters");
    const double station_node_spacing_meters = RequireFiniteNumber(
        root.at("station_node_spacing_meters"),
        "$.station_node_spacing_meters");
    const double rail_reference_lateral_span_meters = RequireFiniteNumber(
        root.at("rail_reference_lateral_span_meters"),
        "$.rail_reference_lateral_span_meters");

    TrackScalarProfile curvature = ParseProfile(
        root.at("curvature_profile"), "$.curvature_profile",
        start_track_station_meters,
        {"curvature_radians_per_meter",
         "start_curvature_radians_per_meter",
         "end_curvature_radians_per_meter"});
    TrackScalarProfile superelevation = ParseProfile(
        root.at("superelevation_profile"), "$.superelevation_profile",
        start_track_station_meters,
        {"superelevation_meters", "start_superelevation_meters",
         "end_superelevation_meters"});
    TrackScalarProfile grade = ParseProfile(
        root.at("centerline_upward_grade_profile"),
        "$.centerline_upward_grade_profile", start_track_station_meters,
        {"centerline_upward_grade", "start_centerline_upward_grade",
         "end_centerline_upward_grade"});

    return track_geometry::TrackGeometry(
        std::move(curvature), std::move(superelevation), std::move(grade),
        rail_reference_lateral_span_meters, station_node_spacing_meters);
}

}  // namespace orvd::configuration
