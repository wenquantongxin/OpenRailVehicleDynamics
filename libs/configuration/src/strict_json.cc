#include "strict_json.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace orvd::configuration::strict_json {
namespace {

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

}  // namespace

std::string ReadWholeFile(const std::filesystem::path& configuration_path,
                          std::string_view what) {
    std::ifstream input(configuration_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open " + std::string(what) + " '" +
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
        throw std::runtime_error("could not read " + std::string(what) + " '" +
                                 configuration_path.string() + "'");
    }
    return document;
}

[[noreturn]] void ThrowExpected(const std::string& path,
                                std::string_view expected) {
    throw std::invalid_argument(path + " must be " + std::string(expected));
}

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

namespace {

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

}  // namespace

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

bool RequireBool(const Json& value, const std::string& path) {
    if (!value.is_boolean()) {
        ThrowExpected(path, "a JSON boolean");
    }
    return value.get<bool>();
}

std::string RequireIdentifier(const Json& value, const std::string& path,
                              std::string_view what) {
    const std::string identifier = RequireString(value, path);
    if (identifier.empty()) {
        ThrowExpected(path, std::string("a non-empty ") + std::string(what));
    }
    for (const char character : identifier) {
        const bool admitted = (character >= 'A' && character <= 'Z') ||
                              (character >= 'a' && character <= 'z') ||
                              (character >= '0' && character <= '9') ||
                              character == '.' || character == '_' ||
                              character == '-';
        if (!admitted) {
            throw std::invalid_argument(
                path + " is a " + std::string(what) + " '" + identifier +
                "' containing a character outside [A-Za-z0-9._-]; an "
                "identifier is a name, not a path or a sentence");
        }
    }
    if (identifier.find("..") != std::string::npos) {
        throw std::invalid_argument(path + " is a " + std::string(what) +
                                    " '" + identifier +
                                    "' containing '..'; an identifier names "
                                    "an object, it does not traverse to one");
    }
    return identifier;
}

Eigen::Vector3d RequireFiniteVector3(const Json& value,
                                     const std::string& path) {
    RequireExactKeys(value, path, {"x", "y", "z"});
    return Eigen::Vector3d(RequireFiniteNumber(value.at("x"), path + ".x"),
                           RequireFiniteNumber(value.at("y"), path + ".y"),
                           RequireFiniteNumber(value.at("z"), path + ".z"));
}

std::string ElementPath(const std::string& array_path, std::size_t index) {
    return array_path + "[" + std::to_string(index) + "]";
}

}  // namespace orvd::configuration::strict_json
