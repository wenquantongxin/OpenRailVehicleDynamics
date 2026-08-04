#pragma once

#include <cstddef>
#include <filesystem>
#include <initializer_list>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

// Strict reading of human-authored product configuration, shared by every
// record this module loads.
//
// This header is not installed and nlohmann's types do not appear in any public
// ORVD interface: a consumer of an installed ORVD receives typed objects and
// never learns which parser produced them. The rules here are the ones a
// human-authored file has to satisfy before its numbers are allowed to decide a
// product result — duplicate keys, unknown keys, missing keys, wrong types, and
// numbers the file states but binary64 cannot hold.
//
// There is deliberately no schema engine, no field table and no reflection. Each
// record spells out its own keys at the point where it reads them, so a reader
// of that record's loader sees the whole contract in one place.

namespace orvd::configuration::strict_json {

using Json = nlohmann::json;

// Reads the whole file as bytes. `what` names the kind of configuration for the
// diagnostic, for example "track geometry configuration".
//
// Throws std::runtime_error when the file cannot be opened or cannot be read to
// its end; both diagnostics name the path.
[[nodiscard]] std::string ReadWholeFile(
    const std::filesystem::path& configuration_path, std::string_view what);

// Parses one document under the strict rules and returns the resulting DOM.
//
// A SAX pre-pass sees what a completed DOM has already normalised away: repeated
// object keys, and the raw text of each number. It therefore refuses a duplicate
// key with the full JSON path of the offending object, and refuses a non-zero
// decimal token that underflows binary64 to zero. An embedded NUL is refused
// too: the lexer would treat it as end of input and silently ignore everything
// after it.
//
// Throws std::invalid_argument for any syntax error, duplicate key, embedded
// NUL, or number the document states but binary64 cannot represent.
[[nodiscard]] Json ParseStrictJson(const std::string& document);

// Throws std::invalid_argument reading "<path> must be <expected>".
[[noreturn]] void ThrowExpected(const std::string& path,
                                std::string_view expected);

void RequireObject(const Json& value, const std::string& path);
void RequireArray(const Json& value, const std::string& path);

// The all-and-only rule for one object: every key present must be expected, and
// every expected key must be present. This is what keeps a mistyped key from
// being read as an absent one, and a field nobody reads from sitting unnoticed
// in a product record.
void RequireExactKeys(const Json& object, const std::string& path,
                      std::initializer_list<std::string_view> expected_keys);

// A number the document states and binary64 holds exactly enough to use.
//
// A JSON integer states an exact value, so it is refused rather than silently
// rounded when it does not fit binary64's significand. A decimal floating token
// keeps the ordinary binary64 rounding semantics — that is what writing 0.1
// means — except that a non-zero token underflowing all the way to zero is
// refused by the pre-pass, because zero is not what the author wrote.
[[nodiscard]] double RequireFiniteNumber(const Json& value,
                                         const std::string& path);

[[nodiscard]] std::string RequireString(const Json& value,
                                        const std::string& path);
[[nodiscard]] bool RequireBool(const Json& value, const std::string& path);
[[nodiscard]] std::size_t RequireIndex(const Json& value,
                                       const std::string& path);

}  // namespace orvd::configuration::strict_json
