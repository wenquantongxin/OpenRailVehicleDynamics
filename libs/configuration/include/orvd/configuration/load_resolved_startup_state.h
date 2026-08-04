#pragma once

#include <filesystem>

#include "orvd/configuration/resolved_startup_state.h"

namespace orvd::configuration {

// Loads one resolved start-up state from the exact path the caller supplies.
//
// It performs no path search, no environment substitution, no default insertion
// and no retained DOM storage. The document is read once and becomes a typed
// value; nothing downstream touches JSON again.
//
// What this layer refuses is what only this layer can name against a JSON path:
// a missing, unknown or repeated key, a wrong type, a number the file states
// but binary64 cannot hold, an identifier outside its character set, a running
// direction outside the closed vocabulary, and the record's own numeric
// invariants — every value finite, each quaternion of unit norm, the speed and
// gravity strictly positive, each target wheel load positive, and no name
// repeated inside its family.
//
// What it deliberately does not do is anything that needs a second object. It
// does not know how many free bodies a vehicle has, does not compare an
// identifier against a vehicle definition, and does not look at a line. Those
// are checked where both sides are in hand, when a start-up context is
// assembled. Checking them twice would give one mistake two diagnostics and let
// the two drift apart.
//
// The returned value is an ordinary C++ value a researcher may edit before
// assembling it. That is why the assembly entry re-checks these invariants
// rather than assuming a record reached it through this function.
//
// Throws std::runtime_error when the file cannot be opened or read.
// Throws std::invalid_argument for every other refusal above; the diagnostic
// names the JSON path and, where there is one, the offending value.
[[nodiscard]] ResolvedStartupState LoadResolvedStartupStateFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
