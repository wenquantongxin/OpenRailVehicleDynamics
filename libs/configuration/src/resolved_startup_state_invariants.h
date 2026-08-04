#pragma once

#include <string>

#include "orvd/configuration/resolved_startup_state.h"

// The invariants a resolved start-up state has to satisfy on its own, with no
// second object in hand.
//
// Both entries into the record run these: the strict loader, so a malformed
// document is refused at the layer that can name a JSON path, and the start-up
// assembly, because the record is an ordinary C++ value a researcher may build
// or edit directly and "it once came through the loader" is not something the
// assembly can assume.
//
// `origin` prefixes each diagnostic. The loader passes the JSON root so a
// message reads as a path into the document; the assembly passes a phrase
// naming the argument.

namespace orvd::configuration::internal {

// Throws std::invalid_argument on the first violation, naming the offending
// field and its value.
void RequireResolvedStartupStateInvariants(const ResolvedStartupState& state,
                                           const std::string& origin);

}  // namespace orvd::configuration::internal
