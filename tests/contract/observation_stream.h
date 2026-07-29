// Transport for observations between two processes of this test harness.
//
// A stream carries only what a side observed:
//   @topology    <name> <integer>
//   @observation <name> <kind> <ieee754_binary64_payload_hex>
//
// The payload is the raw 64-bit pattern because that is a lossless, unambiguous
// transport between two programs we build ourselves; it is NOT the acceptance
// rule. Acceptance is a tolerance judgement made by the comparator.
//
// Parsing is loose on form and strict on substance. The producer is a process
// this harness started, so field order, extra fields and unused fields are
// accepted, and a repeated singleton takes the last value. What is checked is
// what will actually be used: that a required observation is present, that it
// parses, that the dimensions line up, and that a value that must be finite is.
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "contract/observation_semantics.h"

namespace orvd_contract {

struct ObservationStream {
    std::vector<TopologyFact> topology_facts;
    std::vector<Observation> observations;

    [[nodiscard]] const Observation* FindObservation(std::string_view name) const;
    [[nodiscard]] const TopologyFact* FindTopologyFact(std::string_view name) const;
};

// Serialises to the directives above. Names are written as given.
[[nodiscard]] std::string FormatObservationStream(const ObservationStream& stream);

// Reads a stream. Unrecognised directives and trailing fields are ignored; a
// repeated name keeps the last occurrence. Returns false and sets
// `parse_error` only when something that will be used cannot be understood: a
// malformed payload, an unknown observation kind, or a non-finite value.
[[nodiscard]] bool ParseObservationStream(std::string_view text,
                                          ObservationStream* stream,
                                          std::string* parse_error);

}  // namespace orvd_contract
