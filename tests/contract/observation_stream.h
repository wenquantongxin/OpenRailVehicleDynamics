// Transport for observations between two processes of this test harness.
//
// A stream carries only what a side observed:
//   @topology    <name> <integer>
//   @observation <name> <ieee754_binary64_payload_hex>
//
// The payload is the raw 64-bit pattern because that is a lossless, unambiguous
// transport between two programs we build ourselves; it is NOT the acceptance
// rule. Acceptance is a tolerance judgement made by the comparator.
//
// Parsing is loose on form. Field order and unused fields are irrelevant, and a
// repeated singleton takes the last value. A malformed observation is retained
// as unusable so the comparator can fail only if it actually requires that name.
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "contract/observation_semantics.h"

namespace orvd_contract {

struct UnusableObservation {
    std::string name;
    std::string reason;
};

struct ObservationStream {
    std::vector<TopologyFact> topology_facts;
    std::vector<Observation> observations;
    std::vector<UnusableObservation> unusable_observations;

    [[nodiscard]] const Observation* FindObservation(std::string_view name) const;
    [[nodiscard]] const TopologyFact* FindTopologyFact(std::string_view name) const;
    [[nodiscard]] const UnusableObservation* FindUnusableObservation(
        std::string_view name) const;
};

// Serialises to the directives above. Names are written as given.
[[nodiscard]] std::string FormatObservationStream(const ObservationStream& stream);

// Reads a stream. Unrecognised directives and trailing fields are ignored; a
// repeated name keeps the last occurrence, including an unusable last value.
[[nodiscard]] ObservationStream ParseObservationStream(std::string_view text);

}  // namespace orvd_contract
