#include "contract/observation_stream.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>

namespace orvd_contract {
namespace {

std::string FormatBinary64Payload(double value) {
    std::uint64_t payload;
    std::memcpy(&payload, &value, sizeof(payload));
    char text[17];
    std::snprintf(text, sizeof(text), "%016llx",
                  static_cast<unsigned long long>(payload));
    return text;
}

bool ParseBinary64Payload(std::string_view text, double* value) {
    std::uint64_t payload = 0;
    const auto result =
        std::from_chars(text.data(), text.data() + text.size(), payload, 16);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
        return false;
    std::memcpy(value, &payload, sizeof(*value));
    return true;
}

}  // namespace

bool ParseObservationKind(std::string_view name, ObservationKind* kind) {
    constexpr ObservationKind kAllKinds[] = {
        ObservationKind::kAngleRadians,
        ObservationKind::kUnitQuaternionComponent,
        ObservationKind::kTranslationMeters,
        ObservationKind::kForceNewtons,
        ObservationKind::kTorqueNewtonMetres,
        ObservationKind::kRotationMatrixElement,
    };
    for (const ObservationKind candidate : kAllKinds) {
        if (ObservationKindName(candidate) == name) {
            *kind = candidate;
            return true;
        }
    }
    return false;
}

const Observation* ObservationStream::FindObservation(std::string_view name) const {
    for (const Observation& observation : observations)
        if (observation.name == name) return &observation;
    return nullptr;
}

const TopologyFact* ObservationStream::FindTopologyFact(std::string_view name) const {
    for (const TopologyFact& fact : topology_facts)
        if (fact.name == name) return &fact;
    return nullptr;
}

std::string FormatObservationStream(const ObservationStream& stream) {
    std::string text;
    for (const TopologyFact& fact : stream.topology_facts)
        text += "@topology " + fact.name + " " + std::to_string(fact.value) + "\n";
    for (const Observation& observation : stream.observations)
        text += "@observation " + observation.name + " " +
                std::string(ObservationKindName(observation.kind)) + " " +
                FormatBinary64Payload(observation.value) + "\n";
    return text;
}

bool ParseObservationStream(std::string_view text, ObservationStream* stream,
                            std::string* parse_error) {
    std::istringstream lines{std::string(text)};
    std::string line;
    while (std::getline(lines, line)) {
        std::istringstream fields(line);
        std::string directive;
        if (!(fields >> directive)) continue;

        if (directive == "@topology") {
            std::string name;
            long long value = 0;
            // A topology fact that does not parse is a fact we cannot compare,
            // so it is dropped rather than guessed at; a required one missing is
            // caught by the comparator, which knows which ones it needs.
            if (!(fields >> name >> value)) continue;
            TopologyFact fact{name, value};
            bool replaced = false;
            for (TopologyFact& existing : stream->topology_facts) {
                if (existing.name == name) { existing = fact; replaced = true; break; }
            }
            if (!replaced) stream->topology_facts.push_back(fact);
        } else if (directive == "@observation") {
            std::string name, kind_name, payload_text;
            if (!(fields >> name >> kind_name >> payload_text)) continue;

            ObservationKind kind{};
            if (!ParseObservationKind(kind_name, &kind)) {
                *parse_error = "observation '" + name + "' declares unknown kind '" +
                               kind_name + "'";
                return false;
            }
            double value = 0.0;
            if (!ParseBinary64Payload(payload_text, &value)) {
                *parse_error = "observation '" + name + "' has a malformed payload";
                return false;
            }
            if (!std::isfinite(value)) {
                *parse_error = "observation '" + name + "' is not finite";
                return false;
            }
            Observation observation{name, kind, value};
            bool replaced = false;
            for (Observation& existing : stream->observations) {
                if (existing.name == name) { existing = observation; replaced = true; break; }
            }
            if (!replaced) stream->observations.push_back(observation);
        }
        // Any other directive belongs to something this reader does not consume.
    }
    return true;
}

}  // namespace orvd_contract
