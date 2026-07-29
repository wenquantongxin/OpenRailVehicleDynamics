#include "contract/observation_stream.h"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <sstream>

namespace orvd_contract {
namespace {

std::string FormatBinary64Payload(double value) {
    static_assert(sizeof(double) == sizeof(std::uint64_t));
    static_assert(std::numeric_limits<double>::is_iec559);
    const std::uint64_t payload = std::bit_cast<std::uint64_t>(value);
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
    *value = std::bit_cast<double>(payload);
    return true;
}

template <typename Record>
void EraseRecordNamed(std::vector<Record>* records, std::string_view name) {
    std::erase_if(*records, [name](const Record& record) {
        return record.name == name;
    });
}

}  // namespace

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

const UnusableObservation* ObservationStream::FindUnusableObservation(
    std::string_view name) const {
    for (const UnusableObservation& observation : unusable_observations)
        if (observation.name == name) return &observation;
    return nullptr;
}

std::string FormatObservationStream(const ObservationStream& stream) {
    std::string text;
    for (const TopologyFact& fact : stream.topology_facts)
        text += "@topology " + fact.name + " " + std::to_string(fact.value) + "\n";
    for (const Observation& observation : stream.observations)
        text += "@observation " + observation.name + " " +
                FormatBinary64Payload(observation.value) + "\n";
    return text;
}

ObservationStream ParseObservationStream(std::string_view text) {
    ObservationStream stream;
    std::istringstream lines{std::string(text)};
    std::string line;
    while (std::getline(lines, line)) {
        std::istringstream fields(line);
        std::string directive;
        if (!(fields >> directive)) continue;

        if (directive == "@topology") {
            std::string name;
            long long value = 0;
            if (!(fields >> name)) continue;
            EraseRecordNamed(&stream.topology_facts, name);
            if (!(fields >> value)) continue;
            TopologyFact fact{name, value};
            stream.topology_facts.push_back(std::move(fact));
        } else if (directive == "@observation") {
            std::string name;
            if (!(fields >> name)) continue;
            EraseRecordNamed(&stream.observations, name);
            EraseRecordNamed(&stream.unusable_observations, name);

            std::string payload_text;
            if (!(fields >> payload_text)) {
                stream.unusable_observations.push_back(
                    {std::move(name), "payload is missing"});
                continue;
            }
            double value = 0.0;
            if (!ParseBinary64Payload(payload_text, &value)) {
                stream.unusable_observations.push_back(
                    {std::move(name), "payload is malformed"});
                continue;
            }
            if (!std::isfinite(value)) {
                stream.unusable_observations.push_back(
                    {std::move(name), "value is not finite"});
                continue;
            }
            stream.observations.push_back({std::move(name), value});
        }
        // Any other directive belongs to something this reader does not consume.
    }
    return stream;
}

}  // namespace orvd_contract
