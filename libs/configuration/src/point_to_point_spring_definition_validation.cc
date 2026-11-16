#include "point_to_point_spring_definition_validation.h"

#include <stdexcept>

namespace orvd::configuration::internal {
namespace {

template <typename Element>
void RequireNames(const Element& element, const std::string& path) {
    const auto require_name = [&](const std::string& name, const char* key) {
        if (name.empty()) {
            throw std::invalid_argument(path + "." + key + " must be a non-empty name");
        }
    };
    require_name(element.name, "name");
    require_name(element.reference_frame_name, "reference_frame_name");
    require_name(element.opposite_frame_name, "opposite_frame_name");
}

}  // namespace

void RequirePointToPointLinearSpringDamperInvariants(
    const PointToPointLinearSpringDamperDefinition& element, const std::string& path) {
    RequireNames(element, path);
    if (!(element.nominal_length_meters > 0.0) || element.stiffness_newtons_per_meter < 0.0 ||
        element.damping_newton_seconds_per_meter < 0.0) {
        throw std::invalid_argument(
            path + " must be positive nominal length and non-negative stiffness and damping");
    }
}

void RequirePointToPointHystereticSpringInvariants(
    const PointToPointHystereticSpringDefinition& element, const std::string& path) {
    RequireNames(element, path);
    if (!(element.nominal_length_meters > 0.0) || element.stiffness_newtons_per_meter < 0.0 ||
        element.hysteresis_ratio < 0.0 || element.hysteresis_ratio > 1.0 ||
        !(element.hysteresis_transition_speed_meters_per_second > 0.0)) {
        throw std::invalid_argument(
            path +
            " must be positive nominal length and transition speed, non-negative stiffness "
            "and hysteresis ratio in [0, 1]");
    }
}

}  // namespace orvd::configuration::internal
