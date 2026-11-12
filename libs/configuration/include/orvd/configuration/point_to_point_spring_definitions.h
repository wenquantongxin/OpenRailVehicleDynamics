#pragma once

#include <string>

namespace orvd::configuration {

// Frame names are local to a vehicle record or qualified in a consist record.
struct PointToPointLinearSpringDamperDefinition {
    std::string name;
    std::string reference_frame_name;
    std::string opposite_frame_name;
    double nominal_length_meters{};
    double stiffness_newtons_per_meter{};
    double damping_newton_seconds_per_meter{};
};

struct PointToPointHystereticSpringDefinition {
    std::string name;
    std::string reference_frame_name;
    std::string opposite_frame_name;
    double nominal_length_meters{};
    double stiffness_newtons_per_meter{};
    double hysteresis_ratio{};
    double hysteresis_transition_speed_meters_per_second{};
};

}  // namespace orvd::configuration
