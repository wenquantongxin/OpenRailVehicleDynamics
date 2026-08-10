#include "vehicle_definition_invariants.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace orvd::configuration::internal {

void RequireVehicleMechanicalTrackStationLayoutInvariants(
    const VehicleDefinition& vehicle, const std::string& origin) {
    const auto& layout = vehicle.mechanical_track_station_layout;
    const std::string path = origin + ".mechanical_track_station_layout";

    std::unordered_set<std::string> free_body_names;
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        if (body.moves_freely_in_world) {
            free_body_names.insert(body.name);
        }
    }

    std::unordered_map<std::string, double> offset_of_body;
    for (const VehicleFreeBodyStationOffsetDefinition& offset :
         layout.free_body_station_offsets) {
        if (offset.body_name.empty()) {
            throw std::invalid_argument(
                path + ".free_body_station_offsets contains an empty body "
                       "name");
        }
        if (!std::isfinite(offset.station_offset_meters)) {
            throw std::invalid_argument(
                path + ".free_body_station_offsets for '" + offset.body_name +
                "' has a station offset that is not finite");
        }
        if (!free_body_names.contains(offset.body_name)) {
            throw std::invalid_argument(
                path + ".free_body_station_offsets names '" +
                offset.body_name +
                "', which this description does not declare as a rigid body "
                "that moves freely in the world");
        }
        if (!offset_of_body
                 .emplace(offset.body_name, offset.station_offset_meters)
                 .second) {
            throw std::invalid_argument(
                path + ".free_body_station_offsets repeats '" +
                offset.body_name + "'");
        }
    }
    for (const std::string& free_body : free_body_names) {
        if (!offset_of_body.contains(free_body)) {
            throw std::invalid_argument(
                path + ".free_body_station_offsets omits the free body '" +
                free_body + "'");
        }
    }

    if (layout.reference_body_name.empty()) {
        throw std::invalid_argument(path + ".reference_body_name is empty");
    }
    const auto reference = offset_of_body.find(layout.reference_body_name);
    if (reference == offset_of_body.end()) {
        throw std::invalid_argument(
            path + ".reference_body_name names '" +
            layout.reference_body_name +
            "', which is not one of the free bodies this layout places");
    }
    if (reference->second != 0.0) {
        throw std::invalid_argument(
            path + ".reference_body_name names '" +
            layout.reference_body_name + "', whose station offset is " +
            std::to_string(reference->second) +
            " m rather than zero; the other offsets are measured from it");
    }

    std::unordered_set<std::string> station_reference_names;
    for (const std::string& name :
         layout.wheel_pair_station_reference_body_names) {
        if (name.empty()) {
            throw std::invalid_argument(
                path +
                ".wheel_pair_station_reference_body_names contains an empty "
                "name");
        }
        if (!offset_of_body.contains(name)) {
            throw std::invalid_argument(
                path + ".wheel_pair_station_reference_body_names names '" +
                name +
                "', which is not one of the free bodies this layout places");
        }
        if (!station_reference_names.insert(name).second) {
            throw std::invalid_argument(path +
                                        ".wheel_pair_station_reference_body_names "
                                        "repeats '" +
                                        name + "'");
        }
    }
}

}  // namespace orvd::configuration::internal
