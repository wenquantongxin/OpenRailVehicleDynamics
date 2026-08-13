#include "orvd/wheel_rail_contact/wheel_profile_preprocessing.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "orvd/wheel_rail_contact/natural_cubic_spline.h"
#include "orvd/wheel_rail_contact/profile_arc_length.h"

namespace orvd::wheel_rail_contact {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("wheel profile preprocessing: " + detail);
}

void RequireWheelProfile(const ProfilePoints& authored) {
    if (authored.role() != ProfileRole::kWheel) {
        Reject("'" + authored.identifier() +
               "' is a rail profile; only a wheel profile is prepared this way");
    }
}

double RescanEndTolerance(double total, double step) {
    const double scale = std::max({1.0, std::abs(total), std::abs(step)});
    return 16.0 * std::numeric_limits<double>::epsilon() * scale;
}

std::vector<double> SampleStations(double first, double last,
                                   std::size_t sample_count) {
    std::vector<double> stations(sample_count, 0.0);
    const double span = last - first;
    const double divisor = static_cast<double>(sample_count - 1);
    for (std::size_t index = 0; index < sample_count; ++index) {
        stations[index] =
            first + span * (static_cast<double>(index) / divisor);
    }
    return stations;
}

}  // namespace

WheelProfilePreprocessing::WheelProfilePreprocessing(
    WheelProfilePreprocessingConfiguration configuration) {
    const double rescan = configuration.equal_arc_length_rescan_step_meters;
    if (!std::isfinite(rescan) || rescan < 0.0) {
        Reject("the equal arc-length rescan step is " + std::to_string(rescan) +
               " m; it must be finite and not negative, and zero preserves the "
               "authored nodes");
    }
    step_meters_ = rescan;
}

WheelProfileControlNodes WheelProfilePreprocessing::LayControlNodes(
    const ProfilePoints& authored, WheelSide side) const {
    RequireWheelProfile(authored);
    WheelProfileControlNodes nodes;
    if (step_meters_ == 0.0) {
        return nodes;
    }

    // The rescan runs on the physical right-hand ordering, whatever side it is
    // being laid for, and the mirror is applied to its result. Anchoring the
    // station phase on a mirrored list would put the remainder segment at the
    // opposite end of the profile.
    const SideResolvedProfile physical =
        authored.ResolveForSide(WheelSide::kRight);
    const NaturalCubicSpline first_pass(physical.lateral_meters(),
                                        physical.vertical_meters());
    const std::vector<double> cumulative =
        AccumulateProfileArcLength(first_pass, physical.lateral_meters());
    const double total = cumulative.back();
    if (!std::isfinite(total) || !(total > 0.0)) {
        Reject("'" + authored.identifier() + "' has no measurable curve length");
    }
    const double tolerance = RescanEndTolerance(total, step_meters_);

    std::vector<double> lateral;
    std::vector<double> vertical;
    lateral.push_back(physical.lateral_meters().front());
    vertical.push_back(physical.vertical_meters().front());
    for (std::size_t station = 1;; ++station) {
        const double target = static_cast<double>(station) * step_meters_;
        if (target >= total - tolerance) {
            break;
        }
        const double at = FindLateralCoordinateAtArcLength(
            first_pass, physical.lateral_meters(), cumulative, target);
        lateral.push_back(at);
        vertical.push_back(first_pass.Evaluate(at));
    }
    // The physical endpoints are copied, never re-evaluated: they are where
    // the asset says the surface ends, and a spline's value there is only equal
    // to that up to rounding.
    lateral.push_back(physical.lateral_meters().back());
    vertical.push_back(physical.vertical_meters().back());

    if (side == WheelSide::kLeft) {
        std::vector<double> mirrored_lateral(lateral.size());
        std::vector<double> mirrored_vertical(vertical.size());
        for (std::size_t index = 0; index < lateral.size(); ++index) {
            const std::size_t source = lateral.size() - 1 - index;
            mirrored_lateral[index] = -lateral[source];
            mirrored_vertical[index] = vertical[source];
        }
        lateral = std::move(mirrored_lateral);
        vertical = std::move(mirrored_vertical);
    }

    nodes.lateral_meters = std::move(lateral);
    nodes.vertical_meters = std::move(vertical);
    nodes.total_arc_length_meters = total;
    return nodes;
}

WheelProfileOutline WheelProfilePreprocessing::SampleVisibleOutline(
    const ProfilePoints& authored, WheelSide side,
    std::size_t sample_count) const {
    RequireWheelProfile(authored);
    if (sample_count < 2) {
        Reject("the outline needs at least two stations, but " +
               std::to_string(sample_count) + " were asked for");
    }

    const WheelProfileControlNodes nodes = LayControlNodes(authored, side);
    WheelProfileOutline outline;

    if (step_meters_ > 0.0) {
        // The rescan replaced the profile nodes, so the outline is sampled from
        // a spline through the new ones. The span is unchanged because the
        // endpoints were copied.
        const NaturalCubicSpline second_pass(nodes.lateral_meters,
                                             nodes.vertical_meters);
        outline.station_meters = SampleStations(nodes.lateral_meters.front(),
                                                nodes.lateral_meters.back(),
                                                sample_count);
        outline.height_meters.resize(sample_count);
        outline.height_slope.resize(sample_count);
        for (std::size_t index = 0; index < sample_count; ++index) {
            const double station = outline.station_meters[index];
            outline.height_meters[index] = second_pass.Evaluate(station);
            outline.height_slope[index] =
                second_pass.EvaluateFirstDerivative(station);
        }
        return outline;
    }

    const SideResolvedProfile resolved = authored.ResolveForSide(side);
    outline.station_meters = SampleStations(resolved.lateral_meters().front(),
                                            resolved.lateral_meters().back(),
                                            sample_count);
    outline.height_meters.resize(sample_count);
    outline.height_slope.resize(sample_count);

    const NaturalCubicSpline surface(resolved.lateral_meters(),
                                     resolved.vertical_meters());
    for (std::size_t index = 0; index < sample_count; ++index) {
        const double station = outline.station_meters[index];
        outline.height_meters[index] = surface.Evaluate(station);
        outline.height_slope[index] = surface.EvaluateFirstDerivative(station);
    }
    return outline;
}

}  // namespace orvd::wheel_rail_contact
