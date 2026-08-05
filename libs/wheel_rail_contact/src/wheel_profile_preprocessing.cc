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

// The final step of a march is allowed to fall this close to the end before it
// is absorbed into the endpoint instead of producing a node on top of it. The
// floor of one keeps the tolerance absolute for profiles stated in metres,
// where every quantity involved is far below one and a purely relative
// tolerance would collapse to nothing.
double MarchTolerance(double first, double last, double travelled) {
    const double scale = std::max(
        {1.0, std::abs(first), std::abs(last), std::abs(travelled)});
    return 8.0 * std::numeric_limits<double>::epsilon() * scale;
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

// Value of the control polygon at a station, held flat outside its span.
double InterpolateControlValue(const std::vector<double>& lateral,
                               const std::vector<double>& vertical,
                               double station) {
    if (station <= lateral.front()) {
        return vertical.front();
    }
    if (station >= lateral.back()) {
        return vertical.back();
    }
    const auto upper = std::upper_bound(lateral.begin(), lateral.end(), station);
    const std::size_t above = static_cast<std::size_t>(upper - lateral.begin());
    const std::size_t below = above - 1;
    const double left = lateral[below];
    const double right = lateral[above];
    const double parameter = (station - left) / (right - left);
    return vertical[below] + (vertical[above] - vertical[below]) * parameter;
}

// Slope of the control polygon at a station. It is piecewise constant, takes
// the right-hand value at an interior node, and keeps the end segment's slope
// outside the span rather than dropping to zero. That last part is deliberately
// inconsistent with the flat value above: a consumer projecting the outline
// under yaw needs a surface direction outside the control span, and zero is not
// one.
double ControlSlope(const std::vector<double>& lateral,
                    const std::vector<double>& vertical, double station) {
    const std::size_t segment_count = lateral.size() - 1;
    std::size_t segment = 0;
    if (station >= lateral.back()) {
        segment = segment_count - 1;
    } else if (station > lateral.front()) {
        const auto upper =
            std::upper_bound(lateral.begin(), lateral.end(), station);
        segment = static_cast<std::size_t>(upper - lateral.begin()) - 1;
        segment = std::min(segment, segment_count - 1);
    }
    return (vertical[segment + 1] - vertical[segment]) /
           (lateral[segment + 1] - lateral[segment]);
}

}  // namespace

WheelProfilePreprocessing::WheelProfilePreprocessing(
    WheelProfilePreprocessingConfiguration configuration) {
    const double rescan = configuration.equal_arc_length_rescan_step_meters;
    const double rediscretisation =
        configuration.source_lateral_rediscretisation_step_meters;
    if (!std::isfinite(rescan) || rescan < 0.0) {
        Reject("the equal arc-length rescan step is " + std::to_string(rescan) +
               " m; it must be finite and not negative, and zero disables it");
    }
    if (!std::isfinite(rediscretisation) || rediscretisation < 0.0) {
        Reject("the source lateral rediscretisation step is " +
               std::to_string(rediscretisation) +
               " m; it must be finite and not negative, and zero disables it");
    }
    if (rescan > 0.0 && rediscretisation > 0.0) {
        Reject(
            "both the equal arc-length rescan and the source lateral "
            "rediscretisation are selected; they rewrite the same outline, so "
            "at most one may be active");
    }

    if (rescan > 0.0) {
        preparation_ = WheelProfilePreparation::kEqualArcLengthRescan;
        step_meters_ = rescan;
    } else if (rediscretisation > 0.0) {
        preparation_ = WheelProfilePreparation::kSourceLateralRediscretisation;
        step_meters_ = rediscretisation;
    }
}

WheelProfileControlNodes WheelProfilePreprocessing::LayControlNodes(
    const ProfilePoints& authored, WheelSide side) const {
    RequireWheelProfile(authored);
    WheelProfileControlNodes nodes;
    if (preparation_ == WheelProfilePreparation::kAuthoredNodes) {
        return nodes;
    }

    if (preparation_ == WheelProfilePreparation::kEqualArcLengthRescan) {
        // The rescan runs on the physical right-hand ordering, whatever side it
        // is being laid for, and the mirror is applied to its result. Anchoring
        // the station phase on a mirrored list would put the remainder segment
        // at the opposite end of the profile.
        const SideResolvedProfile physical =
            authored.ResolveForSide(WheelSide::kRight);
        const NaturalCubicSpline first_pass(physical.lateral_meters(),
                                            physical.vertical_meters());
        const std::vector<double> cumulative =
            AccumulateProfileArcLength(first_pass, physical.lateral_meters());
        const double total = cumulative.back();
        if (!std::isfinite(total) || !(total > 0.0)) {
            Reject("'" + authored.identifier() +
                   "' has no measurable curve length");
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
        // the asset says the surface ends, and a spline's value there is only
        // equal to that up to rounding.
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

    // Source lateral rediscretisation. The march anchors on the authored first
    // element and runs toward the authored last one, so which end carries the
    // short remainder is a property of how the asset was written.
    const std::span<const double> authored_lateral =
        authored.authored_lateral_meters();
    const double first = authored_lateral.front();
    const double last = authored_lateral.back();
    const double extent = std::abs(last - first);
    if (!(extent > 0.0) || !std::isfinite(extent)) {
        Reject("'" + authored.identifier() +
               "' begins and ends at the same lateral coordinate");
    }
    const double direction = (last > first) ? 1.0 : -1.0;

    std::vector<double> marched;
    marched.push_back(first);
    for (std::size_t station = 1;; ++station) {
        const double travelled = static_cast<double>(station) * step_meters_;
        const double next = first + direction * travelled;
        const double remaining = direction * (last - next);
        if (remaining <= MarchTolerance(first, last, travelled)) {
            break;
        }
        marched.push_back(next);
    }
    marched.push_back(last);

    const SideResolvedProfile resolved = authored.ResolveForSide(side);
    const NaturalCubicSpline surface(resolved.lateral_meters(),
                                     resolved.vertical_meters());

    const double sign = (side == WheelSide::kLeft) ? -1.0 : 1.0;
    for (double& value : marched) {
        value *= sign;
    }
    std::sort(marched.begin(), marched.end());

    std::vector<double> vertical(marched.size(), 0.0);
    for (std::size_t index = 0; index < marched.size(); ++index) {
        vertical[index] = surface.Evaluate(marched[index]);
    }
    nodes.lateral_meters = std::move(marched);
    nodes.vertical_meters = std::move(vertical);
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

    if (preparation_ == WheelProfilePreparation::kEqualArcLengthRescan) {
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

    if (preparation_ == WheelProfilePreparation::kSourceLateralRediscretisation) {
        for (std::size_t index = 0; index < sample_count; ++index) {
            const double station = outline.station_meters[index];
            outline.height_meters[index] = InterpolateControlValue(
                nodes.lateral_meters, nodes.vertical_meters, station);
            outline.height_slope[index] =
                ControlSlope(nodes.lateral_meters, nodes.vertical_meters, station);
        }
        return outline;
    }

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
