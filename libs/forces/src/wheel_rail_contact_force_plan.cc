#include "orvd/forces/wheel_rail_contact_force_plan.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Geometry>

#include "orvd/multibody_model/multibody_frame_spatial_velocity.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"
#include "orvd/wheel_rail_contact/contact_wrench.h"

namespace orvd::forces {
namespace {

using multibody_model::AppliedBodyWrench;
using multibody_model::MultibodyEvaluationContext;
using multibody_model::MultibodyModel;
using wheel_rail_contact::BuildContactPoseScalars;
using wheel_rail_contact::ContactPoseScalars;
using wheel_rail_contact::ProfileTrackRollTransport;
using wheel_rail_contact::PlaceRailProfileLongitudinalOrigin;
using wheel_rail_contact::RailProfileFrame;
using wheel_rail_contact::RailProfileOriginMode;
using wheel_rail_contact::ResolveRollYawPitch;
using wheel_rail_contact::ResolveRollYawPitchRates;
using wheel_rail_contact::RollYawPitchAngles;
using wheel_rail_contact::RollYawPitchRates;
using wheel_rail_contact::SpatialWrench;
using wheel_rail_contact::TrackIrregularity;
using wheel_rail_contact::WheelProfileRigidMotion;
using wheel_rail_contact::WheelRailContactInput;
using wheel_rail_contact::WheelRailContactResult;
using wheel_rail_contact::WheelRailPoseInput;
using wheel_rail_contact::WheelSide;
using wheel_rail_contact::WheelsetPlacement;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("wheel-rail contact force plan: " + detail);
}

Eigen::Matrix3d RotationAboutX(double radians) {
    return Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitX())
        .toRotationMatrix();
}

Eigen::Matrix3d RotationAboutZ(double radians) {
    return Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ())
        .toRotationMatrix();
}

void RequireFinite(const Eigen::Vector3d& value, std::string_view what) {
    if (!value.allFinite()) {
        std::string message("wheel-rail contact force plan: ");
        message.append(what);
        message += " is not finite";
        throw std::runtime_error(message);
    }
}

void RequireFinite(const Eigen::Matrix3d& value, std::string_view what) {
    if (!value.allFinite()) {
        std::string message("wheel-rail contact force plan: ");
        message.append(what);
        message += " is not finite";
        throw std::runtime_error(message);
    }
}

std::string SideName(WheelSide side) {
    return side == WheelSide::kRight ? "right" : "left";
}

}  // namespace

WheelRailContactForceWorkspace::WheelRailContactForceWorkspace(
    const WheelRailContactForcePlan* issuer, std::size_t carrier_count,
    std::size_t interface_count)
    : issuer_(issuer),
      carriers_(carrier_count),
      pending_wrenches_(interface_count) {}

WheelRailContactForceWorkspace::~WheelRailContactForceWorkspace() = default;

WheelRailContactForcePlan::WheelRailContactForcePlan(
    const MultibodyModel& model, track_geometry::TrackGeometry line,
    std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality>
        personality,
    std::vector<WheelRailContactCarrierDefinition> carriers,
    std::vector<WheelRailContactInterfaceDefinition> interfaces,
    double projection_search_half_width_meters)
    : model_(&model),
      line_(std::move(line)),
      personality_(std::move(personality)),
      projection_search_half_width_meters_(
          projection_search_half_width_meters) {
    if (!model.is_finalized()) {
        throw std::logic_error(
            "WheelRailContactForcePlan requires a finalized multibody model");
    }
    if (personality_ == nullptr) {
        Reject("the runtime personality is null");
    }
    if (!std::isfinite(projection_search_half_width_meters_) ||
        !(projection_search_half_width_meters_ > 0.0)) {
        Reject("the projection search half width must be finite and positive");
    }
    if (carriers.empty()) {
        Reject("at least one contact carrier is required");
    }
    if (interfaces.empty()) {
        Reject("at least one wheel-rail interface is required");
    }

    std::unordered_map<std::string, std::size_t> carrier_ordinal;
    carrier_ordinal.reserve(carriers.size());
    carriers_.reserve(carriers.size());
    for (WheelRailContactCarrierDefinition& carrier : carriers) {
        if (carrier.carrier_name.empty() || carrier.body_name.empty()) {
            Reject("a carrier name and its body name must both be non-empty");
        }
        if (!carrier_ordinal
                 .emplace(carrier.carrier_name, carriers_.size())
                 .second) {
            Reject("more than one carrier is named '" + carrier.carrier_name +
                   "'");
        }
        if (!std::isfinite(carrier.initial_projection_station_meters)) {
            Reject("carrier '" + carrier.carrier_name +
                   "' has a non-finite initial projection station");
        }
        const double lower = carrier.initial_projection_station_meters -
                             projection_search_half_width_meters_;
        const double upper = carrier.initial_projection_station_meters +
                             projection_search_half_width_meters_;
        if (lower < line_.start_track_station_meters() ||
            upper > line_.end_track_station_meters()) {
            Reject("carrier '" + carrier.carrier_name +
                   "' has an initial projection window outside the line");
        }
        carriers_.push_back(CarrierBinding{
            std::move(carrier.carrier_name),
            model.GetRigidBodyByName(carrier.body_name),
            carrier.initial_projection_station_meters});
    }

    std::unordered_set<std::string> interface_names;
    std::unordered_set<std::string> carrier_side_pairs;
    std::vector<bool> carrier_is_used(carriers_.size(), false);
    interfaces_.reserve(interfaces.size());
    for (WheelRailContactInterfaceDefinition& interface : interfaces) {
        if (interface.interface_name.empty()) {
            Reject("an interface name is empty");
        }
        if (!interface_names.insert(interface.interface_name).second) {
            Reject("more than one interface is named '" +
                   interface.interface_name + "'");
        }
        const auto carrier = carrier_ordinal.find(interface.carrier_name);
        if (carrier == carrier_ordinal.end()) {
            Reject("interface '" + interface.interface_name +
                   "' names unknown carrier '" + interface.carrier_name +
                   "'");
        }
        const std::string pair_key =
            interface.carrier_name + "\n" + SideName(interface.side);
        if (!carrier_side_pairs.insert(pair_key).second) {
            Reject("carrier '" + interface.carrier_name +
                   "' has more than one " + SideName(interface.side) +
                   " interface");
        }
        if (interface.wheel_body_name.empty()) {
            Reject("interface '" + interface.interface_name +
                   "' has an empty wheel body name");
        }
        carrier_is_used[carrier->second] = true;
        const multibody_model::RigidBodyHandle wheel_body =
            model.GetRigidBodyByName(interface.wheel_body_name);
        if (wheel_body != carriers_[carrier->second].body) {
            Reject("interface '" + interface.interface_name +
                   "' names a wheel body different from its projection "
                   "carrier; split carrier/wheel kinematics is not part of "
                   "the GZ18 force-plan contract");
        }
        interfaces_.push_back(InterfaceBinding{
            std::move(interface.interface_name), carrier->second, wheel_body,
            interface.side});
    }
    for (std::size_t ordinal = 0; ordinal < carriers_.size(); ++ordinal) {
        if (!carrier_is_used[ordinal]) {
            Reject("carrier '" + carriers_[ordinal].name +
                   "' has no wheel-rail interface");
        }
    }
}

std::string_view WheelRailContactForcePlan::carrier_name(int index) const {
    if (index < 0 || index >= carrier_count()) {
        Reject("the requested carrier index is out of range");
    }
    return carriers_[static_cast<std::size_t>(index)].name;
}

std::string_view WheelRailContactForcePlan::interface_name(int index) const {
    if (index < 0 || index >= interface_count()) {
        Reject("the requested interface index is out of range");
    }
    return interfaces_[static_cast<std::size_t>(index)].name;
}

double WheelRailContactForcePlan::initial_projection_station_meters(
    int index) const {
    if (index < 0 || index >= carrier_count()) {
        Reject("the requested carrier index is out of range");
    }
    return carriers_[static_cast<std::size_t>(index)]
        .initial_projection_station_meters;
}

std::unique_ptr<WheelRailContactForceWorkspace>
WheelRailContactForcePlan::CreateWorkspace() const {
    auto workspace = std::unique_ptr<WheelRailContactForceWorkspace>(
        new WheelRailContactForceWorkspace(this, carriers_.size(),
                                           interfaces_.size()));
    personality_->model(WheelSide::kRight)
        .PrepareWorkspace(workspace->contact_workspace_);
    personality_->model(WheelSide::kLeft)
        .PrepareWorkspace(workspace->contact_workspace_);
    return workspace;
}

void WheelRailContactForcePlan::CalcAppliedForces(
    const MultibodyEvaluationContext& context,
    WheelRailContactForceWorkspace& workspace,
    std::span<AppliedBodyWrench> body_wrenches) const {
    if (workspace.issuer_ != this) {
        Reject("the workspace belongs to a different contact force plan");
    }
    if (body_wrenches.size() != interfaces_.size()) {
        Reject("the output span has " + std::to_string(body_wrenches.size()) +
               " entries, but this plan has " +
               std::to_string(interfaces_.size()) + " interfaces");
    }

    // Four GZ18 carriers, each evaluated once.  The plan keeps this first pass
    // separate from interface evaluation so left and right cannot silently
    // project the same wheelset onto different branches.
    for (std::size_t ordinal = 0; ordinal < carriers_.size(); ++ordinal) {
        const CarrierBinding& binding = carriers_[ordinal];
        WheelRailContactForceWorkspace::CarrierScratch& scratch =
            workspace.carriers_[ordinal];
        const auto body_pose = model_->CalcPoseInWorld(context, binding.body);
        const auto body_velocity =
            model_->CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                context, binding.body);
        scratch.body_origin_in_inertial_meters = body_pose.translation();
        scratch.rotation_inertial_from_body = body_pose.rotation();
        scratch.body_origin_velocity_in_inertial_meters_per_second =
            body_velocity
                .translational_velocity_at_frame_origin_meters_per_second();
        scratch.body_angular_velocity_in_inertial_radians_per_second =
            body_velocity.angular_velocity_radians_per_second();

        const auto projection = line_.ProjectPointNearSeed(
            scratch.body_origin_in_inertial_meters,
            binding.initial_projection_station_meters,
            projection_search_half_width_meters_);
        scratch.track_station_meters = projection.track_station_meters();
        const auto track = line_.EvaluateTrackFrame(scratch.track_station_meters);
        scratch.track_origin_in_inertial_meters =
            track.pose().origin_in_inertial_meters();
        scratch.rotation_inertial_from_track =
            track.pose().rotation_inertial_from_track();
        const Eigen::Matrix3d rotation_track_from_inertial =
            scratch.rotation_inertial_from_track.transpose();
        scratch.body_origin_in_track_meters =
            rotation_track_from_inertial *
            (scratch.body_origin_in_inertial_meters -
             scratch.track_origin_in_inertial_meters);
        scratch.body_origin_velocity_in_track_meters_per_second =
            rotation_track_from_inertial *
            scratch.body_origin_velocity_in_inertial_meters_per_second;
        scratch.body_angular_velocity_in_track_radians_per_second =
            rotation_track_from_inertial *
            scratch.body_angular_velocity_in_inertial_radians_per_second;

        const Eigen::Matrix3d rotation_track_from_body =
            rotation_track_from_inertial *
            scratch.rotation_inertial_from_body;
        const RollYawPitchAngles angles =
            ResolveRollYawPitch(rotation_track_from_body);
        scratch.roll_radians = angles.roll_radians;
        scratch.yaw_radians = angles.yaw_radians;

        const Eigen::Vector3d centerline_derivative_in_track =
            rotation_track_from_inertial *
            track.centerline_derivative_in_inertial_meters_per_meter();
        const Eigen::Vector3d track_rotation_rate_in_track =
            rotation_track_from_inertial *
            track.track_frame_rotation_rate_in_inertial_radians_per_meter();
        const Eigen::Vector3d transport =
            track_rotation_rate_in_track.cross(
                scratch.body_origin_in_track_meters);
        const double station_denominator =
            centerline_derivative_in_track.x() + transport.x();
        if (!std::isfinite(station_denominator) ||
            std::abs(station_denominator) < 1.0e-12) {
            throw std::runtime_error(
                "wheel-rail contact force plan: carrier '" + binding.name +
                "' has an unusable station-rate denominator");
        }
        scratch.station_rate_meters_per_second =
            scratch.body_origin_velocity_in_track_meters_per_second.x() /
            station_denominator;
        scratch.path_rate_meters_per_second =
            track.centerline_derivative_in_inertial_meters_per_meter().norm() *
            scratch.station_rate_meters_per_second;

        const Eigen::Vector3d relative_angular_velocity_in_track =
            scratch.body_angular_velocity_in_track_radians_per_second -
            track_rotation_rate_in_track *
                scratch.station_rate_meters_per_second;
        const RollYawPitchRates rates = ResolveRollYawPitchRates(
            relative_angular_velocity_in_track, scratch.roll_radians,
            scratch.yaw_radians);
        scratch.wheel_pitch_rate_radians_per_second =
            rates.pitch_rate_radians_per_second;
        scratch.curvature_radians_per_meter =
            line_.CurvatureRadiansPerMeter(scratch.track_station_meters);

        RequireFinite(scratch.body_origin_in_track_meters,
                      "carrier position in the track frame");
        RequireFinite(scratch.body_origin_velocity_in_track_meters_per_second,
                      "carrier velocity in the track frame");
        if (!std::isfinite(scratch.station_rate_meters_per_second) ||
            !std::isfinite(scratch.path_rate_meters_per_second) ||
            !std::isfinite(scratch.roll_radians) ||
            !std::isfinite(scratch.yaw_radians) ||
            !std::isfinite(scratch.wheel_pitch_rate_radians_per_second)) {
            throw std::runtime_error(
                "wheel-rail contact force plan: carrier '" + binding.name +
                "' produced non-finite track kinematics");
        }
    }

    for (std::size_t ordinal = 0; ordinal < interfaces_.size(); ++ordinal) {
        const InterfaceBinding& interface = interfaces_[ordinal];
        const auto& carrier = workspace.carriers_[interface.carrier_ordinal];
        const auto& constants = personality_->pose_constants(interface.side);
        const double sigma = constants.wheel_lateral_datum_meters;
        const double longitudinal_offset =
            -std::sin(carrier.yaw_radians) * sigma;
        const double lateral_at_profile =
            carrier.body_origin_in_track_meters.y() +
            std::cos(carrier.yaw_radians) * sigma;
        const double effective_denominator =
            1.0 - carrier.curvature_radians_per_meter * lateral_at_profile;
        if (!std::isfinite(effective_denominator) ||
            std::abs(effective_denominator) < 1.0e-12) {
            throw std::runtime_error(
                "wheel-rail contact force plan: interface '" +
                interface.name +
                "' has an unusable effective-station denominator");
        }
        const double effective_station =
            carrier.track_station_meters +
            longitudinal_offset / effective_denominator;
        const auto profile_track = line_.EvaluateTrackFrame(effective_station);
        const Eigen::Matrix3d rotation_track_from_inertial =
            carrier.rotation_inertial_from_track.transpose();

        const ProfileTrackRollTransport roll_transport =
            personality_->roll_transport_strategy().Compute(
                carrier.track_origin_in_inertial_meters,
                carrier.rotation_inertial_from_track,
                carrier.body_origin_in_track_meters,
                profile_track.pose().origin_in_inertial_meters(),
                profile_track.pose().rotation_inertial_from_track());

        WheelRailPoseInput pose_input;
        pose_input.placement = WheelsetPlacement{
            carrier.body_origin_in_track_meters.y(),
            carrier.body_origin_in_track_meters.z(), carrier.roll_radians,
            carrier.yaw_radians};
        pose_input.arc_rate_meters_per_second =
            carrier.path_rate_meters_per_second;
        pose_input.irregularity = TrackIrregularity{};

        const Eigen::Matrix3d rotation_track_from_profile =
            RotationAboutX(carrier.roll_radians) *
            RotationAboutZ(carrier.yaw_radians);

        const Eigen::Vector3d& wheel_body_origin_in_track =
            carrier.body_origin_in_track_meters;
        const Eigen::Vector3d& wheel_body_velocity_in_track =
            carrier.body_origin_velocity_in_track_meters_per_second;
        const Eigen::Vector3d& wheel_angular_velocity_in_track =
            carrier.body_angular_velocity_in_track_radians_per_second;
        const Eigen::Vector3d wheel_profile_offset_in_track =
            rotation_track_from_profile *
            Eigen::Vector3d(0.0, constants.wheel_lateral_datum_meters, 0.0);

        WheelProfileRigidMotion wheel_motion;
        wheel_motion.origin_track_meters =
            wheel_body_origin_in_track + wheel_profile_offset_in_track;
        wheel_motion.rotation_track_from_profile =
            rotation_track_from_profile;
        wheel_motion.origin_velocity_track_meters_per_second =
            wheel_body_velocity_in_track +
            wheel_angular_velocity_in_track.cross(
                wheel_profile_offset_in_track);
        wheel_motion.angular_velocity_track_radians_per_second =
            wheel_angular_velocity_in_track;
        wheel_motion.arc_rate_meters_per_second =
            carrier.path_rate_meters_per_second;
        wheel_motion.wheel_pitch_rate_radians_per_second =
            carrier.wheel_pitch_rate_radians_per_second;

        const Eigen::Vector3d rail_offset_at_profile_station(
            0.0, constants.rail_lateral_datum_meters,
            constants.rail_vertical_datum_meters);
        const Eigen::Vector3d rail_origin_in_inertial =
            profile_track.pose().origin_in_inertial_meters() +
            profile_track.pose().rotation_inertial_from_track() *
                rail_offset_at_profile_station;
        RailProfileFrame rail_frame;
        rail_frame.origin_track_meters =
            rotation_track_from_inertial *
            (rail_origin_in_inertial -
             carrier.track_origin_in_inertial_meters);
        rail_frame.rotation_track_from_profile =
            rotation_track_from_inertial *
            profile_track.pose().rotation_inertial_from_track() *
            RotationAboutX(constants.rail_roll_radians);

        if (personality_->rail_profile_origin_mode() ==
            RailProfileOriginMode::kProfileCoordinate) {
            rail_frame.origin_track_meters =
                PlaceRailProfileLongitudinalOrigin(
                    personality_->rail_profile_origin_mode(),
                    rail_frame.origin_track_meters,
                    rail_frame.rotation_track_from_profile,
                    wheel_body_origin_in_track,
                    effective_station - carrier.track_station_meters);
        }

        RequireFinite(wheel_motion.origin_track_meters,
                      "wheel profile origin");
        RequireFinite(wheel_motion.rotation_track_from_profile,
                      "wheel profile rotation");
        RequireFinite(wheel_motion.origin_velocity_track_meters_per_second,
                      "wheel profile origin velocity");
        RequireFinite(
            wheel_motion.angular_velocity_track_radians_per_second,
            "wheel angular velocity");
        RequireFinite(rail_frame.origin_track_meters, "rail profile origin");
        RequireFinite(rail_frame.rotation_track_from_profile,
                      "rail profile rotation");

        WheelRailContactInput input;
        input.pose = BuildContactPoseScalars(
            constants, pose_input, roll_transport);
        input.rail_frame = rail_frame;
        input.wheel = wheel_motion;
        input.roll_transport = roll_transport;
        const WheelRailContactResult result =
            personality_->model(interface.side)
                .Evaluate(input, workspace.contact_workspace_);
        if (result.count > result.patches.size()) {
            throw std::runtime_error(
                "wheel-rail contact force plan: interface '" +
                interface.name + "' returned too many contact patches");
        }

        SpatialWrench accumulated_in_track;
        for (std::size_t patch = 0; patch < result.count; ++patch) {
            const auto& wheel_wrench =
                result.patches[patch].wrench.rail_on_wheel;
            const Eigen::Vector3d& contact_point =
                result.patches[patch].wrench.contact_point_meters;
            RequireFinite(contact_point, "contact point");
            RequireFinite(wheel_wrench.force_newtons, "contact force");
            RequireFinite(wheel_wrench.moment_newton_meters,
                          "contact moment");
            const SpatialWrench at_body_origin =
                wheel_rail_contact::TransportWrench(
                    wheel_wrench, contact_point,
                    wheel_body_origin_in_track);
            accumulated_in_track.force_newtons +=
                at_body_origin.force_newtons;
            accumulated_in_track.moment_newton_meters +=
                at_body_origin.moment_newton_meters;
        }
        const SpatialWrench accumulated_in_inertial =
            wheel_rail_contact::RotateWrench(
                accumulated_in_track,
                carrier.rotation_inertial_from_track);
        RequireFinite(accumulated_in_inertial.force_newtons,
                      "aggregated contact force");
        RequireFinite(accumulated_in_inertial.moment_newton_meters,
                      "aggregated contact moment");
        workspace.pending_wrenches_[ordinal] = AppliedBodyWrench{
            interface.wheel_body, Eigen::Vector3d::Zero(),
            model_->world_frame(),
            accumulated_in_inertial.moment_newton_meters,
            accumulated_in_inertial.force_newtons};
    }

    std::copy(workspace.pending_wrenches_.begin(),
              workspace.pending_wrenches_.end(), body_wrenches.begin());
}

}  // namespace orvd::forces
