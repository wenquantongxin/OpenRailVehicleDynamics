#include "orvd/configuration/irw_full_state_control_observation_binding.h"

#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>

#include "irw_closed_topology.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"
#include "orvd/track_geometry/track_frame_pose.h"
#include "orvd/wheel_rail_contact/roll_yaw_pitch.h"

namespace orvd::configuration {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument(
        "IRW full-state control observation binding: " + detail);
}

template <std::size_t Size>
void RequireFinite(const std::array<double, Size>& values,
                   const std::string& name) {
    for (const double value : values) {
        if (!std::isfinite(value)) {
            Reject(name + " contains a non-finite value");
        }
    }
}

}  // namespace

IrwFullStateControlObservationBinding::ResolvedBinding
IrwFullStateControlObservationBinding::ResolveBinding(
    const AssembledVehicleSystem& assembled) {
    const auto* contact_plan = assembled.contact_force_plan();
    const auto* torque_plan = assembled.active_torque_plan();
    if (contact_plan == nullptr || torque_plan == nullptr ||
        contact_plan->carrier_count() !=
            static_cast<int>(internal::kIrwClosedCarrierNames.size()) ||
        contact_plan->interface_count() !=
            static_cast<int>(internal::kIrwClosedInterfaces.size()) ||
        torque_plan->channel_count() !=
            static_cast<int>(internal::kIrwClosedInterfaces.size()) ||
        assembled.system().held_active_torque_count() !=
            static_cast<int>(internal::kIrwClosedInterfaces.size())) {
        Reject("the assembled system is not the closed four-carrier, "
               "eight-wheel IRW contact and active-torque topology");
    }

    for (std::size_t ordinal = 0;
         ordinal < internal::kIrwClosedCarrierNames.size(); ++ordinal) {
        if (contact_plan->carrier_name(static_cast<int>(ordinal)) !=
            internal::kIrwClosedCarrierNames[ordinal]) {
            Reject("the contact carrier order differs from the IRW control "
                   "order");
        }
    }
    for (std::size_t ordinal = 0;
         ordinal < internal::kIrwClosedInterfaces.size(); ++ordinal) {
        const auto& expected = internal::kIrwClosedInterfaces[ordinal];
        const int index = static_cast<int>(ordinal);
        if (contact_plan->interface_name(index) != expected.interface_name ||
            torque_plan->channel_name(index) != expected.interface_name ||
            torque_plan->axis_provider_body_name(index) !=
                expected.carrier_body_name ||
            torque_plan->wheel_body_name(index) != expected.wheel_body_name ||
            torque_plan->reaction_frame_body_name(index) !=
                expected.reaction_frame_body_name) {
            Reject("contact, active-torque and control channel " +
                   std::to_string(ordinal) +
                   " do not share the IRW identity");
        }
    }

    const auto& model = assembled.model();
    ResolvedBinding binding{
        .carrier_bodies = {
            model.GetRigidBodyByName(internal::kIrwClosedCarrierNames[0]),
            model.GetRigidBodyByName(internal::kIrwClosedCarrierNames[1]),
            model.GetRigidBodyByName(internal::kIrwClosedCarrierNames[2]),
            model.GetRigidBodyByName(internal::kIrwClosedCarrierNames[3]),
        },
        .wheel_velocity_ranges = {
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[0].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[1].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[2].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[3].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[4].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[5].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[6].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[7].revolute_joint_name)),
        },
    };
    for (const auto& range : binding.wheel_velocity_ranges) {
        if (range.size() != 1) {
            Reject("an independent-wheel joint does not own exactly one "
                   "generalized velocity");
        }
    }
    return binding;
}

IrwFullStateControlObservationBinding::
    IrwFullStateControlObservationBinding(
        const AssembledVehicleSystem& assembled)
    : IrwFullStateControlObservationBinding(assembled,
                                            ResolveBinding(assembled)) {}

IrwFullStateControlObservationBinding::
    IrwFullStateControlObservationBinding(
        const AssembledVehicleSystem& assembled, ResolvedBinding binding)
    : assembled_(&assembled),
      carrier_bodies_(binding.carrier_bodies),
      wheel_velocity_ranges_(binding.wheel_velocity_ranges) {}

IrwFullStateControlMechanicalObservation
IrwFullStateControlObservationBinding::Observe(
    system_assembly::SystemRuntimeContext& context) const {
    const std::span<const double> stations =
        context.wheel_rail_projection_station_hints_meters();
    if (stations.size() != carrier_bodies_.size()) {
        Reject("the context does not hold four carrier projection stations");
    }
    auto component = assembled_->system().GetMultibodyComponentView(
        context, assembled_->system().multibody_component());
    const auto& model = assembled_->model();
    const auto& line = assembled_->contact_force_plan()->track_geometry();

    IrwFullStateControlMechanicalObservation observation;
    for (std::size_t axle = 0; axle < carrier_bodies_.size(); ++axle) {
        const double station = stations[axle];
        if (!std::isfinite(station)) {
            Reject("a carrier projection station is not finite");
        }
        const auto track = line.EvaluateTrackFrame(station);
        const auto body_pose =
            model.CalcPoseInWorld(component.context(), carrier_bodies_[axle]);
        const Eigen::Matrix3d rotation_track_from_inertial =
            track.pose().rotation_inertial_from_track().transpose();
        const Eigen::Vector3d body_origin_in_track =
            rotation_track_from_inertial *
            (body_pose.translation() -
             track.pose().origin_in_inertial_meters());
        const Eigen::Matrix3d rotation_track_from_orvd_body =
            rotation_track_from_inertial * body_pose.rotation();
        // The control convention uses the source physical axle-bridge basis;
        // the multibody assembly deliberately retains the WRL/Drake basis.
        const auto source_angles = wheel_rail_contact::ResolveRollYawPitch(
            rotation_track_from_orvd_body *
            internal::kIrwBodyBasisHalfTurn);
        observation.axle_track_stations_meters[axle] = station;
        observation.mechanical_input.axle_lateral_displacements_meters[axle] =
            body_origin_in_track.y();
        observation.mechanical_input.axle_yaw_angles_radians[axle] =
            source_angles.yaw_radians;
    }

    const Eigen::VectorXd& velocities = context.generalized_velocities();
    for (std::size_t wheel = 0; wheel < wheel_velocity_ranges_.size();
         ++wheel) {
        // The control scalar is the negative of the revolute rate about the
        // axle bridge's local +Y joint axis.
        observation.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                [wheel] = -velocities[wheel_velocity_ranges_[wheel].start()];
    }
    RequireFinite(observation.axle_track_stations_meters,
                  "axle track stations");
    RequireFinite(observation.mechanical_input.axle_lateral_displacements_meters,
                  "axle lateral displacements");
    RequireFinite(observation.mechanical_input.axle_yaw_angles_radians,
                  "axle yaw angles");
    RequireFinite(
        observation.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        "wheel angular speeds");
    return observation;
}

}  // namespace orvd::configuration
