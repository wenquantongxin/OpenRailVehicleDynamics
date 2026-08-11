#include "orvd/configuration/irw_full_state_control_event_session.h"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "irw_closed_topology.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"
#include "orvd/track_geometry/track_frame_pose.h"
#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"

namespace orvd::configuration {
namespace {

constexpr double kFrozenSamplePeriodSeconds = 0.01;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("IRW full-state control event session: " +
                                detail);
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
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

IrwFullStateControlEventSession::ResolvedBinding
IrwFullStateControlEventSession::ResolveBinding(
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
            Reject("the contact carrier order differs from the frozen IRW "
                   "control order");
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
                   " do not share the frozen IRW identity");
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

IrwFullStateControlEventSession::IrwFullStateControlEventSession(
    const AssembledVehicleSystem& assembled,
    control::IrwFullStateWheelSpeedGuidanceController controller,
    actuation::WheelDriveTorqueCommandConditioner conditioner)
    : IrwFullStateControlEventSession(
          assembled, std::move(controller), std::move(conditioner),
          ResolveBinding(assembled)) {}

IrwFullStateControlEventSession::IrwFullStateControlEventSession(
    const AssembledVehicleSystem& assembled,
    control::IrwFullStateWheelSpeedGuidanceController controller,
    actuation::WheelDriveTorqueCommandConditioner conditioner,
    ResolvedBinding binding)
    : assembled_(&assembled),
      controller_(std::move(controller)),
      conditioner_(std::move(conditioner)),
      carrier_bodies_(binding.carrier_bodies),
      wheel_velocity_ranges_(binding.wheel_velocity_ranges),
      sample_period_seconds_(controller_.config().sample_period_seconds) {
    const double conditioner_period =
        conditioner_.config().sample_period_seconds;
    if (!SameBits(sample_period_seconds_, conditioner_period) ||
        !SameBits(sample_period_seconds_, kFrozenSamplePeriodSeconds)) {
        Reject("the controller and conditioner sample periods must be "
               "bitwise-equal frozen 0.01-second values");
    }
}

double IrwFullStateControlEventSession::next_periodic_event_time_seconds()
    const noexcept {
    return static_cast<double>(next_periodic_event_ordinal_) *
           sample_period_seconds_;
}

control::IrwFullStateWheelSpeedGuidanceControllerInput
IrwFullStateControlEventSession::ObserveMechanicalInput(
    system_assembly::SystemRuntimeContext& context) const {
    const std::span<const double> stations =
        context.wheel_rail_projection_station_hints_meters();
    if (stations.size() != carrier_bodies_.size()) {
        Reject("the context does not hold four carrier projection "
               "stations");
    }
    auto component = assembled_->system().GetMultibodyComponentView(
        context, assembled_->system().multibody_component());
    const auto& model = assembled_->model();
    const auto& line = assembled_->contact_force_plan()->track_geometry();

    control::IrwFullStateWheelSpeedGuidanceControllerInput input;
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
        // P179's frozen controller observes the source physical axle-bridge
        // body basis, whereas ORVD deliberately retained the WRL/Drake body
        // basis for multibody assembly. Restore that source basis here rather
        // than absorbing the sign into the controller asset. The body origin,
        // and therefore its Track-T lateral displacement, is basis-independent.
        const auto source_angles = wheel_rail_contact::ResolveRollYawPitch(
            rotation_track_from_orvd_body *
            internal::kIrwBodyBasisHalfTurn);
        input.axle_track_stations_meters[axle] = station;
        input.axle_lateral_displacements_meters[axle] =
            body_origin_in_track.y();
        input.axle_yaw_angles_radians[axle] = source_angles.yaw_radians;
    }

    const Eigen::VectorXd& velocities =
        context.generalized_velocities();
    for (std::size_t wheel = 0; wheel < wheel_velocity_ranges_.size();
         ++wheel) {
        // The frozen controller and conditioner scalar is the negative of the
        // revolute rate about the axle bridge's local +Y joint axis. The
        // controller asset retains its own independent sign arrays.
        input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                [wheel] = -velocities[wheel_velocity_ranges_[wheel].start()];
    }
    RequireFinite(input.axle_track_stations_meters, "axle track stations");
    RequireFinite(input.axle_lateral_displacements_meters,
                  "axle lateral displacements");
    RequireFinite(input.axle_yaw_angles_radians, "axle yaw angles");
    RequireFinite(
        input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        "wheel angular speeds");
    return input;
}

IrwFullStateControlEventAudit
IrwFullStateControlEventSession::ComputeCandidate(
    IrwFullStateControlEventKind kind, std::uint64_t ordinal,
    double event_time_seconds,
    system_assembly::SystemRuntimeContext& accepted_context) const {
    IrwFullStateControlEventAudit audit;
    audit.kind = kind;
    audit.periodic_event_ordinal = ordinal;
    audit.event_time_seconds = event_time_seconds;
    audit.mechanical_input = ObserveMechanicalInput(accepted_context);
    audit.controller_state_before = controller_state_;
    audit.conditioner_memory_before_newton_metres =
        conditioner_memory_newton_metres_;
    audit.controller_result =
        controller_.Step(audit.mechanical_input, controller_state_);
    audit.conditioning_result = conditioner_.Step(
        audit.controller_result.requested_wheel_torques_newton_metres,
        audit.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        conditioner_memory_newton_metres_);
    return audit;
}

IrwFullStateControlEventAudit
IrwFullStateControlEventSession::ApplyInitializationUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ != Phase::kInitializationRequired) {
        Reject("the initialization recurrence has already been committed");
    }
    if (!SameBits(accepted_context.time_seconds(), 0.0)) {
        Reject("the initialization recurrence requires accepted time zero");
    }
    IrwFullStateControlEventAudit audit = ComputeCandidate(
        IrwFullStateControlEventKind::kInitialization, 0, 0.0,
        accepted_context);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    phase_ = Phase::kPeriodicEventRequired;
    return audit;
}

IrwFullStateControlEventAudit
IrwFullStateControlEventSession::ApplyPeriodicUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ == Phase::kInitializationRequired) {
        Reject("the initialization recurrence has not been committed");
    }
    if (phase_ == Phase::kSynchronizationRequired) {
        Reject("the preceding event is committed but its backend is not "
               "synchronized");
    }
    const double expected_time = next_periodic_event_time_seconds();
    if (!SameBits(accepted_context.time_seconds(), expected_time)) {
        Reject("accepted time does not equal the next integer-grid event "
               "time");
    }

    // Once a valid boundary has been presented, any calculation or commit
    // failure leaves this event due and therefore blocks another advance.
    phase_ = Phase::kPeriodicEventRequired;
    IrwFullStateControlEventAudit audit = ComputeCandidate(
        IrwFullStateControlEventKind::kPeriodic,
        next_periodic_event_ordinal_, expected_time, accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    ++next_periodic_event_ordinal_;
    phase_ = Phase::kSynchronizationRequired;
    return audit;
}

bool IrwFullStateControlEventSession::synchronization_required() const
    noexcept {
    return phase_ == Phase::kSynchronizationRequired;
}

void IrwFullStateControlEventSession::ConfirmBackendSynchronized() {
    if (phase_ != Phase::kSynchronizationRequired) {
        Reject("there is no committed event awaiting backend "
               "synchronization");
    }
    phase_ = Phase::kReadyToAdvance;
}

void IrwFullStateControlEventSession::RequireReadyToAdvance() const {
    if (phase_ != Phase::kReadyToAdvance) {
        Reject("the next vehicle advance is blocked until the required "
               "control event and backend synchronization succeed");
    }
}

}  // namespace orvd::configuration
